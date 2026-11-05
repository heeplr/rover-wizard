"""rover_wizard.steps.verify - the "Closed-loop verification & fine tuning" step."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from ..motion.step_tuning import GainTuner, step_response_metrics
from ..rover_defs import GCS_PID_MASK_STEERING, GCS_PID_MASK_THROTTLE, GRAVITY, STEER_SPEED_MIN, TURN_MAX_G_MIN
from ..util.errors import VehicleError
from ..util.values import format_value
from .util.driving import (
    DEFAULT_CIRCLE_DIAMETER_M,
    DEFAULT_RUNWAY_M,
    GAIN_DECIMALS,
    RUNWAY_USE_FRACTION,
    who_drives_field,
)
from ..model import Action, Step, param_field, value_field
from .util.rover_hardware import is_motorised
from .util.ekf_measurement import EKF_ACTION, ekf_fields, log_ekf_health

DEFAULT_TUNING_ROUNDS = 6
MAX_TUNING_ROUNDS = 15

# ArduPilot aimed for less than 95 % of the requested target: say why
AIMED_BELOW_REQUESTED = 0.95
# ... naming the limit within 5 % of what it aimed for
LIMIT_MATCH = 0.05

# [m/s]
DEFAULT_SPEED_STEP = 1.0

# [deg/s]
DEFAULT_YAW_RATE_STEP_DEG = 45

# [Hz] samples per second of a tuning run: more samples average the noise of a gyro better
TUNING_SAMPLE_RATE_HZ = 20

# manual tracking report: ignore stick inputs below 20 % (noise, dead-band)
TRACKING_MIN_INPUT_FRACTION = 0.2

# ... and needs at least this many input/response pairs
TRACKING_MIN_PAIRS = 20

# speed step: standing still before the step ...
STEP_LEAD_SECONDS = 1.0

# ... holding the target speed at most this long ...
STEP_MAX_HOLD_SECONDS = 6.0

# ... and at least this long (else the runway is too short) ...
STEP_MIN_HOLD_SECONDS = 2.0

# ... and stopping again
STEP_STOP_SECONDS = 3.0

# hard distance limit of a speed step run
STEP_RUNWAY_FRACTION = 0.9

# [m/s] a front-wheel-steered car must move to turn
MIN_GUIDED_TURN_SPEED = 0.5

# ask for at most 70 % of speed / minimum turn radius
CAR_TURN_RATE_FRACTION = 0.7

YAW_STEP_START_SECONDS = 1.5

YAW_STEP_END_SECONDS = 5.5

YAW_STEP_RUN_SECONDS = 8.5

# default speed step: 60 % of the cruise speed ...
SPEED_STEP_FRACTION = 0.6

# ... but at least 0.3 m/s
MIN_SPEED_STEP = 0.3

# [m/s] used while no cruise speed is known
FALLBACK_CRUISE_SPEED = 1.0

# default yaw-rate step: half of ACRO_TURN_RATE ...
YAW_STEP_FRACTION = 0.5

# [deg/s] ... or of this while it is unknown
FALLBACK_ACRO_TURN_RATE = 90


def guided_speed_limit(ctx) -> Optional[float]:
    """The fastest Guided mode drives: WP_SPEED (Rover's Mode::get_speed_default -> AR_WPNav), None if unknown.

    Faster speed targets are silently capped by the firmware, so a speed step above it would be misread as a slow
    response.
    """
    if not ctx.vehicle.has_parameter("WP_SPEED"):
        return None
    return float(ctx.param_value("WP_SPEED", 0) or 0)


def _ensure_guided_can_reach(ctx, speed: float) -> None:
    """Refuse a Guided test speed above WP_SPEED (the firmware would cap it without telling)."""
    limit = guided_speed_limit(ctx)
    if limit is not None and speed > limit:
        raise VehicleError(
            f"Guided mode drives at most WP_SPEED = {limit:.1f} m/s, so a {speed:.1f} m/s test would be "
            "capped. Choose a lower speed step, or raise WP_SPEED on the navigation step first."
        )


def default_speed_step(ctx) -> float:
    """Speed-step target: a share of the cruise speed, never below the minimum nor above Guided mode's limit."""
    cruise_speed = ctx.param_value("CRUISE_SPEED", 0) or FALLBACK_CRUISE_SPEED
    # Share of the cruise speed, to 0.1 m/s, but not below the minimum step ...
    step = max(MIN_SPEED_STEP, round(SPEED_STEP_FRACTION * cruise_speed, 1))
    limit = guided_speed_limit(ctx)
    # ... and not above WP_SPEED (to 0.1 m/s, rounded down so it stays within the limit).
    return min(step, math.floor(limit * 10) / 10) if limit else step


def default_yaw_rate_step(ctx) -> float:
    """Yaw-rate-step target: a share of the maximum turn rate (rounded to whole deg/s)."""
    # Share of the maximum turn rate, to whole deg/s.
    return round(YAW_STEP_FRACTION * (ctx.param_value("ACRO_TURN_RATE", 0) or FALLBACK_ACRO_TURN_RATE))


def _manual_tracking_report(ctx):
    """Pilot drives in Acro; report how well achieved turn rate/speed followed the sticks (ratio and RMS error)."""
    samples = ctx.drive.record_manual(
        "Drive normally in ACRO mode (steering stick = turn rate, throttle = speed) with "
        "varied inputs for 20-30 s. Press Continue when done.",
        mode="ACRO",
    )

    # Full stick in Acro means ACRO_TURN_RATE (turning) and SPEED_MAX, or CRUISE_SPEED if unset (speed).
    full_turn_rate = math.radians(ctx.param_value("ACRO_TURN_RATE", 0) or FALLBACK_ACRO_TURN_RATE)
    full_speed = ctx.param_value("SPEED_MAX", 0) or (ctx.param_value("CRUISE_SPEED", 0) or FALLBACK_CRUISE_SPEED)
    for name, stick_attribute, measured_attribute, full_scale in (
        ("Steering rate", "steering_input", "yaw_rate", full_turn_rate),
        ("Speed", "throttle_input", "forward_speed", full_speed),
    ):
        # Desired value = stick position x full-scale value; pair it with what the vehicle actually did.
        pairs = [
            (getattr(sample, stick_attribute) * full_scale, getattr(sample, measured_attribute))
            for sample in samples
            if getattr(sample, stick_attribute) is not None
        ]
        # Ignore small stick inputs: mostly noise and dead-band.
        pairs = [
            (desired, achieved)
            for desired, achieved in pairs
            if abs(desired) > TRACKING_MIN_INPUT_FRACTION * full_scale
        ]
        if len(pairs) < TRACKING_MIN_PAIRS:
            ctx.log(f"{name}: not enough input activity.")
            continue
        # Ratio of total achieved to total desired magnitude: < 1 means the vehicle falls short of the command.
        ratio = sum(abs(achieved) for _, achieved in pairs) / sum(abs(desired) for desired, _ in pairs)
        # Root-mean-square tracking error, relative to full scale.
        rms_error = math.sqrt(sum((desired - achieved) ** 2 for desired, achieved in pairs) / len(pairs)) / full_scale
        ctx.log(
            f"{name}: achieved/desired = {ratio:.2f}, RMS error {rms_error:.0%} of full scale "
            "(ratio < 0.9 -> raise I/FF, jerky or overshooting -> lower P). Edit values above and repeat."
        )


@dataclass(frozen=True)
class _Controller:
    """One of ArduPilot's controllers the step tunes, and how its step responses are read."""

    # DriveSample attributes: the controlled quantity and the controller's own report (PID_TUNING)
    measured: str
    report: str
    # GCS_PID_MASK bit that switches the report on
    report_mask: int
    # tuner gain ("p", "i", "ff") -> parameter
    gains: dict
    # [s] slower to 63 % of the target counts as too slow
    slow_rise_time: float
    # how values are shown: unit and conversion from the measured unit
    unit: str
    shown: Callable = float


STEERING_CONTROLLER = _Controller(
    "yaw_rate",
    "steering_pid",
    GCS_PID_MASK_STEERING,
    {"ff": "ATC_STR_RAT_FF", "p": "ATC_STR_RAT_P", "i": "ATC_STR_RAT_I"},
    slow_rise_time=1.0,
    unit="deg/s",
    shown=math.degrees,
)
# the speed controller's feed-forward comes from CRUISE_SPEED / CRUISE_THROTTLE (measured on the gains step)
SPEED_CONTROLLER = _Controller(
    "forward_speed",
    "speed_pid",
    GCS_PID_MASK_THROTTLE,
    {"p": "ATC_SPEED_P", "i": "ATC_SPEED_I"},
    slow_rise_time=3.0,
    unit="m/s",
)


def _tune_by_step_response(
    ctx,
    controller: _Controller,
    run_step,
    target: float,
    step_on: float,
    step_off: float,
    between_rounds: str,
    limit_note=None,
) -> None:
    """Repeat: run a step test, measure the response, adjust the gains - until it tracks well, the changes have
    become small, the gains can't help (output at its limit) or the rounds run out. limit_note(aimed) explains why
    ArduPilot aimed for less than requested."""
    current = {key: ctx.param_value(name) for key, name in controller.gains.items()}
    missing = [controller.gains[key] for key, value in current.items() if value is None]
    if missing:
        raise VehicleError(f"{' / '.join(missing)} do not exist on this firmware - tune them manually.")

    tuner = GainTuner(current, controller.slow_rise_time)
    rounds = int(ctx.number("rounds", DEFAULT_TUNING_ROUNDS))
    with ctx.vehicle.controller_reports(controller.report_mask) as reporting:
        if not reporting:
            ctx.log("i No GCS_PID_MASK: the responses are compared with the requested target only.")
        for round_number in range(1, rounds + 1):
            samples = run_step()
            reports = [getattr(sample, controller.report) for sample in samples]
            response = step_response_metrics(
                [sample.run_time for sample in samples],
                [getattr(sample, controller.measured) for sample in samples],
                target,
                step_on,
                step_off,
                desired=[report and report.desired for report in reports],
                outputs=[report and report.output for report in reports],
            )
            change = tuner.next_gains(response)
            ctx.log(f"Round {round_number}: {_describe(controller, response)} -> {change.reason}")
            log_ekf_health(ctx, samples)
            aimed = response.target if response else None
            if aimed is not None and abs(aimed) < AIMED_BELOW_REQUESTED * abs(target):
                ctx.log(
                    f"  ArduPilot aimed for {controller.shown(aimed):.3g} instead of {controller.shown(abs(target)):.3g}"
                    f" {controller.unit}" + (f" ({limit_note(aimed)})" if limit_note else "")
                )
            if change.blocked or change.converged:
                break

            for key, name in controller.gains.items():
                new = round(change.gains[key], GAIN_DECIMALS)
                if new != round(current[key], GAIN_DECIMALS):
                    ctx.set_param(name, new)
                    ctx.set_values(**{name: new})
                current[key] = new
            if round_number < rounds:
                ctx.pause(between_rounds)


def _describe(controller: _Controller, response) -> str:
    """One round's measurements for the log."""
    if response is None:
        return "no usable data"
    shown = controller.shown
    return (
        # Fractions are shown as percentages (x 100).
        f"overshoot {response.overshoot * 100:.0f}%, t63 {format_value(response.rise_time_63)} s, steady-state error "
        f"{response.steady_state_error:.0%} (aimed at {shown(response.target):.3g} {controller.unit}, noise "
        f"±{shown(response.noise):.2g} {controller.unit})"
    )


def _steering_limit_note(ctx, speed: float):
    """For the log: which of ArduPilot's limits keeps the turn-rate target below the requested one."""

    def note(aimed: float) -> str:
        """The limit closest to what ArduPilot aimed for."""
        limits = []
        rate_max = ctx.param_value("ATC_STR_RAT_MAX", 0) or 0
        if rate_max > 0:
            limits.append((math.radians(rate_max), f"ATC_STR_RAT_MAX = {rate_max:g} deg/s"))
        turn_max_g = ctx.param_value("ATC_TURN_MAX_G", 0) or 0
        # AR_AttitudeControl: lateral acceleration / speed, at least 0.1 g and 1 m/s
        rate = max(turn_max_g, TURN_MAX_G_MIN) * GRAVITY / max(speed, STEER_SPEED_MIN)
        limits.append((rate, f"ATC_TURN_MAX_G = {turn_max_g:g} g at {max(speed, STEER_SPEED_MIN):g} m/s"))
        nearest, name = min(limits, key=lambda limit: abs(limit[0] - aimed))
        if abs(nearest - aimed) <= LIMIT_MATCH * nearest:
            return f"limited by {name}"
        return "ATC_STR_ACC_MAX may not let it build up within the step"

    return note


def action_verify_speed_controller(ctx):
    """Guided-mode speed steps; refine ATC_SPEED_P/I automatically (or a manual Acro tracking report)."""
    if ctx.value("how_verify", "auto") == "manual":
        _manual_tracking_report(ctx)
        return
    runway = ctx.number("runway", DEFAULT_RUNWAY_M)
    target = ctx.number("v_step", DEFAULT_SPEED_STEP)
    _ensure_guided_can_reach(ctx, target)

    # Hold the step as long as the usable runway allows at the target speed (0.1 m/s floor avoids dividing by 0).
    hold = min(STEP_MAX_HOLD_SECONDS, RUNWAY_USE_FRACTION * runway / max(target, 0.1))
    if hold < STEP_MIN_HOLD_SECONDS:
        raise VehicleError("Runway too short for this speed step - lower 'speed step' or increase the runway.")
    ctx.drive.safety_brief(
        f"The vehicle follows a speed step to {target:.1f} m/s (Guided mode) and stops, "
        f"up to {int(ctx.number('rounds', DEFAULT_TUNING_ROUNDS))} rounds."
    )
    step_end = STEP_LEAD_SECONDS + hold
    _tune_by_step_response(
        ctx,
        SPEED_CONTROLLER,
        lambda: ctx.drive.run_guided(
            lambda t: (target if STEP_LEAD_SECONDS <= t < step_end else 0.0, 0.0),
            step_end + STEP_STOP_SECONDS,
            limit_m=runway * STEP_RUNWAY_FRACTION,
            hz=TUNING_SAMPLE_RATE_HZ,
        ),
        target,
        STEP_LEAD_SECONDS,
        step_end,
        between_rounds="Bring the vehicle back to the start and press Continue for the next round.",
    )


def action_verify_steering_controller(ctx):
    """Guided-mode turn-rate steps; refine ATC_STR_RAT_P/I automatically (or a manual Acro tracking report)."""
    if ctx.value("how_verify", "auto") == "manual":
        _manual_tracking_report(ctx)
        return

    circle = ctx.number("circle_d", DEFAULT_CIRCLE_DIAMETER_M)
    # skid: pivot on the spot; car: must be moving to turn
    speed = 0.0 if ctx.is_skid() else max(MIN_GUIDED_TURN_SPEED, ctx.number("v_step", DEFAULT_SPEED_STEP))
    _ensure_guided_can_reach(ctx, speed)
    rate = math.radians(ctx.number("r_step", DEFAULT_YAW_RATE_STEP_DEG))
    radius = ctx.param_value("TURN_RADIUS", 0) or 0
    if speed > 0 and radius > 0:
        # A car can't turn faster than speed / minimum radius; ask for a safe fraction of that.
        rate = min(rate, CAR_TURN_RATE_FRACTION * speed / radius)
    ctx.log(f"Test: {speed:.1f} m/s, yaw-rate step {math.degrees(rate):.0f} deg/s")
    ctx.drive.safety_brief(
        "The vehicle follows a yaw-rate step to the RIGHT (Guided mode) within the circle entered on the Pre-drive step."
    )
    # Timeline of one round: straight, then turning at `rate`, then straight again.
    _tune_by_step_response(
        ctx,
        STEERING_CONTROLLER,
        lambda: ctx.drive.run_guided(
            lambda t: (speed, rate if YAW_STEP_START_SECONDS <= t < YAW_STEP_END_SECONDS else 0.0),
            YAW_STEP_RUN_SECONDS,
            limit_m=circle,
            hz=TUNING_SAMPLE_RATE_HZ,
        ),
        rate,
        YAW_STEP_START_SECONDS,
        YAW_STEP_END_SECONDS,
        between_rounds="Reposition the vehicle and press Continue for the next round.",
        limit_note=_steering_limit_note(ctx, speed),
    )


STEP = Step(
    "verify",
    "Closed-loop verification & fine tuning",
    [
        who_drives_field("how_verify"),
    ]
    + ekf_fields()
    + [
        value_field(
            "rounds",
            "Max refinement rounds",
            kind="int",
            default=DEFAULT_TUNING_ROUNDS,
            min_value=1,
            max_value=MAX_TUNING_ROUNDS,
        ),
        value_field("v_step", "Speed step target", unit="m/s", default=default_speed_step),
        value_field("r_step", "Yaw-rate step target", unit="deg/s", default=default_yaw_rate_step),
        param_field("ATC_SPEED_P", "Speed P"),
        param_field("ATC_SPEED_I", "Speed I"),
        param_field("ATC_STR_RAT_FF", "Steering rate FF"),
        param_field("ATC_STR_RAT_P", "Steering rate P"),
        param_field("ATC_STR_RAT_I", "Steering rate I"),
    ],
    [
        # first: Guided mode relies on the EKF's heading and position
        EKF_ACTION,
        Action("Verify & tune SPEED controller", action_verify_speed_controller, armed=True),
        Action("Verify & tune STEERING-RATE controller", action_verify_steering_controller, armed=True),
    ],
    # needs a motor: a vehicle with nothing but sails skips it
    applies=is_motorised,
)
