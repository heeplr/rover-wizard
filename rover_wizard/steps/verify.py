"""rover_wizard.steps.verify - the "Closed-loop verification & fine tuning" step."""

from __future__ import annotations

import math
from typing import Optional

from ..motion.analysis import step_response_metrics, suggest_pi_gains
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

DEFAULT_TUNING_ROUNDS = 3

# [m/s]
DEFAULT_SPEED_STEP = 1.0

# [deg/s]
DEFAULT_YAW_RATE_STEP_DEG = 45

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


def _tune_by_step_response(
    ctx,
    run_step,
    measured_attribute: str,
    target: float,
    step_on: float,
    step_off: float,
    p_name: str,
    i_name: str,
    slow_rise_time: float,
    between_rounds: str,
) -> None:
    """Repeat: run a step test, measure the response, adjust P/I - until it tracks well or rounds run out."""
    rounds = int(ctx.number("rounds", DEFAULT_TUNING_ROUNDS))
    for round_number in range(1, rounds + 1):
        samples = run_step()
        response = step_response_metrics(
            [sample.run_time for sample in samples],
            [getattr(sample, measured_attribute) for sample in samples],
            target,
            step_on,
            step_off,
        )
        p_gain = ctx.param_value(p_name)
        i_gain = ctx.param_value(i_name)
        if p_gain is None or i_gain is None:
            raise VehicleError(f"{p_name} / {i_name} do not exist on this firmware - tune them manually.")

        suggestion = suggest_pi_gains(response, p_gain, i_gain, slow_rise_time)
        if response:
            ctx.log(
                # Fractions are shown as percentages (x 100).
                f"Round {round_number}: overshoot {response.overshoot * 100:.0f}%, t63 "
                f"{format_value(response.rise_time_63)} s, steady-state error "
                f"{response.steady_state_error:.0%} -> {suggestion.reason}"
            )
        else:
            ctx.log(f"Round {round_number}: {suggestion.reason}")

        if suggestion.converged or response is None:
            break

        new_p = round(suggestion.p_gain, GAIN_DECIMALS)
        new_i = round(suggestion.i_gain, GAIN_DECIMALS)
        ctx.set_param(p_name, new_p)
        ctx.set_param(i_name, new_i)
        ctx.set_values(**{p_name: new_p, i_name: new_i})
        if round_number < rounds:
            ctx.pause(between_rounds)


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
        f"The rover follows a speed step to {target:.1f} m/s (Guided mode) and stops, "
        f"up to {int(ctx.number('rounds', DEFAULT_TUNING_ROUNDS))} rounds."
    )
    step_end = STEP_LEAD_SECONDS + hold
    _tune_by_step_response(
        ctx,
        lambda: ctx.drive.run_guided(
            lambda t: (target if STEP_LEAD_SECONDS <= t < step_end else 0.0, 0.0),
            step_end + STEP_STOP_SECONDS,
            limit_m=runway * STEP_RUNWAY_FRACTION,
        ),
        "forward_speed",
        target,
        STEP_LEAD_SECONDS,
        step_end,
        "ATC_SPEED_P",
        "ATC_SPEED_I",
        slow_rise_time=3.0,
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
        "The rover follows a yaw-rate step to the RIGHT (Guided mode) within the circle entered on the Pre-drive step."
    )
    # Timeline of one round: straight, then turning at `rate`, then straight again.
    _tune_by_step_response(
        ctx,
        lambda: ctx.drive.run_guided(
            lambda t: (speed, rate if YAW_STEP_START_SECONDS <= t < YAW_STEP_END_SECONDS else 0.0),
            YAW_STEP_RUN_SECONDS,
            limit_m=circle,
        ),
        "yaw_rate",
        rate,
        YAW_STEP_START_SECONDS,
        YAW_STEP_END_SECONDS,
        "ATC_STR_RAT_P",
        "ATC_STR_RAT_I",
        slow_rise_time=1.0,
        between_rounds="Reposition the vehicle and press Continue for the next round.",
    )


STEP = Step(
    "verify",
    "Closed-loop verification & fine tuning",
    [
        who_drives_field("how_verify"),
        value_field(
            "rounds", "Max refinement rounds", kind="int", default=DEFAULT_TUNING_ROUNDS, min_value=1, max_value=8
        ),
        value_field("v_step", "Speed step target", unit="m/s", default=default_speed_step),
        value_field("r_step", "Yaw-rate step target", unit="deg/s", default=default_yaw_rate_step),
        param_field("ATC_SPEED_P", "Speed P"),
        param_field("ATC_SPEED_I", "Speed I"),
        param_field("ATC_STR_RAT_P", "Steering rate P"),
        param_field("ATC_STR_RAT_I", "Steering rate I"),
    ],
    [
        Action("Verify & tune SPEED controller", action_verify_speed_controller, armed=True),
        Action("Verify & tune STEERING-RATE controller", action_verify_steering_controller, armed=True),
    ],
)
