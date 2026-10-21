"""rover_wizard.steps.gains - the "Feed-forward & first PID values" step."""

from __future__ import annotations

from ..motion.driving import ramp_up
from ..motion.analysis import analyze_speed_feedforward, analyze_steering_feedforward
from ..util.errors import VehicleError
from ..rover_defs import RC_OPTION_SCRIPTING_1
from .util.driving import (
    DEFAULT_CIRCLE_DIAMETER_M,
    DEFAULT_RUNWAY_M,
    GAIN_DECIMALS,
    LIMIT_SAFETY_FACTOR,
    RUNWAY_USE_FRACTION,
    _is_speed_settled,
    who_drives_field,
)
from .util.outputs import assign_rc_switch
from .scripting import STEP as SCRIPTING_STEP
from ..model import Action, Step, param_field, value_field
from .util.rover_hardware import is_motorised

MIN_STEADY_TURN_SAMPLES = 15

MIN_STEADY_SPEED_SAMPLES = 10

# front-wheel steering; skid-steer turns on the spot (0 %)
DEFAULT_FEEDFORWARD_THROTTLE_PCT = 35

DEFAULT_CRUISE_TEST_THROTTLE_PCT = 50

# straight before the first circle
FEEDFORWARD_START_SECONDS = 0.5

# from first (40 %) to second (70 %) steering level
FEEDFORWARD_SWITCH_SECONDS = 7.0

FEEDFORWARD_STEERING_LEVELS = (0.4, 0.7)

FEEDFORWARD_SECONDS = 14

# QuickTune's starting steering-rate P and I: half the feed-forward
STARTING_GAIN_FRACTION = 0.5

# rate-controller filters: half the gyro filter frequency
CONTROLLER_FILTER_FRACTION = 0.5

CRUISE_TEST_MAX_SECONDS = 20


def action_measure_steering_feedforward(ctx):
    """Steady part-lock circles: proposes steering-rate FF, starting P/I, and the rate-controller filters."""
    circle = ctx.number("circle_d", DEFAULT_CIRCLE_DIAMETER_M)
    # % -> 0..1
    throttle = ctx.number("ff_thr", 0 if ctx.is_skid() else DEFAULT_FEEDFORWARD_THROTTLE_PCT) / 100
    if ctx.value("how_gain", "auto") == "auto":
        ctx.drive.safety_brief("The vehicle drives circles to the RIGHT with 40% and 70% steering for ~6 s each.")
        # 0-0.5 s straight, 0.5-7 s at 40 % steering, 7-14 s at 70 %: two steady operating points.
        samples = ctx.drive.run_open_loop(
            lambda t: (
                (
                    (
                        FEEDFORWARD_STEERING_LEVELS[0]
                        if t < FEEDFORWARD_SWITCH_SECONDS
                        else FEEDFORWARD_STEERING_LEVELS[1]
                    )
                    if t > FEEDFORWARD_START_SECONDS
                    else 0.0
                ),
                ramp_up(throttle, t),
            ),
            FEEDFORWARD_SECONDS,
            limit_m=circle,
        )
    else:
        samples = ctx.drive.record_manual(
            "Drive steady circles (Manual mode, partial steering ~40-70%, both "
            "directions) for at least 15 s. Press Continue when done."
        )

    result = analyze_steering_feedforward(samples)
    if result.feedforward is None or result.sample_count < MIN_STEADY_TURN_SAMPLES:
        raise VehicleError(f"Not enough steady turning data ({result.sample_count} samples). Turn harder/longer.")
    feedforward = result.feedforward * LIMIT_SAFETY_FACTOR
    # Starting gains as in ArduPilot's QuickTune: P = I = half the feed-forward.
    starting_gain = round(feedforward * STARTING_GAIN_FRACTION, GAIN_DECIMALS)
    proposal = {
        "ATC_STR_RAT_FF": round(feedforward, GAIN_DECIMALS),
        "ATC_STR_RAT_P": starting_gain,
        "ATC_STR_RAT_I": starting_gain,
    }

    gyro_filter = ctx.param_value("INS_GYRO_FILTER")
    if gyro_filter:
        # Controller filters at half the gyro filter frequency. The target filter's field key is
        # ATC_STR_RAT_FLTT even on firmware where the parameter is called ATC_STR_RAT_FILT.
        for field_key, names in (
            ("ATC_STR_RAT_FLTT", ("ATC_STR_RAT_FLTT", "ATC_STR_RAT_FILT")),
            ("ATC_STR_RAT_FLTD", ("ATC_STR_RAT_FLTD",)),
        ):
            if ctx.vehicle.resolve(names):
                # to 0.1 Hz
                proposal[field_key] = round(gyro_filter * CONTROLLER_FILTER_FRACTION, 1)
    ctx.log(f"Steering feed-forward {feedforward:.4f} (from {result.sample_count} steady samples)")
    ctx.set_values(**proposal)
    ctx.log("Review the proposed values and press Apply.")


def action_measure_speed_feedforward(ctx):
    """Steady straight drive at medium throttle: proposes CRUISE_SPEED/THROTTLE and starting speed P/I."""
    runway = ctx.number("runway", DEFAULT_RUNWAY_M)
    # % -> 0..1
    throttle = ctx.number("cruise_pct", DEFAULT_CRUISE_TEST_THROTTLE_PCT) / 100
    if ctx.value("how_gain", "auto") == "auto":
        ctx.drive.safety_brief(
            f"The vehicle drives straight at {throttle:.0%} throttle until the speed is steady "
            f"(max {runway * RUNWAY_USE_FRACTION:.0f} m)."
        )
        # Throttle ramps up over 1 s; stop once speed has been flat (±5 %) for 3 s, after at least 7 s.
        samples = ctx.drive.run_open_loop(
            lambda t: (0.0, ramp_up(throttle, t)),
            CRUISE_TEST_MAX_SECONDS,
            limit_m=runway * RUNWAY_USE_FRACTION,
            stop_if=lambda recorded: _is_speed_settled(recorded, window=3.0, max_spread=0.05, min_run_seconds=7.0),
        )
    else:
        samples = ctx.drive.record_manual(
            "Drive straight at a steady MEDIUM throttle for at least 10 s. " "Press Continue when done."
        )

    result = analyze_speed_feedforward(samples)
    if result.sample_count < MIN_STEADY_SPEED_SAMPLES:
        raise VehicleError("No steady-speed data found (speed kept changing or too slow).")
    # Starting speed P and I = throttle needed per m/s (the controller's natural gain scale).
    gain = round(result.throttle_per_speed, GAIN_DECIMALS)
    ctx.log(f"Cruise: {result.cruise_speed:.2f} m/s at {result.cruise_throttle:.1%} throttle -> speed P=I={gain}")

    ctx.set_values(
        # CRUISE_SPEED to 0.01 m/s
        CRUISE_SPEED=round(result.cruise_speed, 2),
        # CRUISE_THROTTLE is a percentage, to 0.1 %.
        CRUISE_THROTTLE=round(result.cruise_throttle * 100, 1),
        ATC_SPEED_P=gain,
        ATC_SPEED_I=gain,
    )
    ctx.log("Review the proposed values and press Apply.")


# ArduPilot's QuickTune applet for Rover
QUICKTUNE_SCRIPT = "rover-quicktune.lua"


def apply_gains(ctx):
    """If a QuickTune switch channel was chosen, give it the script's switch function and say what's still missing.

    rover-quicktune.lua (ArduPilot's AP_Scripting/applets) reads the switch whose RCx_OPTION equals its
    RTUN_RC_FUNC parameter (300 = Scripting1 by default); its RTUN_ parameters only exist while the script runs.
    """
    channel = int(ctx.number("qt_ch", 0))
    if channel <= 0:
        return

    option = ctx.param_int("RTUN_RC_FUNC", RC_OPTION_SCRIPTING_1) or RC_OPTION_SCRIPTING_1
    # refuses steering/throttle/mode channels and channels with another switch; moves it from an earlier channel
    assign_rc_switch(ctx, option, channel)
    if not ctx.param_int("SCR_ENABLE"):
        ctx.log(
            f"❗ QuickTune needs Lua scripting, which is off: enable it on the {SCRIPTING_STEP.title} step "
            f"(then reboot) and copy {QUICKTUNE_SCRIPT} into APM/scripts on the SD card."
        )
    elif not ctx.vehicle.has_parameter("RTUN_ENABLE"):
        ctx.log(
            f"❗ The QuickTune script isn't running: copy {QUICKTUNE_SCRIPT} into APM/scripts on the SD card "
            "and reboot."
        )
    else:
        ctx.log(
            f"QuickTune ready on RC{channel}: arm, switch to Circle mode, then switch middle = start tuning, "
            "high = save, low = revert."
        )


STEP = Step(
    "gains",
    "Feed-forward & first PID values",
    [
        who_drives_field("how_gain"),
        value_field(
            "cruise_pct",
            "Throttle for speed test",
            unit="%",
            default=DEFAULT_CRUISE_TEST_THROTTLE_PCT,
            min_value=10,
            max_value=100,
        ),
        value_field(
            "ff_thr",
            "Throttle during steering test",
            unit="%",
            default=lambda ctx: 0 if ctx.is_skid() else DEFAULT_FEEDFORWARD_THROTTLE_PCT,
            min_value=0,
            max_value=100,
        ),
        param_field("ATC_STR_RAT_FF", "Steering rate FF"),
        param_field("ATC_STR_RAT_P", "Steering rate P"),
        param_field("ATC_STR_RAT_I", "Steering rate I"),
        param_field("ATC_STR_RAT_D", "Steering rate D"),
        param_field(("ATC_STR_RAT_FLTT", "ATC_STR_RAT_FILT"), "Steering rate target filter", unit="Hz"),
        param_field("ATC_STR_RAT_FLTD", "Steering rate D filter", unit="Hz"),
        param_field("CRUISE_SPEED", "Cruise speed", unit="m/s"),
        param_field("CRUISE_THROTTLE", "Cruise throttle", unit="%"),
        param_field("ATC_SPEED_P", "Speed P"),
        param_field("ATC_SPEED_I", "Speed I"),
        param_field("ATC_SPEED_D", "Speed D"),
        value_field(
            "qt_ch", "RC channel for QuickTune switch (0 = none)", kind="int", default=0, min_value=0, max_value=16
        ),
    ],
    [
        Action("Measure steering feed-forward", action_measure_steering_feedforward, armed=True),
        Action("Measure cruise & speed gains", action_measure_speed_feedforward, armed=True),
    ],
    on_apply=apply_gains,
    # needs a motor: a vehicle with nothing but sails skips it
    applies=is_motorised,
)
