"""rover_wizard.steps.top_speed - the "Top speed & acceleration" step."""

from __future__ import annotations

from ..motion.driving import ramp_up
from ..motion.analysis import analyze_top_speed_run
from ..util.errors import VehicleError
from .util.driving import (
    DEFAULT_RUNWAY_M,
    LIMIT_SAFETY_FACTOR,
    RUNWAY_USE_FRACTION,
    _is_speed_settled,
    who_drives_field,
)
from ..model import Action, Step, param_field
from .util.rover_hardware import is_motorised

# decel/accel ratio inside this -> leave ATC_DECEL_MAX = 0 ("same as accel")
DECEL_SIMILAR_RANGE = (0.7, 1.3)

# 0 -> 100 % throttle without a wheelspin jolt
TOP_SPEED_RAMP_SECONDS = 1.5

TOP_SPEED_MAX_SECONDS = 25

# keep recording while coasting, to measure the deceleration
TOP_SPEED_TAIL_SECONDS = 8


def action_measure_top_speed(ctx):
    """Full throttle straight ahead, then coast: proposes SPEED_MAX, ATC_ACCEL_MAX and ATC_DECEL_MAX."""
    runway = ctx.number("runway", DEFAULT_RUNWAY_M)
    if ctx.value("how_speed", "auto") == "auto":
        ctx.drive.safety_brief(
            f"The vehicle accelerates straight ahead to FULL throttle (max "
            f"{runway * RUNWAY_USE_FRACTION:.0f} m), then coasts to a stop."
        )
        # Throttle ramps 0 -> 100 % in 1.5 s (no wheelspin jolt); stop once speed has been flat for 2 s (after
        # at least 4 s), then keep recording up to 8 s while coasting to measure the deceleration.
        samples = ctx.drive.run_open_loop(
            lambda t: (0.0, ramp_up(1.0, t, TOP_SPEED_RAMP_SECONDS)),
            TOP_SPEED_MAX_SECONDS,
            limit_m=runway * RUNWAY_USE_FRACTION,
            stop_if=lambda recorded: _is_speed_settled(recorded, window=2.0, max_spread=0.06, min_run_seconds=4.0),
            tail=TOP_SPEED_TAIL_SECONDS,
        )
    else:
        samples = ctx.drive.record_manual(
            "Drive straight at FULL throttle until the speed stops rising, release the "
            "throttle and let the vehicle coast to a stop. Press Continue when it has stopped."
        )

    result = analyze_top_speed_run(samples)
    if result is None:
        raise VehicleError("Not enough data recorded.")
    ctx.log(
        f"Top speed {result.top_speed:.2f} m/s ("
        + ("steady" if result.speed_was_steady else "STILL RISING - runway/time too short, value is a lower bound")
        + ")"
    )
    ctx.set_values(SPEED_MAX=round(result.top_speed, 2))

    if result.max_acceleration:
        ctx.log(f"Max acceleration {result.max_acceleration:.2f} m/s^2")
        ctx.set_values(ATC_ACCEL_MAX=round(result.max_acceleration * LIMIT_SAFETY_FACTOR, 1))

    if result.max_deceleration:
        ctx.log(f"Max deceleration {result.max_deceleration:.2f} m/s^2")
        # ATC_DECEL_MAX = 0 means "same as ATC_ACCEL_MAX"; only set it if braking differs by more than ±30 %.
        ratio = result.max_deceleration / result.max_acceleration if result.max_acceleration else None
        similar = ratio is None or DECEL_SIMILAR_RANGE[0] < ratio < DECEL_SIMILAR_RANGE[1]
        ctx.set_values(ATC_DECEL_MAX=0 if similar else round(result.max_deceleration * LIMIT_SAFETY_FACTOR, 1))
    ctx.log("Review the proposed values and press Apply.")


STEP = Step(
    "top_speed",
    "Top speed & acceleration",
    [
        who_drives_field("how_speed"),
        param_field("SPEED_MAX", "Top speed (SPEED_MAX)", unit="m/s"),
        param_field("ATC_ACCEL_MAX", "Max acceleration", unit="m/s/s"),
        param_field("ATC_DECEL_MAX", "Max deceleration (0 = same)", unit="m/s/s"),
    ],
    [Action("Measure top speed & acceleration", action_measure_top_speed, armed=True)],
    # needs a motor: a vehicle with nothing but sails skips it
    applies=is_motorised,
)
