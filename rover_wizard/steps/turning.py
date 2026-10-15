"""rover_wizard.steps.turning - the "Turn radius & turn rate" step."""

from __future__ import annotations

import itertools
import math

from ..motion.driving import ramp_up
from ..motion.analysis import analyze_turn_run
from ..util.errors import VehicleError
from .util.driving import DEFAULT_CIRCLE_DIAMETER_M, LIMIT_SAFETY_FACTOR, who_drives_field
from ..model import Action, Step, param_field, value_field

# for the turn-rate build-up limit
STEER_ACCEL_SAFETY_FACTOR = 0.8

# [deg/s^2] never propose a lower ATC_STR_ACC_MAX
MIN_STEER_ACCEL_LIMIT = 30

DEFAULT_RADIUS_TEST_THROTTLE_PCT = 20

# front-wheel steering; skid-steer pivots on the spot (0 %)
DEFAULT_MAX_RATE_THROTTLE_PCT = 40

# full lock once rolling
RADIUS_STEER_START_SECONDS = 0.8

RADIUS_MAX_SECONDS = 22

# stop after 1.5 laps
RADIUS_STOP_ANGLE = 3 * math.pi

MAX_RATE_STEER_START_SECONDS = 1.0

MAX_RATE_SECONDS = 8


def _total_turned_angle(samples: list) -> float:
    """Total angle [rad] turned so far: the integral of |yaw rate| over time (rectangle rule per sample interval)."""
    # For each consecutive pair: |turn rate| x elapsed time = angle turned in that interval; add them all up.
    return sum(
        abs(later.yaw_rate) * max(0.0, later.time - earlier.time) for earlier, later in itertools.pairwise(samples)
    )


def action_measure_turn_radius(ctx):
    """Full-lock circle at low speed: proposes TURN_RADIUS (= speed / turn rate)."""
    circle = ctx.number("circle_d", DEFAULT_CIRCLE_DIAMETER_M)
    # % -> 0..1
    throttle = ctx.number("low_thr", DEFAULT_RADIUS_TEST_THROTTLE_PCT) / 100
    if ctx.value("how_turn", "auto") == "auto":
        ctx.drive.safety_brief(
            f"The rover drives a full-lock circle to the RIGHT at low speed (stops {circle:.0f} m from "
            "its start point at the latest)."
        )
        # Throttle ramps up over 1 s; full right lock from 0.8 s; stop after 1.5 laps (3*pi rad) or the limit.
        samples = ctx.drive.run_open_loop(
            lambda t: (1.0 if t > RADIUS_STEER_START_SECONDS else 0.0, ramp_up(throttle, t)),
            RADIUS_MAX_SECONDS,
            limit_m=circle,
            stop_if=lambda recorded: _total_turned_angle(recorded) > RADIUS_STOP_ANGLE,
        )
    else:
        samples = ctx.drive.record_manual(
            "Drive the SMALLEST possible circle (full steering lock) slowly, both "
            "directions if possible. Press Continue when done."
        )

    result = analyze_turn_run(samples)
    if result is None or result.radius is None:
        raise VehicleError("No usable circle data (did the vehicle turn at full lock and move?).")
    ctx.log(
        f"Turn radius {result.radius:.2f} m"
        + ("  (skid steer vehicles can pivot; this is the driving radius)" if ctx.is_skid() else "")
    )
    ctx.set_values(TURN_RADIUS=round(result.radius, 2))


def action_measure_max_turn_rate(ctx):
    """Sharpest possible turn: proposes ACRO_TURN_RATE, ATC_STR_RAT_MAX and ATC_STR_ACC_MAX."""
    circle = ctx.number("circle_d", DEFAULT_CIRCLE_DIAMETER_M)
    # % -> 0..1
    throttle = ctx.number("turn_thr", 0 if ctx.is_skid() else DEFAULT_MAX_RATE_THROTTLE_PCT) / 100
    if ctx.value("how_turn", "auto") == "auto":
        ctx.drive.safety_brief(
            "The rover performs a full-lock turn " + ("on the spot (pivot)." if throttle == 0 else "at medium speed.")
        )
        # Full right lock from 1 s on; throttle ramps up over 1 s (none for a pivot turn).
        samples = ctx.drive.run_open_loop(
            lambda t: (1.0 if t > MAX_RATE_STEER_START_SECONDS else 0.0, ramp_up(throttle, t)),
            MAX_RATE_SECONDS,
            limit_m=circle,
        )
    else:
        samples = ctx.drive.record_manual(
            "Turn as sharply as possible at full steering (pivot if skid-steer, medium "
            "speed otherwise). Press Continue when done."
        )

    result = analyze_turn_run(samples)
    if result is None:
        raise VehicleError("Not enough data recorded.")

    rate = math.degrees(result.max_yaw_rate)
    ctx.log(f"Maximum turn rate {rate:.0f} deg/s")
    proposal = {
        "ACRO_TURN_RATE": round(rate * LIMIT_SAFETY_FACTOR),
        "ATC_STR_RAT_MAX": round(rate * LIMIT_SAFETY_FACTOR),
    }
    if result.max_yaw_acceleration:
        acceleration = math.degrees(result.max_yaw_acceleration)
        ctx.log(f"Max steering acceleration {acceleration:.0f} deg/s^2")
        # 80 % of the measured build-up, rounded to 10 deg/s^2, but never below the floor.
        proposal["ATC_STR_ACC_MAX"] = max(MIN_STEER_ACCEL_LIMIT, round(acceleration * STEER_ACCEL_SAFETY_FACTOR, -1))
    ctx.set_values(**proposal)
    ctx.log("Review the proposed values and press Apply.")


STEP = Step(
    "turning",
    "Turn radius & turn rate",
    [
        who_drives_field("how_turn"),
        value_field(
            "low_thr",
            "Throttle for radius test",
            unit="%",
            default=DEFAULT_RADIUS_TEST_THROTTLE_PCT,
            min_value=1,
            max_value=100,
        ),
        value_field(
            "turn_thr",
            "Throttle for max-rate test",
            unit="%",
            default=lambda ctx: 0 if ctx.is_skid() else DEFAULT_MAX_RATE_THROTTLE_PCT,
            min_value=0,
            max_value=100,
        ),
        param_field("TURN_RADIUS", "Turn radius", unit="m"),
        param_field("ACRO_TURN_RATE", "Acro turn rate", unit="deg/s"),
        param_field("ATC_STR_RAT_MAX", "Max steering rate", unit="deg/s"),
        param_field("ATC_STR_ACC_MAX", "Max steering accel", unit="deg/s/s"),
        param_field("ATC_TURN_MAX_G", "Max lateral acceleration", unit="g"),
    ],
    [
        Action("Measure turn radius", action_measure_turn_radius, armed=True),
        Action("Measure max turn rate", action_measure_max_turn_rate, armed=True),
    ],
)
