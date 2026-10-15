"""rover_wizard.steps.direction_check - the "Direction check (drive + steering)" step."""

from __future__ import annotations

import math

from ..motion.driving import ramp_up
from ..util.errors import VehicleError
from ..ardupilot_defs import (
    SERVO_FUNCTION_GROUND_STEERING,
    SERVO_FUNCTION_THROTTLE,
    SERVO_FUNCTION_THROTTLE_LEFT,
    SERVO_FUNCTION_THROTTLE_RIGHT,
)
from .util.driving import DEFAULT_RUNWAY_M, who_drives_field
from ..model import Action, Step

# [m/s] slower than this, the direction of travel is not trustworthy
DIRECTION_MIN_SPEED = 0.05

# throttle commands below 10 % don't count as "driving" in the direction check
DIRECTION_MIN_THROTTLE = 0.1

# steering commands below 30 % don't count as "turning" in the direction check
DIRECTION_MIN_STEERING = 0.3

# at least 15 % throttle for the direction check ...
DIRECTION_CREEP_THROTTLE = 0.15

# ... and 12 percentage points above MOT_THR_MIN
DIRECTION_THROTTLE_ABOVE_DEADZONE = 0.12

# straight throttle ramp, then the turn
DIRECTION_RAMP_SECONDS = 1.5

# 60 % right steering
DIRECTION_TURN_STEERING = 0.6

DIRECTION_RUN_SECONDS = 3.8

# the direction check stays within 20 % of the runway ...
DIRECTION_RUNWAY_FRACTION = 0.2

# ... but is allowed at least 3 m
DIRECTION_MIN_LIMIT_M = 3.0


def _measure_direction_response(samples: list) -> tuple:
    """(mean forward speed per forward command, mean yaw rate per right-steer command) - positive = correct.

    Each sample's speed is multiplied by the sign of its throttle command, so "moved the way it was told" is
    positive. The yaw rate is multiplied by the sign of the steering command and by the sign of the actual travel
    direction (a car turns the other way when reversing), so "turned the way it was told" is positive too.
    """
    speeds = []
    yaw_rates = []
    for sample in samples:
        # What was commanded: the wizard's own command if it drove, otherwise the pilot's sticks.
        steering, throttle = (
            sample.command if sample.command else (sample.steering_input or 0, sample.throttle_input or 0)
        )
        speed = sample.forward_speed
        if abs(throttle) > DIRECTION_MIN_THROTTLE and abs(speed) > DIRECTION_MIN_SPEED:
            # speed x sign(throttle): positive when moving the commanded way.
            speeds.append(speed * math.copysign(1, throttle))
        if (
            abs(steering) > DIRECTION_MIN_STEERING
            and abs(throttle) > DIRECTION_MIN_THROTTLE
            and abs(speed) > DIRECTION_MIN_SPEED
        ):
            # yaw x sign(steering) x sign(travel): positive when turning the commanded way.
            yaw_rates.append(sample.yaw_rate * math.copysign(1, steering) * math.copysign(1, speed))
    # Averages over all qualifying samples (None if there were none).
    mean_speed = sum(speeds) / len(speeds) if speeds else None
    mean_yaw_rate = sum(yaw_rates) / len(yaw_rates) if yaw_rates else None
    return mean_speed, mean_yaw_rate


def _toggle_reversed(ctx, output: int) -> None:
    """Flip SERVO<output>_REVERSED between 0 and 1."""
    ctx.set_param(f"SERVO{output}_REVERSED", 1 - ctx.param_int(f"SERVO{output}_REVERSED"))


def action_check_directions(ctx):
    """Drive forward, then steer right; offer to fix an inverted drive or steering direction."""
    if ctx.value("how_dir", "auto") == "auto":
        # Just enough throttle to move: a bit above the configured dead-zone (MOT_THR_MIN is in %).
        dead_zone = (ctx.param_value("MOT_THR_MIN", 0) or 0) / 100
        throttle = max(DIRECTION_CREEP_THROTTLE, dead_zone + DIRECTION_THROTTLE_ABOVE_DEADZONE)
        ctx.drive.safety_brief("The rover creeps forward for ~1.5 s, then drives a gentle right turn for ~2 s.")
        # Straight until DIRECTION_RAMP_SECONDS (throttle ramps up over the first second), then right steering at
        # the same throttle.
        samples = ctx.drive.run_open_loop(
            lambda t: (
                (0.0, ramp_up(throttle, t)) if t < DIRECTION_RAMP_SECONDS else (DIRECTION_TURN_STEERING, throttle)
            ),
            DIRECTION_RUN_SECONDS,
            limit_m=max(DIRECTION_MIN_LIMIT_M, ctx.number("runway", DEFAULT_RUNWAY_M) * DIRECTION_RUNWAY_FRACTION),
        )
    else:
        samples = ctx.drive.record_manual(
            "Drive slowly FORWARD, then steer to the RIGHT while moving forward. " "Press Continue when done."
        )

    forward, yaw = _measure_direction_response(samples)
    if forward is None:
        raise VehicleError("The vehicle did not move. Raise MOT_THR_MIN / check motors, then repeat.")
    skid = ctx.is_skid()
    ctx.log(f"Forward command -> speed {forward:+.2f} m/s ({'OK' if forward > 0 else 'WRONG DIRECTION'})")
    ctx.log(
        "Right steering -> yaw "
        + (f"{math.degrees(yaw):+.0f} deg/s ({'OK' if yaw > 0 else 'WRONG'})" if yaw is not None else "not measurable")
    )

    # the outputs a fix would change; a skipped output step can leave one unassigned
    motor_functions = (
        (SERVO_FUNCTION_THROTTLE_LEFT, SERVO_FUNCTION_THROTTLE_RIGHT) if skid else (SERVO_FUNCTION_THROTTLE,)
    )
    motor_outputs = [ctx.output_with_function(function) for function in motor_functions]
    steering_output = 0 if skid else ctx.output_with_function(SERVO_FUNCTION_GROUND_STEERING)
    if 0 in motor_outputs or (not skid and not steering_output):
        raise VehicleError(
            "Not every steering / motor function has an output - assign them on the Steering output "
            "and Motor output steps first."
        )

    if forward < 0:
        outputs = motor_outputs
        if ctx.confirm(f"Toggle 'reversed' on output(s) {outputs} to fix the drive direction?"):
            for output in outputs:
                _toggle_reversed(ctx, output)

    if yaw is not None and yaw < 0:
        if skid:
            if ctx.confirm("Swap LEFT/RIGHT motor outputs (SERVOx_FUNCTION 73 <-> 74) to fix the steering direction?"):
                left, right = motor_outputs
                ctx.set_param(f"SERVO{left}_FUNCTION", SERVO_FUNCTION_THROTTLE_RIGHT, reboot=True)
                ctx.set_param(f"SERVO{right}_FUNCTION", SERVO_FUNCTION_THROTTLE_LEFT, reboot=True)
        else:
            if ctx.confirm(
                f"Toggle 'reversed' on the steering servo (SERVO{steering_output}) to fix the steering " "direction?"
            ):
                _toggle_reversed(ctx, steering_output)
    ctx.log("Repeat this test after a fix to confirm it.")


STEP = Step(
    "direction_check",
    "Direction check (drive + steering)",
    [who_drives_field("how_dir")],
    [Action("Run direction check", action_check_directions, armed=True)],
)
