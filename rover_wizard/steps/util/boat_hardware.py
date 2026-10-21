"""rover_wizard.steps.util.boat_hardware - detecting the hardware of boats and sailing vehicles (sails, Torqeedo motor,
depth sounder) from the vehicle's parameters; util.rover_hardware lists them with the other optional hardware."""

from __future__ import annotations

from ...rover_defs import (
    RANGEFINDER_INSTANCES,
    RANGEFINDER_ORIENT_DOWN,
    SERVO_FUNCTION_THROTTLE,
    SERVO_FUNCTION_THROTTLE_LEFT,
    SERVO_FUNCTION_THROTTLE_RIGHT,
)

# the first Torqeedo's type: TRQ1_TYPE since firmware 4.7 (two motors), TRQD_TYPE before
TORQEEDO_TYPE_PARAMETER = ("TRQ1_TYPE", "TRQD_TYPE")

# Sailboat::set_motor_state: a motor is there when an output drives the throttle or skid-steering motors
MOTOR_FUNCTIONS = (SERVO_FUNCTION_THROTTLE, SERVO_FUNCTION_THROTTLE_LEFT, SERVO_FUNCTION_THROTTLE_RIGHT)


def are_sails_enabled(ctx) -> bool:
    """ArduPilot's sailing support is switched on (SAIL_ENABLE)."""
    return bool(ctx.param_int("SAIL_ENABLE"))


def is_motor_assigned(ctx) -> bool:
    """An output drives a motor (throttle or a skid-steering side)."""
    return any(ctx.output_with_function(function) for function in MOTOR_FUNCTIONS)


def is_torqeedo_configured(ctx) -> bool:
    """A Torqeedo motor is set up."""
    return bool(ctx.param_int(TORQEEDO_TYPE_PARAMETER))


def depth_rangefinder(ctx) -> str:
    """Instance ("1".."A") of the configured rangefinder pointing down - the depth sounder; "" if there is none."""
    return next(
        (
            instance
            for instance in RANGEFINDER_INSTANCES
            if ctx.param_int(f"RNGFND{instance}_TYPE")
            and ctx.param_int(f"RNGFND{instance}_ORIENT") == RANGEFINDER_ORIENT_DOWN
        ),
        "",
    )
