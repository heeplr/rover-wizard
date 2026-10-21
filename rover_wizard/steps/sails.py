"""rover_wizard.steps.sails - the "Sails" step: the sail actuator, how ArduPilot trims the sail, and the RC switches
of a sailing vehicle (boat or land yacht).

ArduPilot's sailing code (Rover/sailboat.cpp) is switched on with SAIL_ENABLE (the Vehicle type step's "Vehicle has
sails"); its other SAIL_ settings exist only after a reboot. The sail is moved by one kind of actuator per vehicle
(AP_MotorsUGV): a winch on the main sheet, a wing sail elevator, or a servo turning the mast.
"""

from __future__ import annotations

from ..util.errors import VehicleError
from ..boat_defs import (
    MOTOR_TEST_INSTANCE_MAINSAIL,
    RC_OPTION_MAINSAIL,
    RC_OPTION_SAILBOAT_MOTOR_3POS,
    RC_OPTION_SAILBOAT_TACK,
    SAIL_FUNCTIONS,
    SERVO_FUNCTION_MAINSAIL,
    SERVO_FUNCTION_MAST_ROTATION,
    SERVO_FUNCTION_WINGSAIL_ELEVATOR,
)
from ..motion.sail_test import sail_range
from .util.rover_hardware import has
from .util.outputs import (
    RC_SWITCH_CHOICES,
    OutputRole,
    assign_exclusive_output,
    assign_rc_switch,
    build_servo_direction_fields,
    rc_channel_with_option,
)
from ..model import Action, Step, param_field, value_field

SAIL_ACTUATOR_CHOICES = [
    ("Sail winch on the main sheet", SERVO_FUNCTION_MAINSAIL),
    ("Wing sail elevator (flap)", SERVO_FUNCTION_WINGSAIL_ELEVATOR),
    ("Servo turning the mast / wing directly", SERVO_FUNCTION_MAST_ROTATION),
]

SAIL_ROLES = [
    OutputRole(SERVO_FUNCTION_MAINSAIL, "mainsail", "Sail winch"),
    OutputRole(SERVO_FUNCTION_WINGSAIL_ELEVATOR, "wingsail", "Wing sail elevator"),
    OutputRole(SERVO_FUNCTION_MAST_ROTATION, "mast", "Mast rotation"),
]

# [s] a sail test holds the position this long
SAIL_TEST_SECONDS = 3.0


def sail_output(ctx) -> int:
    """The output that moves the sail (0 = none yet)."""
    return next((output for output in map(ctx.output_with_function, SAIL_FUNCTIONS) if output), 0)


def sail_actuator(ctx) -> int:
    """SERVOx_FUNCTION of the sail actuator: chosen in the form, else the one assigned, else a winch."""
    chosen = ctx.values.get("sail_actuator")
    if chosen:
        return int(chosen)
    return next(
        (function for function in SAIL_FUNCTIONS if ctx.output_with_function(function)), SERVO_FUNCTION_MAINSAIL
    )


def are_sail_settings_loaded(ctx) -> bool:
    """ArduPilot shows its sailing settings: SAIL_ENABLE is set and the flight controller has rebooted since."""
    return ctx.vehicle.has_parameter("SAIL_ANGLE_MIN")


def _is_winch(ctx) -> bool:
    """The sail is trimmed with a winch: only then do the sail angles apply."""
    return are_sail_settings_loaded(ctx) and sail_actuator(ctx) == SERVO_FUNCTION_MAINSAIL


def _has_motor(ctx) -> bool:
    """The vehicle has a motor besides the sails (motor assistance, motor switch)."""
    return are_sail_settings_loaded(ctx) and has("has_motor")(ctx)


def reboot_note(ctx) -> str:
    """Shown until the sailing settings appear."""
    if ctx.param_int("SAIL_ENABLE"):
        return "SAIL_ENABLE is set - reboot the flight controller (Reboot button) to see the sailing settings."
    return "Tick 'Vehicle has sails' on the Vehicle type step (sets SAIL_ENABLE), then reboot."


def make_sail_test(direction: int):
    """A Test - / Test + button: move the sail output to its lowest / highest position with the sail motor test."""

    def test_sail(ctx):
        """Drive the sail output to one end for a few seconds (the motor test arms the vehicle meanwhile)."""
        if not sail_output(ctx):
            raise VehicleError("Assign the sail output first (enter its number and press Apply).")
        low, high = sail_range(sail_actuator(ctx))
        percent = high if direction > 0 else low
        ctx.servos.run_motor_test(MOTOR_TEST_INSTANCE_MAINSAIL, percent, SAIL_TEST_SECONDS)
        ctx.log(f"Sail output at {percent:+.0f}% for {SAIL_TEST_SECONDS:g} s (sail motor test, arms the vehicle).")

    return test_sail


def apply_sails(ctx) -> None:
    """Give the chosen output the sail function (and no other output any sail function); the RC switches."""
    if not are_sail_settings_loaded(ctx):
        return
    output = int(ctx.number("sail_output", 0))
    if not output:
        raise VehicleError("Enter the servo output the sail actuator is connected to.")
    actuator = sail_actuator(ctx)
    # first take the other sail functions away (also from this output), then give it the chosen one
    for function in SAIL_FUNCTIONS:
        if function != actuator:
            assign_exclusive_output(ctx, function, 0)
    assign_exclusive_output(ctx, actuator, output)
    assign_rc_switch(ctx, RC_OPTION_SAILBOAT_TACK, int(ctx.number("tack_switch", 0)))
    if has("has_motor")(ctx):
        assign_rc_switch(ctx, RC_OPTION_MAINSAIL, int(ctx.number("mainsail_channel", 0)))
        assign_rc_switch(ctx, RC_OPTION_SAILBOAT_MOTOR_3POS, int(ctx.number("motor_switch", 0)))


STEP = Step(
    "sails",
    "Sails",
    [
        value_field(
            "sail_reboot",
            "Sailing settings",
            kind="display",
            default=reboot_note,
            when=lambda ctx: not are_sail_settings_loaded(ctx),
        ),
        value_field(
            "sail_actuator",
            "The sail is moved by",
            kind="choice",
            choices=SAIL_ACTUATOR_CHOICES,
            default=sail_actuator,
            refresh=True,
            when=are_sail_settings_loaded,
        ),
        value_field(
            "sail_output",
            "Sail servo output #",
            kind="int",
            min_value=1,
            max_value=16,
            default=lambda ctx: sail_output(ctx) or None,
            when=are_sail_settings_loaded,
            inline_actions=[
                Action("Test -", make_sail_test(-1), armed=False),
                Action("Test +", make_sail_test(+1), armed=False),
            ],
        ),
    ]
    + build_servo_direction_fields(SAIL_ROLES)
    + [
        param_field("SAIL_ANGLE_MIN", "Boom angle, sheeted in", unit="deg", when=_is_winch),
        param_field("SAIL_ANGLE_MAX", "Boom angle, let out", unit="deg", when=_is_winch),
        param_field("SAIL_ANGLE_IDEAL", "Ideal angle between sail and wind", unit="deg", when=_is_winch),
        param_field("SAIL_HEEL_MAX", "Maximum heel angle", unit="deg", when=are_sail_settings_loaded),
        param_field("ATC_SAIL_P", "Heel control P", when=are_sail_settings_loaded),
        param_field("ATC_SAIL_I", "Heel control I", when=are_sail_settings_loaded),
        param_field("ATC_SAIL_IMAX", "Heel control I maximum", when=are_sail_settings_loaded),
        param_field(
            "SAIL_NO_GO_ANGLE", "Closest angle to the wind (no-go zone)", unit="deg", when=are_sail_settings_loaded
        ),
        param_field(
            "SAIL_XTRACK_MAX", "Tack at this cross-track error (0 = off)", unit="m", when=are_sail_settings_loaded
        ),
        param_field("SAIL_LOIT_RADIUS", "Loiter radius while sailing", unit="m", when=are_sail_settings_loaded),
        param_field("SAIL_WNDSPD_MIN", "Motor on below this wind speed (0 = never)", unit="m/s", when=_has_motor),
        value_field(
            "tack_switch",
            "Tack switch",
            kind="choice",
            choices=RC_SWITCH_CHOICES,
            default=lambda ctx: rc_channel_with_option(ctx, RC_OPTION_SAILBOAT_TACK),
            when=are_sail_settings_loaded,
        ),
        value_field(
            "mainsail_channel",
            "Sail controlled by RC channel (instead of the throttle stick)",
            kind="choice",
            choices=RC_SWITCH_CHOICES,
            default=lambda ctx: rc_channel_with_option(ctx, RC_OPTION_MAINSAIL),
            when=_has_motor,
        ),
        value_field(
            "motor_switch",
            "Motor switch (never / assist / always)",
            kind="choice",
            choices=RC_SWITCH_CHOICES,
            default=lambda ctx: rc_channel_with_option(ctx, RC_OPTION_SAILBOAT_MOTOR_3POS),
            when=_has_motor,
        ),
    ],
    [],
    on_apply=apply_sails,
    applies=has("has_sails"),
)
