"""rover_wizard.steps.gripper - a servo or EPM gripper: positions, output, RC switch, grab/release test."""

from __future__ import annotations

from ..util.errors import VehicleError
from ..rover_defs import RC_OPTION_GRIPPER, SERVO_FUNCTION_GRIPPER
from ..mavlink_defs import ACCEPTED_RESULTS, GRIPPER_ACTION_GRAB, GRIPPER_ACTION_RELEASE
from .util.choices import DISABLED_ENABLED_CHOICES
from .util.outputs import (
    RC_SWITCH_CHOICES,
    RELAY_PIN_OUTPUT_CHOICES,
    assign_exclusive_output,
    assign_rc_switch,
    rc_channel_with_option,
)
from .util.rover_hardware import has
from ..model import Action, Step, param_field, value_field

GRIPPER_TYPE_CHOICES = [("None", 0), ("Servo", 1), ("EPM (electro-permanent magnet)", 2)]

GRIPPER_TYPE_SERVO = 1

GRIPPER_TYPE_EPM = 2


def gripper_type(ctx) -> int:
    """Gripper type chosen in the form (or configured on the vehicle)."""
    return ctx.form_int("GRIP_TYPE")


def gripper_output(ctx) -> int:
    """The servo output currently assigned to the gripper (0 = none)."""
    return ctx.output_with_function(SERVO_FUNCTION_GRIPPER)


def gripper_switch_channel(ctx) -> int:
    """RC channel whose switch operates the gripper (0 = none)."""
    return rc_channel_with_option(ctx, RC_OPTION_GRIPPER)


def apply_gripper(ctx):
    """Refresh the parameter list after enabling, then assign the servo output and the RC switch."""
    if ctx.param_int("GRIP_ENABLE") and not ctx.vehicle.has_parameter("GRIP_TYPE"):
        ctx.log("Gripper enabled - reading its parameters ...")
        # GRIP_ENABLE reveals the other GRIP_ parameters
        ctx.fetch_params()
    assign_exclusive_output(ctx, SERVO_FUNCTION_GRIPPER, int(ctx.number("gripper_output", 0)))
    assign_rc_switch(ctx, RC_OPTION_GRIPPER, int(ctx.number("gripper_rc_channel", 0)))


def make_gripper_action(grab: bool):
    """Grab / Release test button (MAV_CMD_DO_GRIPPER)."""

    def operate_gripper(ctx):
        """Close or open the gripper and report the result."""
        result = ctx.vehicle.gripper(GRIPPER_ACTION_GRAB if grab else GRIPPER_ACTION_RELEASE)
        if result not in ACCEPTED_RESULTS:
            raise VehicleError(
                f"Gripper refused (result {result}): enable it, choose its type and output, apply "
                f"and reboot first. {ctx.vehicle.last_text()}"
            )
        ctx.log(f"Gripper {'grab' if grab else 'release'} sent.")

    return operate_gripper


STEP = Step(
    "gripper",
    "Gripper",
    [
        param_field("GRIP_ENABLE", "Gripper enabled", kind="choice", choices=DISABLED_ENABLED_CHOICES, refresh=True),
        param_field(
            "GRIP_TYPE", "Gripper type", kind="choice", choices=GRIPPER_TYPE_CHOICES, reboot=True, refresh=True
        ),
        value_field(
            "gripper_output",
            "Servo output",
            kind="choice",
            choices=RELAY_PIN_OUTPUT_CHOICES,
            default=gripper_output,
            when=lambda ctx: gripper_type(ctx) == GRIPPER_TYPE_SERVO,
        ),
        param_field(
            "GRIP_GRAB",
            "Grab (closed) PWM",
            kind="int",
            unit="us",
            when=lambda ctx: gripper_type(ctx) == GRIPPER_TYPE_SERVO,
        ),
        param_field(
            "GRIP_RELEASE",
            "Release (open) PWM",
            kind="int",
            unit="us",
            when=lambda ctx: gripper_type(ctx) == GRIPPER_TYPE_SERVO,
        ),
        param_field(
            "GRIP_NEUTRAL",
            "Neutral PWM",
            kind="int",
            unit="us",
            when=lambda ctx: gripper_type(ctx) == GRIPPER_TYPE_SERVO,
        ),
        param_field("GRIP_AUTOCLOSE", "Close again after release", unit="s"),
        param_field(
            "GRIP_REGRAB", "Re-grab interval (EPM)", unit="s", when=lambda ctx: gripper_type(ctx) == GRIPPER_TYPE_EPM
        ),
        value_field(
            "gripper_rc_channel",
            "Operated by RC channel",
            kind="choice",
            choices=RC_SWITCH_CHOICES,
            default=gripper_switch_channel,
        ),
    ],
    [Action("Test: grab", make_gripper_action(True)), Action("Test: release", make_gripper_action(False))],
    on_apply=apply_gripper,
    applies=has("has_gripper"),
)
