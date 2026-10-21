"""rover_wizard.steps.util.outputs - servo outputs and RC channels: output functions, RC switches, and the servo
direction / end-point fields."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
from ...util.errors import VehicleError
from ...rover_defs import (
    RC_CHANNELS,
    SERVO_FUNCTION_GPIO,
    SERVO_FUNCTION_GROUND_STEERING,
    SERVO_FUNCTION_RCIN_BASE,
    SERVO_FUNCTION_RC_PASSTHRU,
    SERVO_FUNCTION_THROTTLE,
    SERVO_FUNCTION_THROTTLE_LEFT,
    SERVO_FUNCTION_THROTTLE_RIGHT,
    SERVO_OUTPUTS,
    SERVO_OUTPUTS_MAX,
)
from ...model import Action, Field, param_field

# ArduPilot's MODE_CH default: flight modes on RC8
DEFAULT_MODE_CHANNEL = 8


SERVO_OUTPUT_CHOICES = [(f"SERVO{output}", output) for output in SERVO_OUTPUTS]


RELAY_PIN_OUTPUT_CHOICES = [("None - a dedicated GPIO pin", 0)] + SERVO_OUTPUT_CHOICES


RC_SWITCH_CHOICES = [("No RC switch", 0)] + [(f"RC{channel}", channel) for channel in RC_CHANNELS]


# SERVOx_FUNCTION SprayerPump / SprayerSpinner
SPRAYER_SERVO_FUNCTIONS = {"pump": 22, "spinner": 23}


# [us] accepted range for SERVOx_MIN/MAX/TRIM entries
SERVO_PWM_RANGE = (800, 2200)


@dataclass(frozen=True)
class OutputRole:
    """One kind of SERVO output the wizard configures (steering servo, throttle, left/right motor)."""

    # its SERVOx_FUNCTION value
    function: int
    # used in field keys, e.g. "rev_<tag>"
    tag: str
    # shown to the pilot
    label: str
    # True: only used on skid-steering vehicles; False: only on front-wheel steering; None: on both (e.g. a sail)
    skid: Optional[bool] = None


def servo_role_field(
    role: OutputRole, suffix: str, key: str, label: str, is_front_wheel_selected=None, **kwargs
) -> Field:
    """A field bound to SERVO<n>_<suffix>, where n is whichever output currently has the role's function.

    Shown when the steering type does_fit the role - by default the saved type; `is_front_wheel_selected` (ctx -> bool)
    lets a step that has its own steering-type selector use the selector's current value instead.
    """

    def does_fit(ctx):
        """Whether this role belongs to the vehicle's steering type."""
        if role.skid is None:
            return True
        front_wheel = is_front_wheel_selected(ctx) if is_front_wheel_selected else not ctx.is_skid()
        return front_wheel != role.skid

    return param_field(
        lambda ctx: f"SERVO{ctx.output_with_function(role.function) or 0}_{suffix}",
        label,
        key=key,
        when=does_fit,
        **kwargs,
    )


def _endpoint_button(role: OutputRole, suffix: str, key: str, direction: int) -> Action:
    """A "+" / "-" button that changes SERVOn_<suffix> right away and moves the output to show it."""
    return Action(
        "+" if direction > 0 else "-",
        lambda ctx: ctx.servos.nudge_endpoint(role.function, suffix, key, direction),
        armed=False,
        help=f"Change {suffix} by 10 us immediately (written to the vehicle); the steering servo is "
        "moved there (disarmed), a sail with the sail motor test; motor outputs are not driven.",
    )


def build_servo_direction_fields(roles, is_front_wheel_selected=None) -> list:
    """For each role: a 'Reverse output' checkbox and its MIN/MAX/TRIM pulse widths (see servo_role_field); their help
    comes from wizard_text.json ("<step>.rev_<tag>", "<step>.min_<tag>", ...), as each role needs its own."""
    fields = []
    for role in roles:
        fields.append(
            servo_role_field(
                role,
                "REVERSED",
                f"rev_{role.tag}",
                "Reverse output",
                is_front_wheel_selected=is_front_wheel_selected,
                kind="bool",
            )
        )
        for suffix in ("MIN", "MAX", "TRIM"):
            key = f"{suffix.lower()}_{role.tag}"
            fields.append(
                servo_role_field(
                    role,
                    suffix,
                    key,
                    f"{role.label}: {suffix} PWM",
                    is_front_wheel_selected=is_front_wheel_selected,
                    inline_actions=[_endpoint_button(role, suffix, key, +1), _endpoint_button(role, suffix, key, -1)],
                    kind="int",
                    unit="us",
                    min_value=SERVO_PWM_RANGE[0],
                    max_value=SERVO_PWM_RANGE[1],
                )
            )
    return fields


AUX_EDITABLE_FUNCTIONS = frozenset(
    [0, SERVO_FUNCTION_RC_PASSTHRU, SERVO_FUNCTION_GPIO]
    + [SERVO_FUNCTION_RCIN_BASE + channel for channel in RC_CHANNELS]
)


FUNCTION_NAMES = {
    0: "not used",
    SERVO_FUNCTION_RC_PASSTHRU: "RC pass-through (same channel number)",
    SERVO_FUNCTION_GPIO: "GPIO pin (relay)",
    SERVO_FUNCTION_GROUND_STEERING: "steering servo (Steering output step)",
    SERVO_FUNCTION_THROTTLE: "throttle (Motor output step)",
    SERVO_FUNCTION_THROTTLE_LEFT: "LEFT motor (Motor output step)",
    SERVO_FUNCTION_THROTTLE_RIGHT: "RIGHT motor (Motor output step)",
}


# ... and RCIN1..RCIN16: the output follows that RC channel
FUNCTION_NAMES |= {SERVO_FUNCTION_RCIN_BASE + channel: f"follows RC channel {channel}" for channel in RC_CHANNELS}


def mode_channel(ctx) -> int:
    """RC channel of the flight-mode switch (MODE_CH)."""
    return ctx.param_int("MODE_CH", DEFAULT_MODE_CHANNEL) or DEFAULT_MODE_CHANNEL


def describe_function(function: int) -> str:
    """Readable name of a SERVOx_FUNCTION value."""
    return FUNCTION_NAMES.get(function, f"function {function} (set elsewhere)")


def output_function(ctx, output: int) -> int:
    """Current SERVOx_FUNCTION of an output (0 if unknown)."""
    return ctx.param_int(f"SERVO{output}_FUNCTION")


def assign_output_function(ctx, output: int, function: int) -> None:
    """Give a free servo output a function (e.g. GPIO, gripper), refusing outputs that something else uses."""
    current = output_function(ctx, output)
    if current != function and current not in AUX_EDITABLE_FUNCTIONS:
        raise VehicleError(f"SERVO{output} is used as {describe_function(current)} - choose another output.")
    ctx.set_param(f"SERVO{output}_FUNCTION", function, reboot=True)


def assign_exclusive_output(ctx, function: int, output: int) -> None:
    """Give exactly one output (0 = none) the function, removing it from any other output first."""
    for other in SERVO_OUTPUTS:
        if other != output and output_function(ctx, other) == function:
            ctx.set_param(f"SERVO{other}_FUNCTION", 0, reboot=True)

    if output:
        assign_output_function(ctx, output, function)


def rc_channel_with_option(ctx, option: int) -> int:
    """The RC channel whose RCx_OPTION is `option` (0 = none)."""
    return next((channel for channel in RC_CHANNELS if ctx.param_int(f"RC{channel}_OPTION") == option), 0)


def assign_rc_switch(ctx, option: int, channel: int) -> None:
    """Let RC channel `channel` trigger an RCx_OPTION (0 = no switch); the option is removed from every other channel.

    Refuses the steering, throttle and mode-switch channels and channels that already have another option.
    """
    if channel:
        steering_channel, throttle_channel = ctx.vehicle.rc_mapping()
        if channel in (steering_channel, throttle_channel, mode_channel(ctx)):
            raise VehicleError(
                f"RC{channel} is used for steering, throttle or the mode switch - choose another channel."
            )
        current = ctx.param_int(f"RC{channel}_OPTION")
        if current not in (0, option):
            raise VehicleError(f"RC{channel} already has RC{channel}_OPTION = {current} - choose another channel.")

    for other in RC_CHANNELS:
        if other != channel and ctx.param_int(f"RC{other}_OPTION") == option:
            ctx.set_param(f"RC{other}_OPTION", 0, required=False)

    if channel:
        ctx.set_param(f"RC{channel}_OPTION", option, required=False)


ROVER_OUTPUT_FUNCTIONS = {
    SERVO_FUNCTION_GROUND_STEERING: "GroundSteering",
    SERVO_FUNCTION_THROTTLE: "Throttle",
    SERVO_FUNCTION_THROTTLE_LEFT: "ThrottleLeft",
    SERVO_FUNCTION_THROTTLE_RIGHT: "ThrottleRight",
}


def assign_rover_outputs(ctx, wanted: dict, functions_of_this_step: tuple) -> None:
    """Give each output in `wanted` ({output: function}) its function and clear those functions elsewhere.

    Refuses an output that currently carries a rover function configured by a *different* step (e.g. putting the
    steering servo on the throttle output), because silently overwriting it would break that other step's setup.
    """
    for output in wanted:
        if not 1 <= output <= SERVO_OUTPUTS_MAX:
            raise VehicleError(f"Choose output channels between 1 and {SERVO_OUTPUTS_MAX}.")

        current = output_function(ctx, output)
        if current in ROVER_OUTPUT_FUNCTIONS and current not in functions_of_this_step:
            raise VehicleError(
                f"SERVO{output} is already used as {ROVER_OUTPUT_FUNCTIONS[current]} - choose another output."
            )
    # Clear this step's functions from outputs that no longer carry them, then assign the new ones.
    for output in SERVO_OUTPUTS:
        if output_function(ctx, output) in functions_of_this_step and output not in wanted:
            ctx.set_param(f"SERVO{output}_FUNCTION", 0, reboot=True)

    for output, function in wanted.items():
        ctx.set_param(f"SERVO{output}_FUNCTION", function, reboot=True)
