"""rover_wizard.steps.servo_outputs - the "Servo outputs" step."""

from __future__ import annotations

from ..ardupilot_defs import RC_CHANNELS, SERVO_FUNCTION_GPIO, SERVO_FUNCTION_RCIN_BASE, SERVO_OUTPUTS
from .util.outputs import AUX_EDITABLE_FUNCTIONS, SERVO_OUTPUT_CHOICES, describe_function, mode_channel, output_function
from .util.hardware import has
from ..model import Action, Step, param_field, value_field

AUX_FUNCTION_CHOICES = (
    [("Not used", 0), ("RC pass-through (same channel number)", 1), ("GPIO pin (for a relay)", -1)]
    # RCIN1..16
    + [(f"Follows RC channel {channel}", SERVO_FUNCTION_RCIN_BASE + channel) for channel in RC_CHANNELS]
)


def _first_free_output(ctx) -> int:
    """Default output to edit: the first one already used for an extra device, else the first unused one."""
    editable = [output for output in SERVO_OUTPUTS if output_function(ctx, output) in AUX_EDITABLE_FUNCTIONS]
    in_use = [output for output in editable if output_function(ctx, output) != 0]
    return (in_use or editable or [1])[0]


def selected_output(ctx) -> int:
    """The output the step's fields currently describe."""
    return int(ctx.values.get("aux_output") or _first_free_output(ctx))


def is_selected_output_editable(ctx) -> bool:
    """Whether the selected output is free for extra devices (not used by steering, motors, a gimbal ...)."""
    return output_function(ctx, selected_output(ctx)) in AUX_EDITABLE_FUNCTIONS


def is_selected_output_pwm(ctx) -> bool:
    """Editable and, per the 'Driven by' choice on screen, a PWM output (not a GPIO pin)."""
    if not is_selected_output_editable(ctx):
        return False
    chosen = ctx.values.get("aux_function")
    function = int(chosen) if chosen is not None else output_function(ctx, selected_output(ctx))
    return function != SERVO_FUNCTION_GPIO


def action_show_output_overview(ctx):
    """Fill the overview with every output and what drives it."""
    count = ctx.servos.output_count()
    lines = [f"SERVO{output}: {describe_function(output_function(ctx, output))}" for output in range(1, count + 1)]
    ctx.set_values(aux_overview="\n".join(lines))


def locked_output_text(ctx) -> str:
    """Why the selected output can't be edited here."""
    output = selected_output(ctx)
    return f"SERVO{output} is used as {describe_function(output_function(ctx, output))} - it can't be changed here."


def make_output_test_action(direction: int):
    """Test +/- : move the selected output one step (DO_SET_SERVO; works for unused and RC-driven outputs)."""

    def move_selected_output(ctx):
        """Nudge the selected output and log where it is now."""
        ctx.log(ctx.servos.nudge(selected_output(ctx), direction))

    return move_selected_output


def apply_aux_servo(ctx):
    """After the parameters were written: warn about RC channels that already have a job, refresh the overview."""
    function = output_function(ctx, selected_output(ctx))
    channel = function - SERVO_FUNCTION_RCIN_BASE
    if channel in RC_CHANNELS:
        steering_channel, throttle_channel = ctx.vehicle.rc_mapping()
        busy = {steering_channel: "steering", throttle_channel: "throttle", mode_channel(ctx): "the flight-mode switch"}
        if channel in busy:
            ctx.log(f"WARNING: RC channel {channel} is {busy[channel]} - the output will follow that stick/switch.")

    if function == SERVO_FUNCTION_GPIO:
        ctx.log("This output is now a GPIO pin: set it up as a relay on the Relays step.")

    action_show_output_overview(ctx)


STEP = Step(
    "servo_outputs",
    "Servo outputs",
    [
        value_field("aux_overview", "Outputs", kind="display", default="(reading ...)"),
        value_field(
            "aux_output",
            "Output to configure",
            kind="choice",
            choices=SERVO_OUTPUT_CHOICES,
            default=lambda ctx: ctx.values.get("aux_output") or selected_output(ctx),
            reload_on_change=True,
            inline_actions=[
                Action("+", make_output_test_action(+1), when=is_selected_output_pwm),
                Action("-", make_output_test_action(-1), when=is_selected_output_pwm),
            ],
        ),
        value_field(
            "aux_locked",
            "In use",
            kind="display",
            default=locked_output_text,
            when=lambda ctx: not is_selected_output_editable(ctx),
        ),
        param_field(
            lambda ctx: f"SERVO{selected_output(ctx)}_FUNCTION",
            "Driven by",
            key="aux_function",
            kind="choice",
            choices=AUX_FUNCTION_CHOICES,
            reboot=True,
            refresh=True,
            when=is_selected_output_editable,
        ),
        param_field(
            lambda ctx: f"SERVO{selected_output(ctx)}_MIN",
            "MIN PWM",
            key="aux_min",
            kind="int",
            unit="us",
            min_value=800,
            max_value=2200,
            when=is_selected_output_pwm,
        ),
        param_field(
            lambda ctx: f"SERVO{selected_output(ctx)}_MAX",
            "MAX PWM",
            key="aux_max",
            kind="int",
            unit="us",
            min_value=800,
            max_value=2200,
            when=is_selected_output_pwm,
        ),
        param_field(
            lambda ctx: f"SERVO{selected_output(ctx)}_TRIM",
            "TRIM PWM",
            key="aux_trim",
            kind="int",
            unit="us",
            min_value=800,
            max_value=2200,
            when=is_selected_output_pwm,
        ),
        param_field(
            lambda ctx: f"SERVO{selected_output(ctx)}_REVERSED",
            "Reverse output",
            key="aux_reversed",
            kind="bool",
            when=is_selected_output_pwm,
        ),
    ],
    [],
    on_apply=apply_aux_servo,
    on_enter=action_show_output_overview,
    applies=has("has_aux_servos"),
)
