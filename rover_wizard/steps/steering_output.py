"""rover_wizard.steps.steering_output - the "Steering output" step."""

from __future__ import annotations

from ..rover_defs import SERVO_FUNCTION_GROUND_STEERING
from ..motion.servo_test import DEFAULT_STEERING_TEST_PERCENT, steering_test_action
from .util.outputs import OutputRole, assign_rover_outputs, build_servo_direction_fields
from ..model import Action, Step, Wording, param_field, value_field

# proposed while no output steers yet: ArduPilot Rover's default layout has the steering servo on output 1
DEFAULT_STEERING_OUTPUT = 1

STEERING_TYPE_CHOICES = [
    (Wording("Front-wheel steering (Ackermann)", "Rudder or steered motor (outboard)"), "ackermann"),
    (Wording("Skid steering (differential)", "Two motors (differential thrust)"), "skid"),
]

# the steering type in a few words, e.g. for the status line
STEERING_KIND = {"ackermann": Wording("FRONT-WHEEL", "RUDDER"), "skid": Wording("SKID", "TWIN-MOTOR")}

STEERING_ROLE = OutputRole(SERVO_FUNCTION_GROUND_STEERING, "steering", "Steering servo", skid=False)


def apply_steering_output(ctx):
    """Save the steering type; front-wheel: assign GroundSteering to the chosen output, skid: remove it anywhere."""
    steering_type = ctx.value("steer")
    ctx.store.set("steer", steering_type)
    ctx.log(f"Steering type: {steering_type}")
    wanted = {} if steering_type == "skid" else {int(ctx.number("ch_steer", 1)): SERVO_FUNCTION_GROUND_STEERING}
    assign_rover_outputs(ctx, wanted, (SERVO_FUNCTION_GROUND_STEERING,))


def steering_kind(ctx) -> str:
    """The vehicle's steering type in a few words ("FRONT-WHEEL", on a boat "RUDDER", ...)."""
    return ctx.words(STEERING_KIND["skid" if ctx.is_skid() else "ackermann"])


def _is_front_wheel_selected(ctx) -> bool:
    """Front-wheel steering chosen in the Steering-type selector of this step (saved choice until it is shown).

    The steering fields follow the selector itself, so switching the type shows/hides them at once, before Apply.
    """
    selected = ctx.values.get("steer")
    return selected == "ackermann" if selected else not ctx.is_skid()


STEP = Step(
    "steering_output",
    "Steering output",
    [
        value_field(
            "steer",
            "Steering type",
            kind="choice",
            refresh=True,
            choices=STEERING_TYPE_CHOICES,
            default=lambda ctx: "skid" if ctx.is_skid() else "ackermann",
        ),
        value_field(
            "test_pct",
            "Testing deflection",
            unit="%",
            min_value=1,
            max_value=100,
            default=DEFAULT_STEERING_TEST_PERCENT,
            when=_is_front_wheel_selected,
        ),
        value_field(
            "ch_steer",
            "Steering servo output #",
            kind="int",
            min_value=1,
            max_value=16,
            default=lambda ctx: ctx.output_with_function(SERVO_FUNCTION_GROUND_STEERING) or DEFAULT_STEERING_OUTPUT,
            when=_is_front_wheel_selected,
            inline_actions=[
                Action("Test L", steering_test_action(-1), armed=False),
                Action("Test R", steering_test_action(+1), armed=False),
            ],
        ),
    ]
    + build_servo_direction_fields([STEERING_ROLE], is_front_wheel_selected=_is_front_wheel_selected)
    + [
        # boats whose steering servo turns the motor itself (AP_MotorsUGV: boats only)
        param_field(
            "MOT_VEC_ANGLEMAX",
            "Vectored thrust: motor angle at full steering (0 = off)",
            unit="deg",
            min_value=0,
            max_value=90,
            when=lambda ctx: ctx.is_boat() and _is_front_wheel_selected(ctx),
        )
    ],
    [],
    on_apply=apply_steering_output,
)
