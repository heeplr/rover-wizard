"""rover_wizard.steps.motor_output - the "Motor output" step."""

from __future__ import annotations

from ..util.errors import VehicleError
from ..ardupilot_defs import (
    MOTOR_TEST_INSTANCE_THROTTLE,
    MOTOR_TEST_INSTANCE_THROTTLE_LEFT,
    MOTOR_TEST_INSTANCE_THROTTLE_RIGHT,
    SERVO_FUNCTION_THROTTLE,
    SERVO_FUNCTION_THROTTLE_LEFT,
    SERVO_FUNCTION_THROTTLE_RIGHT,
)
from ..mavlink_defs import ACCEPTED_RESULTS
from ..motion.servo_test import (
    DEFAULT_MOTOR_TEST_PERCENT,
    MOTOR_TEST_REQUIREMENTS,
    MOTOR_TEST_SECONDS,
    MOTOR_TEST_STOP_SECONDS,
    arming_checks_skipped,
)
from ..util.timebase import now
from .util.outputs import OutputRole, assign_rover_outputs, build_servo_direction_fields
from ..model import Action, Step, param_field, value_field

DEFAULT_THROTTLE_OUTPUT = 3

DEFAULT_LEFT_MOTOR_OUTPUT = 1

DEFAULT_RIGHT_MOTOR_OUTPUT = 3

MOTOR_PWM_TYPE_CHOICES = [
    ("Normal PWM (ESC / servo)", 0),
    ("OneShot", 1),
    ("OneShot125", 2),
    ("Brushed with relay", 3),
    ("Brushed bipolar (H-bridge)", 4),
    ("DShot150", 5),
    ("DShot300", 6),
    ("DShot600", 7),
    ("DShot1200", 8),
]

# MOT_PWM_TYPE values that need MOT_PWM_FREQ
BRUSHED_PWM_TYPES = (3, 4)

MOTOR_ROLES = (
    OutputRole(SERVO_FUNCTION_THROTTLE, "throttle", "Throttle / ESC", skid=False),
    OutputRole(SERVO_FUNCTION_THROTTLE_LEFT, "left", "LEFT motor", skid=True),
    OutputRole(SERVO_FUNCTION_THROTTLE_RIGHT, "right", "RIGHT motor", skid=True),
)

# All motor functions belong to the motor step, so switching steering type replaces throttle <-> left/right cleanly.
MOTOR_FUNCTIONS = (SERVO_FUNCTION_THROTTLE, SERVO_FUNCTION_THROTTLE_LEFT, SERVO_FUNCTION_THROTTLE_RIGHT)

# give up if the wheel hasn't moved at this motor power
DEADZONE_SEARCH_MAX_PERCENT = 40
# how long each power level is applied
DEADZONE_STEP_SECONDS = 2.0
# each step's motor test outlasts the step, so the test never lapses between
DEADZONE_TEST_OVERLAP = 1.0
# how often the dead-zone ramp checks for Continue
DEADZONE_POLL_SECONDS = 0.05
# Each ramp step is logged with the first output report received this long after the step started (an earlier
# one may still show the previous power).
DEADZONE_LOG_DELAY_SECONDS = 0.3

# What the ESC / motor driver can do below neutral (saved as the 'esc_dir' setting).
ESC_REVERSE = "reverse"
ESC_BRAKE = "brake"
ESC_FORWARD_ONLY = "forward"
ESC_DIRECTION_CHOICES = [
    ("Forward and reverse", ESC_REVERSE),
    ("Forward and brake (car ESC)", ESC_BRAKE),
    ("Forward only", ESC_FORWARD_ONLY),
]


def apply_motor_output(ctx):
    """Assign the throttle function (or the left/right motor functions for skid steering) to the chosen outputs."""
    if ctx.is_skid():
        left = int(ctx.number("ch_left", DEFAULT_LEFT_MOTOR_OUTPUT))
        right = int(ctx.number("ch_right", DEFAULT_RIGHT_MOTOR_OUTPUT))
        if left == right:
            raise VehicleError("LEFT and RIGHT motor need two DIFFERENT outputs.")
        assign_rover_outputs(
            ctx, {left: SERVO_FUNCTION_THROTTLE_LEFT, right: SERVO_FUNCTION_THROTTLE_RIGHT}, MOTOR_FUNCTIONS
        )
    else:
        assign_rover_outputs(
            ctx, {int(ctx.number("ch_thr", DEFAULT_THROTTLE_OUTPUT)): SERVO_FUNCTION_THROTTLE}, MOTOR_FUNCTIONS
        )
    apply_esc_direction(ctx)


def _motor_outputs(ctx) -> list:
    """The SERVO outputs that currently drive a motor."""
    return [output for output in (ctx.output_with_function(function) for function in MOTOR_FUNCTIONS) if output]


def guess_esc_direction(ctx) -> str:
    """The saved ESC type; otherwise forward-only if the wizard's forward-only settings are found, else reverse."""
    saved = ctx.store.setting("esc_dir")
    if saved in (ESC_REVERSE, ESC_BRAKE, ESC_FORWARD_ONLY):
        return saved
    outputs = _motor_outputs(ctx)
    trims_at_minimum = bool(outputs) and all(
        ctx.param_value(f"SERVO{output}_TRIM", 0) <= ctx.param_value(f"SERVO{output}_MIN", 0) for output in outputs
    )
    return ESC_FORWARD_ONLY if trims_at_minimum and ctx.param_value("ATC_BRAKE", 1) == 0 else ESC_REVERSE


def is_reversible(ctx) -> bool:
    """Whether the ESC chosen in the form can drive (or brake) below neutral."""
    return (ctx.value("esc_dir") or guess_esc_direction(ctx)) != ESC_FORWARD_ONLY


def apply_esc_direction(ctx) -> None:
    """Forward-only ESC: zero throttle is the lowest pulse (TRIM = MIN) and the speed controller mustn't brake.
    Back from forward-only: put the neutral pulse back in the middle of the range."""
    esc = ctx.value("esc_dir") or guess_esc_direction(ctx)
    previous = guess_esc_direction(ctx)
    for output in _motor_outputs(ctx):
        minimum, maximum, trim = (ctx.param_value(f"SERVO{output}_{suffix}") for suffix in ("MIN", "MAX", "TRIM"))
        if None in (minimum, maximum, trim):
            continue
        if esc == ESC_FORWARD_ONLY and trim != minimum:
            ctx.set_param(f"SERVO{output}_TRIM", minimum)
            ctx.log(
                f"Forward-only ESC: SERVO{output}_TRIM = SERVO{output}_MIN ({minimum:g} us), so zero throttle is "
                "the lowest pulse."
            )
        elif esc != ESC_FORWARD_ONLY and previous == ESC_FORWARD_ONLY and trim <= minimum:
            middle = round((minimum + maximum) / 2)
            ctx.set_param(f"SERVO{output}_TRIM", middle)
            ctx.log(
                f"Reversible ESC: SERVO{output}_TRIM back to {middle} us (middle of the range) - check the "
                "ESC's neutral with the TRIM button on the Servo outputs step."
            )
    if esc == ESC_FORWARD_ONLY:
        ctx.set_param("ATC_BRAKE", 0)
    ctx.store.update_settings(esc_dir=esc)


def make_motor_test_action(instance: int, direction: int, motor_name: str):
    """A Test up/down button: run ArduPilot's motor test on one motor output forward (+1) or in reverse (-1)."""

    def run_motor_test(ctx):
        """Run the motor test for this button (ArduPilot arms the vehicle for the test and disarms it after)."""
        # +power = forward, -power = reverse
        percent = direction * ctx.number("test_pct", DEFAULT_MOTOR_TEST_PERCENT)
        ctx.log(
            f"Motor test: {motor_name} {'forward' if direction > 0 else 'reverse'} {abs(percent):.0f}% for "
            f"{MOTOR_TEST_SECONDS:g} s (ArduPilot arms the vehicle for the test)"
        )
        ctx.servos.run_motor_test(instance, percent, MOTOR_TEST_SECONDS)

    return run_motor_test


def _reported_motor_pulse(ctx, function: int) -> str:
    """'SERVOn: <pulse> us' of the motor's output as the flight controller reports it."""
    output = ctx.output_with_function(function)
    pulse = ctx.vehicle.servo_pulse(output)
    return f"SERVO{output}: {pulse} us" if pulse else "output not reported"


def action_find_throttle_deadzone(ctx):
    """Ramp motor power 1 % every DEADZONE_STEP_SECONDS until the lifted wheel turns; propose the power just below
    as MOT_THR_MIN.

    MOT_THR_MIN is set to 0 during the search (otherwise the motor test would already add it) and is always
    restored afterwards - the result is only a proposal in the form until the pilot presses Apply.
    """
    instance = MOTOR_TEST_INSTANCE_THROTTLE_LEFT if ctx.is_skid() else MOTOR_TEST_INSTANCE_THROTTLE
    function = SERVO_FUNCTION_THROTTLE_LEFT if ctx.is_skid() else SERVO_FUNCTION_THROTTLE
    if not ctx.confirm("Driven wheels off the ground and free to spin? Start the motor ramp?"):
        ctx.log("Cancelled - no changes made.")
        return
    original = ctx.param_value("MOT_THR_MIN", 0)
    ctx.set_param("MOT_THR_MIN", 0)
    found = None
    ctx.begin_wait("Watch the wheel: press Continue the moment it starts to turn.")
    try:
        # The first step arms the vehicle for the motor test; later steps continue the running test.
        with arming_checks_skipped(ctx.vehicle):
            for percent in range(1, DEADZONE_SEARCH_MAX_PERCENT + 1):
                result = ctx.vehicle.motor_test(instance, percent, DEADZONE_STEP_SECONDS + DEADZONE_TEST_OVERLAP)
                if result not in ACCEPTED_RESULTS:
                    raise VehicleError(
                        f"Motor test refused (result {result}): {MOTOR_TEST_REQUIREMENTS} " f"{ctx.vehicle.last_text()}"
                    )
                step_start = now()
                logged = False
                while now() - step_start < DEADZONE_STEP_SECONDS and found is None:
                    ctx.check()
                    # log every step - with a report that arrived late enough to show this step's power
                    reported_at = now() - ctx.vehicle.age("SERVO_OUTPUT_RAW")
                    if not logged and reported_at >= step_start + DEADZONE_LOG_DELAY_SECONDS:
                        ctx.log(f"Motor power {percent}% - {_reported_motor_pulse(ctx, function)}")
                        logged = True
                    if ctx.was_continue_pressed():
                        # The pilot reacts a little late, so the previous (lower) level is the better estimate.
                        found = max(1, percent - 1)
                    ctx.sleep(DEADZONE_POLL_SECONDS)
                if found is not None:
                    break
                if not logged:
                    # no fresh report during the whole step
                    ctx.log(f"Motor power {percent}% - output not reported")
    finally:
        ctx.end_wait()
        # stop the wheel now, not when the test lapses
        ctx.vehicle.motor_test(instance, 0, MOTOR_TEST_STOP_SECONDS)
        ctx.set_param("MOT_THR_MIN", original)
    if found is not None:
        ctx.log(f"Wheel started to move at about {found}% -> suggested MOT_THR_MIN = {found}")
        ctx.set_values(MOT_THR_MIN=found)
        ctx.log("Press 'Apply' to write it.")
    else:
        ctx.log(
            f"No movement detected up to {DEADZONE_SEARCH_MAX_PERCENT}% - check wiring/ESC arming, "
            "MOT_THR_MIN left unchanged."
        )


STEP = Step(
    "motor_output",
    "Motor output",
    [
        param_field(
            "MOT_PWM_TYPE",
            "Motor driver / ESC signal",
            kind="choice",
            choices=MOTOR_PWM_TYPE_CHOICES,
            reboot=True,
            refresh=True,
        ),
        param_field(
            "MOT_PWM_FREQ",
            "Brushed motor PWM frequency",
            kind="int",
            unit="kHz",
            when=lambda ctx: ctx.number("MOT_PWM_TYPE", 0) in BRUSHED_PWM_TYPES,
        ),
        value_field(
            "test_pct", "Motor test power", unit="%", min_value=1, max_value=100, default=DEFAULT_MOTOR_TEST_PERCENT
        ),
        value_field(
            "ch_thr",
            "Throttle / ESC output #",
            kind="int",
            min_value=1,
            max_value=16,
            default=lambda ctx: ctx.output_with_function(SERVO_FUNCTION_THROTTLE) or DEFAULT_THROTTLE_OUTPUT,
            when=lambda ctx: not ctx.is_skid(),
            inline_actions=[
                Action("Test ↑", make_motor_test_action(MOTOR_TEST_INSTANCE_THROTTLE, 1, "throttle")),
                Action("Test ↓", make_motor_test_action(MOTOR_TEST_INSTANCE_THROTTLE, -1, "throttle")),
            ],
        ),
        value_field(
            "ch_left",
            "LEFT motor output #",
            kind="int",
            min_value=1,
            max_value=16,
            default=lambda ctx: ctx.output_with_function(SERVO_FUNCTION_THROTTLE_LEFT) or DEFAULT_LEFT_MOTOR_OUTPUT,
            when=lambda ctx: ctx.is_skid(),
            inline_actions=[
                Action("Test ↑", make_motor_test_action(MOTOR_TEST_INSTANCE_THROTTLE_LEFT, 1, "LEFT motor")),
                Action("Test ↓", make_motor_test_action(MOTOR_TEST_INSTANCE_THROTTLE_LEFT, -1, "LEFT motor")),
            ],
        ),
        value_field(
            "ch_right",
            "RIGHT motor output #",
            kind="int",
            min_value=1,
            max_value=16,
            default=lambda ctx: ctx.output_with_function(SERVO_FUNCTION_THROTTLE_RIGHT) or DEFAULT_RIGHT_MOTOR_OUTPUT,
            when=lambda ctx: ctx.is_skid(),
            inline_actions=[
                Action("Test ↑", make_motor_test_action(MOTOR_TEST_INSTANCE_THROTTLE_RIGHT, 1, "RIGHT motor")),
                Action("Test ↓", make_motor_test_action(MOTOR_TEST_INSTANCE_THROTTLE_RIGHT, -1, "RIGHT motor")),
            ],
        ),
    ]
    + build_servo_direction_fields(list(MOTOR_ROLES))
    + [
        param_field(
            "MOT_THR_MIN",
            "Minimum throttle (dead-zone)",
            unit="%",
            inline_actions=[Action("Probe", action_find_throttle_deadzone, armed=False)],
        ),
        param_field("MOT_THR_MAX", "Maximum throttle", unit="%"),
        param_field("MOT_SLEWRATE", "Throttle slew rate", unit="%/s"),
        value_field(
            "esc_dir",
            "ESC / motor driver can",
            kind="choice",
            choices=ESC_DIRECTION_CHOICES,
            default=guess_esc_direction,
            refresh=True,
        ),
        param_field(
            "ATC_BRAKE",
            "Speed controller may brake / reverse",
            kind="choice",
            choices=[("No - only coast to slow down", 0), ("Yes - reverse output to slow down", 1)],
            when=is_reversible,
        ),
        param_field(
            "MOT_REV_DELAY", "Pause before changing direction", unit="s", min_value=0, max_value=1, when=is_reversible
        ),
        param_field("MOT_STR_THR_MIX", "Steering vs throttle priority", when=lambda ctx: ctx.is_skid()),
    ],
    [],
    on_apply=apply_motor_output,
)
