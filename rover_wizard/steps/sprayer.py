"""rover_wizard.steps.sprayer - crop sprayer: pump and spinner outputs and rates."""

from __future__ import annotations

from .util.choices import DISABLED_ENABLED_CHOICES
from .util.outputs import RELAY_PIN_OUTPUT_CHOICES, SPRAYER_SERVO_FUNCTIONS, assign_exclusive_output
from .util.hardware import has
from ..model import Step, param_field, value_field


def is_sprayer_enabled(ctx) -> bool:
    """Sprayer enabled in the form (or on the vehicle)."""
    return bool(ctx.form_int("SPRAY_ENABLE"))


def sprayer_output(kind: str):
    """Default for the pump/spinner output field: the output that has that function now."""
    return lambda ctx: ctx.output_with_function(SPRAYER_SERVO_FUNCTIONS[kind])


def apply_sprayer(ctx):
    """Read the sprayer's parameters after enabling it, then assign the pump and spinner outputs."""
    if is_sprayer_enabled(ctx) and not ctx.vehicle.has_parameter("SPRAY_PUMP_RATE"):
        ctx.log("Sprayer enabled - reading its parameters ...")
        # SPRAY_ENABLE reveals the other SPRAY_ parameters
        ctx.fetch_params()

    for kind, function in SPRAYER_SERVO_FUNCTIONS.items():
        assign_exclusive_output(ctx, function, int(ctx.number(f"sprayer_{kind}_output", 0)))


STEP = Step(
    "sprayer",
    "Sprayer",
    [
        param_field("SPRAY_ENABLE", "Sprayer enabled", kind="choice", choices=DISABLED_ENABLED_CHOICES, refresh=True),
        value_field(
            "sprayer_pump_output",
            "Pump output",
            kind="choice",
            choices=RELAY_PIN_OUTPUT_CHOICES,
            default=sprayer_output("pump"),
            when=is_sprayer_enabled,
        ),
        value_field(
            "sprayer_spinner_output",
            "Spinner output (optional)",
            kind="choice",
            choices=RELAY_PIN_OUTPUT_CHOICES,
            default=sprayer_output("spinner"),
            when=is_sprayer_enabled,
        ),
        param_field("SPRAY_PUMP_RATE", "Pump rate at 1 m/s", unit="%", when=is_sprayer_enabled),
        param_field("SPRAY_PUMP_MIN", "Minimum pump rate", unit="%", when=is_sprayer_enabled),
        param_field("SPRAY_SPEED_MIN", "Start spraying above", unit="cm/s", when=is_sprayer_enabled),
        param_field("SPRAY_SPINNER", "Spinner PWM when on", kind="int", unit="us", when=is_sprayer_enabled),
    ],
    [],
    on_apply=apply_sprayer,
    applies=has("has_sprayer"),
)
