"""rover_wizard.steps.relays - the "Relays" step."""

from __future__ import annotations

from ..util.errors import VehicleError
from ..rover_defs import LEGACY_RELAY_PIN, RC_OPTION_RELAY, RELAYS, SERVO_FUNCTION_GPIO
from ..mavlink_defs import ACCEPTED_RESULTS
from .util.outputs import (
    RC_SWITCH_CHOICES,
    RELAY_PIN_OUTPUT_CHOICES,
    assign_output_function,
    assign_rc_switch,
    rc_channel_with_option,
)
from .util.rover_hardware import has
from ..model import Action, Step, param_field, value_field

RELAY_CHOICES = [(f"Relay {relay}", relay) for relay in RELAYS]

RELAY_FUNCTION_CHOICES = [("None", 0), ("Relay (switched output)", 1)]

RELAY_DEFAULT_CHOICES = [("Off", 0), ("On", 1), ("No change", 2)]


def selected_relay(ctx) -> int:
    """The relay (1..6) the step's fields currently describe."""
    return int(ctx.values.get("relay") or 1)


def relay_pin_names(ctx) -> tuple:
    """RELAYn_PIN, or the pre-4.5 name of the same parameter."""
    relay = selected_relay(ctx)
    return (f"RELAY{relay}_PIN", LEGACY_RELAY_PIN[relay])


def relay_switch_channel(ctx) -> int:
    """RC channel whose RCx_OPTION toggles the selected relay (0 = none)."""
    return rc_channel_with_option(ctx, RC_OPTION_RELAY[selected_relay(ctx)])


def apply_relay(ctx):
    """After the RELAYn_* parameters were written: pin output and RC switch for the selected relay."""
    relay = selected_relay(ctx)
    output = int(ctx.number("relay_servo_output", 0))
    if output:
        assign_output_function(ctx, output, SERVO_FUNCTION_GPIO)
    assign_rc_switch(ctx, RC_OPTION_RELAY[relay], int(ctx.number("relay_rc_channel", 0)))

    if ctx.vehicle.has_parameter(f"RELAY{relay}_FUNCTION") and ctx.param_int(f"RELAY{relay}_FUNCTION") == 0:
        ctx.log(f"Note: RELAY{relay}_FUNCTION is None - set it to 'Relay' or the relay stays inactive.")


def switch_relay(ctx, relay: int, on: bool) -> None:
    """Switch a relay (MAV_CMD_DO_SET_RELAY) and log it; VehicleError if the flight controller refuses."""
    result = ctx.vehicle.set_relay(relay, on)
    if result not in ACCEPTED_RESULTS:
        raise VehicleError(
            f"Relay {relay} refused (result {result}): apply its pin and function first "
            f"(a pin change needs a reboot). {ctx.vehicle.last_text()}"
        )
    ctx.log(f"Relay {relay} switched {'ON' if on else 'OFF'}.")


def toggle_relay(ctx, relay: int) -> None:
    """Switch a relay to the opposite of its reported state (the relay buttons in the side panel)."""
    switch_relay(ctx, relay, not ctx.vehicle.relays().get(relay, False))


def make_relay_test_action(on: bool):
    """Test On / Off for the selected relay (MAV_CMD_DO_SET_RELAY)."""

    def switch_selected_relay(ctx):
        """Switch the selected relay and report whether the flight controller accepted it."""
        switch_relay(ctx, selected_relay(ctx), on)

    return switch_selected_relay


STEP = Step(
    "relays",
    "Relays",
    [
        value_field(
            "relay",
            "Relay to configure",
            kind="choice",
            choices=RELAY_CHOICES,
            default=lambda ctx: ctx.values.get("relay") or 1,
            reload_on_change=True,
            inline_actions=[Action("On", make_relay_test_action(True)), Action("Off", make_relay_test_action(False))],
        ),
        param_field(
            lambda ctx: f"RELAY{selected_relay(ctx)}_FUNCTION",
            "Function",
            key="relay_function",
            kind="choice",
            choices=RELAY_FUNCTION_CHOICES,
            reboot=True,
        ),
        param_field(relay_pin_names, "GPIO pin number", key="relay_pin", kind="int", reboot=True),
        value_field(
            "relay_servo_output",
            "Servo output used as this pin",
            kind="choice",
            choices=RELAY_PIN_OUTPUT_CHOICES,
            default=0,
        ),
        param_field(
            lambda ctx: f"RELAY{selected_relay(ctx)}_DEFAULT",
            "State at start-up",
            key="relay_default",
            kind="choice",
            choices=RELAY_DEFAULT_CHOICES,
        ),
        param_field(
            lambda ctx: f"RELAY{selected_relay(ctx)}_INVERTED",
            "Inverted (ON = low level)",
            key="relay_inverted",
            kind="bool",
        ),
        value_field(
            "relay_rc_channel",
            "Switched by RC channel",
            kind="choice",
            choices=RC_SWITCH_CHOICES,
            default=relay_switch_channel,
        ),
    ],
    [],
    on_apply=apply_relay,
    applies=has("has_relays"),
)
