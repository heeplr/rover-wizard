"""Extra devices on servo outputs (PWM lights, additional servos) and relays."""

import pytest

from rover_wizard.util.errors import VehicleError
from rover_wizard.flow import StepFlow
from rover_wizard.ui.step_catalog import STEPS

from util.helpers import BY_ID, act, apply, enter, make, wait_until

INDEX = {step.id: position for position, step in enumerate(STEPS)}


@pytest.fixture
def wizard():
    """A front-wheel simulated vehicle (SERVO1 steering, SERVO3 throttle) and its wizard context."""
    vehicle, ctx = make()
    yield vehicle, ctx
    vehicle.disconnect()


def select(ctx, step_id, key, value):
    """Pick the output/relay to edit and reload the step, as the UI does for a reload_on_change field."""
    ctx.set_field_value(key, value)
    ctx.enter_step(BY_ID[step_id], reload=True)


def visible(ctx, step_id):
    """Keys of the fields currently shown on a step."""
    return {field.key for field in BY_ID[step_id].fields if ctx.is_visible(field)}


def test_steps_follow_the_detected_hardware(wizard):
    vehicle, ctx = wizard
    flow = StepFlow(STEPS, ctx)
    assert not flow.does_apply(INDEX["servo_outputs"]) and not flow.does_apply(INDEX["relays"])
    # RCIN7 -> an extra device is configured
    vehicle.set_param("SERVO5_FUNCTION", 57)
    # relay 1 switch
    vehicle.set_param("RC7_OPTION", 28)
    assert flow.does_apply(INDEX["servo_outputs"]) and flow.does_apply(INDEX["relays"])


def test_output_follows_an_rc_channel(wizard):
    vehicle, ctx = wizard
    enter(ctx, "servo_outputs")
    assert ctx.values["aux_output"] == 2, "first free output is proposed"
    select(ctx, "servo_outputs", "aux_output", 5)
    # follows RC channel 7
    ctx.set_field_value("aux_function", 57)
    apply(ctx, "servo_outputs")
    assert vehicle.get("SERVO5_FUNCTION") == 57
    vehicle.raw_rc_override = [1500] * 6 + [1800] + [1500] * 5
    assert wait_until(lambda: vehicle.servo_raw()[4] == 1800), "output 5 passes RC7 through"
    assert "SERVO5: follows RC channel 7" in ctx.values["aux_overview"]


def test_switching_output_reloads_its_settings(wizard):
    vehicle, ctx = wizard
    vehicle.set_param("SERVO6_MIN", 1000)
    enter(ctx, "servo_outputs")
    select(ctx, "servo_outputs", "aux_output", 5)
    # an unsaved edit for output 5 ...
    ctx.set_field_value("aux_min", 1234)
    select(ctx, "servo_outputs", "aux_output", 6)
    assert ctx.values["aux_min"] == 1000, "... must not be shown as output 6's value"


def test_outputs_in_use_are_locked(wizard):
    vehicle, ctx = wizard
    enter(ctx, "servo_outputs")
    # the throttle output
    select(ctx, "servo_outputs", "aux_output", 3)
    shown = visible(ctx, "servo_outputs")
    assert "aux_locked" in shown and "aux_function" not in shown and "aux_min" not in shown
    assert "throttle" in ctx.values["aux_locked"]


def test_gpio_output_hides_pwm_settings(wizard):
    vehicle, ctx = wizard
    enter(ctx, "servo_outputs")
    select(ctx, "servo_outputs", "aux_output", 5)
    ctx.set_field_value("aux_function", -1)
    assert not {"aux_min", "aux_max", "aux_trim", "aux_reversed"} & visible(ctx, "servo_outputs")


def test_test_buttons_move_an_rc_driven_output(wizard):
    vehicle, ctx = wizard
    vehicle.set_param("SERVO5_FUNCTION", 57)
    enter(ctx, "servo_outputs")
    select(ctx, "servo_outputs", "aux_output", 5)
    assert act(ctx, "servo_outputs", "+")
    assert wait_until(lambda: vehicle.servo_raw()[4] == 1525)


def test_relay_setup_pin_output_and_switch(wizard):
    vehicle, ctx = wizard
    enter(ctx, "relays")
    ctx.values.update(relay_function=1, relay_pin=50, relay_servo_output=9, relay_rc_channel=7)
    apply(ctx, "relays")
    assert (vehicle.get("RELAY1_FUNCTION"), vehicle.get("RELAY1_PIN")) == (1, 50)
    assert vehicle.get("SERVO9_FUNCTION") == -1, "the servo output became the relay's GPIO pin"
    assert vehicle.get("RC7_OPTION") == 28
    enter(ctx, "relays")
    ctx.values.update(relay_rc_channel=9)
    apply(ctx, "relays")
    assert (vehicle.get("RC7_OPTION"), vehicle.get("RC9_OPTION")) == (0, 28), "the switch moved, not duplicated"
    assert act(ctx, "relays", "On") and vehicle.relay_states[1] is True
    assert act(ctx, "relays", "Off") and vehicle.relay_states[1] is False


def test_relay_numbers_map_to_their_rc_options(wizard):
    vehicle, ctx = wizard
    enter(ctx, "relays")
    select(ctx, "relays", "relay", 5)
    ctx.values.update(relay_function=1, relay_pin=54, relay_rc_channel=10)
    apply(ctx, "relays")
    assert vehicle.get("RELAY5_PIN") == 54 and vehicle.get("RC10_OPTION") == 66


@pytest.mark.parametrize(
    "field, value, message",
    [
        # RC1 steers
        ("relay_rc_channel", 1, "steering, throttle or the mode switch"),
        # MODE_CH = 8
        ("relay_rc_channel", 8, "steering, throttle or the mode switch"),
        # SERVO1 steers
        ("relay_servo_output", 1, "steering servo"),
    ],
)
def test_relay_refuses_channels_and_outputs_in_use(wizard, field, value, message):
    vehicle, ctx = wizard
    enter(ctx, "relays")
    ctx.values.update(relay_function=1, relay_pin=50, **{field: value})
    with pytest.raises(VehicleError, match=message):
        BY_ID["relays"].on_apply(ctx)


def test_relay_refuses_a_channel_with_another_option(wizard):
    vehicle, ctx = wizard
    # already an arm/disarm switch
    vehicle.set_param("RC10_OPTION", 41)
    enter(ctx, "relays")
    ctx.values.update(relay_rc_channel=10)
    with pytest.raises(VehicleError, match="RC10_OPTION = 41"):
        BY_ID["relays"].on_apply(ctx)


def test_unconfigured_relay_test_is_refused(wizard):
    vehicle, ctx = wizard
    enter(ctx, "relays")
    select(ctx, "relays", "relay", 2)
    assert not act(ctx, "relays", "On")
