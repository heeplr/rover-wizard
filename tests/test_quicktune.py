"""QuickTune switch in the "Feed-forward & first PID values" step: Lua scripting itself is set on the Scripting step;
Apply gives the switch the script's function and says what is still missing (scripting off, script not running)."""

import pytest

from rover_wizard.util.errors import VehicleError

from util.helpers import BY_ID, apply, enter, make

# a free channel: not steering (1), throttle (3) or the mode channel (8)
SWITCH_CHANNEL = 6
# PARAM_TYPE for the script's RTUN_ parameters in the simulator
FLOAT_PARAM = 9


@pytest.fixture
def wizard():
    """A simulated vehicle; the log goes into a list."""
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    yield vehicle, ctx, log
    vehicle.disconnect()


def apply_switch(ctx, channel):
    """Enter the step, choose the QuickTune channel, Apply."""
    enter(ctx, "gains")
    ctx.set_field_value("qt_ch", str(channel))
    apply(ctx, "gains")


def test_scripting_is_set_on_the_scripting_step_only():
    assert "SCR_ENABLE" not in {field.key for field in BY_ID["gains"].fields}
    assert "SCR_ENABLE" in {field.key for field in BY_ID["scripting"].fields}
    assert BY_ID["gains"].fields[-1].key == "qt_ch" and BY_ID["gains"].fields[-1].help


def test_no_channel_changes_nothing(wizard):
    vehicle, ctx, log = wizard
    apply_switch(ctx, 0)
    assert vehicle.get(f"RC{SWITCH_CHANNEL}_OPTION") == 0 and not any("QuickTune" in line for line in log)


def test_scripting_off_is_pointed_out(wizard):
    vehicle, ctx, log = wizard
    vehicle.set_param("SCR_ENABLE", 0)
    apply_switch(ctx, SWITCH_CHANNEL)
    assert vehicle.get(f"RC{SWITCH_CHANNEL}_OPTION") == 300
    assert any("Lua scripting, which is off" in line and "Scripting step" in line for line in log)


def test_script_not_running_is_pointed_out(wizard):
    vehicle, ctx, log = wizard
    vehicle.set_param("SCR_ENABLE", 1)
    apply_switch(ctx, SWITCH_CHANNEL)
    assert any("script isn't running" in line for line in log)


def test_running_script_uses_its_switch_function(wizard):
    vehicle, ctx, log = wizard
    vehicle.set_param("SCR_ENABLE", 1)
    # the script runs (its parameters exist) and listens to Scripting3
    for name, value in (("RTUN_ENABLE", 1.0), ("RTUN_RC_FUNC", 302.0)):
        vehicle._store_param(name, value, FLOAT_PARAM, 0, 0)
    apply_switch(ctx, SWITCH_CHANNEL)
    assert vehicle.get(f"RC{SWITCH_CHANNEL}_OPTION") == 302
    assert any(line.startswith(f"QuickTune ready on RC{SWITCH_CHANNEL}") for line in log)


@pytest.mark.parametrize("channel", [1, 3, 8], ids=["steering", "throttle", "mode channel"])
def test_a_channel_in_use_is_refused(wizard, channel):
    vehicle, ctx, log = wizard
    with pytest.raises(VehicleError, match="choose another channel"):
        apply_switch(ctx, channel)
    assert vehicle.get(f"RC{channel}_OPTION") == 0


def test_a_channel_with_another_switch_is_refused(wizard):
    # e.g. RC6 already switches relay 1: QuickTune must not silently take it over
    vehicle, ctx, log = wizard
    vehicle.set_param(f"RC{SWITCH_CHANNEL}_OPTION", 28)
    with pytest.raises(VehicleError, match="already has"):
        apply_switch(ctx, SWITCH_CHANNEL)
    assert vehicle.get(f"RC{SWITCH_CHANNEL}_OPTION") == 28


def test_moving_the_switch_frees_the_old_channel(wizard):
    vehicle, ctx, log = wizard
    apply_switch(ctx, SWITCH_CHANNEL)
    apply_switch(ctx, SWITCH_CHANNEL + 1)
    assert (vehicle.get(f"RC{SWITCH_CHANNEL}_OPTION"), vehicle.get(f"RC{SWITCH_CHANNEL + 1}_OPTION")) == (0, 300)
