"""Safeguards against setup mistakes other ground stations' users ran into: values left unapplied, a wrong RC
mapping, switch functions on control channels, contradictory battery thresholds, declination without GPS, a stale
level calibration, and choosing the compasses to use."""

import math

import pytest

from rover_wizard.util.errors import VehicleError
from rover_wizard.mavlink_defs import MAV_SEVERITY_INFO
from rover_wizard.steps.compass import describe_compasses, orientation_name
from rover_wizard.context import _severity_colour
from rover_wizard.session.store import STEP_DONE

from util.helpers import BY_ID, act, apply, enter, make

# MAV_SEVERITY: CRITICAL (ArduPilot's PreArm messages), WARNING
SEVERITY_CRITICAL = 2
SEVERITY_WARNING = 4
CENTRE_PWM = 1490


@pytest.fixture
def wizard():
    """A simulated vehicle; the log goes into a list."""
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    yield vehicle, ctx, log
    vehicle.disconnect()


# ---- values not applied ----------------------------------------------------------------------------------------------


def test_edited_and_proposed_values_count_as_unapplied_until_apply(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "battery")
    assert ctx.unapplied_fields(BY_ID["battery"]) == []
    # typed by the pilot (as text), and proposed by a measurement
    ctx.set_field_value("BATT_CAPACITY", "6000")
    ctx.set_values(BATT_VOLT_MULT=10.5)
    assert ctx.unapplied_fields(BY_ID["battery"]) == ["Voltage multiplier", "Battery capacity"]
    apply(ctx, "battery")
    assert ctx.unapplied_fields(BY_ID["battery"]) == []


def test_a_changed_checkbox_counts_as_unapplied(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "failsafe")
    key = next(
        field.key
        for field in BY_ID["failsafe"].fields
        if field.bit is not None and ctx.is_visible(field) and field.label == "Battery level"
    )
    ctx.set_field_value(key, not ctx.value(key))
    assert ctx.unapplied_fields(BY_ID["failsafe"]) == ["Battery level"]


# ---- radio calibration -----------------------------------------------------------------------------------------------


def scripted_pilot(vehicle, throttle_channel, steering_channel):
    """Moves the sticks as asked; full throttle / right steering appear on the given (real) channels."""

    def pause(message, during=None):
        """Put the receiver's channels where the prompt asks."""
        raw = [CENTRE_PWM] * 16
        if "Move EVERY" in message:
            for pwm in (1000, 2000):
                vehicle.raw_rc_override = [pwm] * 16
                during()
        elif "THROTTLE" in message:
            raw[throttle_channel - 1] = 2000
        elif "STEERING" in message:
            raw[steering_channel - 1] = 2000
        vehicle.raw_rc_override = raw

    return pause


def test_a_channel_that_does_not_move_with_its_stick_gets_no_direction(wizard):
    vehicle, ctx, log = wizard
    vehicle._params.update(RC3_REVERSED=1)
    # the transmitter's throttle is on channel 2, the wizard expects 3
    ctx.pause = scripted_pilot(vehicle, throttle_channel=2, steering_channel=1)
    enter(ctx, "rc")
    assert act(ctx, "rc", "Calibrate radio")
    assert vehicle.get("RC3_REVERSED") == 1, "left as it was"
    assert any("RC3 didn't move with the THROTTLE stick" in line and "RC2 moved instead" in line for line in log)
    assert vehicle.get("RC1_REVERSED") == 0


def test_no_receiver_is_reported_before_any_question(wizard, monkeypatch):
    vehicle, ctx, log = wizard
    monkeypatch.setattr(vehicle, "rc_raw", lambda: None)
    ctx.pause = lambda message, during=None: pytest.fail("no question without a receiver")
    enter(ctx, "rc")
    with pytest.raises(VehicleError, match="No RC_CHANNELS telemetry"):
        BY_ID["rc"].actions[0].fn(ctx)


def test_sticks_that_never_move_are_reported(wizard):
    vehicle, ctx, log = wizard

    def pause(message, during=None):
        """A pilot who doesn't touch the transmitter."""
        vehicle.raw_rc_override = [CENTRE_PWM] * 16
        if during:
            during()

    ctx.pause = pause
    enter(ctx, "rc")
    assert act(ctx, "rc", "Calibrate radio")
    assert any("didn't move with the THROTTLE stick" in line and "No channel moved" in line for line in log)
    assert any("didn't move with the STEERING stick" in line for line in log)


def test_mode_switch_on_a_control_channel_is_refused(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "rc")
    ctx.set_field_value("MODE_CH", "3")
    with pytest.raises(VehicleError, match="share channel RC3"):
        apply(ctx, "rc")
    assert vehicle.get("MODE_CH") != 3


def test_switch_functions_on_control_channels_are_removed(wizard):
    vehicle, ctx, log = wizard
    mode_channel = vehicle.get_int("MODE_CH")
    vehicle._params.update(RC1_OPTION=4, **{f"RC{mode_channel}_OPTION": 54})
    enter(ctx, "rc")
    apply(ctx, "rc")
    assert vehicle.get("RC1_OPTION") == 0 and vehicle.get(f"RC{mode_channel}_OPTION") == 0
    assert any("RC1 is the steering channel" in line and "'RTL'" in line for line in log)


# ---- battery ---------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("fields", [{"BATT_CRT_VOLT": "10.5"}, {"BATT_LOW_MAH": "500", "BATT_CRT_MAH": "800"}])
def test_critical_must_be_below_low(wizard, fields):
    vehicle, ctx, log = wizard
    enter(ctx, "battery")
    for key, value in fields.items():
        ctx.set_field_value(key, value)
    with pytest.raises(VehicleError, match="must be below"):
        apply(ctx, "battery")


# ---- compass ---------------------------------------------------------------------------------------------------------


def visible_keys(ctx, step_id):
    """Keys of the step's fields shown now."""
    return {field.key for field in BY_ID[step_id].fields if ctx.is_visible(field)}


def test_declination_is_asked_for_without_gps_and_written_in_radians(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "compass")
    assert "COMPASS_DEC" not in visible_keys(ctx, "compass")
    ctx.store.set("has_gps", False)
    enter(ctx, "compass")
    assert "COMPASS_DEC" in visible_keys(ctx, "compass")
    ctx.set_field_value("COMPASS_DEC", "4.2")
    apply(ctx, "compass")
    assert vehicle.get("COMPASS_DEC") == pytest.approx(math.radians(4.2), rel=1e-5)
    enter(ctx, "compass")
    assert ctx.value("COMPASS_DEC") == pytest.approx(4.2)
    assert ctx.unapplied_fields(BY_ID["compass"]) == []


def test_second_compass_is_switched_through_its_priority(wizard):
    vehicle, ctx, log = wizard
    # an internal compass detected first (slot 1) is used second; the external one (slot 2) is used first
    vehicle._params.update(
        COMPASS_DEV_ID=111,
        COMPASS_EXTERNAL=0,
        COMPASS_DEV_ID2=222,
        COMPASS_EXTERN2=1,
        COMPASS_PRIO1_ID=222,
        COMPASS_PRIO2_ID=111,
    )
    enter(ctx, "compass")
    assert describe_compasses(ctx) == (
        "Compass 1: internal, priority 2\n" "Compass 2: external, priority 1 - used first"
    )
    assert orientation_name(ctx) == "COMPASS_ORIENT2"
    ctx.set_field_value("compass_use1", 0)
    apply(ctx, "compass")
    assert vehicle.get("COMPASS_USE2") == 0 and vehicle.get("COMPASS_USE") == 1


def test_one_compass_must_stay_in_use(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "compass")
    ctx.set_field_value("compass_use1", 0)
    with pytest.raises(VehicleError, match="At least one compass"):
        apply(ctx, "compass")


def test_orientation_field_hidden_for_an_internal_compass(wizard):
    vehicle, ctx, log = wizard
    vehicle._params.update(COMPASS_EXTERNAL=0)
    enter(ctx, "compass")
    assert "COMPASS_ORIENT" not in visible_keys(ctx, "compass")


# ---- board orientation after the level calibration -------------------------------------------------------------------


def test_new_board_orientation_asks_for_a_new_level_calibration(wizard):
    vehicle, ctx, log = wizard
    ctx.pause = lambda message, during=None: None
    assert act(ctx, "accel", "Level calibration")
    ctx.store.mark_step("accel", STEP_DONE)
    enter(ctx, "orientation")
    apply(ctx, "orientation")
    assert ctx.store.step_status("accel") == STEP_DONE, "unchanged orientation: nothing to redo"
    ctx.set_field_value("AHRS_ORIENTATION", 2)
    apply(ctx, "orientation")
    assert ctx.store.step_status("accel") is None
    assert any("redo the 'Level calibration'" in line for line in log)


# ---- flight-controller messages --------------------------------------------------------------------------------------


def test_flight_controller_messages_are_coloured_by_severity(wizard):
    vehicle, ctx, log = wizard
    assert [_severity_colour(level) for level in (SEVERITY_CRITICAL, SEVERITY_WARNING, MAV_SEVERITY_INFO)] == [
        "red",
        "yellow",
        "white",
    ]
    vehicle._add_status_text("PreArm: Compass not calibrated", SEVERITY_CRITICAL)
    ctx.flush_texts()
    assert log[-1] == "FC: [italic red]PreArm: Compass not calibrated[/]"
