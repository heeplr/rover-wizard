"""Arming checks, RC switch functions, GPS and gimbal instances, compass-orientation probe, serial ports, logging,
scripting, gripper and temperature sensors."""

import pytest

from rover_wizard.util.errors import VehicleError
from rover_wizard.flow import StepFlow
from rover_wizard.util.rotation import ORIENTATION_CHOICES
from rover_wizard.vehicle.simulator import SimVehicle
from rover_wizard.ui.step_catalog import STEPS
from rover_wizard.steps.gimbal import action_probe_gimbal
from rover_wizard.steps.compass import action_probe_compass_orientation
from rover_wizard.steps.gps import action_probe_gps_type
from rover_wizard.steps.failsafe import ARMING_CHECK_BITS

from util.helpers import BY_ID, act, apply, enter, make

INDEX = {step.id: position for position, step in enumerate(STEPS)}
ALL_LISTED_CHECKS = sum(bit for bit, _ in ARMING_CHECK_BITS)


@pytest.fixture
def wizard():
    """A front-wheel simulated vehicle and its wizard context."""
    vehicle, ctx = make()
    yield vehicle, ctx
    vehicle.disconnect()


def select(ctx, step_id, key, value):
    """Pick the item to edit and reload the step, as the UI does for a reload_on_change field."""
    ctx.set_field_value(key, value)
    ctx.enter_step(BY_ID[step_id], reload=True)


def visible(ctx, step_id):
    """Keys of the fields currently shown on a step."""
    return {field.key for field in BY_ID[step_id].fields if ctx.is_visible(field)}


# ---------------------------------------------------------------------------------------------- 1. arming checks
def test_arming_check_all_expands_to_individual_checks(wizard):
    # the simulator has ARMING_CHECK = 1 (all)
    vehicle, ctx = wizard
    enter(ctx, "failsafe")
    shown = visible(ctx, "failsafe")
    assert "arm_check_4" in shown and "arm_skip_4" not in shown, "only the firmware's parameter is shown"
    assert all(ctx.values[f"arm_check_{bit}"] for bit, _ in ARMING_CHECK_BITS), "'All' ticks every check"
    # compass check off
    ctx.set_field_value("arm_check_4", False)
    apply(ctx, "failsafe")
    assert vehicle.get("ARMING_CHECK") == ALL_LISTED_CHECKS - 4
    enter(ctx, "failsafe")
    ctx.set_field_value("arm_check_4", True)
    apply(ctx, "failsafe")
    assert vehicle.get("ARMING_CHECK") == 1, "everything ticked is written as 'All'"


def test_arming_skipchk_on_newer_firmware_keeps_unknown_bits(wizard):
    vehicle, ctx = wizard
    vehicle._params.pop("ARMING_CHECK")
    # skips GPS lock + an unknown bit
    vehicle._store_param("ARMING_SKIPCHK", float(8 | (1 << 22)), 6, 0, 0)
    enter(ctx, "failsafe")
    assert ctx.values["arm_skip_8"] is False and ctx.values["arm_skip_4"] is True
    # also skip the compass check
    ctx.set_field_value("arm_skip_4", False)
    apply(ctx, "failsafe")
    assert vehicle.get("ARMING_SKIPCHK") == 8 | 4 | (1 << 22)


# ---------------------------------------------------------------------------------------------- 2. RC switches
def test_rc_switch_functions(wizard):
    vehicle, ctx = wizard
    enter(ctx, "rc")
    shown = visible(ctx, "rc")
    assert not {"RC1_OPTION", "RC3_OPTION", "RC8_OPTION"} & shown, "steering, throttle and mode channels excluded"
    assert {"RC2_OPTION", "RC7_OPTION", "RC16_OPTION"} <= shown
    # RTL
    ctx.set_field_value("RC7_OPTION", 4)
    apply(ctx, "rc")
    assert vehicle.get("RC7_OPTION") == 4


# ---------------------------------------------------------------------------------------------- 3. gimbals
def test_second_gimbal(wizard):
    vehicle, ctx = wizard
    # a Siyi as gimbal 2
    vehicle.true_gimbal2_type = 8
    ctx.confirm = lambda message: (vehicle.gimbal_pitches.get(2, 0) != 0) if "did the gimbal tilt" in message else True
    enter(ctx, "gimbal")
    select(ctx, "gimbal", "gimbal_count", 2)
    select(ctx, "gimbal", "gimbal", 2)
    ctx.set_field_value("gimbal_port", 6)
    action_probe_gimbal(ctx)
    assert vehicle.get("MNT2_TYPE") == 8 and vehicle.get("MNT1_TYPE") == 0, "the probe configured gimbal 2"
    enter(ctx, "gimbal")
    # the fields now describe gimbal 1 again
    select(ctx, "gimbal", "gimbal_count", 1)
    assert ctx.values["gimbal_type"] == 0
    apply(ctx, "gimbal")
    assert vehicle.get("MNT2_TYPE") == 0, "one gimbal: the second is switched off"


# ---------------------------------------------------------------------------------------------- 6. compass orientation
def heading_following_pause(ctx, vehicle, cancel_at=None):
    """Stand-in pilot: turns the simulated vehicle to the heading each prompt asks for."""

    def pause(message, during=None):
        """Face north or south as instructed (or press Cancel)."""
        ctx.log(">> " + message)
        if cancel_at and cancel_at in message:
            from rover_wizard.util.errors import Cancelled

            raise Cancelled()
        if "NORTH" in message:
            vehicle.held_heading_deg = 0.0
        elif "SOUTH" in message:
            vehicle.held_heading_deg = 180.0

    return pause


@pytest.mark.parametrize(
    "mounting, southern",
    [("Yaw90", False), ("None", False), ("Roll180", False), ("Yaw225", False), ("Yaw45Roll90", True)],
)
def test_compass_orientation_probe(monkeypatch, mounting, southern):
    monkeypatch.setattr(SimVehicle, "TRUE_COMPASS_ORIENTATION", mounting)
    # southern hemisphere: the field points UP
    if southern:
        monkeypatch.setattr(SimVehicle, "LATITUDE_DEG", -33.9)
        monkeypatch.setattr(SimVehicle, "EARTH_FIELD_DOWN", -300.0)
    vehicle, ctx = make()
    ctx.confirm = lambda message: True
    ctx.pause = heading_following_pause(ctx, vehicle)
    enter(ctx, "compass")
    action_probe_compass_orientation(ctx)
    assert vehicle.get("COMPASS_ORIENT") == dict(ORIENTATION_CHOICES)[mounting]
    vehicle.disconnect()


def test_compass_probe_restores_on_cancel_and_refuses_internal_compass(wizard):
    vehicle, ctx = wizard
    vehicle.set_param("COMPASS_ORIENT", 4)
    ctx.confirm = lambda message: True
    ctx.pause = heading_following_pause(ctx, vehicle, cancel_at="SOUTH")
    enter(ctx, "compass")
    assert not act(ctx, "compass", "Probe")
    assert vehicle.get("COMPASS_ORIENT") == 4, "cancelled probe restores the original orientation"
    vehicle.set_param("COMPASS_EXTERNAL", 0)
    with pytest.raises(VehicleError, match="internal"):
        action_probe_compass_orientation(ctx)


# ---------------------------------------------------------------------------------------------- 7/8. GPS receivers
def test_second_gps_port_type_and_probe(wizard):
    # SERIAL3 = GPS 1 in the simulator
    vehicle, ctx = wizard
    # an NMEA receiver as GPS 2
    vehicle.true_gps2_type = 5
    ctx.confirm = lambda message: True
    enter(ctx, "gps")
    select(ctx, "gps", "gps_count", 2)
    select(ctx, "gps", "gps", 2)
    assert "GPS_AUTO_SWITCH" in visible(ctx, "gps")
    assert ctx.values["gps_port"] == 4, "the second GPS-protocol port (SERIAL4) belongs to GPS 2"
    ctx.values.update(gps_port=4, gps_baud=115)
    apply(ctx, "gps")
    assert (vehicle.get("SERIAL4_PROTOCOL"), vehicle.get("SERIAL4_BAUD")) == (5, 115)
    action_probe_gps_type(ctx)
    assert vehicle.get("GPS2_TYPE") == 5 and vehicle.get("GPS1_TYPE") == 1, "the probe configured GPS 2 only"


# SERIAL1 = telemetry
@pytest.mark.parametrize(
    "instance, port, message",
    [
        (2, 1, "protocol 2"),
        # below GPS 1's SERIAL3
        (2, 2, "higher-numbered"),
        (1, 9, "no SERIAL9"),
    ],
)
def test_gps_port_refusals(wizard, instance, port, message):
    vehicle, ctx = wizard
    enter(ctx, "gps")
    select(ctx, "gps", "gps_count", 2)
    select(ctx, "gps", "gps", instance)
    ctx.values.update(gps_port=port)
    vehicle.set_param("SERIAL2_PROTOCOL", -1)
    with pytest.raises(VehicleError, match=message):
        BY_ID["gps"].on_apply(ctx)


def test_one_gps_switches_the_second_off(wizard):
    vehicle, ctx = wizard
    vehicle.set_param("GPS2_TYPE", 1)
    enter(ctx, "gps")
    assert ctx.values["gps_count"] == 2, "a configured second receiver is detected"
    select(ctx, "gps", "gps_count", 1)
    apply(ctx, "gps")
    assert vehicle.get("GPS2_TYPE") == 0


# ---------------------------------------------------------------------------------------------- 9. serial ports
def test_serial_port_protocol_and_baud(wizard):
    vehicle, ctx = wizard
    enter(ctx, "serial_ports")
    select(ctx, "serial_ports", "serial_port", 5)
    # RC input on a UART
    ctx.values.update(serial_protocol=23, serial_baud=115)
    apply(ctx, "serial_ports")
    assert (vehicle.get("SERIAL5_PROTOCOL"), vehicle.get("SERIAL5_BAUD")) == (23, 115)
    assert "SERIAL5: RC input @ 115200" in ctx.values["serial_overview"]


# ---------------------------------------------------------------------------------------------- 4/5. logging, scripting
def test_logging_bits(wizard):
    # LOG_BACKEND_TYPE = 1 (file), LOG_BITMASK = 65535
    vehicle, ctx = wizard
    enter(ctx, "log_settings")
    ctx.values.update(log_backend_2=True, log_bit_8192=False)
    apply(ctx, "log_settings")
    assert vehicle.get("LOG_BACKEND_TYPE") == 3 and vehicle.get("LOG_BITMASK") == 65535 - 8192


def test_scripting_enable_then_settings_after_reboot(wizard):
    vehicle, ctx = wizard
    enter(ctx, "scripting")
    assert visible(ctx, "scripting") == {"SCR_ENABLE"}
    ctx.set_field_value("SCR_ENABLE", 1)
    apply(ctx, "scripting")
    assert vehicle.get("SCR_ENABLE") == 1 and "SCR_HEAP_SIZE" not in visible(ctx, "scripting")
    assert act(ctx, "save", "Reboot")
    enter(ctx, "scripting")
    assert {"SCR_HEAP_SIZE", "scr_dir_2", "SCR_USER6"} <= visible(ctx, "scripting")
    # don't run scripts from the SD card
    ctx.set_field_value("scr_dir_2", False)
    apply(ctx, "scripting")
    assert vehicle.get("SCR_DIR_DISABLE") == 2


# ---------------------------------------------------------------------------------------------- 10. peripherals
def test_new_peripheral_steps_follow_the_hardware(wizard):
    vehicle, ctx = wizard
    flow = StepFlow(STEPS, ctx)
    assert not flow.does_apply(INDEX["gripper"]) and not flow.does_apply(INDEX["temperature"])
    vehicle.set_param("GRIP_ENABLE", 1)
    vehicle.set_param("TEMP2_TYPE", 7)
    assert flow.does_apply(INDEX["gripper"]) and flow.does_apply(INDEX["temperature"])


def test_gripper_setup_and_test(wizard):
    vehicle, ctx = wizard
    enter(ctx, "gripper")
    ctx.values.update(GRIP_ENABLE=1, GRIP_TYPE=1, gripper_output=6, gripper_rc_channel=7)
    apply(ctx, "gripper")
    assert (vehicle.get("SERVO6_FUNCTION"), vehicle.get("RC7_OPTION")) == (28, 19)
    assert act(ctx, "gripper", "grab") and vehicle.gripper_state == "grab"
    assert act(ctx, "gripper", "release") and vehicle.gripper_state == "release"
    enter(ctx, "gripper")
    # the steering servo
    ctx.values.update(gripper_output=1)
    with pytest.raises(VehicleError, match="steering servo"):
        BY_ID["gripper"].on_apply(ctx)


def test_temperature_sensor_selector(wizard):
    vehicle, ctx = wizard
    enter(ctx, "temperature")
    select(ctx, "temperature", "temp_sensor", 2)
    ctx.values.update(temp_type=7, temp_addr=90, temp_bus=1)
    apply(ctx, "temperature")
    assert (vehicle.get("TEMP2_TYPE"), vehicle.get("TEMP2_ADDR"), vehicle.get("TEMP2_BUS")) == (7, 90, 1)
    assert vehicle.get("TEMP1_TYPE") == 0


def test_arming_bits_match_ardupilot():
    # ArduPilot's own list of ARMING_SKIPCHK bits for Rover (bit number -> name)
    official = docs_bitmask("ARMING_SKIPCHK")
    assert sorted(bit for bit, _ in ARMING_CHECK_BITS) == sorted(1 << int(number) for number in official)


def test_every_arming_check_has_its_own_help_and_a_plain_label():
    step = BY_ID["failsafe"]
    checks = [field for field in step.fields if field.key.startswith(("arm_skip_", "arm_check_"))]
    assert len(checks) == 2 * len(ARMING_CHECK_BITS)
    # one text per check (the two parameters' checkboxes for the same check share it)
    texts = {field.help for field in checks if field.key.startswith("arm_skip_")}
    assert len(texts) == len(ARMING_CHECK_BITS) and all(texts)
    assert all(
        field.help == step_field(step, field.key.replace("arm_check_", "arm_skip_")).help
        for field in checks
        if field.key.startswith("arm_check_")
    )
    assert not any(field.label.startswith("Arming check") for field in checks)
    heading = step_field(step, "arm_heading")
    assert heading.kind == "heading" and heading.help and step.fields.index(heading) < step.fields.index(checks[0])


def test_arming_heading_only_with_an_arming_parameter(wizard):
    vehicle, ctx = wizard
    heading = step_field(BY_ID["failsafe"], "arm_heading")
    assert ctx.is_visible(heading)
    for name in ("ARMING_CHECK", "ARMING_SKIPCHK"):
        vehicle._params.pop(name, None)
    assert not ctx.is_visible(heading)


def step_field(step, key):
    """A step's field by key."""
    return next(field for field in step.fields if field.key == key)


def docs_bitmask(name):
    """ArduPilot's bit names of a bitmask parameter, from the wizard's documentation file."""
    import json
    from rover_wizard.ui.param_docs import DOCS_FILE

    with open(DOCS_FILE, encoding="utf-8") as handle:
        return json.load(handle)[name]["bitmask"]
