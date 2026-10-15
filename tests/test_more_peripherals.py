"""Sprayer, optical flow, visual odometry, beacons, EKF position sources, OSD, Remote ID, RPM sensors, generator."""

import pytest

from rover_wizard.util.errors import VehicleError
from rover_wizard.flow import StepFlow
from rover_wizard.ui.step_catalog import STEPS

from util.helpers import BY_ID, act, apply, enter, make

INDEX = {step.id: position for position, step in enumerate(STEPS)}
NEW_STEPS = ("sprayer", "optical_flow", "visual_odometry", "beacon", "osd", "remote_id", "rpm", "generator")


@pytest.fixture
def wizard():
    """A front-wheel simulated vehicle (standard firmware: no Remote ID parameters) and its wizard context."""
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


def add_remote_id_parameters(vehicle):
    """Turn the simulator into a firmware built with OpenDroneID."""
    for name, value in (("DID_ENABLE", 0), ("DID_MAVPORT", -1), ("DID_CANDRIVER", 0), ("DID_OPTIONS", 0)):
        vehicle._store_param(name, float(value), 2, 0, 0)


def test_steps_follow_the_hardware(wizard):
    vehicle, ctx = wizard
    flow = StepFlow(STEPS, ctx)
    assert not any(flow.does_apply(INDEX[step]) for step in NEW_STEPS)
    # the EKF step is always shown; its sensor settings only for the sensors the vehicle has
    vehicle.set_param("WENC_TYPE", 0)
    vehicle.set_param("WENC2_TYPE", 0)
    assert flow.does_apply(INDEX["ekf_sources"])
    enter(ctx, "ekf_sources")
    assert not {"ekf_sensors", "EK3_WENC_VERR", "EK3_FLOW_DELAY"} & visible(ctx, "ekf_sources")
    vehicle.set_param("FLOW_TYPE", 5)
    assert flow.does_apply(INDEX["optical_flow"])
    assert {"ekf_sensors", "EK3_FLOW_DELAY"} <= visible(ctx, "ekf_sources")
    assert "EK3_WENC_VERR" not in visible(ctx, "ekf_sources")


def test_sprayer_outputs_rates_and_switch(wizard):
    vehicle, ctx = wizard
    enter(ctx, "sprayer")
    assert visible(ctx, "sprayer") == {"SPRAY_ENABLE"}, "everything else waits for 'enabled'"
    ctx.set_field_value("SPRAY_ENABLE", 1)
    assert {"sprayer_pump_output", "SPRAY_PUMP_RATE"} <= visible(ctx, "sprayer")
    ctx.values.update(sprayer_pump_output=5, sprayer_spinner_output=6, SPRAY_PUMP_RATE=20)
    apply(ctx, "sprayer")
    assert (vehicle.get("SERVO5_FUNCTION"), vehicle.get("SERVO6_FUNCTION")) == (22, 23)
    assert vehicle.get("SPRAY_PUMP_RATE") == 20
    enter(ctx, "sprayer")
    ctx.values.update(sprayer_pump_output=9)
    apply(ctx, "sprayer")
    assert (vehicle.get("SERVO5_FUNCTION"), vehicle.get("SERVO9_FUNCTION")) == (0, 22), "the pump moved, not duplicated"
    # the steering servo
    ctx.values.update(sprayer_spinner_output=1)
    with pytest.raises(VehicleError, match="steering servo"):
        BY_ID["sprayer"].on_apply(ctx)


def test_optical_flow_address_only_for_px4flow(wizard):
    vehicle, ctx = wizard
    enter(ctx, "optical_flow")
    ctx.set_field_value("FLOW_TYPE", 5)
    assert "FLOW_ADDR" not in visible(ctx, "optical_flow")
    ctx.set_field_value("FLOW_TYPE", 1)
    assert "FLOW_ADDR" in visible(ctx, "optical_flow")
    ctx.values.update(FLOW_ORIENT_YAW=9000, FLOW_POS_Z=0.1)
    apply(ctx, "optical_flow")
    assert (vehicle.get("FLOW_TYPE"), vehicle.get("FLOW_ORIENT_YAW"), vehicle.get("FLOW_POS_Z")) == (1, 9000, 0.1)


def test_beacons_serial_port(wizard):
    vehicle, ctx = wizard
    enter(ctx, "beacon")
    ctx.values.update(BCN_TYPE=2, beacon_port=5, BCN_LATITUDE=48.1)
    apply(ctx, "beacon")
    assert (vehicle.get("BCN_TYPE"), vehicle.get("SERIAL5_PROTOCOL"), vehicle.get("SERIAL5_BAUD")) == (2, 13, 115)
    enter(ctx, "beacon")
    # the GPS port
    ctx.values.update(beacon_port=3)
    with pytest.raises(VehicleError, match="protocol 5"):
        BY_ID["beacon"].on_apply(ctx)


def test_position_sources(wizard):
    vehicle, ctx = wizard
    enter(ctx, "ekf_sources")
    # set 1 without GPS: velocity from the wheel encoders
    ctx.values.update(src_velxy=7, src_posxy=0)
    apply(ctx, "ekf_sources")
    assert (vehicle.get("EK3_SRC1_VELXY"), vehicle.get("EK3_SRC1_POSXY")) == (7, 0)


def test_osd(wizard):
    vehicle, ctx = wizard
    enter(ctx, "osd")
    ctx.values.update(OSD_TYPE=5, OSD_CHAN=7, OSD_UNITS=2)
    apply(ctx, "osd")
    assert (vehicle.get("OSD_TYPE"), vehicle.get("OSD_CHAN"), vehicle.get("OSD_UNITS")) == (5, 7, 2)


def test_remote_id_hidden_on_standard_firmware(wizard):
    vehicle, ctx = wizard
    enter(ctx, "remote_id")
    assert visible(ctx, "remote_id") == set(), "no DID_ parameters: nothing to show but the 'not available' notice"


def test_remote_id_on_a_remote_id_build(wizard):
    vehicle, ctx = wizard
    add_remote_id_parameters(vehicle)
    enter(ctx, "remote_id")
    ctx.values.update(DID_ENABLE=1, DID_MAVPORT=5, did_option_2=True)
    apply(ctx, "remote_id")
    assert (vehicle.get("DID_ENABLE"), vehicle.get("DID_MAVPORT"), vehicle.get("DID_OPTIONS")) == (1, 5, 2)
    assert vehicle.get("SERIAL5_PROTOCOL") == 2, "the module's unused port is switched to MAVLink2"
    enter(ctx, "remote_id")
    # the GPS port
    ctx.values.update(DID_MAVPORT=3)
    with pytest.raises(VehicleError, match="protocol 5"):
        apply(ctx, "remote_id")


def test_rpm_sensor_gpio_pin_and_live_reading(wizard):
    vehicle, ctx = wizard
    messages = []
    ctx.set_output(log=messages.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    enter(ctx, "rpm")
    select(ctx, "rpm", "rpm_sensor", 2)
    # GPIO
    ctx.set_field_value("rpm_type", 2)
    shown = visible(ctx, "rpm")
    assert {"rpm_pin", "rpm_servo_output", "rpm_scaling"} <= shown and "rpm_esc_mask" not in shown
    ctx.values.update(rpm_pin=54, rpm_servo_output=13, rpm_scaling=0.5)
    apply(ctx, "rpm")
    assert (vehicle.get("RPM2_TYPE"), vehicle.get("RPM2_PIN"), vehicle.get("RPM2_SCALING")) == (2, 54, 0.5)
    assert vehicle.get("SERVO13_FUNCTION") == -1 and vehicle.get("RPM1_TYPE") == 0
    # ESC telemetry
    ctx.set_field_value("rpm_type", 5)
    assert "rpm_esc_mask" in visible(ctx, "rpm") and "rpm_pin" not in visible(ctx, "rpm")
    assert act(ctx, "rpm", "Show RPM")
    assert any("RPM 1: -1   RPM 2: 1200" in line for line in messages), messages[-3:]


def test_generator_port_and_options(wizard):
    vehicle, ctx = wizard
    enter(ctx, "generator")
    ctx.values.update(GEN_TYPE=3, generator_port=5, generator_baud=19, gen_option_1=True)
    apply(ctx, "generator")
    assert (vehicle.get("GEN_TYPE"), vehicle.get("SERIAL5_PROTOCOL"), vehicle.get("SERIAL5_BAUD")) == (3, 30, 19)
    assert vehicle.get("GEN_OPTIONS") == 1
    enter(ctx, "generator")
    # the telemetry radio
    ctx.values.update(generator_port=1)
    with pytest.raises(VehicleError, match="protocol 2"):
        BY_ID["generator"].on_apply(ctx)


def test_second_source_set_switch_and_options(wizard):
    vehicle, ctx = wizard
    enter(ctx, "ekf_sources")
    select(ctx, "ekf_sources", "ekf_source_set", 2)
    assert ctx.values["src_posxy"] == 0, "set 2's own (empty) values are shown"
    ctx.values.update(src_posxy=4, src_posz=1, src_yaw=1, ekf_source_rc_channel=6, src_fuse_all_velocities=False)
    apply(ctx, "ekf_sources")
    assert (vehicle.get("EK3_SRC2_POSXY"), vehicle.get("EK3_SRC2_POSZ"), vehicle.get("EK3_SRC1_POSXY")) == (4, 1, 3)
    assert vehicle.get("RC6_OPTION") == 90 and vehicle.get("EK3_SRC_OPTIONS") == 0
    assert act(ctx, "ekf_sources", "Use this source set now") and vehicle.ekf_source_set == 2
