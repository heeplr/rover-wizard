"""The 'Distance sensors & object avoidance' step: rangefinders (10 instances), proximity sensors (5 instances),
the RangeFinder bridge between them, and object avoidance."""

import pytest

from rover_wizard.util.errors import VehicleError
from rover_wizard.steps.distance_sensors import action_probe_rangefinder
from rover_wizard.flow import StepFlow
from rover_wizard.steps.gimbal import action_probe_gimbal
from rover_wizard.ui.step_catalog import STEPS

from util.helpers import BY_ID, apply, enter, make

STEP_ID = "distance_sensors"
INDEX = {step.id: position for position, step in enumerate(STEPS)}


@pytest.fixture
def wizard():
    """A simulated vehicle and its wizard context, with both kinds of sensor ticked on 'Vehicle type'."""
    vehicle, ctx = make()
    ctx.store.set("has_rangefinder", True)
    ctx.store.set("has_proximity", True)
    yield vehicle, ctx
    vehicle.disconnect()


def select(ctx, key, value):
    """Pick the instance to edit and reload the step, as the UI does for a reload_on_change field."""
    ctx.set_field_value(key, value)
    ctx.enter_step(BY_ID[STEP_ID], reload=True)


def visible(ctx):
    """Keys of the fields currently shown."""
    return {field.key for field in BY_ID[STEP_ID].fields if ctx.is_visible(field)}


@pytest.mark.parametrize(
    "rangefinder, proximity, shown", [(False, False, False), (True, False, True), (False, True, True)]
)
def test_step_follows_either_kind_of_sensor(rangefinder, proximity, shown):
    vehicle, ctx = make()
    ctx.store.set("has_rangefinder", rangefinder)
    ctx.store.set("has_proximity", proximity)
    assert StepFlow(STEPS, ctx).does_apply(INDEX[STEP_ID]) is shown
    vehicle.disconnect()


@pytest.mark.parametrize(
    "kind, port_field, address_field",
    [(8, True, False), (25, False, True), (10, False, False)],
    ids=["serial (LightWareSerial)", "I2C (BenewakeTFmini-I2C)", "neither (MAVLink)"],
)
def test_rangefinder_fields_follow_the_driver(wizard, kind, port_field, address_field):
    vehicle, ctx = wizard
    enter(ctx, STEP_ID)
    ctx.set_field_value("rng_type", kind)
    assert ("rng_port" in visible(ctx), "rng_address" in visible(ctx)) == (port_field, address_field)


def test_second_rangefinder_is_its_own_instance(wizard):
    vehicle, ctx = wizard
    vehicle.set_param("RNGFND2_ORIENT", 25)
    enter(ctx, STEP_ID)
    select(ctx, "rangefinder", 2)
    assert ctx.values["rng_orient"] == 25
    ctx.values.update(rng_type=16, rng_address=41)
    apply(ctx, STEP_ID)
    assert (vehicle.get("RNGFND2_TYPE"), vehicle.get("RNGFND2_ADDR"), vehicle.get("RNGFND1_TYPE")) == (16, 41, 0)
    enter(ctx, STEP_ID)
    assert "Rangefinder 2: VL53L0X or VL53L1X, down (e.g. echosounder)" in ctx.values["rng_overview"]


def test_probe_finds_a_sensor_as_the_selected_instance(wizard):
    vehicle, ctx = wizard
    # a VL53L0X wired as rangefinder 2
    vehicle.true_rangefinder = ("2", 16, 0)
    ctx.confirm = lambda message: True
    enter(ctx, STEP_ID)
    select(ctx, "rangefinder", 2)
    action_probe_rangefinder(ctx)
    assert (vehicle.get("RNGFND2_TYPE"), vehicle.get("RNGFND1_TYPE")) == (16, 0)


@pytest.mark.parametrize(
    "field, port, message",
    [("rng", 3, "protocol 5"), ("prx", 1, "protocol 2")],
    ids=["rangefinder on the GPS port", "lidar on the telemetry port"],
)
def test_serial_ports_in_use_are_refused(wizard, field, port, message):
    vehicle, ctx = wizard
    enter(ctx, STEP_ID)
    # LightWareSerial rangefinder / RPLidar proximity sensor
    ctx.values.update(**{f"{field}_type": 8 if field == "rng" else 5, f"{field}_port": port})
    with pytest.raises(VehicleError, match=message):
        BY_ID[STEP_ID].on_apply(ctx)


def test_serial_sensors_get_their_ports(wizard):
    vehicle, ctx = wizard
    enter(ctx, STEP_ID)
    ctx.values.update(rng_type=8, rng_port=5, prx_type=5, prx_port=6, prx_baud=256)
    apply(ctx, STEP_ID)
    assert (vehicle.get("SERIAL5_PROTOCOL"), vehicle.get("SERIAL6_PROTOCOL"), vehicle.get("SERIAL6_BAUD")) == (
        9,
        11,
        256,
    )


def test_rangefinder_bridge_warns_without_a_horizontal_rangefinder(wizard):
    vehicle, ctx = wizard
    messages = []
    ctx.set_output(log=messages.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    enter(ctx, STEP_ID)
    ctx.values.update(prx_type=4)
    apply(ctx, STEP_ID)
    assert vehicle.get("PRX1_TYPE") == 4 and any("pointing horizontally" in line for line in messages)
    messages.clear()
    # an I2C LightWare pointing forward
    vehicle.set_param("RNGFND1_TYPE", 7)
    vehicle.set_param("RNGFND1_ORIENT", 0)
    enter(ctx, STEP_ID)
    apply(ctx, STEP_ID)
    assert not any("pointing horizontally" in line for line in messages), "a forward rangefinder feeds the bridge"


def test_avoidance_sources_are_checkboxes(wizard):
    vehicle, ctx = wizard
    # the simulator has AVOID_ENABLE = 3: fence + proximity sensors
    enter(ctx, STEP_ID)
    assert (ctx.values["avoid_source_1"], ctx.values["avoid_source_2"], ctx.values["avoid_source_4"]) == (
        True,
        True,
        False,
    )
    ctx.values.update(avoid_source_1=False, AVOID_BEHAVE=1)
    apply(ctx, STEP_ID)
    assert (vehicle.get("AVOID_ENABLE"), vehicle.get("AVOID_BEHAVE")) == (2, 1)


def test_gimbal_probe_refuses_the_telemetry_port(wizard):
    vehicle, ctx = wizard
    ctx.confirm = lambda message: True
    enter(ctx, "gimbal")
    # SERIAL1 carries MAVLink2 telemetry in the simulator
    ctx.set_field_value("gimbal_port", 1)
    with pytest.raises(VehicleError, match="telemetry link"):
        action_probe_gimbal(ctx)
    assert vehicle.get("SERIAL1_PROTOCOL") == 2
