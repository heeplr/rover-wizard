"""The 'Vehicle type' checkboxes start from what the vehicle's parameters (and, where there is no parameter, its
telemetry) say is fitted."""

import tempfile
from types import SimpleNamespace

import pytest

from rover_wizard.context import Ctx
from rover_wizard.vehicle.simulator import SimVehicle
from rover_wizard.steps.util.hardware import HARDWARE_BY_KEY
from rover_wizard.session.store import Store
from rover_wizard.vehicle.link import Vehicle

from util.helpers import BY_ID, enter


def quiet_vehicle(**parameters):
    """A simulated vehicle that isn't running (no telemetry unless a test stores some), with parameters set."""
    vehicle = SimVehicle()
    for name, value in parameters.items():
        if vehicle.has_parameter(name):
            vehicle.set_param(name, value)
        else:
            # an instance the simulator doesn't define
            vehicle._store_param(name, float(value), 2, 0, 0)
    return vehicle, Ctx(vehicle, Store(1, tempfile.mkdtemp()))


def detected(key, vehicle_and_ctx):
    """What the checkbox for `key` would start with."""
    return HARDWARE_BY_KEY[key].detect(vehicle_and_ctx[1])


@pytest.mark.parametrize(
    "parameters, fix_type, expected",
    [
        # no GPS configured
        ({"GPS1_TYPE": 0}, None, False),
        # configured, no telemetry yet: trust the configuration
        ({"GPS1_TYPE": 1}, None, True),
        # Auto, but "no GPS connected" (ArduPilot's default type)
        ({"GPS1_TYPE": 1}, 0, False),
        # connected, no fix yet
        ({"GPS1_TYPE": 1}, 1, True),
        # only a second receiver
        ({"GPS1_TYPE": 0, "GPS2_TYPE": 5}, None, True),
    ],
    ids=["none", "configured", "auto-but-absent", "no-fix", "second-only"],
)
def test_gps(parameters, fix_type, expected):
    vehicle_and_ctx = quiet_vehicle(**parameters)
    if fix_type is not None:
        vehicle_and_ctx[0]._store_message("GPS_RAW_INT", SimpleNamespace(fix_type=fix_type))
    assert detected("has_gps", vehicle_and_ctx) is expected


@pytest.mark.parametrize(
    "parameters, expected",
    [
        ({"COMPASS_ENABLE": 1, "COMPASS_DEV_ID": 97539}, True),
        # enabled by default, but nothing detected
        ({"COMPASS_ENABLE": 1, "COMPASS_DEV_ID": 0}, False),
        ({"COMPASS_ENABLE": 1, "COMPASS_DEV_ID": 0, "COMPASS_DEV_ID3": 1234}, True),
        ({"COMPASS_ENABLE": 0, "COMPASS_DEV_ID": 97539}, False),
    ],
    ids=["detected", "enabled-but-none", "third-only", "disabled"],
)
def test_compass(parameters, expected):
    assert detected("has_compass", quiet_vehicle(**parameters)) is expected


@pytest.mark.parametrize(
    "channel_count, expected",
    [(None, True), (0, False), (8, True)],
    ids=["no-telemetry-yet", "no-channels", "receiving"],
)
def test_rc_receiver(channel_count, expected):
    # the real RC_CHANNELS handling (the simulator fakes it)
    vehicle = Vehicle()
    if channel_count is not None:
        channels = {f"chan{n}_raw": 1500 for n in range(1, 19)}
        vehicle._store_message("RC_CHANNELS", SimpleNamespace(chancount=channel_count, **channels))
    assert detected("has_rc", (vehicle, Ctx(vehicle, Store(1, tempfile.mkdtemp())))) is expected


@pytest.mark.parametrize(
    "key, parameters",
    [
        ("has_battery_monitor", {"BATT_MONITOR": 0, "BATT2_MONITOR": 4}),
        ("has_wenc", {"WENC_TYPE": 0, "WENC2_TYPE": 1}),
        ("has_rangefinder", {"RNGFNDA_TYPE": 7}),
        ("has_proximity", {"PRX3_TYPE": 5}),
        ("has_gimbal", {"MNT2_TYPE": 8}),
        ("has_temperature", {"TEMP7_TYPE": 7}),
        ("has_rpm", {"RPM4_TYPE": 2}),
    ],
    ids=lambda value: value if isinstance(value, str) else "",
)
def test_later_instances_count_too(key, parameters):
    assert detected(key, quiet_vehicle(**{"WENC_TYPE": 0, **parameters})) is True


def test_nothing_configured_means_unticked():
    # the simulator has both encoders
    vehicle_and_ctx = quiet_vehicle(WENC_TYPE=0, WENC2_TYPE=0, BATT_MONITOR=0)
    for key in (
        "has_battery_monitor",
        "has_wenc",
        "has_rangefinder",
        "has_proximity",
        "has_gimbal",
        "has_temperature",
        "has_rpm",
        "has_gripper",
        "has_sprayer",
        "has_optical_flow",
        "has_visual_odometry",
        "has_beacons",
        "has_osd",
        "has_remote_id",
        "has_generator",
        "has_aux_servos",
        "has_relays",
    ):
        assert detected(key, vehicle_and_ctx) is False, key


def test_checkboxes_show_the_detection():
    vehicle, ctx = quiet_vehicle(COMPASS_DEV_ID=0, RNGFND3_TYPE=7)
    enter(ctx, "vehicle_type")
    assert ctx.values["has_compass"] is False and ctx.values["has_rangefinder"] is True
    # the pilot's saved answer wins over the detection
    ctx.store.set("has_compass", True)
    enter(ctx, "vehicle_type")
    assert ctx.values["has_compass"] is True


def test_unticking_switches_every_instance_off():
    vehicle, ctx = quiet_vehicle(RNGFND1_TYPE=7, RNGFND3_TYPE=20)
    enter(ctx, "vehicle_type")
    ctx.set_field_value("has_rangefinder", False)
    BY_ID["vehicle_type"].on_apply(ctx)
    assert (vehicle.get("RNGFND1_TYPE"), vehicle.get("RNGFND3_TYPE")) == (0, 0)
