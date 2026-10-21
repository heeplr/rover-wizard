"""The Barometer step: the barometers found (decoded device IDs), which one is used, an external I2C barometer
looked for at boot, and the live pressure readout."""

import pytest

from rover_wizard.rover_defs import BARO_DEVICE_TYPE_BMP280, BARO_PROBE_EXT_BMP280, DEVICE_BUS_TYPE_I2C
from rover_wizard.steps.barometer import decode_device_id, describe_device
from rover_wizard.steps.util.actions import action_reboot_flight_controller
from rover_wizard.ui.monitor import barometer_text
from rover_wizard.vehicle.rover_simulator import INTERNAL_BAROMETER_DEVID, make_device_id

from util.helpers import BY_ID, act, apply, enter, make

# where the simulated external barometer sits: I2C bus 0, the BMP280's usual address
EXTERNAL_BUS = 0
BMP280_ADDRESS = 0x76
EXTERNAL_BAROMETER = (BARO_PROBE_EXT_BMP280, BARO_DEVICE_TYPE_BMP280, EXTERNAL_BUS, BMP280_ADDRESS)


@pytest.fixture
def wizard():
    """A simulated vehicle with its internal barometer; the log goes into a list."""
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    yield vehicle, ctx, log
    vehicle.disconnect()


def visible(ctx):
    """Keys of the barometer step's fields shown now."""
    return {field.key for field in BY_ID["barometer"].fields if ctx.is_visible(field)}


# ---- device IDs (AP_HAL::Device: bus type, bus, address, device type) ----------------------------------------------


@pytest.mark.parametrize(
    "device_id, text",
    [
        (INTERNAL_BAROMETER_DEVID, "MS5611 on SPI bus 1"),
        (
            make_device_id(DEVICE_BUS_TYPE_I2C, 0, BMP280_ADDRESS, BARO_DEVICE_TYPE_BMP280),
            "BMP280 on I2C bus 0, address 0x76",
        ),
        # DroneCAN / MSP (AP_Baro_DroneCAN.cpp, AP_Baro_MSP.cpp): no device type, the address is the node / instance
        (make_device_id(3, 0, 125, 0), "DroneCAN barometer, node 125"),
        (make_device_id(5, 0, 1, 0), "MSP barometer 1"),
        (make_device_id(2, 0, 0, 0x99), "type 153 on SPI bus 0"),
    ],
)
def test_device_ids_in_words(device_id, text):
    assert describe_device(device_id) == text


def test_device_id_fields_round_trip():
    assert decode_device_id(make_device_id(DEVICE_BUS_TYPE_I2C, 5, 0x77, 0x14)) == (DEVICE_BUS_TYPE_I2C, 5, 0x77, 0x14)


# ---- the barometers found ---------------------------------------------------------------------------------------------


def test_internal_barometer_is_listed_and_used(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "barometer")
    assert ctx.values["baro_list"] == "Barometer 1: MS5611 on SPI bus 1 - used"
    assert "BARO_PRIMARY" not in visible(ctx), "nothing to choose with one barometer"
    assert not ctx.values["baro_external"] and "BARO_EXT_BUS" not in visible(ctx)


def test_no_barometer_found(wizard):
    vehicle, ctx, log = wizard
    vehicle.set_param("BARO1_DEVID", 0)
    enter(ctx, "barometer")
    assert ctx.values["baro_list"].startswith("none found")
    assert act(ctx, "barometer", "Show pressure")
    assert log[-1] == "No barometer found - see the help of this step."


def test_external_barometer_is_looked_for_after_a_reboot(wizard):
    vehicle, ctx, log = wizard
    vehicle.true_external_baro = EXTERNAL_BAROMETER
    enter(ctx, "barometer")
    ctx.set_field_value("baro_external", True)
    assert {"baro_probe", "BARO_EXT_BUS", f"baro_probe_{BARO_PROBE_EXT_BMP280}"} <= visible(ctx)
    assert "baro_probe_256" not in visible(ctx), "Keller: only ArduSub looks for it"
    ctx.set_field_value(f"baro_probe_{BARO_PROBE_EXT_BMP280}", True)
    apply(ctx, "barometer")
    assert vehicle.get("BARO_PROBE_EXT") == BARO_PROBE_EXT_BMP280
    assert "BARO_PROBE_EXT" in ctx.changes.reboot_reasons

    action_reboot_flight_controller(ctx)
    enter(ctx, "barometer")
    assert ctx.values["baro_list"].splitlines() == [
        "Barometer 1: MS5611 on SPI bus 1 - used",
        "Barometer 2: BMP280 on I2C bus 0, address 0x76",
    ]
    assert ctx.values["baro_external"], "ticked while BARO_PROBE_EXT looks for one"
    # two barometers: choose the external one
    assert "BARO_PRIMARY" in visible(ctx)
    ctx.set_field_value("BARO_PRIMARY", 1)
    apply(ctx, "barometer")
    enter(ctx, "barometer")
    assert ctx.values["baro_list"].splitlines()[1].endswith("- used")


def test_external_barometer_on_another_bus_is_not_found(wizard):
    vehicle, ctx, log = wizard
    vehicle.true_external_baro = EXTERNAL_BAROMETER
    vehicle.set_param("BARO_PROBE_EXT", BARO_PROBE_EXT_BMP280)
    vehicle.set_param("BARO_EXT_BUS", EXTERNAL_BUS + 1)
    action_reboot_flight_controller(ctx)
    assert vehicle.get("BARO2_DEVID") == 0


def test_unticking_the_external_barometer_stops_looking_for_it(wizard):
    vehicle, ctx, log = wizard
    vehicle.set_param("BARO_PROBE_EXT", BARO_PROBE_EXT_BMP280)
    enter(ctx, "barometer")
    assert ctx.values["baro_external"]
    ctx.set_field_value("baro_external", False)
    apply(ctx, "barometer")
    assert vehicle.get("BARO_PROBE_EXT") == 0 and "BARO_PROBE_EXT" in ctx.changes.reboot_reasons


# ---- live readout -----------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("unhealthy, health", [(False, "(healthy)"), (True, "(NOT healthy)")])
def test_show_pressure_logs_every_barometer_and_its_health(wizard, unhealthy, health):
    vehicle, ctx, log = wizard
    vehicle.barometers_unhealthy = unhealthy
    vehicle.true_external_baro = EXTERNAL_BAROMETER
    vehicle.set_param("BARO_PROBE_EXT", BARO_PROBE_EXT_BMP280)
    action_reboot_flight_controller(ctx)
    enter(ctx, "barometer")
    assert act(ctx, "barometer", "Show pressure")
    assert log[-1] == f"Baro 1: 1013.3 hPa, 24.5 deg C   Baro 2: 1013.4 hPa, 24.5 deg C   {health}"


@pytest.mark.parametrize(
    "reading, healthy, text",
    [
        ((1013.25, 24.46), True, "1013.2 hPa  24.5 C"),
        ((1013.25, 24.46), False, "1013.2 hPa  24.5 C  NOT healthy"),
        (None, None, "no reading"),
    ],
)
def test_side_panel_text(reading, healthy, text):
    assert barometer_text(reading, healthy) == text
