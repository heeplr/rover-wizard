"""rover_wizard.steps.barometer - the "Barometer" step.

ArduPilot finds its barometers at boot (AP_Baro::init) - the flight controller's own, plus external ones it is told
to look for with BARO_PROBE_EXT - and records each one's device ID in BARO1_DEVID .. BARO3_DEVID. Nothing needs
calibrating: the ground pressure is measured at every boot.
"""

from __future__ import annotations

from typing import Optional

from ..rover_defs import (
    BARO_DEVICE_TYPES,
    BARO_EXT_BUS_ALL,
    BARO_INSTANCES,
    BARO_PROBE_EXT_BITS,
    DEVICE_BUS_TYPE_DRONECAN,
    DEVICE_BUS_TYPE_I2C,
    DEVICE_BUS_TYPE_MSP,
    DEVICE_BUS_TYPES,
    DEVID_BUS_BITS,
    DEVID_BUS_TYPE_BITS,
    DEVID_BYTE_BITS,
)
from .util.probing import LIVE_DISPLAY_SECONDS
from ..model import Action, Step, bit_field, heading_field, param_field, value_field

# the external I2C buses ArduPilot can be told to search (BARO_EXT_BUS; -1 = all of them)
HIGHEST_I2C_BUS = 6


def decode_device_id(device_id: int) -> tuple:
    """(bus type, bus number, address, device type) packed in a device ID (AP_HAL::Device::DeviceStructure)."""
    bus_type = device_id & ((1 << DEVID_BUS_TYPE_BITS) - 1)
    bus = (device_id >> DEVID_BUS_TYPE_BITS) & ((1 << DEVID_BUS_BITS) - 1)
    address = (device_id >> (DEVID_BUS_TYPE_BITS + DEVID_BUS_BITS)) & ((1 << DEVID_BYTE_BITS) - 1)
    device_type = (device_id >> (DEVID_BUS_TYPE_BITS + DEVID_BUS_BITS + DEVID_BYTE_BITS)) & ((1 << DEVID_BYTE_BITS) - 1)
    return bus_type, bus, address, device_type


def describe_device(device_id: int) -> str:
    """A barometer's device ID in words, e.g. "MS5611 on SPI bus 1" or "BMP280 on I2C bus 0, address 0x76"."""
    bus_type, bus, address, device_type = decode_device_id(device_id)
    sensor = BARO_DEVICE_TYPES.get(device_type, f"type {device_type}")
    if bus_type == DEVICE_BUS_TYPE_I2C:
        return f"{sensor} on I2C bus {bus}, address 0x{address:02X}"
    # DroneCAN and MSP barometers have no device type: the address is the node / the MSP instance
    if bus_type == DEVICE_BUS_TYPE_DRONECAN:
        return f"DroneCAN barometer, node {address}"
    if bus_type == DEVICE_BUS_TYPE_MSP:
        return f"MSP barometer {address}"
    if bus_type in DEVICE_BUS_TYPES:
        return f"{sensor} on {DEVICE_BUS_TYPES[bus_type]} bus {bus}"
    return f"{sensor} (device ID {device_id})"


def detected_barometers(ctx) -> list:
    """Numbers (1..3) of the barometers found at the last boot."""
    return [instance for instance in BARO_INSTANCES if ctx.param_int(f"BARO{instance}_DEVID")]


def barometer_in_use(ctx) -> Optional[int]:
    """The barometer ArduPilot uses: BARO_PRIMARY (0 = the first) if there is such a barometer, else the first."""
    found = detected_barometers(ctx)
    if not found:
        return None
    wanted = ctx.param_int("BARO_PRIMARY") + 1
    return wanted if wanted in found else found[0]


def describe_barometers(ctx) -> str:
    """One line per barometer found: what and where it is, which one is used."""
    in_use = barometer_in_use(ctx)
    lines = [
        f"Barometer {instance}: {describe_device(ctx.param_int(f'BARO{instance}_DEVID'))}"
        + (" - used" if instance == in_use else "")
        for instance in detected_barometers(ctx)
    ]
    return "\n".join(lines) or "none found (ArduPilot reports 'Baro: unable to initialise driver')"


def has_several(ctx) -> bool:
    """More than one barometer found: BARO_PRIMARY chooses."""
    return len(detected_barometers(ctx)) > 1


def looks_for_external(ctx) -> bool:
    """The 'external barometer' checkbox is ticked: show what to look for, and where."""
    return bool(ctx.value("baro_external"))


def apply_barometer(ctx) -> None:
    """External barometer unticked: stop looking for one at boot."""
    if not looks_for_external(ctx) and ctx.param_int("BARO_PROBE_EXT"):
        ctx.set_param("BARO_PROBE_EXT", 0, reboot=True)


def describe_pressure(reading: Optional[tuple]) -> str:
    """'1013.2 hPa, 24.5 deg C' - or 'no reading'."""
    if reading is None:
        return "no reading"
    pressure, temperature = reading
    return f"{pressure:.1f} hPa, {temperature:.1f} deg C"


def action_show_pressure(ctx):
    """Log the pressure and temperature of every barometer once per second for 10 seconds."""
    found = detected_barometers(ctx)
    if not found:
        ctx.log("No barometer found - see the help of this step.")
        return

    for _ in range(LIVE_DISPLAY_SECONDS):
        readings = [f"Baro {instance}: {describe_pressure(ctx.vehicle.pressure(instance))}" for instance in found]
        healthy = ctx.vehicle.barometers_healthy()
        health = "" if healthy is None else ("   (healthy)" if healthy else "   (NOT healthy)")
        ctx.log("   ".join(readings) + health)
        ctx.sleep(1)


STEP = Step(
    "barometer",
    "Barometer",
    [
        value_field("baro_list", "Barometers found", kind="display", default=describe_barometers),
        param_field(
            "BARO_PRIMARY",
            "Barometer to use",
            kind="choice",
            choices=[(f"Barometer {instance}", instance - 1) for instance in BARO_INSTANCES],
            when=has_several,
        ),
        value_field(
            "baro_external",
            "An external barometer is connected",
            kind="bool",
            default=lambda ctx: ctx.param_int("BARO_PROBE_EXT") != 0,
            refresh=True,
        ),
        heading_field("baro_probe", "Look for these external barometers at boot", when=looks_for_external),
    ]
    + [
        bit_field("BARO_PROBE_EXT", bit, label, key=f"baro_probe_{bit}", reboot=True, when=looks_for_external)
        for bit, label in BARO_PROBE_EXT_BITS
    ]
    + [
        param_field(
            "BARO_EXT_BUS",
            f"I2C bus to search ({BARO_EXT_BUS_ALL} = all external)",
            kind="int",
            min_value=BARO_EXT_BUS_ALL,
            max_value=HIGHEST_I2C_BUS,
            reboot=True,
            when=looks_for_external,
        ),
    ],
    [Action("Show pressure (10 s)", action_show_pressure)],
    on_apply=apply_barometer,
)
