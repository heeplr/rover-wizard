"""rover_wizard.steps.beacon - the "Positioning beacons" step."""

from __future__ import annotations

from ..util.errors import VehicleError
from ..ardupilot_defs import SERIAL_PROTOCOL_BEACON
from .util.ports import SERIAL_PORT_CHOICES, chosen_serial_port, configure_serial, ensure_port_free, port_with_protocol
from .util.hardware import has
from ..model import Step, param_field, value_field

BEACON_TYPE_CHOICES = [("None", 0), ("Pozyx", 1), ("Marvelmind", 2), ("Nooploop", 3)]

# the beacon systems' documented 115200 baud
BEACON_BAUD = 115


def apply_beacons(ctx):
    """Put the beacon receiver's serial port on the Beacon protocol."""
    if int(ctx.number("BCN_TYPE", 0)):
        port = chosen_serial_port(ctx, "beacon_port")
        if not port:
            raise VehicleError("Choose the serial port the beacon receiver is connected to.")
        ensure_port_free(ctx, port, SERIAL_PROTOCOL_BEACON)
        configure_serial(ctx, port, SERIAL_PROTOCOL_BEACON, BEACON_BAUD)


STEP = Step(
    "beacon",
    "Positioning beacons",
    [
        param_field("BCN_TYPE", "Beacon system", kind="choice", choices=BEACON_TYPE_CHOICES, reboot=True),
        value_field(
            "beacon_port",
            "Receiver connected to serial port",
            kind="choice",
            choices=SERIAL_PORT_CHOICES,
            default=port_with_protocol(SERIAL_PROTOCOL_BEACON),
        ),
        param_field("BCN_LATITUDE", "Beacon origin latitude", unit="deg"),
        param_field("BCN_LONGITUDE", "Beacon origin longitude", unit="deg"),
        param_field("BCN_ALT", "Beacon origin altitude (above sea level)", unit="m"),
        param_field("BCN_ORIENT_YAW", "Beacon system rotation from north", unit="deg"),
    ],
    [],
    on_apply=apply_beacons,
    applies=has("has_beacons"),
)
