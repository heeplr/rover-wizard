"""rover_wizard.steps.remote_id - OpenDroneID Remote ID module (needs firmware built with Remote ID)."""

from __future__ import annotations

from ..util.errors import VehicleError
from ..ardupilot_defs import SERIAL_PORTS, SERIAL_PROTOCOL_MAVLINK2, SERIAL_PROTOCOL_NONE
from .util.choices import DISABLED_ENABLED_CHOICES
from .util.hardware import has
from ..model import Step, bit_field, param_field

REMOTE_ID_OPTION_BITS = [
    (1, "Enforce Remote ID arming checks"),
    (2, "Allow arming without a GPS position"),
    (4, "Lock the UAS ID on the first Basic ID received"),
]

REMOTE_ID_CAN_CHOICES = [("Not on DroneCAN", 0), ("First CAN driver", 1), ("Second CAN driver", 2)]

# DID_MAVPORT documents SERIAL0..SERIAL6 only
REMOTE_ID_LAST_PORT = 6
REMOTE_ID_PORT_CHOICES = [("Not on a serial port", -1), ("SERIAL0 (USB)", 0)] + [
    (f"SERIAL{port}", port) for port in SERIAL_PORTS if port <= REMOTE_ID_LAST_PORT
]

# MAVLink1 / MAVLink2
MAVLINK_PROTOCOLS = (1, SERIAL_PROTOCOL_MAVLINK2)


def apply_remote_id(ctx):
    """A module on a serial port talks MAVLink: make sure that port uses MAVLink (never override another protocol)."""
    port = ctx.param_int("DID_MAVPORT", -1)
    if port <= 0 or not ctx.vehicle.has_parameter(f"SERIAL{port}_PROTOCOL"):
        return

    protocol = ctx.param_int(f"SERIAL{port}_PROTOCOL", SERIAL_PROTOCOL_NONE)
    if protocol in MAVLINK_PROTOCOLS:
        return

    if protocol not in (SERIAL_PROTOCOL_NONE, 0):
        raise VehicleError(
            f"SERIAL{port} is used for protocol {protocol} - choose another port for the Remote ID module."
        )
    ctx.set_param(f"SERIAL{port}_PROTOCOL", SERIAL_PROTOCOL_MAVLINK2, reboot=True)


STEP = Step(
    "remote_id",
    "Remote ID",
    [
        param_field("DID_ENABLE", "Remote ID enabled", kind="choice", choices=DISABLED_ENABLED_CHOICES, reboot=True),
        param_field(
            "DID_MAVPORT", "Module on serial port (MAVLink)", kind="choice", choices=REMOTE_ID_PORT_CHOICES, reboot=True
        ),
        param_field("DID_CANDRIVER", "Module on DroneCAN", kind="choice", choices=REMOTE_ID_CAN_CHOICES, reboot=True),
    ]
    + [bit_field("DID_OPTIONS", bit, label, key=f"did_option_{bit}") for bit, label in REMOTE_ID_OPTION_BITS],
    [],
    on_apply=apply_remote_id,
    applies=has("has_remote_id"),
)
