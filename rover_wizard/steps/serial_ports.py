"""rover_wizard.steps.serial_ports - protocol and baud rate of every serial port (SERIAL1..SERIAL8)."""

from __future__ import annotations

from ..rover_defs import SERIAL_PORTS, SERIAL_PROTOCOL_NONE
from .util.ports import SERIAL_BAUD_FULL_CHOICES, baud_label
from ..model import Step, param_field, value_field

# Values from AP_SerialManager.cpp.
SERIAL_PROTOCOL_CHOICES = [
    ("None", -1),
    ("MAVLink1", 1),
    ("MAVLink2", 2),
    ("FrSky D", 3),
    ("FrSky SPort", 4),
    ("GPS", 5),
    ("Alexmos gimbal", 7),
    ("Gimbal", 8),
    ("Rangefinder", 9),
    ("FrSky SPort passthrough", 10),
    ("Lidar360", 11),
    ("Beacon", 13),
    ("Volz servo out", 14),
    ("SBus servo out", 15),
    ("ESC telemetry", 16),
    ("Devo telemetry", 17),
    ("Optical flow", 18),
    ("Robotis servo", 19),
    ("NMEA output", 20),
    ("Wind vane", 21),
    ("SLCAN", 22),
    ("RC input", 23),
    ("EFI serial", 24),
    ("LTM", 25),
    ("RunCam", 26),
    ("HoTT telemetry", 27),
    ("Scripting", 28),
    ("Crossfire VTX", 29),
    ("Generator", 30),
    ("Winch", 31),
    ("MSP", 32),
    ("DJI FPV", 33),
    ("Airspeed", 34),
    ("ADSB", 35),
    ("AHRS", 36),
    ("SmartAudio", 37),
    ("FETtec OneWire", 38),
    ("Torqeedo", 39),
    ("AIS", 40),
    ("CoDev ESC", 41),
    ("DisplayPort", 42),
    ("MAVLink high latency", 43),
    ("IRC Tramp", 44),
    ("DDS XRCE", 45),
    ("IMU data", 46),
    ("PPP", 48),
    ("i-BUS telemetry", 49),
]

PROTOCOL_NAMES = {value: label for label, value in SERIAL_PROTOCOL_CHOICES}


def selected_serial_port(ctx) -> int:
    """The serial port the step's fields currently describe."""
    return int(ctx.values.get("serial_port") or 1)


def serial_parameter(suffix: str):
    """SERIALn_<suffix> of the selected port."""
    return lambda ctx: f"SERIAL{selected_serial_port(ctx)}_{suffix}"


def action_show_serial_overview(ctx):
    """Fill the overview with each port's protocol and baud rate."""
    lines = []
    for port in SERIAL_PORTS:
        if not ctx.vehicle.has_parameter(f"SERIAL{port}_PROTOCOL"):
            continue
        protocol = ctx.param_int(f"SERIAL{port}_PROTOCOL", SERIAL_PROTOCOL_NONE)
        baud = ctx.param_int(f"SERIAL{port}_BAUD")
        lines.append(f"SERIAL{port}: {PROTOCOL_NAMES.get(protocol, f'protocol {protocol}')} @ " f"{baud_label(baud)}")
    ctx.set_values(serial_overview="\n".join(lines) or "(no serial port parameters)")


STEP = Step(
    "serial_ports",
    "Serial ports",
    [
        value_field("serial_overview", "Serial ports", kind="display", default="(reading ...)"),
        value_field(
            "serial_port",
            "Port to configure",
            kind="choice",
            choices=[(f"SERIAL{port}", port) for port in SERIAL_PORTS],
            default=lambda ctx: ctx.values.get("serial_port") or 1,
            reload_on_change=True,
        ),
        param_field(
            serial_parameter("PROTOCOL"),
            "Protocol",
            key="serial_protocol",
            kind="choice",
            choices=SERIAL_PROTOCOL_CHOICES,
            reboot=True,
        ),
        param_field(
            serial_parameter("BAUD"),
            "Baud rate",
            key="serial_baud",
            kind="choice",
            choices=SERIAL_BAUD_FULL_CHOICES,
            reboot=True,
        ),
    ],
    [],
    on_apply=action_show_serial_overview,
    on_enter=action_show_serial_overview,
)
