"""rover_wizard.steps.util.ports - serial ports: choosing a port, its protocol and baud rate, and the baud rates a probe
tries."""

from __future__ import annotations

from collections.abc import Callable
from ...util.errors import VehicleError
from ...rover_defs import SERIAL_PORTS
from .probing import snapshot

SERIAL_PORT_CHOICES = [("Not connected to a serial port", 0)] + [(f"SERIAL{port}", port) for port in SERIAL_PORTS]


SERIAL_BAUD_FULL_CHOICES = [
    ("1200", 1),
    ("2400", 2),
    ("4800", 4),
    ("9600", 9),
    ("19200", 19),
    ("38400", 38),
    ("57600", 57),
    ("111100", 111),
    ("115200", 115),
    ("230400", 230),
    ("256000", 256),
    ("460800", 460),
    ("500000", 500),
    ("921600", 921),
    ("1500000", 1500),
    ("2000000", 2000),
]


# SERIALx_BAUD encoding for 115200
DEFAULT_SERIAL_BAUD = 115


# Baud rates the serial probes fall back to (SERIALx_BAUD encoding), most common first. Rangefinder, proximity and
# gimbal drivers only open their port at its configured rate (unlike ArduPilot's GPS driver, which cycles itself).
COMMON_PROBE_BAUDS = (115, 57, 38, 19, 9, 230, 460, 921)


def baud_rates_to_try(ctx, port: int, preferred=()) -> list:
    """SERIALx_BAUD values for a probe: the port's current rate, then the device's usual ones, then common rates."""
    current = ctx.param_int(f"SERIAL{port}_BAUD", DEFAULT_SERIAL_BAUD) or DEFAULT_SERIAL_BAUD
    ordered = []
    for baud in (current, *preferred, *COMMON_PROBE_BAUDS):
        if baud not in ordered:
            ordered.append(baud)
    return ordered


def baud_label(baud: int) -> str:
    """Bits per second of a SERIALx_BAUD value, for messages (e.g. 115 -> "115200")."""
    return {value: label for label, value in SERIAL_BAUD_FULL_CHOICES}.get(baud, f"{baud}")


def chosen_serial_port(ctx, key: str) -> int:
    """The serial port chosen in a field (0 = none); refuses ports this board doesn't have."""
    port = int(ctx.number(key, 0))
    if port and not ctx.vehicle.has_parameter(f"SERIAL{port}_PROTOCOL"):
        raise VehicleError(f"This board has no SERIAL{port}.")
    return port


def ensure_port_free(ctx, port: int, protocol: int) -> None:
    """Refuse a serial port that is already used for a different protocol (e.g. GPS or the telemetry link)."""
    current = ctx.param_int(f"SERIAL{port}_PROTOCOL", -1)
    if current not in (-1, 0, protocol):
        raise VehicleError(
            f"SERIAL{port} is used for protocol {current} - choose another port or free it on the " "Serial ports step."
        )


def configure_serial(ctx, port: int, protocol: int, baud: int) -> None:
    """Set a serial port's protocol and baud rate (both take effect after a reboot)."""
    ctx.set_param(f"SERIAL{port}_PROTOCOL", protocol, reboot=True)
    ctx.set_param(f"SERIAL{port}_BAUD", baud, reboot=True)


def configure_serial_keeping_baud(ctx, port: int, protocol: int, default_baud: int) -> None:
    """Like configure_serial, but a port that already runs this protocol keeps its baud rate (e.g. the one a probe
    found); otherwise it gets `default_baud`."""
    already_set_up = ctx.param_int(f"SERIAL{port}_PROTOCOL", -1) == protocol
    baud = ctx.param_value(f"SERIAL{port}_BAUD") if already_set_up else None
    configure_serial(ctx, port, protocol, int(baud or default_baud))


def port_with_protocol(protocol: int) -> Callable:
    """A field default: the serial port set to `protocol` now (0 = none), so a configured device's port shows."""

    def first_port(ctx) -> int:
        """The first serial port running the protocol."""
        return next((port for port in SERIAL_PORTS if ctx.param_int(f"SERIAL{port}_PROTOCOL", -1) == protocol), 0)

    return first_port


def _serial_snapshot(ctx, port: int) -> dict:
    """Current protocol/baud of a serial port, to restore after a probe."""
    return snapshot(ctx, f"SERIAL{port}_PROTOCOL", f"SERIAL{port}_BAUD") if port else {}
