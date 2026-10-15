"""rover_wizard.steps.generator - generators and fuel cells: type and serial port."""

from __future__ import annotations

from ..util.errors import VehicleError
from ..ardupilot_defs import SERIAL_PROTOCOL_GENERATOR
from .util.ports import (
    DEFAULT_SERIAL_BAUD,
    SERIAL_BAUD_FULL_CHOICES,
    SERIAL_PORT_CHOICES,
    chosen_serial_port,
    configure_serial,
    ensure_port_free,
    port_with_protocol,
)
from .util.hardware import has
from ..model import Step, bit_field, param_field, value_field

GENERATOR_TYPE_CHOICES = [
    ("Disabled", 0),
    ("IE 650 W / 800 W fuel cell", 1),
    ("IE 2.4 kW fuel cell", 2),
    ("RichenPower", 3),
    ("Loweheiser", 4),
    ("Cortex", 5),
]


def apply_generator(ctx):
    """Serial port and baud rate of the generator."""
    if int(ctx.number("GEN_TYPE", 0)):
        port = chosen_serial_port(ctx, "generator_port")
        if not port:
            raise VehicleError("Choose the serial port the generator is connected to.")
        ensure_port_free(ctx, port, SERIAL_PROTOCOL_GENERATOR)
        configure_serial(ctx, port, SERIAL_PROTOCOL_GENERATOR, int(ctx.number("generator_baud", DEFAULT_SERIAL_BAUD)))


STEP = Step(
    "generator",
    "Generator / fuel cell",
    [
        param_field("GEN_TYPE", "Generator type", kind="choice", choices=GENERATOR_TYPE_CHOICES, reboot=True),
        value_field(
            "generator_port",
            "Connected to serial port",
            kind="choice",
            choices=SERIAL_PORT_CHOICES,
            default=port_with_protocol(SERIAL_PROTOCOL_GENERATOR),
        ),
        value_field(
            "generator_baud",
            "Serial baud rate",
            kind="choice",
            choices=SERIAL_BAUD_FULL_CHOICES,
            default=DEFAULT_SERIAL_BAUD,
        ),
        bit_field("GEN_OPTIONS", 1, "Suppress maintenance-required warnings", key="gen_option_1"),
    ],
    [],
    on_apply=apply_generator,
    applies=has("has_generator"),
)
