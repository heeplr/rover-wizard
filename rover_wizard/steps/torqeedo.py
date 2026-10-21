"""rover_wizard.steps.torqeedo - the "Torqeedo motor" step: a Torqeedo electric outboard driven over its RS485 bus.

AP_Torqeedo talks to the motor (or its battery's tiller connector) over a serial port at 19200 baud through an
RS485 converter, and passes the throttle output on to it. Its first motor's settings are TRQ1_* since firmware 4.7
(two motors) and TRQD_* before; the type switches it on, the other settings exist only after a reboot.
"""

from __future__ import annotations

from ..util.errors import VehicleError
from ..rover_defs import SERVO_FUNCTION_THROTTLE, SERVO_FUNCTION_THROTTLE_LEFT, SERVO_FUNCTION_THROTTLE_RIGHT
from ..boat_defs import (
    BATTERY_MONITOR_TORQEEDO,
    RC_OPTION_TORQEEDO_CLEAR_ERROR,
    SERIAL_PROTOCOL_TORQEEDO,
    TORQEEDO_SERIAL_BAUD,
)
from ..vehicle.boat_telemetry import BOAT_TELEMETRY_MAX_AGE_SECONDS
from .util.boat_hardware import TORQEEDO_TYPE_PARAMETER
from .util.rover_hardware import has
from .util.outputs import RC_SWITCH_CHOICES, assign_rc_switch, rc_channel_with_option
from .util.ports import SERIAL_PORT_CHOICES, chosen_serial_port, configure_serial, ensure_port_free, port_with_protocol
from .util.probing import LIVE_DISPLAY_SECONDS
from ..model import Action, Step, bit_field, param_field, value_field

TORQEEDO_TYPE_CHOICES = [("Disabled", 0), ("Tiller connector (Torqeedo battery)", 1), ("Motor connector", 2)]
# TRQ1_TYPE 2: connected to the motor itself (TRQ1_POWER only applies then)
TORQEEDO_TYPE_MOTOR = 2

# TRQ1_ONOFF_PIN / TRQ1_DE_PIN: -1 = the serial port's RTS / CTS flow-control pin, 50.. = AUX1..
TORQEEDO_PIN_CHOICES = [("The serial port's flow-control pin", -1)] + [
    (f"AUX{number}", pin) for number, pin in enumerate(range(50, 56), 1)
]

TORQEEDO_SERVO_FUNCTION_CHOICES = [
    ("Throttle", SERVO_FUNCTION_THROTTLE),
    ("Left motor", SERVO_FUNCTION_THROTTLE_LEFT),
    ("Right motor", SERVO_FUNCTION_THROTTLE_RIGHT),
]

# TRQ1_OPTIONS bits
TORQEEDO_OPTION_BITS = [(1, "Log the motor's reports"), (2, "Send the motor's reports to the ground station")]

# ESC_TELEMETRY_1_TO_4 covers the outputs 1..4, ESC_TELEMETRY_5_TO_8 5..8 (AP_ESC_Telem)
ESC_TELEMETRY_BLOCK = 4
CENTI = 100


def torqeedo_parameter(suffix: str) -> tuple:
    """The first Torqeedo's setting in current (TRQ1_) and older (TRQD_) firmware."""
    return (f"TRQ1_{suffix}", f"TRQD_{suffix}")


def are_torqeedo_settings_loaded(ctx) -> bool:
    """ArduPilot shows the Torqeedo settings: the type is set and the flight controller has rebooted since."""
    return ctx.vehicle.resolve(torqeedo_parameter("SLEW_TIME")) is not None


def reboot_note(ctx) -> str:
    """Shown until the Torqeedo settings appear."""
    if ctx.param_int(TORQEEDO_TYPE_PARAMETER):
        return "The Torqeedo type is set - reboot the flight controller (Reboot button) to see its settings."
    return "Choose how the Torqeedo is connected and press Apply, then reboot."


def apply_torqeedo(ctx) -> None:
    """Serial port (Torqeedo protocol, 19200 baud), battery monitor, error-clearing switch."""
    if not ctx.form_int("trq_type", TORQEEDO_TYPE_PARAMETER):
        return
    port = chosen_serial_port(ctx, "trq_port")
    if not port:
        raise VehicleError("Choose the serial port the Torqeedo's RS485 converter is connected to.")
    ensure_port_free(ctx, port, SERIAL_PROTOCOL_TORQEEDO)
    configure_serial(ctx, port, SERIAL_PROTOCOL_TORQEEDO, TORQEEDO_SERIAL_BAUD)
    if ctx.value("trq_battery"):
        ctx.set_param("BATT_MONITOR", BATTERY_MONITOR_TORQEEDO, reboot=True)
    assign_rc_switch(ctx, RC_OPTION_TORQEEDO_CLEAR_ERROR, int(ctx.number("trq_clear_switch", 0)))


def describe_motor(ctx) -> str:
    """The Torqeedo's ESC telemetry (AP_Torqeedo_TQBus::update_esc_telem: on its throttle output's index)."""
    function = ctx.param_int(torqeedo_parameter("SERVO_FN"), SERVO_FUNCTION_THROTTLE)
    output = ctx.output_with_function(function) or 1
    first = (output - 1) // ESC_TELEMETRY_BLOCK * ESC_TELEMETRY_BLOCK + 1
    message = ctx.vehicle.latest(
        f"ESC_TELEMETRY_{first}_TO_{first + ESC_TELEMETRY_BLOCK - 1}", max_age=BOAT_TELEMETRY_MAX_AGE_SECONDS
    )
    if message is None:
        return "No motor telemetry - check the RS485 wiring, the serial port and the type; then reboot."
    index = (output - 1) % ESC_TELEMETRY_BLOCK
    return (
        f"Torqeedo: {message.rpm[index]} rpm, {message.voltage[index] / CENTI:.1f} V, "
        f"{message.current[index] / CENTI:.1f} A, {message.temperature[index]} deg C"
    )


def action_show_motor(ctx):
    """Log the motor's telemetry once per second for 10 seconds."""
    for _ in range(LIVE_DISPLAY_SECONDS):
        ctx.log(describe_motor(ctx))
        ctx.flush_texts()
        ctx.sleep(1)


STEP = Step(
    "torqeedo",
    "Torqeedo motor",
    [
        param_field(
            torqeedo_parameter("TYPE"),
            "Connected to",
            key="trq_type",
            kind="choice",
            choices=TORQEEDO_TYPE_CHOICES,
            reboot=True,
            refresh=True,
        ),
        value_field(
            "trq_reboot",
            "Torqeedo settings",
            kind="display",
            default=reboot_note,
            when=lambda ctx: not are_torqeedo_settings_loaded(ctx),
        ),
        value_field(
            "trq_port",
            "Serial port (RS485 converter)",
            kind="choice",
            choices=SERIAL_PORT_CHOICES,
            default=port_with_protocol(SERIAL_PROTOCOL_TORQEEDO),
            when=lambda ctx: bool(ctx.form_int("trq_type", TORQEEDO_TYPE_PARAMETER)),
        ),
        param_field(
            torqeedo_parameter("ONOFF_PIN"),
            "On/off pin",
            key="trq_onoff_pin",
            kind="choice",
            choices=TORQEEDO_PIN_CHOICES,
            reboot=True,
            when=are_torqeedo_settings_loaded,
        ),
        param_field(
            torqeedo_parameter("DE_PIN"),
            "RS485 driver-enable (DE) pin",
            key="trq_de_pin",
            kind="choice",
            choices=TORQEEDO_PIN_CHOICES,
            reboot=True,
            when=are_torqeedo_settings_loaded,
        ),
        param_field(
            torqeedo_parameter("SERVO_FN"),
            "Follows the output",
            key="trq_servo_fn",
            kind="choice",
            choices=TORQEEDO_SERVO_FUNCTION_CHOICES,
            when=are_torqeedo_settings_loaded,
        ),
        param_field(
            torqeedo_parameter("POWER"),
            "Motor power",
            key="trq_power",
            unit="%",
            min_value=0,
            max_value=100,
            when=lambda ctx: are_torqeedo_settings_loaded(ctx)
            and ctx.form_int("trq_type", TORQEEDO_TYPE_PARAMETER) == TORQEEDO_TYPE_MOTOR,
        ),
        param_field(
            torqeedo_parameter("SLEW_TIME"),
            "Time from 0 to full throttle",
            key="trq_slew",
            unit="s",
            when=are_torqeedo_settings_loaded,
        ),
        param_field(
            torqeedo_parameter("DIR_DELAY"),
            "Pause when changing direction",
            key="trq_dir_delay",
            unit="s",
            when=are_torqeedo_settings_loaded,
        ),
    ]
    + [
        bit_field(torqeedo_parameter("OPTIONS"), bit, label, key=f"trq_option_{bit}", when=are_torqeedo_settings_loaded)
        for bit, label in TORQEEDO_OPTION_BITS
    ]
    + [
        value_field(
            "trq_battery",
            "Battery monitor reads the Torqeedo (BATT_MONITOR = 23)",
            kind="bool",
            default=lambda ctx: ctx.param_int("BATT_MONITOR") == BATTERY_MONITOR_TORQEEDO,
            when=lambda ctx: bool(ctx.form_int("trq_type", TORQEEDO_TYPE_PARAMETER)),
        ),
        value_field(
            "trq_clear_switch",
            "RC switch clearing motor errors",
            kind="choice",
            choices=RC_SWITCH_CHOICES,
            default=lambda ctx: rc_channel_with_option(ctx, RC_OPTION_TORQEEDO_CLEAR_ERROR),
            when=lambda ctx: bool(ctx.form_int("trq_type", TORQEEDO_TYPE_PARAMETER)),
        ),
    ],
    [Action("Show motor status (10 s)", action_show_motor, when=are_torqeedo_settings_loaded)],
    on_apply=apply_torqeedo,
    applies=has("has_torqeedo"),
)
