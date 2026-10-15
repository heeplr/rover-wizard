"""rover_wizard.steps.gps - one or two GPS receivers: type, serial port, baud rate, antenna offsets, switching.

ArduPilot assigns receivers to serial ports in port order: the first port whose SERIALx_PROTOCOL is GPS carries
GPS 1, the second one GPS 2 (and it stops looking for GPS 1 if it isn't on that first port). The wizard therefore
keeps GPS 1 on the lower-numbered port and refuses ports that another device already uses.
"""

from __future__ import annotations

from ..util.errors import VehicleError
from ..ardupilot_defs import SERIAL_PORTS, SERIAL_PROTOCOL_GPS, SERIAL_PROTOCOL_NONE
from ..mavlink_defs import GPS_FIX_TYPE_NAMES, GPS_RAW_INT_EPH_SCALE
from ..util.timebase import now
from .util.choices import COUNT_ONE_TWO_CHOICES
from .util.probing import LIVE_DISPLAY_SECONDS, restore_on_failure
from .util.ports import SERIAL_BAUD_FULL_CHOICES
from .util.actions import action_reboot_flight_controller
from .util.hardware import has
from ..model import Action, Step, bit_field, heading_field, param_field, value_field

GPS_TYPE_AUTO = 1

GPS_TYPE_NMEA = 5

GPS_TYPE_CHOICES = [
    ("None", 0),
    ("Auto", GPS_TYPE_AUTO),
    ("u-blox", 2),
    ("NMEA", GPS_TYPE_NMEA),
    ("DroneCAN", 9),
    ("MAVLink", 14),
    # two receivers measuring heading (moving baseline): the base on GPS 1, the rover on GPS 2
    ("u-blox moving baseline: base", 17),
    ("u-blox moving baseline: rover", 18),
    ("DroneCAN moving baseline: base", 22),
    ("DroneCAN moving baseline: rover", 23),
]

# EK3_GPS_CHECK bits (AP_NavEKF3.cpp)
GPS_CHECK_BITS = [
    (1, "Number of satellites"),
    (2, "HDoP"),
    (4, "Speed error"),
    (8, "Position error"),
    (16, "Yaw error"),
    (32, "Position drift"),
    (64, "Vertical speed"),
    (128, "Horizontal speed"),
]

GPS_AUTO_SWITCH_CHOICES = [("Use primary", 0), ("Use best", 1), ("Blend", 2), ("Use primary if 3D fix or better", 4)]

GPS_PRIMARY_CHOICES = [("GPS 1", 0), ("GPS 2", 1)]

GPS_PORT_CHOICES = [("Not on a serial port (DroneCAN ...)", 0)] + [(f"SERIAL{port}", port) for port in SERIAL_PORTS]

# ArduPilot's GPS driver finds the receiver's baud rate itself: starting at the configured rate it tries 8 more
# (AP_GPS::_baudrates), each for 1.2 s (GPS_BAUD_TIME_MS). The probe waits two full rounds of that.
ARDUPILOT_GPS_BAUD_RATES = 8
ARDUPILOT_GPS_BAUD_STEP_SECONDS = 1.2
GPS_DETECTION_SECONDS = 2 * (ARDUPILOT_GPS_BAUD_RATES + 1) * ARDUPILOT_GPS_BAUD_STEP_SECONDS

GPS_TELEMETRY_FRESH_SECONDS = 2.0
# how often the GPS probe checks for a receiver
GPS_POLL_SECONDS = 0.5


def _gps_message_type(instance: int) -> str:
    """Telemetry message that reports receiver 1 or 2."""
    return "GPS_RAW_INT" if instance == 1 else "GPS2_RAW"


def _is_receiver_detected(ctx) -> bool:
    """Wait up to GPS_DETECTION_SECONDS for fresh GPS data showing a receiver (any fix state or any satellites)."""
    start = now()
    while now() - start < GPS_DETECTION_SECONDS:
        ctx.check()
        gps = ctx.vehicle.latest(_gps_message_type(selected_gps(ctx)), max_age=GPS_TELEMETRY_FRESH_SECONDS)
        # fix_type 0 = "no GPS" (nothing talking); 1 = "no fix" already means a receiver answered.
        if gps is not None and (gps.fix_type > 0 or gps.satellites_visible > 0):
            return True
        ctx.sleep(GPS_POLL_SECONDS)
    return False


def action_probe_gps_type(ctx):
    """Try GPS type Auto, then NMEA (ArduPilot's Auto never tries NMEA), with a reboot after each attempt."""
    # the selected receiver's type parameter
    parameter = gps_type_names(ctx)
    original = ctx.param_value(parameter)
    if not ctx.confirm("Start the GPS type probe (Auto, then NMEA; a reboot each, about a minute per type)?"):
        ctx.log("Cancelled - no changes made.")
        return

    detected_but_declined = []
    with restore_on_failure(ctx, {parameter: original}):
        for gps_type, name in ((GPS_TYPE_AUTO, "Auto"), (GPS_TYPE_NMEA, "NMEA")):
            ctx.log(f"Trying GPS type {name} ...")
            ctx.set_param(parameter, gps_type)
            action_reboot_flight_controller(ctx)
            ctx.log(f"Waiting up to {GPS_DETECTION_SECONDS:.0f} s for a receiver to be detected ...")
            if not _is_receiver_detected(ctx):
                ctx.log(f"GPS type {name}: no receiver detected.")
                continue
            gps = ctx.vehicle.latest(_gps_message_type(selected_gps(ctx)))
            ctx.log(f"GPS type {name} detected a receiver (fix_type={gps.fix_type}, {gps.satellites_visible} sats).")
            if ctx.confirm(f"Keep GPS_TYPE = {name}?"):
                ctx.set_values(gps_type=gps_type)
                ctx.log("GPS type applied.")
                return
            detected_but_declined.append(name)
        # Nothing kept: restore the original type (the guard only handles exceptions).
        ctx.log("Restoring the original GPS type and rebooting back ...")
        ctx.set_param(parameter, original)
        action_reboot_flight_controller(ctx)

    if detected_but_declined:
        ctx.log(f"Detected with {', '.join(detected_but_declined)} but not kept - original GPS type restored.")
        return
    raise VehicleError("Could not auto-detect the GPS type - check the wiring/baud rate, or set it manually.")


def action_show_gps_status(ctx):
    """Log fix type, satellites and HDOP once per second for 10 seconds."""
    for _ in range(LIVE_DISPLAY_SECONDS):
        gps = ctx.vehicle.latest("GPS_RAW_INT")
        if gps is None:
            ctx.log("No GPS_RAW_INT received (GPS not detected?)")
        else:
            fix = GPS_FIX_TYPE_NAMES.get(gps.fix_type, gps.fix_type)
            # eph is HDOP x 100; divide to show the usual dimensionless HDOP.
            ctx.log(f"GPS: {fix}, {gps.satellites_visible} sats, HDOP {gps.eph / GPS_RAW_INT_EPH_SCALE:.1f}")
        ctx.sleep(1)


# Parameter names per receiver: firmware 4.6+ name first, the older name second.
GPS_TYPE_NAMES = {1: ("GPS1_TYPE", "GPS_TYPE"), 2: ("GPS2_TYPE", "GPS_TYPE2")}

# SERIALx_BAUD encoding for 230400
DEFAULT_GPS_BAUD = 230


def detected_gps_count(ctx) -> int:
    """2 if a second receiver is configured on the vehicle, else 1."""
    return 2 if ctx.param_int(GPS_TYPE_NAMES[2]) else 1


def gps_count(ctx) -> int:
    """Number of receivers chosen in the form (the vehicle's configuration until chosen)."""
    return int(ctx.values.get("gps_count") or detected_gps_count(ctx))


def selected_gps(ctx) -> int:
    """The receiver (1 or 2) the step's fields currently describe."""
    return min(int(ctx.values.get("gps") or 1), gps_count(ctx))


def gps_type_names(ctx) -> tuple:
    """Type parameter of the selected receiver."""
    return GPS_TYPE_NAMES[selected_gps(ctx)]


def gps_offset_names(axis: str):
    """Antenna-offset parameter names of the selected receiver for one axis (X, Y or Z)."""
    return lambda ctx: (f"GPS{selected_gps(ctx)}_POS_{axis}", f"GPS_POS{selected_gps(ctx)}_{axis}")


def gps_ports(ctx) -> list:
    """Serial ports set to the GPS protocol, in port order (index 0 = GPS 1)."""
    return [
        port
        for port in SERIAL_PORTS
        if ctx.param_int(f"SERIAL{port}_PROTOCOL", SERIAL_PROTOCOL_NONE) == SERIAL_PROTOCOL_GPS
    ]


def default_gps_port(ctx) -> int:
    """The serial port the selected receiver uses now (0 = none, e.g. DroneCAN)."""
    ports = gps_ports(ctx)
    return ports[selected_gps(ctx) - 1] if len(ports) >= selected_gps(ctx) else 0


def default_gps_baud(ctx) -> int:
    """The selected receiver's current baud setting."""
    port = default_gps_port(ctx)
    return ctx.param_int(f"SERIAL{port}_BAUD", DEFAULT_GPS_BAUD) or DEFAULT_GPS_BAUD if port else DEFAULT_GPS_BAUD


def _move_gps_port(ctx, instance: int, port: int, baud: int) -> None:
    """Put receiver `instance` on serial port `port` (0 = none) with `baud`, keeping ArduPilot's port order valid."""
    ports = gps_ports(ctx)
    old = ports[instance - 1] if len(ports) >= instance else 0
    if port and port != old:
        if not ctx.vehicle.has_parameter(f"SERIAL{port}_PROTOCOL"):
            raise VehicleError(f"This board has no SERIAL{port}.")
        protocol = ctx.param_int(f"SERIAL{port}_PROTOCOL", SERIAL_PROTOCOL_NONE)
        if protocol == SERIAL_PROTOCOL_GPS:
            raise VehicleError(f"SERIAL{port} already carries the other GPS.")
        if protocol not in (SERIAL_PROTOCOL_NONE, 0):
            raise VehicleError(
                f"SERIAL{port} is used for protocol {protocol} - free it on the Serial ports step first."
            )

    # GPS 1 must stay on the lower-numbered GPS port: check the order the ports would have after the change.
    new_ports = sorted([p for p in ports if p != old] + ([port] if port else []))
    if port and new_ports.index(port) + 1 != instance:
        raise VehicleError(
            f"ArduPilot uses the lowest-numbered GPS port for GPS 1 - put GPS {instance} on a "
            f"{'lower' if instance == 1 else 'higher'}-numbered port than the other GPS."
        )

    if old and old != port:
        ctx.set_param(f"SERIAL{old}_PROTOCOL", SERIAL_PROTOCOL_NONE, reboot=True)

    if port:
        ctx.set_param(f"SERIAL{port}_PROTOCOL", SERIAL_PROTOCOL_GPS, reboot=True)
        ctx.set_param(f"SERIAL{port}_BAUD", baud, reboot=True)


def apply_gps(ctx):
    """Number of receivers, and serial port + baud of the selected one (type/offsets are ordinary fields)."""
    if gps_count(ctx) == 1:
        ctx.set_param(GPS_TYPE_NAMES[2], 0, reboot=True, required=False)
    _move_gps_port(
        ctx, selected_gps(ctx), int(ctx.number("gps_port", 0)), int(ctx.number("gps_baud", DEFAULT_GPS_BAUD))
    )


STEP = Step(
    "gps",
    "GPS",
    [
        value_field(
            "gps_count",
            "Number of GPS receivers",
            kind="choice",
            choices=COUNT_ONE_TWO_CHOICES,
            default=lambda ctx: ctx.values.get("gps_count") or detected_gps_count(ctx),
            reload_on_change=True,
        ),
        value_field(
            "gps",
            "GPS to configure",
            kind="choice",
            choices=[("GPS 1", 1), ("GPS 2", 2)],
            default=lambda ctx: ctx.values.get("gps") or 1,
            reload_on_change=True,
            when=lambda ctx: gps_count(ctx) == 2,
        ),
        param_field(
            gps_type_names,
            "GPS type",
            key="gps_type",
            kind="choice",
            choices=GPS_TYPE_CHOICES,
            reboot=True,
            inline_actions=[Action("Probe", action_probe_gps_type, armed=False, is_probe=True)],
        ),
        value_field(
            "gps_port", "Connected to serial port", kind="choice", choices=GPS_PORT_CHOICES, default=default_gps_port
        ),
        value_field(
            "gps_baud",
            "Serial baud rate",
            kind="choice",
            choices=SERIAL_BAUD_FULL_CHOICES,
            default=default_gps_baud,
            when=lambda ctx: int(ctx.number("gps_port", 0)) > 0,
        ),
        param_field(gps_offset_names("X"), "Antenna offset X (forward)", key="gps_pos_x", unit="m"),
        param_field(gps_offset_names("Y"), "Antenna offset Y (right)", key="gps_pos_y", unit="m"),
        param_field(gps_offset_names("Z"), "Antenna offset Z (down)", key="gps_pos_z", unit="m"),
        param_field(
            "GPS_AUTO_SWITCH",
            "Using two receivers",
            kind="choice",
            choices=GPS_AUTO_SWITCH_CHOICES,
            when=lambda ctx: gps_count(ctx) == 2,
        ),
        param_field(
            "GPS_PRIMARY",
            "Primary receiver",
            kind="choice",
            choices=GPS_PRIMARY_CHOICES,
            when=lambda ctx: gps_count(ctx) == 2,
        ),
    ]
    + [heading_field("gps_checks", "GPS checks before the EKF uses the GPS")]
    + [bit_field("EK3_GPS_CHECK", bit, label, key=f"gps_check_{bit}") for bit, label in GPS_CHECK_BITS]
    + [param_field("EK3_CHECK_SCALE", "Accepted GPS error", kind="int", unit="%", min_value=50, max_value=200)],
    [Action("Show GPS status (10 s)", action_show_gps_status)],
    on_apply=apply_gps,
    applies=has("has_gps"),
)
