"""rover_wizard.steps.distance_sensors - the "Distance sensors & object avoidance" step.

Rangefinders (RNGFNDn_*) measure one distance in one fixed direction - forward for obstacles, down as a boat's
echosounder. Proximity sensors (PRXn_*) report distances in several directions around the vehicle, and they are
what ArduPilot's object avoidance uses. The proximity type "RangeFinder" bridges the two: it feeds every
horizontally pointing rangefinder (orientation 0-7) into the proximity system (AP_Proximity_RangeFinder.cpp).

Type lists use ArduPilot's own names; which types need a serial port or an I2C address was read from the driver
selection in ArduPilot's AP_RangeFinder.cpp / AP_Proximity.cpp.
"""

from __future__ import annotations

from ..util.errors import VehicleError
from ..ardupilot_defs import RANGEFINDER_INSTANCES, SERIAL_PROTOCOL_LIDAR360, SERIAL_PROTOCOL_RANGEFINDER
from ..util.timebase import now
from .util.ports import (
    COMMON_PROBE_BAUDS,
    DEFAULT_SERIAL_BAUD,
    SERIAL_BAUD_FULL_CHOICES,
    baud_label,
    baud_rates_to_try,
    SERIAL_PORT_CHOICES,
    _serial_snapshot,
    chosen_serial_port,
    configure_serial,
    configure_serial_keeping_baud,
    port_with_protocol,
    ensure_port_free,
)
from .util.probing import LIVE_DISPLAY_SECONDS, restore, restore_on_failure, snapshot
from .util.actions import action_reboot_flight_controller
from .util.hardware import has
from ..model import Action, Step, bit_field, param_field, value_field

# RNGFNDn_TYPE values and names from ArduPilot's parameter definitions (simulator-only types left out).
RANGEFINDER_TYPE_CHOICES = [
    ("None", 0),
    ("Analog", 1),
    ("MaxbotixI2C", 2),
    ("LidarLite-I2C", 3),
    ("PWM", 5),
    ("BBB-PRU", 6),
    ("LightWareI2C", 7),
    ("LightWareSerial", 8),
    ("Bebop", 9),
    ("MAVLink", 10),
    ("USD1_Serial", 11),
    ("LeddarOne", 12),
    ("MaxbotixSerial", 13),
    ("TeraRangerI2C", 14),
    ("LidarLiteV3-I2C", 15),
    ("VL53L0X or VL53L1X", 16),
    ("NMEA", 17),
    ("WASP-LRF", 18),
    ("BenewakeTF02", 19),
    ("BenewakeTFmini-Serial", 20),
    ("LidarLightV3HP", 21),
    ("PWM", 22),
    ("BlueRoboticsPing", 23),
    ("DroneCAN", 24),
    ("BenewakeTFmini-I2C", 25),
    ("LanbaoPSK-CM8JL65-CC5", 26),
    ("BenewakeTF03", 27),
    ("VL53L1X-ShortRange", 28),
    ("LeddarVu8-Serial", 29),
    ("HC-SR04", 30),
    ("GYUS42v2", 31),
    ("MSP", 32),
    ("USD1_CAN", 33),
    ("Benewake_CAN", 34),
    ("TeraRangerSerial", 35),
    ("Lua_Scripting", 36),
    ("NoopLoop_TOFSense", 37),
    ("NoopLoop_TOFSense_CAN", 38),
    ("NRA24_CAN", 39),
    ("NoopLoop_TOFSenseF_I2C", 40),
    ("JRE_Serial", 41),
    ("Ainstein_LR_D1", 42),
    ("RDS02UF", 43),
    ("HexsoonRadar", 44),
    ("LightWare-GRF", 45),
    ("BenewakeTFS20L-I2C", 46),
    ("DTS6012M-Serial", 47),
    ("LightWare-GRF-I2C", 48),
]

# Types whose driver reads a serial port (AP_RangeFinder.cpp assigns them a serial_create_fn) ...
RANGEFINDER_SERIAL_TYPES = frozenset([8, 11, 12, 13, 17, 18, 19, 20, 23, 26, 27, 29, 31, 35, 37, 41, 42, 43, 45, 47])

# ... and types whose driver is found on an I2C bus.
RANGEFINDER_I2C_TYPES = frozenset([2, 3, 7, 14, 15, 16, 21, 25, 28, 40, 46, 48])

# RNGFNDx_ORIENT uses ArduPilot's rotation enum: 0-7 = around the vehicle in 45-degree steps, 24 = up, 25 = down.
RANGEFINDER_ORIENTATION_CHOICES = [
    ("Forward", 0),
    ("Forward-right", 1),
    ("Right", 2),
    ("Back-right", 3),
    ("Back", 4),
    ("Back-left", 5),
    ("Left", 6),
    ("Forward-left", 7),
    ("Up", 24),
    ("Down (e.g. echosounder)", 25),
]

# Orientations 0..7 point horizontally: those rangefinders feed the proximity type "RangeFinder".
HORIZONTAL_ORIENTATIONS = range(8)

# (type, I2C address, name): the common I2C sensors and their default addresses (from the drivers).
RANGEFINDER_I2C_CANDIDATES = [
    (7, 102, "LightWare I2C"),
    (25, 16, "Benewake I2C"),
    (16, 41, "VL53L0X / VL53L1X"),
    (2, 112, "Maxbotix I2C sonar"),
]

RANGEFINDER_SERIAL_CANDIDATES = [(8, "LightWare serial"), (20, "Benewake serial")]

# Proximity sensor instances: PRX1..PRX5.
PROXIMITY_INSTANCES = range(1, 6)

# PRXn_TYPE values and names from ArduPilot's parameter definitions (simulator-only types left out).
PROXIMITY_TYPE_CHOICES = [
    ("None", 0),
    ("MAVLink", 2),
    ("TeraRangerTower", 3),
    ("RangeFinder", 4),
    ("RPLidarA2", 5),
    ("TeraRangerTowerEvo", 6),
    ("LightwareSF40c", 7),
    ("LightwareSF45B", 8),
    ("CygbotD1", 13),
    ("DroneCAN", 14),
    ("Scripting", 15),
    ("LD06", 16),
    ("MR72_CAN", 17),
    ("HexsoonRadar", 18),
]

# Types whose driver is a serial backend (AP_Proximity_Backend_Serial): their port uses the Lidar360 protocol.
PROXIMITY_SERIAL_TYPES = frozenset([3, 5, 6, 7, 8, 13, 16])

# The proximity type that takes its distances from the horizontally pointing rangefinders.
PROXIMITY_TYPE_RANGEFINDER = 4

PROXIMITY_ORIENTATION_CHOICES = [("Top of the vehicle, right way up", 0), ("Bottom, mounted upside down", 1)]


# AVOID_ENABLE bits (ArduPilot's parameter definition)
AVOID_SOURCE_BITS = [(1, "Avoid: the fence"), (2, "Avoid: proximity sensors"), (4, "Avoid: the beacon fence")]

AVOID_BEHAVE_CHOICES = [("Slide along obstacles", 0), ("Stop", 1)]

# a proximity sensor reports several directions; a rangefinder only one
MIN_PROXIMITY_SECTORS = 3

# (type, its usual baud rates, name): RPLidar A2 115200, S1/A2M12 256000, C1 460800; SF40/C 921600
PROXIMITY_PROBE_TYPES = [(5, (115, 256, 460), "RPLidar"), (7, (921,), "LightWare SF40/C")]

CENTIMETRES_PER_METRE = 100
# after a reboot, how long a candidate gets to report readings
DETECTION_SECONDS = 10
# how often a candidate is checked within that time
DETECTION_POLL_SECONDS = 0.5


def _wait_for(ctx, condition) -> bool:
    """Poll `condition()` for up to DETECTION_SECONDS; True as soon as it holds."""
    start = now()
    while now() - start < DETECTION_SECONDS:
        ctx.check()
        if condition():
            return True
        ctx.sleep(DETECTION_POLL_SECONDS)
    return False


def _describe_reading(message) -> str:
    """One distance reading as text (DISTANCE_SENSOR distances are in centimetres)."""
    # cm -> m
    metres = message.current_distance / CENTIMETRES_PER_METRE
    return f"sensor {message.id} (orientation {message.orientation}): {metres:.2f} m"


def action_show_distances(ctx):
    """Log every fresh distance reading once per second for 10 seconds."""
    for _ in range(LIVE_DISPLAY_SECONDS):
        readings = ctx.vehicle.distance_sensors()
        ctx.log(
            "; ".join(_describe_reading(message) for message in readings)
            if readings
            else "No DISTANCE_SENSOR readings (type set? rebooted? wiring?)"
        )
        ctx.sleep(1)


# ============================================================================ which instance the fields describe
def selected_rangefinder(ctx) -> str:
    """Instance character ("1".."9", "A") of the rangefinder the fields currently describe."""
    return RANGEFINDER_INSTANCES[int(ctx.values.get("rangefinder") or 1) - 1]


def rangefinder_parameter(suffix: str):
    """RNGFNDn_<suffix> of the selected rangefinder."""
    return lambda ctx: f"RNGFND{selected_rangefinder(ctx)}_{suffix}"


def selected_proximity(ctx) -> int:
    """Number of the proximity sensor the fields currently describe."""
    return int(ctx.values.get("proximity") or 1)


def proximity_type_names(ctx) -> tuple:
    """Type parameter of the selected proximity sensor (before firmware 4.3: PRX_TYPE for the only one)."""
    instance = selected_proximity(ctx)
    return (f"PRX{instance}_TYPE", "PRX_TYPE") if instance == 1 else (f"PRX{instance}_TYPE",)


def proximity_parameter(suffix: str):
    """PRXn_<suffix> of the selected proximity sensor (with the pre-4.3 name for the first one)."""
    return lambda ctx: (
        (f"PRX1_{suffix}", f"PRX_{suffix}")
        if selected_proximity(ctx) == 1
        else f"PRX{selected_proximity(ctx)}_{suffix}"
    )


def is_rangefinder_serial(ctx) -> bool:
    """The selected rangefinder's type reads a serial port."""
    return ctx.form_int("rng_type", rangefinder_parameter("TYPE")(ctx)) in RANGEFINDER_SERIAL_TYPES


def is_rangefinder_i2c(ctx) -> bool:
    """The selected rangefinder's type sits on an I2C bus."""
    return ctx.form_int("rng_type", rangefinder_parameter("TYPE")(ctx)) in RANGEFINDER_I2C_TYPES


def is_proximity_serial(ctx) -> bool:
    """The selected proximity sensor's type reads a serial port."""
    return ctx.form_int("prx_type", proximity_type_names(ctx)) in PROXIMITY_SERIAL_TYPES


def _type_name(choices, value: int) -> str:
    """ArduPilot's name of a type value."""
    return next((label for label, choice in choices if choice == value), f"type {value}")


def rangefinder_overview(ctx) -> str:
    """Every configured rangefinder: instance, type and direction."""
    directions = {value: label for label, value in RANGEFINDER_ORIENTATION_CHOICES}
    lines = []
    for instance in RANGEFINDER_INSTANCES:
        kind = ctx.param_int(f"RNGFND{instance}_TYPE")
        if kind:
            orientation = ctx.param_int(f"RNGFND{instance}_ORIENT")
            lines.append(
                f"Rangefinder {instance}: {_type_name(RANGEFINDER_TYPE_CHOICES, kind)}, "
                f"{directions.get(orientation, f'orientation {orientation}').lower()}"
            )
    return "\n".join(lines) or "(none configured)"


def proximity_overview(ctx) -> str:
    """Every configured proximity sensor: instance and type."""
    lines = []
    for instance in PROXIMITY_INSTANCES:
        names = (f"PRX{instance}_TYPE", "PRX_TYPE") if instance == 1 else f"PRX{instance}_TYPE"
        kind = ctx.param_int(names)
        if kind:
            lines.append(f"Proximity sensor {instance}: {_type_name(PROXIMITY_TYPE_CHOICES, kind)}")
    return "\n".join(lines) or "(none configured)"


# ============================================================================ apply
def _has_horizontal_rangefinder(ctx) -> bool:
    """Whether a configured rangefinder points horizontally (and so feeds the proximity type RangeFinder)."""
    return any(
        ctx.param_int(f"RNGFND{instance}_TYPE") and ctx.param_int(f"RNGFND{instance}_ORIENT") in HORIZONTAL_ORIENTATIONS
        for instance in RANGEFINDER_INSTANCES
    )


def apply_distance_sensors(ctx):
    """Serial ports of the selected rangefinder / proximity sensor; warn about a RangeFinder bridge without input."""
    if has("has_rangefinder")(ctx) and is_rangefinder_serial(ctx):
        port = chosen_serial_port(ctx, "rng_port")
        if not port:
            raise VehicleError("A serial rangefinder needs the serial port it is connected to.")
        ensure_port_free(ctx, port, SERIAL_PROTOCOL_RANGEFINDER)
        configure_serial_keeping_baud(ctx, port, SERIAL_PROTOCOL_RANGEFINDER, DEFAULT_SERIAL_BAUD)

    if has("has_proximity")(ctx) and is_proximity_serial(ctx):
        port = chosen_serial_port(ctx, "prx_port")
        if not port:
            raise VehicleError("Choose the serial port the proximity sensor is connected to.")
        ensure_port_free(ctx, port, SERIAL_PROTOCOL_LIDAR360)
        configure_serial(ctx, port, SERIAL_PROTOCOL_LIDAR360, int(ctx.number("prx_baud", DEFAULT_SERIAL_BAUD)))

    if (
        has("has_proximity")(ctx)
        and ctx.form_int("prx_type", proximity_type_names(ctx)) == PROXIMITY_TYPE_RANGEFINDER
        and not _has_horizontal_rangefinder(ctx)
    ):
        ctx.log(
            "Note: the proximity type RangeFinder uses rangefinders pointing horizontally (forward, sideways or "
            "back) - none is configured yet."
        )


# ============================================================================ probes
def _rangefinder_reading(ctx):
    """A fresh reading from the selected rangefinder's direction, or None."""
    orientation = ctx.param_int(rangefinder_parameter("ORIENT")(ctx))
    return next(
        (
            message
            for message in ctx.vehicle.distance_sensors()
            if message.orientation == orientation and message.current_distance > 0
        ),
        None,
    )


def action_probe_rangefinder(ctx):
    """Try the common I2C rangefinders (and serial ones on the chosen port) as the selected rangefinder."""
    port = chosen_serial_port(ctx, "rng_port")
    type_name, address_name = rangefinder_parameter("TYPE")(ctx), rangefinder_parameter("ADDR")(ctx)
    # (type, I2C address, serial port, baud, name): the I2C sensors, then each serial type at each baud rate
    candidates = [(kind, address, 0, 0, name) for kind, address, name in RANGEFINDER_I2C_CANDIDATES]
    if port:
        candidates += [
            (kind, 0, port, baud, f"{name} at {baud_label(baud)} baud")
            for baud in baud_rates_to_try(ctx, port, preferred=(DEFAULT_SERIAL_BAUD,))
            for kind, name in RANGEFINDER_SERIAL_CANDIDATES
        ]

    if not ctx.confirm(
        f"Start the rangefinder probe (up to {len(candidates)} attempts, a reboot each, about " "20-30 s each)?"
    ):
        ctx.log("Cancelled - no changes made.")
        return

    if port:
        ensure_port_free(ctx, port, SERIAL_PROTOCOL_RANGEFINDER)

    originals = {**snapshot(ctx, type_name, address_name), **_serial_snapshot(ctx, port)}
    with restore_on_failure(ctx, originals):
        for kind, address, serial_port, baud, name in candidates:
            ctx.log(f"Trying {name} ...")
            ctx.set_param(type_name, kind, reboot=True)
            if address:
                ctx.set_param(address_name, address, required=False)
            if serial_port:
                configure_serial(ctx, serial_port, SERIAL_PROTOCOL_RANGEFINDER, baud)
            action_reboot_flight_controller(ctx)
            if _wait_for(ctx, lambda: _rangefinder_reading(ctx) is not None):
                reading = _rangefinder_reading(ctx)
                if ctx.confirm(f"{name} answers: {_describe_reading(reading)}. Keep it?"):
                    ctx.set_values(rng_type=kind, **({"rng_address": address} if address else {}))
                    ctx.log(f"Rangefinder {selected_rangefinder(ctx)} set to {name}.")
                    return
            else:
                ctx.log(f"{name}: no reading.")
        ctx.log("No rangefinder kept - restoring the original settings.")
        restore(ctx, originals)


def _proximity_sectors(ctx) -> int:
    """How many different directions currently report distances (a proximity sensor fills several)."""
    return len({message.orientation for message in ctx.vehicle.distance_sensors()})


def _proximity_attempts(ctx, port: int) -> list:
    """(type, baud) to try: every type at the port's current rate, then at its usual rates, then at common rates."""
    current = baud_rates_to_try(ctx, port)[0]
    stages = [
        [(kind, current) for kind, _, _ in PROXIMITY_PROBE_TYPES],
        [(kind, baud) for kind, usual, _ in PROXIMITY_PROBE_TYPES for baud in usual],
        [(kind, baud) for baud in COMMON_PROBE_BAUDS for kind, _, _ in PROXIMITY_PROBE_TYPES],
    ]
    attempts = []
    for stage in stages:
        attempts += [attempt for attempt in stage if attempt not in attempts]
    return attempts


def action_probe_proximity(ctx):
    """Try the common serial proximity sensors and baud rates on the chosen port as the selected sensor."""
    port = chosen_serial_port(ctx, "prx_port")
    if not port:
        raise VehicleError("Choose the serial port the proximity sensor is connected to first.")
    ensure_port_free(ctx, port, SERIAL_PROTOCOL_LIDAR360)
    attempts = _proximity_attempts(ctx, port)

    if not ctx.confirm(
        f"Start the proximity sensor probe on SERIAL{port} (up to {len(attempts)} attempts, a reboot "
        "each, about 20-30 s each)?"
    ):
        ctx.log("Cancelled - no changes made.")
        return

    type_names = proximity_type_names(ctx)
    originals = {**snapshot(ctx, type_names), **_serial_snapshot(ctx, port)}
    with restore_on_failure(ctx, originals):
        names = {kind: name for kind, _, name in PROXIMITY_PROBE_TYPES}
        for kind, baud in attempts:
            name = f"{names[kind]} at {baud_label(baud)} baud"
            ctx.log(f"Trying {name} ...")
            ctx.set_param(type_names, kind, reboot=True)
            configure_serial(ctx, port, SERIAL_PROTOCOL_LIDAR360, baud)
            action_reboot_flight_controller(ctx)
            if _wait_for(ctx, lambda: _proximity_sectors(ctx) >= MIN_PROXIMITY_SECTORS):
                if ctx.confirm(f"{name} reports {_proximity_sectors(ctx)} directions. Keep it?"):
                    ctx.set_values(prx_type=kind, prx_baud=baud)
                    ctx.log(f"Proximity sensor {selected_proximity(ctx)} set to {name}.")
                    return
            else:
                ctx.log(f"{name}: no data.")
        ctx.log("No proximity sensor kept - restoring the original settings.")
        restore(ctx, originals)


def _has_rangefinders(ctx) -> bool:
    """The rangefinder section is shown."""
    return has("has_rangefinder")(ctx)


def _has_proximity_sensors(ctx) -> bool:
    """The proximity section is shown."""
    return has("has_proximity")(ctx)


def _does_apply(ctx) -> bool:
    """The step is shown when the vehicle has a rangefinder or a proximity sensor."""
    return _has_rangefinders(ctx) or _has_proximity_sensors(ctx)


STEP = Step(
    "distance_sensors",
    "Distance sensors & object avoidance",
    [
        value_field(
            "rng_overview", "Rangefinders", kind="display", default=rangefinder_overview, when=_has_rangefinders
        ),
        value_field(
            "rangefinder",
            "Rangefinder to configure",
            kind="choice",
            choices=[(f"Rangefinder {instance}", number) for number, instance in enumerate(RANGEFINDER_INSTANCES, 1)],
            default=lambda ctx: ctx.values.get("rangefinder") or 1,
            reload_on_change=True,
            when=_has_rangefinders,
        ),
        param_field(
            rangefinder_parameter("TYPE"),
            "Rangefinder type",
            key="rng_type",
            kind="choice",
            choices=RANGEFINDER_TYPE_CHOICES,
            reboot=True,
            refresh=True,
            when=_has_rangefinders,
            inline_actions=[Action("Probe", action_probe_rangefinder, armed=False, is_probe=True)],
        ),
        value_field(
            "rng_port",
            "Rangefinder serial port",
            kind="choice",
            choices=SERIAL_PORT_CHOICES,
            default=port_with_protocol(SERIAL_PROTOCOL_RANGEFINDER),
            when=lambda ctx: _has_rangefinders(ctx) and is_rangefinder_serial(ctx),
        ),
        param_field(
            rangefinder_parameter("ADDR"),
            "I2C address (decimal)",
            key="rng_address",
            kind="int",
            when=lambda ctx: _has_rangefinders(ctx) and is_rangefinder_i2c(ctx),
        ),
        param_field(
            rangefinder_parameter("ORIENT"),
            "Pointing direction",
            key="rng_orient",
            kind="choice",
            choices=RANGEFINDER_ORIENTATION_CHOICES,
            when=_has_rangefinders,
        ),
        param_field(
            rangefinder_parameter("MIN"), "Minimum reliable distance", key="rng_min", unit="m", when=_has_rangefinders
        ),
        param_field(
            rangefinder_parameter("MAX"), "Maximum reliable distance", key="rng_max", unit="m", when=_has_rangefinders
        ),
        param_field(
            rangefinder_parameter("MIN_CM"),
            "Minimum reliable distance",
            key="rng_min_cm",
            unit="cm",
            when=_has_rangefinders,
        ),
        param_field(
            rangefinder_parameter("MAX_CM"),
            "Maximum reliable distance",
            key="rng_max_cm",
            unit="cm",
            when=_has_rangefinders,
        ),
        param_field(
            rangefinder_parameter("POS_X"), "Position X (forward)", key="rng_pos_x", unit="m", when=_has_rangefinders
        ),
        param_field(
            rangefinder_parameter("POS_Y"), "Position Y (right)", key="rng_pos_y", unit="m", when=_has_rangefinders
        ),
        param_field(
            rangefinder_parameter("POS_Z"), "Position Z (down)", key="rng_pos_z", unit="m", when=_has_rangefinders
        ),
        value_field(
            "prx_overview", "Proximity sensors", kind="display", default=proximity_overview, when=_has_proximity_sensors
        ),
        value_field(
            "proximity",
            "Proximity sensor to configure",
            kind="choice",
            choices=[(f"Proximity sensor {instance}", instance) for instance in PROXIMITY_INSTANCES],
            default=lambda ctx: ctx.values.get("proximity") or 1,
            reload_on_change=True,
            when=_has_proximity_sensors,
        ),
        param_field(
            proximity_type_names,
            "Proximity sensor type",
            key="prx_type",
            kind="choice",
            choices=PROXIMITY_TYPE_CHOICES,
            reboot=True,
            refresh=True,
            when=_has_proximity_sensors,
            inline_actions=[Action("Probe", action_probe_proximity, armed=False, is_probe=True)],
        ),
        value_field(
            "prx_port",
            "Proximity sensor serial port",
            kind="choice",
            choices=SERIAL_PORT_CHOICES,
            default=port_with_protocol(SERIAL_PROTOCOL_LIDAR360),
            when=lambda ctx: _has_proximity_sensors(ctx) and is_proximity_serial(ctx),
        ),
        value_field(
            "prx_baud",
            "Serial baud rate",
            kind="choice",
            choices=SERIAL_BAUD_FULL_CHOICES,
            default=DEFAULT_SERIAL_BAUD,
            when=lambda ctx: _has_proximity_sensors(ctx) and is_proximity_serial(ctx),
        ),
        param_field(
            proximity_parameter("ORIENT"),
            "Mounting",
            key="prx_orient",
            kind="choice",
            choices=PROXIMITY_ORIENTATION_CHOICES,
            when=_has_proximity_sensors,
        ),
        param_field(
            proximity_parameter("YAW_CORR"),
            "Forward direction correction",
            key="prx_yaw_corr",
            unit="deg",
            when=_has_proximity_sensors,
        ),
        param_field(
            proximity_parameter("MIN"),
            "Minimum range (0 = sensor's own)",
            key="prx_min",
            unit="m",
            when=_has_proximity_sensors,
        ),
        param_field(
            proximity_parameter("MAX"),
            "Maximum range (0 = sensor's own)",
            key="prx_max",
            unit="m",
            when=_has_proximity_sensors,
        ),
    ]
    + [bit_field("AVOID_ENABLE", bit, label, key=f"avoid_source_{bit}") for bit, label in AVOID_SOURCE_BITS]
    + [
        param_field("AVOID_MARGIN", "Avoidance margin", unit="m"),
        param_field("AVOID_BEHAVE", "Avoidance behaviour", kind="choice", choices=AVOID_BEHAVE_CHOICES),
        param_field("AVOID_BACKUP_SPD", "Back away from obstacles at up to", unit="m/s"),
        param_field("AVOID_ACCEL_MAX", "Maximum avoidance acceleration (0 = no limit)", unit="m/s/s"),
    ],
    [Action("Show distances (10 s)", action_show_distances)],
    on_apply=apply_distance_sensors,
    applies=_does_apply,
)
