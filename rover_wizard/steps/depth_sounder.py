"""rover_wizard.steps.depth_sounder - the "Depth sounder" step of a boat: an echosounder is a rangefinder pointing down.

ArduPilot measures the water depth with any rangefinder whose orientation is "down" (RNGFNDx_ORIENT = 25); on a boat
it logs the depth with the position (DPTH) and reports it to the ground station (WATER_DEPTH). The settings of the
common echosounders come from their pages in ArduPilot's documentation.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..util.errors import VehicleError
from ..rover_defs import RANGEFINDER_INSTANCES, RANGEFINDER_ORIENT_DOWN, SERIAL_PROTOCOL_RANGEFINDER
from ..vehicle.boat_telemetry import water_depth
from .distance_sensors import CENTIMETRES_PER_METRE, RANGEFINDER_SERIAL_TYPES, RANGEFINDER_TYPE_CHOICES
from .util.boat_hardware import depth_rangefinder
from .util.rover_hardware import has
from .util.ports import (
    DEFAULT_SERIAL_BAUD,
    SERIAL_BAUD_FULL_CHOICES,
    SERIAL_PORT_CHOICES,
    chosen_serial_port,
    configure_serial,
    ensure_port_free,
    port_with_protocol,
)
from .util.probing import LIVE_DISPLAY_SECONDS
from ..model import Action, Step, param_field, value_field

# RNGFNDx_TYPE of most echosounders and of the Blue Robotics Ping
RANGEFINDER_TYPE_NMEA = 17
RANGEFINDER_TYPE_PING = 23


@dataclass(frozen=True)
class Echosounder:
    """An echosounder ArduPilot documents, with the settings its page gives."""

    name: str
    # RNGFNDx_TYPE
    type: int
    # SERIALx_BAUD
    baud: int
    # RNGFNDx_MIN / _MAX [m]
    min_m: float
    max_m: float


ECHOSOUNDERS = [
    Echosounder("Blue Robotics Ping", RANGEFINDER_TYPE_PING, 115, 0.3, 96),
    Echosounder("CEE-LINE", RANGEFINDER_TYPE_NMEA, 9, 0.5, 70),
    Echosounder("EchoLogger ECT400", RANGEFINDER_TYPE_NMEA, 115, 0.13, 100),
    Echosounder("Hondex", RANGEFINDER_TYPE_NMEA, 9, 0.13, 100),
    Echosounder("Kogger", RANGEFINDER_TYPE_NMEA, 115, 0.3, 50),
    Echosounder("Lowrance Elite Ti2", RANGEFINDER_TYPE_NMEA, 38, 0.13, 300),
    Echosounder("Lowrance HST-WSBL + NMEA digitizer", RANGEFINDER_TYPE_NMEA, 4, 0.13, 300),
    Echosounder("L04xMTW / GL04xMT", RANGEFINDER_TYPE_NMEA, 115, 0.05, 6),
    Echosounder("Airmar triducer (NMEA)", RANGEFINDER_TYPE_NMEA, 4, 0.5, 7),
]
# the choice for anything else: the fields are filled in by hand
OTHER_ECHOSOUNDER = -1
ECHOSOUNDER_CHOICES = [(sounder.name, number) for number, sounder in enumerate(ECHOSOUNDERS)] + [
    ("Other (settings below)", OTHER_ECHOSOUNDER)
]


def _first_free_rangefinder(ctx) -> str:
    """The first rangefinder instance not in use."""
    return next((i for i in RANGEFINDER_INSTANCES if not ctx.param_int(f"RNGFND{i}_TYPE")), RANGEFINDER_INSTANCES[0])


def selected_instance(ctx) -> str:
    """Instance ("1".."A") the fields describe: chosen in the form, else the depth sounder, else a free one."""
    chosen = ctx.values.get("depth_rangefinder")
    if chosen:
        return RANGEFINDER_INSTANCES[int(chosen) - 1]
    return depth_rangefinder(ctx) or _first_free_rangefinder(ctx)


def depth_parameter(suffix: str):
    """RNGFNDn_<suffix> of the selected instance."""
    return lambda ctx: f"RNGFND{selected_instance(ctx)}_{suffix}"


def _is_serial(ctx) -> bool:
    """The chosen type reads a serial port."""
    return ctx.form_int("depth_type", depth_parameter("TYPE")(ctx)) in RANGEFINDER_SERIAL_TYPES


def action_use_echosounder_settings(ctx):
    """Fill type, baud rate and range from the chosen echosounder's documentation page."""
    choice = int(ctx.number("depth_model", OTHER_ECHOSOUNDER))
    if choice == OTHER_ECHOSOUNDER:
        raise VehicleError("Choose an echosounder from the list first (or enter the settings yourself).")
    sounder = ECHOSOUNDERS[choice]
    ctx.set_values(
        depth_type=sounder.type,
        depth_baud=sounder.baud,
        depth_min=sounder.min_m,
        depth_max=sounder.max_m,
        depth_min_cm=round(sounder.min_m * CENTIMETRES_PER_METRE),
        depth_max_cm=round(sounder.max_m * CENTIMETRES_PER_METRE),
    )
    ctx.log(f"{sounder.name}: settings filled in - press 'Apply' to write them.")


def apply_depth_sounder(ctx) -> None:
    """Point the rangefinder down and set up its serial port."""
    instance = selected_instance(ctx)
    ctx.set_param(f"RNGFND{instance}_ORIENT", RANGEFINDER_ORIENT_DOWN)
    if _is_serial(ctx):
        port = chosen_serial_port(ctx, "depth_port")
        if not port:
            raise VehicleError("Choose the serial port the echosounder is connected to.")
        ensure_port_free(ctx, port, SERIAL_PROTOCOL_RANGEFINDER)
        configure_serial(ctx, port, SERIAL_PROTOCOL_RANGEFINDER, int(ctx.number("depth_baud", DEFAULT_SERIAL_BAUD)))


def describe_depth(ctx) -> str:
    """The depth reported now."""
    depth = water_depth(ctx.vehicle)
    if depth is None:
        return "No depth reported (type and port set? rebooted? sounder in the water?)"
    metres, temperature, healthy = depth
    return f"Depth {metres:.2f} m, water {temperature:.1f} deg C" + ("" if healthy else " (NOT healthy)")


def action_show_depth(ctx):
    """Log the depth once per second for 10 seconds."""
    for _ in range(LIVE_DISPLAY_SECONDS):
        ctx.log(describe_depth(ctx))
        ctx.sleep(1)


STEP = Step(
    "depth_sounder",
    "Depth sounder",
    [
        value_field(
            "depth_rangefinder",
            "Rangefinder used as depth sounder",
            kind="choice",
            choices=[(f"Rangefinder {instance}", number) for number, instance in enumerate(RANGEFINDER_INSTANCES, 1)],
            default=lambda ctx: RANGEFINDER_INSTANCES.index(selected_instance(ctx)) + 1,
            reload_on_change=True,
        ),
        value_field(
            "depth_model",
            "Echosounder",
            kind="choice",
            choices=ECHOSOUNDER_CHOICES,
            default=OTHER_ECHOSOUNDER,
            inline_actions=[Action("Fill in", action_use_echosounder_settings, armed=False)],
        ),
        param_field(
            depth_parameter("TYPE"),
            "Rangefinder type",
            key="depth_type",
            kind="choice",
            choices=RANGEFINDER_TYPE_CHOICES,
            reboot=True,
            refresh=True,
        ),
        value_field(
            "depth_port",
            "Serial port",
            kind="choice",
            choices=SERIAL_PORT_CHOICES,
            default=port_with_protocol(SERIAL_PROTOCOL_RANGEFINDER),
            when=_is_serial,
        ),
        value_field(
            "depth_baud",
            "Baud rate",
            kind="choice",
            choices=SERIAL_BAUD_FULL_CHOICES,
            default=lambda ctx: ctx.param_int(f"SERIAL{int(ctx.number('depth_port', 0))}_BAUD") or DEFAULT_SERIAL_BAUD,
            when=_is_serial,
        ),
        param_field(depth_parameter("MIN"), "Shallowest reliable depth", key="depth_min", unit="m"),
        param_field(depth_parameter("MAX"), "Deepest reliable depth", key="depth_max", unit="m"),
        param_field(depth_parameter("MIN_CM"), "Shallowest reliable depth", key="depth_min_cm", unit="cm"),
        param_field(depth_parameter("MAX_CM"), "Deepest reliable depth", key="depth_max_cm", unit="cm"),
        param_field(depth_parameter("POS_X"), "Position X (forward)", key="depth_pos_x", unit="m"),
        param_field(depth_parameter("POS_Y"), "Position Y (right)", key="depth_pos_y", unit="m"),
        param_field(depth_parameter("POS_Z"), "Position Z (down)", key="depth_pos_z", unit="m"),
    ],
    [Action("Show depth (10 s)", action_show_depth)],
    on_apply=apply_depth_sounder,
    applies=has("has_depth_sounder"),
)
