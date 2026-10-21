"""rover_wizard.steps.wind_vane - the "Wind vane" step of a sailing vehicle: the wind direction and speed sensors,
their calibration and a live readout.

AP_WindVane knows the apparent wind (relative to the vehicle) from a direction sensor (WNDVN_TYPE) and optionally a
speed sensor (WNDVN_SPEED_TYPE), and works out the true wind from it. WNDVN_TYPE switches the wind vane on; its
other settings exist only after a reboot. Calibrating (WNDVN_CAL) runs on the flight controller, disarmed.
"""

from __future__ import annotations

import math

from ..util.errors import VehicleError
from ..util.timebase import now
from ..boat_defs import (
    RC_OPTION_WIND_VANE_DIRECTION_OFFSET,
    SERIAL_PROTOCOL_WIND_VANE,
    WIND_SPEED_TYPE_MODERN_DEVICES,
    WIND_SPEED_TYPE_NMEA,
    WIND_SPEED_TYPE_NONE,
    WIND_VANE_CALIBRATE_DIRECTION,
    WIND_VANE_CALIBRATE_SPEED,
    WIND_VANE_DIRECTION_CALIBRATION_SECONDS,
    WIND_VANE_TYPE_ANALOG,
    WIND_VANE_TYPE_NMEA,
    WIND_VANE_TYPE_NONE,
    WIND_VANE_TYPE_RC_OFFSET_HOME_HEADING,
)
from ..vehicle.boat_telemetry import apparent_wind, true_wind
from .util.rover_hardware import has
from .util.outputs import RC_SWITCH_CHOICES, assign_rc_switch, rc_channel_with_option
from .util.ports import (
    SERIAL_BAUD_FULL_CHOICES,
    SERIAL_PORT_CHOICES,
    chosen_serial_port,
    configure_serial,
    ensure_port_free,
    port_with_protocol,
)
from .util.probing import LIVE_DISPLAY_SECONDS
from ..model import Action, Step, param_field, value_field

WIND_VANE_TYPE_CHOICES = [
    ("None", WIND_VANE_TYPE_NONE),
    ("No sensor: wind from the heading when armed", 1),
    ("No sensor: heading when armed, offset by an RC knob", WIND_VANE_TYPE_RC_OFFSET_HOME_HEADING),
    ("Analog wind vane (potentiometer)", WIND_VANE_TYPE_ANALOG),
    ("NMEA 0183 wind sensor (serial)", WIND_VANE_TYPE_NMEA),
]

WIND_SPEED_TYPE_CHOICES = [
    ("None", WIND_SPEED_TYPE_NONE),
    ("Airspeed sensor (pitot)", 1),
    ("Modern Devices Wind Sensor rev. P", WIND_SPEED_TYPE_MODERN_DEVICES),
    ("Cup anemometer on RPM sensor 1", 3),
    ("NMEA 0183 wind sensor (serial)", WIND_SPEED_TYPE_NMEA),
]

# SERIALx_BAUD 4800: the NMEA 0183 standard rate
NMEA_DEFAULT_BAUD = 4

# how long a calibration may take beyond its nominal time before the wizard gives up [s]
CALIBRATION_MARGIN_SECONDS = 10
# a speed calibration only stores the voltage it reads now [s]
SPEED_CALIBRATION_SECONDS = 2
# how often the wizard asks whether WNDVN_CAL is back to 0 [s]
CALIBRATION_POLL_SECONDS = 1.0
# readings averaged for the headwind offset
OFFSET_SAMPLES = 5
FULL_CIRCLE_DEG = 360


def are_wind_vane_settings_loaded(ctx) -> bool:
    """ArduPilot shows the wind vane settings: WNDVN_TYPE is set and the flight controller has rebooted since."""
    return ctx.vehicle.has_parameter("WNDVN_SPEED_TYPE")


def _direction_type(ctx) -> int:
    """WNDVN_TYPE as chosen in the form."""
    return ctx.form_int("WNDVN_TYPE")


def _speed_type(ctx) -> int:
    """WNDVN_SPEED_TYPE as chosen in the form."""
    return ctx.form_int("WNDVN_SPEED_TYPE") if are_wind_vane_settings_loaded(ctx) else WIND_SPEED_TYPE_NONE


def _is_analog(ctx) -> bool:
    """An analog wind vane: pins, voltages, calibration."""
    return are_wind_vane_settings_loaded(ctx) and _direction_type(ctx) == WIND_VANE_TYPE_ANALOG


def _measures_direction(ctx) -> bool:
    """A sensor measures the direction (analog or NMEA): it has a headwind offset."""
    return are_wind_vane_settings_loaded(ctx) and _direction_type(ctx) in (WIND_VANE_TYPE_ANALOG, WIND_VANE_TYPE_NMEA)


def _is_nmea(ctx) -> bool:
    """An NMEA sensor gives the direction or the speed: it needs a serial port."""
    return _direction_type(ctx) == WIND_VANE_TYPE_NMEA or _speed_type(ctx) == WIND_SPEED_TYPE_NMEA


def _is_modern_devices(ctx) -> bool:
    """The Modern Devices hot-wire sensor: analog pins and a zero-wind offset."""
    return _speed_type(ctx) == WIND_SPEED_TYPE_MODERN_DEVICES


def _has_speed_sensor(ctx) -> bool:
    """Some sensor measures the wind speed."""
    return _speed_type(ctx) != WIND_SPEED_TYPE_NONE


def reboot_note(ctx) -> str:
    """Shown until the wind vane settings appear."""
    if ctx.param_int("WNDVN_TYPE"):
        return "WNDVN_TYPE is set - reboot the flight controller (Reboot button) to see the wind vane settings."
    return "Choose the wind vane type and press Apply, then reboot."


def apply_wind_vane(ctx) -> None:
    """Serial port of an NMEA sensor; the RC knob of the 'heading when armed' type."""
    if _is_nmea(ctx):
        port = chosen_serial_port(ctx, "wind_port")
        if not port:
            raise VehicleError("Choose the serial port the NMEA wind sensor is connected to.")
        ensure_port_free(ctx, port, SERIAL_PROTOCOL_WIND_VANE)
        configure_serial(ctx, port, SERIAL_PROTOCOL_WIND_VANE, int(ctx.number("wind_baud", NMEA_DEFAULT_BAUD)))
    if _direction_type(ctx) == WIND_VANE_TYPE_RC_OFFSET_HOME_HEADING:
        assign_rc_switch(ctx, RC_OPTION_WIND_VANE_DIRECTION_OFFSET, int(ctx.number("wind_offset_channel", 0)))


# ============================================================================ calibration
def _calibrate(ctx, mode: int, seconds: float) -> None:
    """Start a WNDVN_CAL calibration and wait until the flight controller has finished it (it resets WNDVN_CAL).

    WNDVN_CAL is a command, not a setting: written directly, so it isn't listed as a change to revert."""
    if ctx.vehicle.armed:
        raise VehicleError("The wind vane only calibrates while disarmed.")
    marker = ctx.vehicle.status_text_marker()
    ctx.vehicle.set_param("WNDVN_CAL", mode)
    deadline = now() + seconds + CALIBRATION_MARGIN_SECONDS
    while now() < deadline:
        ctx.sleep(CALIBRATION_POLL_SECONDS)
        ctx.flush_texts()
        if not ctx.vehicle.read_param("WNDVN_CAL"):
            break
    else:
        raise VehicleError("The flight controller didn't finish the calibration - is the wind vane type set up?")
    texts = " ".join(ctx.vehicle.status_texts_since(marker))
    if "failed" in texts:
        raise VehicleError("Calibration failed - see the flight controller's message above.")


def action_calibrate_direction(ctx):
    """Analog vane: the flight controller records the lowest and highest voltage while the vane is turned for 30 s
    (AP_WindVane_Analog::calibrate) and stores them in WNDVN_DIR_V_MIN / _MAX."""
    ctx.pause(ctx.prompt("wind_vane.rotate_vane"))
    _calibrate(ctx, WIND_VANE_CALIBRATE_DIRECTION, WIND_VANE_DIRECTION_CALIBRATION_SECONDS)
    low, high = (ctx.vehicle.read_param(f"WNDVN_DIR_V_{end}") for end in ("MIN", "MAX"))
    ctx.set_values(WNDVN_DIR_V_MIN=low, WNDVN_DIR_V_MAX=high)
    ctx.log(f"Wind vane voltage range {low:.2f} .. {high:.2f} V stored by the flight controller.")


def action_calibrate_speed(ctx):
    """Modern Devices sensor: the flight controller stores the voltage it reads now as zero wind (WNDVN_SPEED_OFS)."""
    ctx.pause(ctx.prompt("wind_vane.shelter_sensor"))
    _calibrate(ctx, WIND_VANE_CALIBRATE_SPEED, SPEED_CALIBRATION_SECONDS)
    offset = ctx.vehicle.read_param("WNDVN_SPEED_OFS")
    ctx.set_values(WNDVN_SPEED_OFS=offset)
    ctx.log(f"Zero-wind voltage {offset:.2f} V stored by the flight controller.")


def _mean_angle(degrees: list) -> float:
    """Mean of angles [deg, -180..180] (averaged as directions, so -179 and 179 give 180, not 0)."""
    x = sum(math.cos(math.radians(angle)) for angle in degrees)
    y = sum(math.sin(math.radians(angle)) for angle in degrees)
    return math.degrees(math.atan2(y, x))


def action_set_headwind_offset(ctx):
    """With the vane pointing at the bow, the apparent wind direction should read 0: correct WNDVN_DIR_OFS by what it
    reads now (the wiki's procedure for analog and NMEA vanes)."""
    ctx.pause(ctx.prompt("wind_vane.point_to_bow"))
    readings = []
    while len(readings) < OFFSET_SAMPLES:
        # readings from after the vane was put in place (the direction is filtered, WNDVN_DIR_FILT)
        ctx.sleep(CALIBRATION_POLL_SECONDS)
        wind = apparent_wind(ctx.vehicle)
        if wind is None:
            raise VehicleError(
                "No apparent wind reported - is the wind vane set up and the flight controller rebooted?"
            )
        readings.append(wind[0])
    reading = _mean_angle(readings)
    offset = (ctx.param_value("WNDVN_DIR_OFS", 0) - reading) % FULL_CIRCLE_DEG
    ctx.log(f"The vane reads {reading:+.0f} deg pointing at the bow -> headwind offset {offset:.0f} deg.")
    ctx.set_values(WNDVN_DIR_OFS=round(offset))
    ctx.log("Press 'Apply' to write it.")


def describe_wind(vehicle) -> str:
    """One line: the apparent and the true wind."""
    apparent, true = apparent_wind(vehicle), true_wind(vehicle)
    if apparent is None and true is None:
        return "No wind reported (wind vane type set? rebooted?)"
    parts = []
    if apparent is not None:
        parts.append(f"apparent: from {apparent[0]:+.0f} deg off the bow at {apparent[1]:.1f} m/s")
    if true is not None:
        parts.append(f"true: from {true[0]:.0f} deg at {true[1]:.1f} m/s")
    return "Wind " + ", ".join(parts)


def action_show_wind(ctx):
    """Log the wind once per second for 10 seconds."""
    for _ in range(LIVE_DISPLAY_SECONDS):
        ctx.log(describe_wind(ctx.vehicle))
        ctx.sleep(1)


STEP = Step(
    "wind_vane",
    "Wind vane",
    [
        param_field(
            "WNDVN_TYPE",
            "Wind direction from",
            kind="choice",
            choices=WIND_VANE_TYPE_CHOICES,
            reboot=True,
            refresh=True,
        ),
        value_field(
            "wind_reboot",
            "Wind vane settings",
            kind="display",
            default=reboot_note,
            when=lambda ctx: not are_wind_vane_settings_loaded(ctx),
        ),
        value_field(
            "wind_offset_channel",
            "RC knob offsetting the wind direction",
            kind="choice",
            choices=RC_SWITCH_CHOICES,
            default=lambda ctx: rc_channel_with_option(ctx, RC_OPTION_WIND_VANE_DIRECTION_OFFSET),
            when=lambda ctx: _direction_type(ctx) == WIND_VANE_TYPE_RC_OFFSET_HOME_HEADING,
        ),
        param_field("WNDVN_DIR_PIN", "Direction: analog input pin", kind="int", when=_is_analog),
        param_field("WNDVN_DIR_V_MIN", "Direction: lowest voltage", unit="V", when=_is_analog),
        param_field("WNDVN_DIR_V_MAX", "Direction: highest voltage", unit="V", when=_is_analog),
        param_field("WNDVN_DIR_DZ", "Direction: dead zone of the potentiometer", unit="deg", when=_is_analog),
        param_field("WNDVN_DIR_OFS", "Headwind offset", unit="deg", when=_measures_direction),
        param_field("WNDVN_DIR_FILT", "Direction filter (-1 = off)", unit="Hz", when=are_wind_vane_settings_loaded),
        param_field(
            "WNDVN_SPEED_TYPE",
            "Wind speed from",
            kind="choice",
            choices=WIND_SPEED_TYPE_CHOICES,
            reboot=True,
            refresh=True,
            when=are_wind_vane_settings_loaded,
        ),
        param_field("WNDVN_SPEED_PIN", "Speed: analog input pin", kind="int", when=_is_modern_devices),
        param_field("WNDVN_TEMP_PIN", "Speed: temperature pin (-1 = none)", kind="int", when=_is_modern_devices),
        param_field("WNDVN_SPEED_OFS", "Speed: voltage at zero wind", unit="V", when=_is_modern_devices),
        param_field(
            "WNDVN_SPEED_MIN", "Ignore the direction below this wind speed", unit="m/s", when=_has_speed_sensor
        ),
        param_field("WNDVN_SPEED_FILT", "Speed filter (-1 = off)", unit="Hz", when=_has_speed_sensor),
        param_field("WNDVN_TRUE_FILT", "True wind filter (-1 = off)", unit="Hz", when=are_wind_vane_settings_loaded),
        value_field(
            "wind_port",
            "NMEA sensor serial port",
            kind="choice",
            choices=SERIAL_PORT_CHOICES,
            default=port_with_protocol(SERIAL_PROTOCOL_WIND_VANE),
            when=_is_nmea,
        ),
        value_field(
            "wind_baud",
            "NMEA sensor baud rate",
            kind="choice",
            choices=SERIAL_BAUD_FULL_CHOICES,
            default=lambda ctx: ctx.param_int(f"SERIAL{port_with_protocol(SERIAL_PROTOCOL_WIND_VANE)(ctx)}_BAUD")
            or NMEA_DEFAULT_BAUD,
            when=_is_nmea,
        ),
    ],
    [
        Action("Calibrate direction (30 s)", action_calibrate_direction, armed=False, when=_is_analog),
        Action("Calibrate zero wind speed", action_calibrate_speed, armed=False, when=_is_modern_devices),
        Action("Set headwind offset", action_set_headwind_offset, armed=False, when=_measures_direction),
        Action("Show wind (10 s)", action_show_wind, when=are_wind_vane_settings_loaded),
    ],
    on_apply=apply_wind_vane,
    applies=has("has_sails"),
)
