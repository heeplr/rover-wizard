"""rover_wizard.vehicle.sail_simulator - sailing for the simulated vehicles: SimSailboat and SimLandYacht.

SailSimulation adds what a sailing vehicle has to a simulated boat or rover: the sail settings (they appear after a
reboot with SAIL_ENABLE set, like ArduPilot's), the sail motor test, and a wind vane the wizard has to set up. The
wind blows steadily; an analog vane is mounted turned by TRUE_VANE_ZERO_DEG, which WNDVN_DIR_OFS must take out, and
its voltage range is found by the direction calibration (WNDVN_CAL = 1, AP_WindVane_Analog: 30 s).
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Optional

from ..util import timebase
from ..boat_defs import (
    ANGLE_SAIL_FUNCTIONS,
    APPARENT_WIND_DIRECTION_NAME,
    APPARENT_WIND_SPEED_NAME,
    MOTOR_TEST_INSTANCE_MAINSAIL,
    SAIL_FUNCTIONS,
    SAIL_TEST_PERCENT_MAX,
    WIND_VANE_CALIBRATE_DIRECTION,
    WIND_VANE_CALIBRATE_SPEED,
    WIND_VANE_DIRECTION_CALIBRATION_SECONDS,
    WIND_VANE_TYPE_ANALOG,
)
from ..mavlink_defs import MAV_PARAM_TYPE_REAL32, PARAM_INDEX_UNKNOWN
from .boat_simulator import SimBoat
from .link import named_value_key
from .rover_simulator import SimVehicle

HALF_CIRCLE_DEG = 180
FULL_CIRCLE_DEG = 360

# ArduPilot's sailing settings, which appear once SAIL_ENABLE is set and the vehicle rebooted (sailboat.cpp defaults)
SAIL_SETTINGS = {
    "SAIL_ANGLE_MIN": 0,
    "SAIL_ANGLE_MAX": 90,
    "SAIL_ANGLE_IDEAL": 25,
    "SAIL_HEEL_MAX": 15,
    "SAIL_NO_GO_ANGLE": 45,
    "SAIL_WNDSPD_MIN": 0,
    "SAIL_XTRACK_MAX": 10,
    "SAIL_LOIT_RADIUS": 5,
}
# ... and the wind vane's, once WNDVN_TYPE is set (AP_WindVane.cpp defaults)
WIND_VANE_SETTINGS = {
    "WNDVN_DIR_PIN": -1,
    "WNDVN_DIR_V_MIN": 0,
    "WNDVN_DIR_V_MAX": 3.3,
    "WNDVN_DIR_OFS": 0,
    "WNDVN_DIR_FILT": 0.5,
    "WNDVN_CAL": 0,
    "WNDVN_DIR_DZ": 0,
    "WNDVN_SPEED_MIN": 0,
    "WNDVN_SPEED_TYPE": 0,
    "WNDVN_SPEED_PIN": -1,
    "WNDVN_TEMP_PIN": -1,
    "WNDVN_SPEED_OFS": 1.346,
    "WNDVN_SPEED_FILT": 2,
    "WNDVN_TRUE_FILT": 10,
}
# [s] a speed calibration only stores what it reads now
SPEED_CALIBRATION_SECONDS = 1.0


def _wrap_180(degrees: float) -> float:
    """An angle in -180..180 deg."""
    return (degrees + HALF_CIRCLE_DEG) % FULL_CIRCLE_DEG - HALF_CIRCLE_DEG


class SailSimulation:
    """Mixin for a simulated vehicle with sails and a wind vane; see the module docstring."""

    # the steady wind: where it comes from [deg] and its speed [m/s]
    TRUE_WIND_FROM_DEG = 270.0
    TRUE_WIND_SPEED = 4.0
    # the analog vane reads this much too far clockwise until WNDVN_DIR_OFS corrects it
    TRUE_VANE_ZERO_DEG = 40.0
    # its real voltage range and the Modern Devices sensor's zero-wind voltage
    TRUE_VANE_VOLTS = (0.21, 3.12)
    TRUE_SPEED_SENSOR_ZERO_VOLTS = 1.28

    def __init__(self, *args, sails_enabled: bool = True, **kwargs) -> None:
        """A sailing vehicle; sails_enabled=False starts with SAIL_ENABLE = 0 (as a fresh rover)."""
        self.sails_enabled_at_start = sails_enabled
        # where the vane is held relative to the bow [deg] (None: it follows the wind)
        self.held_vane_deg: Optional[float] = None
        # (WNDVN_CAL value, time it finishes) of a running calibration
        self._wind_calibration = None
        super().__init__(*args, **kwargs)

    def _build_params(self) -> None:
        """The vehicle's parameters plus SAIL_ENABLE, the wind vane type and the sail heel controller."""
        super()._build_params()
        self._add_params(
            {
                "SAIL_ENABLE": int(self.sails_enabled_at_start),
                "WNDVN_TYPE": 0,
                "ATC_SAIL_P": 1.0,
                "ATC_SAIL_I": 0.1,
                "ATC_SAIL_IMAX": 1.0,
            }
        )
        if self.sails_enabled_at_start:
            self._add_params(SAIL_SETTINGS)

    def _reveal(self, settings: dict) -> None:
        """Parameters appearing after a reboot (keeping their values if they exist already)."""
        for name, value in settings.items():
            if not self.has_parameter(name):
                self._store_param(name, float(value), MAV_PARAM_TYPE_REAL32, PARAM_INDEX_UNKNOWN, self._param_total)

    def reboot(self, progress=None) -> None:
        """Like ArduPilot: sailing and wind vane settings exist only after a reboot with SAIL_ENABLE / WNDVN_TYPE."""
        if self.get_int("SAIL_ENABLE"):
            self._reveal(SAIL_SETTINGS)
        if self.get_int("WNDVN_TYPE"):
            self._reveal(WIND_VANE_SETTINGS)
        super().reboot(progress)

    def set_param(self, name: str, value: float) -> float:
        """WNDVN_CAL starts a calibration (the flight controller resets it when done)."""
        stored = super().set_param(name, value)
        if name == "WNDVN_CAL" and value:
            seconds = (
                WIND_VANE_DIRECTION_CALIBRATION_SECONDS
                if int(value) == WIND_VANE_CALIBRATE_DIRECTION
                else SPEED_CALIBRATION_SECONDS
            )
            self._wind_calibration = (int(value), timebase.now() + seconds)
        return stored

    # ------------------------------------------------------------------ sail motor test
    def _motor_test_output(self, instance: int):
        """The sail motor test drives whichever output has a sail function."""
        if instance == MOTOR_TEST_INSTANCE_MAINSAIL:
            return next(filter(None, map(self._output_with_function, SAIL_FUNCTIONS)), None)
        return super()._motor_test_output(instance)

    def _pwm_for_percent(self, output: int, percent: float) -> int:
        """A sail winch is a range output: 0 % = MIN .. 100 % = MAX (reversed: the other way round)."""
        function = self.get_int(f"SERVO{output}_FUNCTION")
        if function not in SAIL_FUNCTIONS or function in ANGLE_SAIL_FUNCTIONS:
            return super()._pwm_for_percent(output, percent)
        low, high = self.get(f"SERVO{output}_MIN"), self.get(f"SERVO{output}_MAX")
        fraction = percent / SAIL_TEST_PERCENT_MAX
        if self.get(f"SERVO{output}_REVERSED"):
            fraction = 1 - fraction
        return int(low + fraction * (high - low))

    # ------------------------------------------------------------------ wind vane
    def _publish_telemetry(self, timestamp: float) -> None:
        """The vehicle's telemetry, the wind (WIND, and the apparent wind as named values) and calibrations."""
        super()._publish_telemetry(timestamp)
        self._advance_wind_calibration(timestamp)
        if not self.get_int("WNDVN_TYPE") or not self.has_parameter("WNDVN_DIR_OFS"):
            return
        heading = math.degrees(self.heading)
        actual = _wrap_180(self.TRUE_WIND_FROM_DEG - heading) if self.held_vane_deg is None else self.held_vane_deg
        reported = actual
        if self.get_int("WNDVN_TYPE") == WIND_VANE_TYPE_ANALOG:
            reported = _wrap_180(actual + self.TRUE_VANE_ZERO_DEG + self.get("WNDVN_DIR_OFS"))
        self._store_message(
            "WIND",
            SimpleNamespace(direction=(heading + reported) % FULL_CIRCLE_DEG, speed=self.TRUE_WIND_SPEED, speed_z=0.0),
            timestamp,
        )
        for name, value in ((APPARENT_WIND_DIRECTION_NAME, reported), (APPARENT_WIND_SPEED_NAME, self.TRUE_WIND_SPEED)):
            self._store_message(named_value_key(name), SimpleNamespace(name=name, value=value), timestamp)

    def _advance_wind_calibration(self, timestamp: float) -> None:
        """Finish a running WNDVN_CAL calibration like AP_WindVane: store the result, report it, reset WNDVN_CAL."""
        if self._wind_calibration is None or timestamp < self._wind_calibration[1]:
            return
        mode = self._wind_calibration[0]
        self._wind_calibration = None
        if mode == WIND_VANE_CALIBRATE_DIRECTION and self.get_int("WNDVN_TYPE") == WIND_VANE_TYPE_ANALOG:
            low, high = self.TRUE_VANE_VOLTS
            super().set_param("WNDVN_DIR_V_MIN", low)
            super().set_param("WNDVN_DIR_V_MAX", high)
            self._add_status_text(f"WindVane: Calibration complete (volt min:{low:.1f} max:{high:.1f})")
        elif mode == WIND_VANE_CALIBRATE_SPEED and self.get_int("WNDVN_SPEED_TYPE"):
            super().set_param("WNDVN_SPEED_OFS", self.TRUE_SPEED_SENSOR_ZERO_VOLTS)
            self._add_status_text(
                f"WindVane: rev P. zero wind voltage offset set to {self.TRUE_SPEED_SENSOR_ZERO_VOLTS:.1f}"
            )
        else:
            self._add_status_text("WindVane: No cal required")
        super().set_param("WNDVN_CAL", 0)


class SimSailboat(SailSimulation, SimBoat):
    """A simulated sailing boat."""


class SimLandYacht(SailSimulation, SimVehicle):
    """A simulated land yacht: a rover with sails."""
