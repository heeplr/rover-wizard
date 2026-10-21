"""rover_wizard.vehicle.boat_simulator - a simulated boat (FRAME_CLASS Boat) for --demo-boat and the tests.

Besides the rover's hidden truths it hides a Torqeedo motor and an echosounder the wizard has to set up: the
Torqeedo only reports (ESC telemetry) once its type and its serial port (protocol, 19200 baud) are right, and the
depth only arrives (WATER_DEPTH, as Rover sends it for boats) from the hidden rangefinder pointing down. Like
ArduPilot, the Torqeedo's settings beyond its type appear only after a reboot with the type set.
"""

from __future__ import annotations

from types import SimpleNamespace

from ..rover_defs import FRAME_CLASS_BOAT, RANGEFINDER_ORIENT_DOWN, SERVO_FUNCTION_THROTTLE
from ..boat_defs import MISSION_DONE_LOITER, SERIAL_PROTOCOL_TORQEEDO, TORQEEDO_SERIAL_BAUD
from ..mavlink_defs import MAV_PARAM_TYPE_REAL32, MAV_TYPE_SURFACE_BOAT, PARAM_INDEX_UNKNOWN
from .rover_simulator import SimVehicle

# ESC_TELEMETRY_1_TO_4 carries four motors; the Torqeedo reports on its output's index
ESC_TELEMETRY_SLOTS = 4
CENTI = 100


class SimBoat(SimVehicle):
    """A simulated boat; see the module docstring."""

    # [m] water below the simulated echosounder, and its temperature [deg C]
    TRUE_DEPTH = 12.5
    WATER_TEMPERATURE = 14.0
    # what the simulated Torqeedo reports
    TORQEEDO_RPM = 850
    TORQEEDO_VOLTS = 29.4
    TORQEEDO_AMPS = 12.5
    TORQEEDO_TEMPERATURE = 31
    # firmware before 4.7 names the (only) Torqeedo's settings TRQD_*
    TORQEEDO_PREFIX = "TRQD_"

    def __init__(self, skid: bool = False, wired_wrong: bool = False) -> None:
        """A boat with no Torqeedo and no echosounder fitted (tests set true_torqeedo / true_rangefinder)."""
        # (TRQD_TYPE, serial port) of a Torqeedo that answers when set up like that (None = not fitted)
        self.true_torqeedo = None
        super().__init__(skid=skid, wired_wrong=wired_wrong)
        self.vehicle_type = MAV_TYPE_SURFACE_BOAT

    def _build_params(self) -> None:
        """The rover's parameters, with Rover's boat defaults (Rover::init_ardupilot) and the boat-only ones."""
        super()._build_params()
        self._params["FRAME_CLASS"] = float(FRAME_CLASS_BOAT)
        self._add_params(
            {
                "LOIT_TYPE": 0,
                "LOIT_RADIUS": 2,
                "LOIT_SPEED_GAIN": 0.5,
                "MIS_DONE_BEHAVE": MISSION_DONE_LOITER,
                "MOT_VEC_ANGLEMAX": 0,
                # AP_GPS's default: Airborne 4G
                "GPS_NAVFILTER": 8,
                f"{self.TORQEEDO_PREFIX}TYPE": 0,
            }
        )

    def reboot(self, progress=None) -> None:
        """Like ArduPilot: the Torqeedo's settings exist only after a reboot with its type set."""
        if self.get_int(f"{self.TORQEEDO_PREFIX}TYPE") and not self.has_parameter(f"{self.TORQEEDO_PREFIX}SLEW_TIME"):
            extra = {"ONOFF_PIN": -1, "DE_PIN": -1, "OPTIONS": 1, "POWER": 100, "SLEW_TIME": 2.0, "DIR_DELAY": 1.0}
            for suffix, value in extra.items():
                self._store_param(
                    self.TORQEEDO_PREFIX + suffix,
                    float(value),
                    MAV_PARAM_TYPE_REAL32,
                    PARAM_INDEX_UNKNOWN,
                    self._param_total,
                )
        super().reboot(progress)

    def _publish_telemetry(self, timestamp: float) -> None:
        """The rover's telemetry, plus the depth and the Torqeedo's reports when they are set up right."""
        super()._publish_telemetry(timestamp)
        instance = self._answering_rangefinder()
        if (
            instance is not None
            and self.get_int("FRAME_CLASS") == FRAME_CLASS_BOAT
            and self.get_int(f"RNGFND{instance}_ORIENT") == RANGEFINDER_ORIENT_DOWN
        ):
            self._store_message(
                "WATER_DEPTH",
                SimpleNamespace(distance=self.TRUE_DEPTH, temperature=self.WATER_TEMPERATURE, healthy=1),
                timestamp,
            )
        if self._is_torqeedo_answering():
            self._store_message("ESC_TELEMETRY_1_TO_4", self._torqeedo_telemetry(), timestamp)

    def _is_torqeedo_answering(self) -> bool:
        """The hidden Torqeedo is fitted, its type is right and its serial port talks the Torqeedo protocol."""
        if self.true_torqeedo is None:
            return False
        kind, port = self.true_torqeedo
        return self.get_int(f"{self.TORQEEDO_PREFIX}TYPE") == kind and self._is_serial_port_configured(
            port, SERIAL_PROTOCOL_TORQEEDO, TORQEEDO_SERIAL_BAUD
        )

    def _torqeedo_telemetry(self) -> SimpleNamespace:
        """ESC_TELEMETRY_1_TO_4 with the Torqeedo on its throttle output's slot."""
        slot = ((self._output_with_function(SERVO_FUNCTION_THROTTLE) or 1) - 1) % ESC_TELEMETRY_SLOTS

        def values(value):
            """The value in the Torqeedo's slot, 0 elsewhere."""
            return [value if index == slot else 0 for index in range(ESC_TELEMETRY_SLOTS)]

        return SimpleNamespace(
            rpm=values(self.TORQEEDO_RPM),
            voltage=values(round(self.TORQEEDO_VOLTS * CENTI)),
            current=values(round(self.TORQEEDO_AMPS * CENTI)),
            temperature=values(self.TORQEEDO_TEMPERATURE),
        )
