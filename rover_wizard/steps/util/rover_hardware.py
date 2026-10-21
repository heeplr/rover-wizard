"""rover_wizard.steps.util.rover_hardware - optional hardware of an ArduPilot Rover - land vehicle or boat: what the
vehicle has (detected from its parameters, or the pilot's answer); decides which steps, fields and actions are shown.
The detectors of boat and sailing hardware are in boat_hardware."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
from collections.abc import Callable
from ...rover_defs import (
    RC_CHANNELS,
    RC_OPTION_RELAY,
    RELAYS,
    RPM_SENSORS,
    SERVO_FUNCTION_GPIO,
    SERVO_FUNCTION_GRIPPER,
    SERVO_FUNCTION_RCIN_BASE,
    SERVO_FUNCTION_RC_PASSTHRU,
    SERVO_OUTPUTS,
)
from ...mavlink_defs import GPS_FIX_TYPE_NO_FIX
from .outputs import SPRAYER_SERVO_FUNCTIONS, output_function
from .boat_hardware import (
    TORQEEDO_TYPE_PARAMETER,
    are_sails_enabled,
    depth_rangefinder,
    is_motor_assigned,
    is_torqeedo_configured,
)


def _parameter_nonzero(names, default=0) -> Callable:
    """Detector: the hardware counts as present when its type/enable parameter is not 0."""
    return lambda ctx: bool(ctx.param_int(names, default))


_RC_DRIVEN_FUNCTIONS = frozenset(
    [SERVO_FUNCTION_RC_PASSTHRU] + [SERVO_FUNCTION_RCIN_BASE + channel for channel in RC_CHANNELS]
)


def _does_any_output_have_function(ctx, functions) -> bool:
    """Detector: some servo output has one of these SERVOx_FUNCTION values."""
    return any(output_function(ctx, output) in functions for output in SERVO_OUTPUTS)


def _are_relays_configured(ctx) -> bool:
    """Detector: a relay is enabled, switched by RC, or an output is set up as a GPIO pin."""
    relay_enabled = any(ctx.param_int(f"RELAY{relay}_FUNCTION") for relay in RELAYS)
    relay_switch = any(ctx.param_int(f"RC{channel}_OPTION") in RC_OPTION_RELAY.values() for channel in RC_CHANNELS)
    return relay_enabled or relay_switch or _does_any_output_have_function(ctx, {SERVO_FUNCTION_GPIO})


# OptionalHardware.frame: only offered on land vehicles / on boats (None: on both)
LAND = "land"
WATER = "water"


@dataclass(frozen=True)
class OptionalHardware:
    """A piece of hardware the vehicle may or may not have."""

    # store/field key, e.g. "has_gps"
    key: str
    # checkbox label
    label: str
    # ctx -> bool: does the vehicle's configuration say it is there?
    detect: Callable
    # ((parameter names, value), ...) written when the box is unchecked
    disable: tuple = ()
    # ((parameter names, value), ...) written when the box is checked (a feature ArduPilot has to switch on first)
    enable: tuple = ()
    # LAND / WATER: only offered for that kind of vehicle (FRAME_CLASS Rover / Boat)
    frame: Optional[str] = None
    # key of another item this one only exists with (e.g. a motor besides sails)
    needs: Optional[str] = None

    def is_offered(self, boat: bool, has_needed: bool) -> bool:
        """Whether the item makes sense on this vehicle: the right kind of vehicle, and what it needs is there."""
        right_frame = self.frame is None or self.frame == (WATER if boat else LAND)
        return right_frame and (self.needs is None or has_needed)


# Device instances ArduPilot Rover offers (from its parameter metadata): a device counts as configured when ANY of
# its instances is. Tuples hold the current name and an older firmware's name of the same parameter.
GPS_TYPE_PARAMETERS = (("GPS1_TYPE", "GPS_TYPE"), ("GPS2_TYPE", "GPS_TYPE2"))


COMPASS_DEVICE_ID_PARAMETERS = ("COMPASS_DEV_ID",) + tuple(f"COMPASS_DEV_ID{number}" for number in range(2, 9))


BATTERY_MONITOR_PARAMETERS = ("BATT_MONITOR",) + tuple(f"BATT{instance}_MONITOR" for instance in "23456789ABCDEFG")


WHEEL_ENCODER_TYPE_PARAMETERS = ("WENC_TYPE", "WENC2_TYPE")


RANGEFINDER_TYPE_PARAMETERS = tuple(f"RNGFND{instance}_TYPE" for instance in "123456789A")


PROXIMITY_TYPE_PARAMETERS = (("PRX1_TYPE", "PRX_TYPE"),) + tuple(f"PRX{number}_TYPE" for number in range(2, 6))


TEMPERATURE_TYPE_PARAMETERS = tuple(f"TEMP{number}_TYPE" for number in range(1, 10))


def _any_parameter_nonzero(parameters) -> Callable:
    """Detector: present when any of these parameters (a name, or a tuple of alternative names) is not 0."""
    return lambda ctx: any(ctx.param_int(names) for names in parameters)


def _all_set_to_zero(parameters) -> tuple:
    """'disable' entries that switch every instance off (only parameters that change are written)."""
    return tuple((names, 0) for names in parameters)


def _is_gps_fitted(ctx) -> bool:
    """A GPS type is set and - once GPS telemetry has arrived - a receiver is actually connected.

    ArduPilot's default GPS type is Auto, so the type alone doesn't prove there is a receiver; the fix type in
    GPS_RAW_INT / GPS2_RAW is "no GPS connected" (0) when none answers.
    """
    if not _any_parameter_nonzero(GPS_TYPE_PARAMETERS)(ctx):
        return False
    reports = [report for report in (ctx.vehicle.latest("GPS_RAW_INT"), ctx.vehicle.latest("GPS2_RAW")) if report]
    return not reports or any(report.fix_type >= GPS_FIX_TYPE_NO_FIX for report in reports)


def _is_compass_fitted(ctx) -> bool:
    """Compasses are enabled and ArduPilot has detected one: it records each detected compass's device id.

    COMPASS_ENABLE alone isn't enough - it defaults to 1 with or without a compass.
    """
    if not ctx.param_int("COMPASS_ENABLE", 1):
        return False
    known_ids = [name for name in COMPASS_DEVICE_ID_PARAMETERS if ctx.vehicle.has_parameter(name)]
    # Without device-id parameters (unknown firmware) the enable switch is all there is to go by.
    return not known_ids or _any_parameter_nonzero(known_ids)(ctx)


# ArduPilot records the device id of each detected accelerometer / gyro (instances 1-3).
ACCELEROMETER_ID_PARAMETERS = ("INS_ACC_ID", "INS_ACC2_ID", "INS_ACC3_ID")


GYRO_ID_PARAMETERS = ("INS_GYR_ID", "INS_GYR2_ID", "INS_GYR3_ID")


def _is_sensor_detected(ctx, id_parameters) -> bool:
    """A detected sensor has a device id; without those parameters (unknown firmware), IMU telemetry decides."""
    known_ids = [name for name in id_parameters if ctx.vehicle.has_parameter(name)]
    if known_ids:
        return _any_parameter_nonzero(known_ids)(ctx)
    return ctx.vehicle.imu() is not None


def is_accelerometer_fitted(ctx) -> bool:
    """ArduPilot has detected an accelerometer."""
    return _is_sensor_detected(ctx, ACCELEROMETER_ID_PARAMETERS)


def is_gyro_fitted(ctx) -> bool:
    """ArduPilot has detected a gyro."""
    return _is_sensor_detected(ctx, GYRO_ID_PARAMETERS)


def _is_rc_receiver_fitted(ctx) -> bool:
    """The receiver delivers channels (RC_CHANNELS.chancount > 0); there is no parameter for this.

    Before any RC telemetry has arrived the answer is unknown: assume a receiver, so the RC step stays visible.
    """
    raw = ctx.vehicle.rc_raw()
    return True if raw is None else bool(raw)


OPTIONAL_HARDWARE = (
    # sailing vehicles - boats and land yachts (sailboat.cpp)
    OptionalHardware("has_sails", "Vehicle has sails", are_sails_enabled, (("SAIL_ENABLE", 0),), (("SAIL_ENABLE", 1),)),
    OptionalHardware("has_motor", "Vehicle also has a motor (besides the sails)", is_motor_assigned, needs="has_sails"),
    OptionalHardware("has_gps", "Vehicle has a GPS", _is_gps_fitted, _all_set_to_zero(GPS_TYPE_PARAMETERS)),
    OptionalHardware("has_compass", "Vehicle has a compass", _is_compass_fitted, (("COMPASS_ENABLE", 0),)),
    OptionalHardware("has_rc", "Vehicle has an RC receiver", _is_rc_receiver_fitted, ()),
    OptionalHardware(
        "has_battery_monitor",
        "Vehicle has a battery monitor",
        _any_parameter_nonzero(BATTERY_MONITOR_PARAMETERS),
        _all_set_to_zero(BATTERY_MONITOR_PARAMETERS),
    ),
    OptionalHardware(
        "has_torqeedo",
        "Boat has a Torqeedo electric motor",
        is_torqeedo_configured,
        ((TORQEEDO_TYPE_PARAMETER, 0),),
        frame=WATER,
    ),
    OptionalHardware(
        "has_wenc",
        "Vehicle has wheel encoder(s)",
        _any_parameter_nonzero(WHEEL_ENCODER_TYPE_PARAMETERS),
        _all_set_to_zero(WHEEL_ENCODER_TYPE_PARAMETERS),
        frame=LAND,
    ),
    OptionalHardware(
        "has_rangefinder",
        "Vehicle has a rangefinder (distance sensor)",
        _any_parameter_nonzero(RANGEFINDER_TYPE_PARAMETERS),
        _all_set_to_zero(RANGEFINDER_TYPE_PARAMETERS),
    ),
    OptionalHardware(
        "has_depth_sounder",
        "Boat has a depth sounder (echosounder)",
        lambda ctx: bool(depth_rangefinder(ctx)),
        frame=WATER,
    ),
    OptionalHardware(
        "has_proximity",
        "Vehicle has a proximity sensor",
        _any_parameter_nonzero(PROXIMITY_TYPE_PARAMETERS),
        _all_set_to_zero(PROXIMITY_TYPE_PARAMETERS),
    ),
    OptionalHardware(
        "has_gimbal",
        "Vehicle has camera gimbal(s)",
        _any_parameter_nonzero((("MNT1_TYPE", "MNT_TYPE"), "MNT2_TYPE")),
        ((("MNT1_TYPE", "MNT_TYPE"), 0), ("MNT2_TYPE", 0)),
    ),
    OptionalHardware(
        "has_aux_servos",
        "Vehicle has extra servos / PWM devices (lights ...)",
        lambda ctx: _does_any_output_have_function(ctx, _RC_DRIVEN_FUNCTIONS),
        (),
    ),
    OptionalHardware("has_relays", "Vehicle has relays / switched outputs", _are_relays_configured, ()),
    OptionalHardware(
        "has_gripper",
        "Vehicle has a gripper",
        lambda ctx: bool(ctx.param_int("GRIP_ENABLE")) or _does_any_output_have_function(ctx, {SERVO_FUNCTION_GRIPPER}),
        (("GRIP_ENABLE", 0),),
    ),
    OptionalHardware(
        "has_temperature", "Vehicle has temperature sensor(s)", _any_parameter_nonzero(TEMPERATURE_TYPE_PARAMETERS), ()
    ),
    OptionalHardware(
        "has_sprayer",
        "Vehicle has a sprayer",
        lambda ctx: bool(ctx.param_int("SPRAY_ENABLE"))
        or _does_any_output_have_function(ctx, set(SPRAYER_SERVO_FUNCTIONS.values())),
        (("SPRAY_ENABLE", 0),),
    ),
    OptionalHardware(
        "has_optical_flow", "Vehicle has an optical flow sensor", _parameter_nonzero("FLOW_TYPE"), (("FLOW_TYPE", 0),)
    ),
    OptionalHardware(
        "has_visual_odometry",
        "Vehicle has visual odometry (tracking camera)",
        _parameter_nonzero("VISO_TYPE"),
        (("VISO_TYPE", 0),),
    ),
    OptionalHardware(
        "has_beacons", "Vehicle uses positioning beacons", _parameter_nonzero("BCN_TYPE"), (("BCN_TYPE", 0),)
    ),
    OptionalHardware(
        "has_osd", "Vehicle has an on-screen display (OSD)", _parameter_nonzero("OSD_TYPE"), (("OSD_TYPE", 0),)
    ),
    OptionalHardware(
        "has_remote_id", "Vehicle has a Remote ID module", _parameter_nonzero("DID_ENABLE"), (("DID_ENABLE", 0),)
    ),
    OptionalHardware(
        "has_rpm", "Vehicle has RPM sensor(s)", lambda ctx: any(ctx.param_int(f"RPM{n}_TYPE") for n in RPM_SENSORS), ()
    ),
    OptionalHardware(
        "has_generator", "Vehicle has a generator / fuel cell", _parameter_nonzero("GEN_TYPE"), (("GEN_TYPE", 0),)
    ),
)


HARDWARE_BY_KEY = {hardware.key: hardware for hardware in OPTIONAL_HARDWARE}


def has(key: str) -> Callable:
    """Condition for steps/fields/actions: the pilot's saved answer, or the detected value if not answered yet - for
    hardware this kind of vehicle can have (a boat has no wheel encoders, a vehicle without sails no extra motor)."""

    def is_there(ctx) -> bool:
        """The vehicle has the item."""
        hardware = HARDWARE_BY_KEY[key]
        has_needed = hardware.needs is not None and has(hardware.needs)(ctx)
        if not hardware.is_offered(ctx.is_boat(), has_needed):
            return False
        return bool(ctx.store.get(key, hardware.detect(ctx)))

    return is_there


def is_motorised(ctx) -> bool:
    """The vehicle has a motor: every vehicle without sails, a sailing one only when it has one besides the sails."""
    return not has("has_sails")(ctx) or has("has_motor")(ctx)
