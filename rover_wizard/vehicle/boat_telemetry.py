"""rover_wizard.vehicle.boat_telemetry - what a boat or sailing vehicle reports besides the usual telemetry: the wind
(WIND and two NAMED_VALUE_FLOATs, AP_WindVane::send_wind) and the water depth (WATER_DEPTH, boats only:
GCS_MAVLINK_Rover::send_water_depth). All of them come with the telemetry group Vehicle.setup_streams() requests."""

from __future__ import annotations

from typing import Optional

from ..boat_defs import APPARENT_WIND_DIRECTION_NAME, APPARENT_WIND_SPEED_NAME
from .link import MONITOR_MAX_AGE_SECONDS, named_value_key

# [s] the wind and the depth arrive with the 2 Hz telemetry group: a few lost messages are no gap yet
BOAT_TELEMETRY_MAX_AGE_SECONDS = 2 * MONITOR_MAX_AGE_SECONDS


def true_wind(vehicle) -> Optional[tuple]:
    """(direction the true wind comes from [deg, 0 = north], speed [m/s]), or None without fresh WIND messages."""
    message = vehicle.latest("WIND", max_age=BOAT_TELEMETRY_MAX_AGE_SECONDS)
    return None if message is None else (message.direction, message.speed)


def apparent_wind(vehicle) -> Optional[tuple]:
    """(direction relative to the bow [deg, -180..180, + = from starboard], speed [m/s]) of the wind the vane feels,
    or None without fresh reports."""
    direction = vehicle.latest(named_value_key(APPARENT_WIND_DIRECTION_NAME), max_age=BOAT_TELEMETRY_MAX_AGE_SECONDS)
    speed = vehicle.latest(named_value_key(APPARENT_WIND_SPEED_NAME), max_age=BOAT_TELEMETRY_MAX_AGE_SECONDS)
    return None if direction is None or speed is None else (direction.value, speed.value)


def water_depth(vehicle) -> Optional[tuple]:
    """(depth below the depth sounder [m], water temperature [deg C], healthy) from fresh WATER_DEPTH, or None."""
    message = vehicle.latest("WATER_DEPTH", max_age=BOAT_TELEMETRY_MAX_AGE_SECONDS)
    return None if message is None else (message.distance, message.temperature, bool(message.healthy))
