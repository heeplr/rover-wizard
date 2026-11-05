"""rover_wizard.steps.util.heading_calibration - compass calibration from a known true heading (Large Vehicle MagCal,
MAV_CMD_FIXED_MAG_CAL_YAW), shared by the compass step and the straight-run compass step."""

from __future__ import annotations

from ...mavlink_defs import ACCEPTED_RESULTS, GPS_FIX_TYPE_3D_FIX, MAV_CMD_FIXED_MAG_CAL_YAW, MAV_RESULT_UNSUPPORTED
from ...util.errors import VehicleError


def require_gps_fix(vehicle) -> None:
    """Refuse without a 3D GPS fix: ArduPilot looks up the magnetic field for the current position."""
    gps = vehicle.latest("GPS_RAW_INT")
    if gps is None or gps.fix_type < GPS_FIX_TYPE_3D_FIX:
        raise VehicleError(
            "Large Vehicle MagCal needs a GPS fix (3D or better) - it looks up magnetic declination "
            "for the current location. Get a GPS fix first (go outside, wait for satellites)."
        )


def calibrate_from_heading(ctx, heading: float) -> None:
    """One-shot compass calibration for the vehicle pointing to `heading` [deg, true north] (World Magnetic Model)."""
    vehicle = ctx.vehicle
    require_gps_fix(vehicle)
    # param1 = heading [deg], param2 = compass mask (0 = all), param3/4 = lat/lon (0 = use the current position).
    result = vehicle.command_long(MAV_CMD_FIXED_MAG_CAL_YAW, heading, 0, 0, 0, timeout=5)
    if result == MAV_RESULT_UNSUPPORTED:
        raise VehicleError(
            "This firmware/vehicle type does not support Large Vehicle MagCal - use the normal "
            "rotate-by-hand compass calibration instead."
        )

    if result not in ACCEPTED_RESULTS:
        raise VehicleError(f"Vehicle refused Large Vehicle MagCal (result {result}). {vehicle.last_text()}")
    # An accepted command has already computed and saved the new offsets (Compass::mag_cal_fixed_yaw); it sends no
    # MAG_CAL_REPORT - one arriving now would be left over from an earlier rotate-by-hand calibration.
    ctx.changes.mark_reboot_needed("compass calibration")
    ctx.log(
        f"Large Vehicle MagCal done for heading {heading:.0f} deg true: the compass offsets are set and saved. "
        "A reboot is recommended."
    )
