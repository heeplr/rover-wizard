"""rover_wizard.steps.accel - the "Accelerometer & gyro calibration" step."""

from __future__ import annotations

from ..util.errors import Cancelled, VehicleError
from ..mavlink_defs import (
    ACCELCAL_VEHICLE_POS_BACK,
    ACCELCAL_VEHICLE_POS_FAILED,
    ACCELCAL_VEHICLE_POS_LEFT,
    ACCELCAL_VEHICLE_POS_LEVEL,
    ACCELCAL_VEHICLE_POS_NOSEDOWN,
    ACCELCAL_VEHICLE_POS_NOSEUP,
    ACCELCAL_VEHICLE_POS_RIGHT,
    ACCELCAL_VEHICLE_POS_SUCCESS,
    MAV_CMD_ACCELCAL_VEHICLE_POS,
    MAV_CMD_PREFLIGHT_CALIBRATION,
    PREFLIGHT_CALIBRATION_ACCEL,
    PREFLIGHT_CALIBRATION_GYRO,
    PREFLIGHT_CALIBRATION_LEVEL,
)
from ..util.timebase import now
from ..vehicle.link import INBOX_ACCEL_CALIBRATION
from ..mavlink_defs import ACCEPTED_RESULTS
from .util.probing import CALIBRATION_POLL_SECONDS
from ..model import Action, Step

ACCEL_POSITION_TEXT = {
    ACCELCAL_VEHICLE_POS_LEVEL: "LEVEL",
    ACCELCAL_VEHICLE_POS_LEFT: "on its LEFT side",
    ACCELCAL_VEHICLE_POS_RIGHT: "on its RIGHT side",
    ACCELCAL_VEHICLE_POS_NOSEDOWN: "NOSE DOWN",
    ACCELCAL_VEHICLE_POS_NOSEUP: "NOSE UP",
    ACCELCAL_VEHICLE_POS_BACK: "UPSIDE DOWN (on its back)",
}

# wait after the gyro / level calibration command before going on
CALIBRATION_SETTLE_SECONDS = 3

# [s] the 6-position accelerometer calibration must finish within this
ACCEL_CALIBRATION_TIMEOUT = 300
# [s] the gyro / level calibration command answers when the calibration is done (it takes a few seconds)
QUICK_CALIBRATION_TIMEOUT = 10


def _run_quick_calibration(ctx, name: str, *params: float) -> None:
    """Send a gyro / level calibration (MAV_CMD_PREFLIGHT_CALIBRATION) and refuse to go on if it wasn't accepted."""
    result = ctx.vehicle.command_long(MAV_CMD_PREFLIGHT_CALIBRATION, *params, timeout=QUICK_CALIBRATION_TIMEOUT)
    if result not in ACCEPTED_RESULTS:
        raise VehicleError(
            f"{name} calibration refused or not answered (result {result}) - keep the vehicle still "
            f"and disarmed, then try again. {ctx.vehicle.last_text()}"
        )
    ctx.log(f"{name} calibration done.")
    ctx.sleep(CALIBRATION_SETTLE_SECONDS)


def remember_level_orientation(ctx) -> None:
    """The level reference (AHRS_TRIM, set by the level and the 6-position calibration) is in vehicle axes: note the
    board orientation it was made with (the Board orientation step warns when that changes)."""
    ctx.store.update_settings(level_orientation=ctx.param_int("AHRS_ORIENTATION"))


def action_calibrate_gyro(ctx):
    """Gyro calibration (the vehicle must be completely still)."""
    ctx.pause("Keep the vehicle completely STILL.")
    _run_quick_calibration(ctx, "Gyro", PREFLIGHT_CALIBRATION_GYRO)


def action_calibrate_level(ctx):
    """Level (AHRS trim) calibration with the vehicle standing level in its driving attitude."""
    ctx.pause("Place the vehicle LEVEL in its normal driving position (on flat ground), keep it still.")
    # param5 = level (AHRS trim)
    _run_quick_calibration(ctx, "Level", 0, 0, 0, 0, PREFLIGHT_CALIBRATION_LEVEL)
    remember_level_orientation(ctx)


# ArduPilot asks for six positions: ACCELCAL_VEHICLE_POS LEVEL (1) .. BACK (6)
ACCEL_POSITION_COUNT = 6

# ArduPilot has no command to stop the accelerometer calibration (AP_AccelCal::cancel() runs only when the vehicle is
# armed), and it waits for a position without a time limit; a new start is ignored while it runs (still ACCEPTED)
STILL_RUNNING_NOTE = (
    "Note: the flight controller keeps waiting for the requested position. Starting the "
    "calibration again continues there; a reboot ends it."
)


def action_calibrate_accelerometer(ctx):
    """ArduPilot's 6-position accelerometer calibration: the FC asks for each pose, the pilot confirms it."""
    vehicle = ctx.vehicle
    # before starting: the first request may arrive before the ACK
    vehicle.clear_inbox(INBOX_ACCEL_CALIBRATION)
    result = vehicle.command_long(MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, PREFLIGHT_CALIBRATION_ACCEL, timeout=5)
    if result not in ACCEPTED_RESULTS:
        raise VehicleError(f"Vehicle refused accelerometer calibration (result {result}). {vehicle.last_text()}")

    try:
        succeeded = _guide_accel_positions(ctx)
    except (Cancelled, VehicleError):
        ctx.log(STILL_RUNNING_NOTE)
        raise

    if not succeeded:
        raise VehicleError("Accelerometer calibration FAILED - try again.")
    ctx.log("Accelerometer calibration SUCCESSFUL.")
    remember_level_orientation(ctx)
    ctx.changes.mark_reboot_needed("accel calibration")


def _guide_accel_positions(ctx) -> bool:
    """Ask the pilot for every position the FC requests; True when the FC reports success, False on failure."""
    vehicle = ctx.vehicle
    start = now()
    confirmed = None
    while now() - start < ACCEL_CALIBRATION_TIMEOUT:
        ctx.check()
        position = vehicle.pop_inbox(INBOX_ACCEL_CALIBRATION)
        if position is None:
            ctx.sleep(CALIBRATION_POLL_SECONDS)
            continue

        if position == ACCELCAL_VEHICLE_POS_SUCCESS:
            ctx.log(f"Position {confirmed} of {ACCEL_POSITION_COUNT} done.")
            return True

        if position == ACCELCAL_VEHICLE_POS_FAILED:
            return False

        if position == confirmed:
            # ArduPilot repeats a request every second until it has the confirmation (AP_AccelCal::update), so
            # repeats of the position just confirmed are answered without asking again. Once the samples are being
            # collected ArduPilot rejects them harmlessly; if a confirmation got lost, this repeats it.
            vehicle.command_long(MAV_CMD_ACCELCAL_VEHICLE_POS, position, wait=False)
            continue

        if confirmed is not None:
            ctx.log(f"Position {confirmed} of {ACCEL_POSITION_COUNT} done.")
        place = ACCEL_POSITION_TEXT.get(position, position)
        ctx.pause(f"Place the vehicle {place}, hold it still, then press Continue.")
        vehicle.command_long(MAV_CMD_ACCELCAL_VEHICLE_POS, position, wait=False)
        confirmed = position
        ctx.log(
            f"Position {position} of {ACCEL_POSITION_COUNT} ({place}) confirmed - collecting samples, keep it "
            "still ..."
        )
    raise VehicleError("Accelerometer calibration timed out.")


STEP = Step(
    "accel",
    "Accelerometer & gyro calibration",
    [],
    [
        Action("Gyro calibration", action_calibrate_gyro, armed=False),
        Action("Level calibration", action_calibrate_level, armed=False),
        Action("6-position accelerometer calibration", action_calibrate_accelerometer, armed=False),
    ],
)
