"""rover_wizard.steps.orientation - the "Board orientation" step."""

from __future__ import annotations

from ..util.errors import VehicleError
from ..mavlink_defs import MILLI_G_PER_G
from ..util.rotation import ORIENTATION_CHOICES, best_matching_rotation, gravity_direction_from_accelerometer
from .util.probing import IMU_MIN_SAMPLES, MIN_ORIENTATION_CONFIDENCE, collect_still_readings, restore_on_failure
from .util.actions import action_reboot_flight_controller
from ..model import Action, Step, param_field

IMU_AVERAGING_SECONDS = 1.5


def _average_accelerometer(ctx, seconds: float = IMU_AVERAGING_SECONDS) -> tuple:
    """Mean SCALED_IMU acceleration (x, y, z) in g over a short still period (averaging removes vibration noise)."""

    def read():
        """The latest acceleration, if fresh."""
        imu = ctx.vehicle.latest("SCALED_IMU", max_age=1.0)
        return None if imu is None else (imu.xacc, imu.yacc, imu.zacc)

    readings = collect_still_readings(ctx, read, seconds)
    if len(readings) < IMU_MIN_SAMPLES:
        raise VehicleError("No SCALED_IMU telemetry - check the flight controller has an IMU and the link is stable.")

    # SCALED_IMU reports milli-g; convert the means to g.
    return tuple(sum(reading[axis] for reading in readings) / len(readings) / MILLI_G_PER_G for axis in range(3))


def action_probe_orientation(ctx):
    """Find AHRS_ORIENTATION from two poses: level, then nose pointing straight down.

    With AHRS_ORIENTATION temporarily at None (and a reboot, since it only applies at boot) the accelerometer
    reports in the board's own axes. Level, gravity points along the vehicle's "down"; nose-down, it points along
    the vehicle's "forward". The orientation whose rotation maps the two measured directions onto vehicle-down and
    vehicle-forward is the answer.
    """
    original = ctx.param_value("AHRS_ORIENTATION")
    if not ctx.confirm("Start the board orientation probe (two reboots, about a minute)?"):
        ctx.log("Cancelled - no changes made.")
        return

    with restore_on_failure(ctx, {"AHRS_ORIENTATION": original}):
        ctx.log("Step 1/4: resetting orientation to None and rebooting to read the board's own axes ...")
        ctx.set_param("AHRS_ORIENTATION", 0)
        action_reboot_flight_controller(ctx)

        ctx.pause(
            "Step 2/4: place the vehicle LEVEL in its normal driving position, hold it still, then press Continue."
        )
        level_reading = _average_accelerometer(ctx)
        ctx.log("  measured (level): x={:+.2f} y={:+.2f} z={:+.2f} g".format(*level_reading))
        ctx.pause(
            "Step 3/4: now tilt the vehicle so its NOSE points straight DOWN (about 90 deg pitch), hold it "
            "still, then press Continue."
        )
        nose_down_reading = _average_accelerometer(ctx)
        ctx.log("  measured (nose-down): x={:+.2f} y={:+.2f} z={:+.2f} g".format(*nose_down_reading))

        # The accelerometer measures specific force (pointing up at rest); gravity is the opposite direction.
        # Level: gravity = vehicle-down. Nose-down: gravity = vehicle-forward.
        down_direction = gravity_direction_from_accelerometer(level_reading)
        forward_direction = gravity_direction_from_accelerometer(nose_down_reading)
        name, confidence = best_matching_rotation(
            [name for name, _ in ORIENTATION_CHOICES], down_direction, forward_direction
        )
        if name is None or confidence < MIN_ORIENTATION_CONFIDENCE:
            raise VehicleError(
                f"Could not determine the orientation confidently (best guess {name}, confidence "
                f"{confidence:.0%}) - hold the vehicle very still in each pose, away from "
                "magnets/vibration, and try again."
            )
        value = dict(ORIENTATION_CHOICES)[name]
        ctx.log(f"Step 4/4: detected orientation '{name}' (confidence {confidence:.0%}).")
        keep = ctx.confirm(f"Apply AHRS_ORIENTATION = {name} and reboot?")
        ctx.set_param("AHRS_ORIENTATION", value if keep else original)
        action_reboot_flight_controller(ctx)

    if keep:
        ctx.set_values(AHRS_ORIENTATION=value)
        warn_if_level_calibration_stale(ctx)
        ctx.log("Orientation applied.")
    else:
        ctx.log("Discarded - restored the original setting and rebooted back.")


def warn_if_level_calibration_stale(ctx) -> None:
    """A new board orientation invalidates the level reference (AHRS_TRIM is in vehicle axes; ArduPilot: "After
    changing you will need to re-level your vehicle"). The accelerometer offsets are in sensor axes and stay valid."""
    calibrated = ctx.store.setting("level_orientation")
    if calibrated is not None and calibrated != ctx.param_int("AHRS_ORIENTATION"):
        ctx.log(
            "The board orientation changed after the level calibration: after the reboot, redo the 'Level "
            "calibration' on the Accelerometer & gyro step (the 6-position calibration stays valid)."
        )
        ctx.store.clear_step("accel")


STEP = Step(
    "orientation",
    "Board orientation",
    [
        param_field(
            "AHRS_ORIENTATION",
            "Board orientation",
            kind="choice",
            choices=ORIENTATION_CHOICES,
            reboot=True,
            inline_actions=[Action("Probe", action_probe_orientation, armed=False, is_probe=True)],
        )
    ],
    [],
    on_apply=warn_if_level_calibration_stale,
)
