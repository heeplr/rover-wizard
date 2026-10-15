"""rover_wizard.steps.compass - the "Compass calibration" step, including compass-motor interference
compensation (CompassMot)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from ..ardupilot_defs import (
    COMPASS_MOTCT_CURRENT,
    COMPASS_MOTCT_DISABLED,
    COMPASS_MOTCT_THROTTLE,
    MOTOR_TEST_INSTANCE_THROTTLE,
    MOTOR_TEST_INSTANCE_THROTTLE_LEFT,
    MOTOR_TEST_INSTANCE_THROTTLE_RIGHT,
    SERVO_FUNCTION_THROTTLE,
    SERVO_FUNCTION_THROTTLE_LEFT,
    SERVO_FUNCTION_THROTTLE_RIGHT,
)
from ..mavlink_defs import (
    ACCEPTED_RESULTS,
    GPS_FIX_TYPE_3D_FIX,
    MAG_CAL_SUCCESS,
    MAV_CMD_DO_CANCEL_MAG_CAL,
    MAV_CMD_DO_START_MAG_CAL,
    MAV_CMD_FIXED_MAG_CAL_YAW,
    MAV_RESULT_UNSUPPORTED,
    SYS_STATUS_CENTIAMPS_PER_AMP,
)
from ..model import Action, Step, param_field, value_field
from ..motion.servo_test import MOTOR_NAMES, MOTOR_TEST_REQUIREMENTS, MOTOR_TEST_STOP_SECONDS, arming_checks_skipped
from ..util.errors import Cancelled, VehicleError
from ..util.rotation import ORIENTATION_CHOICES, best_matching_rotation
from ..util.timebase import now
from ..vehicle.link import INBOX_MAG_CAL_PROGRESS, INBOX_MAG_CAL_REPORT
from .util.actions import action_reboot_flight_controller
from .util.choices import DISABLED_ENABLED_CHOICES
from .util.compass_slots import (
    COMPASS_SLOTS,
    detected_slots,
    is_external,
    primary_compass_slot,
    priority_of_slot,
    slot_name,
    use_name,
)
from .util.hardware import has
from .util.probing import (
    CALIBRATION_POLL_SECONDS,
    IMU_MIN_SAMPLES,
    MIN_ORIENTATION_CONFIDENCE,
    SENSOR_SAMPLE_SPACING,
    collect_still_readings,
    restore_on_failure,
)

COMPASS_AUTO_ROT_CHOICES = [
    ("Disabled", 0),
    ("Check only", 1),
    ("Check and fix", 2),
    ("Check and fix, including 45-degree mountings", 3),
]

COMPASS_AVERAGING_SECONDS = 2.0

# near the magnetic equator the field is almost horizontal: no usable "down"
MIN_VERTICAL_TO_HORIZONTAL = 0.15

# GPS_RAW_INT.lat is in degrees x 10^7
DEGREES_E7 = 1e7


def _average_compass(ctx, seconds: float = COMPASS_AVERAGING_SECONDS) -> tuple:
    """Mean RAW_IMU magnetic field (x, y, z) [mGauss] of the first compass over a still period."""

    def read():
        """The latest field, if fresh."""
        imu = ctx.vehicle.latest("RAW_IMU", max_age=1.0)
        return None if imu is None else (imu.xmag, imu.ymag, imu.zmag)

    readings = collect_still_readings(ctx, read, seconds)
    if len(readings) < IMU_MIN_SAMPLES:
        raise VehicleError("No RAW_IMU compass telemetry - is the compass enabled and working?")

    # Component-wise mean over all readings.
    return tuple(sum(reading[axis] for reading in readings) / len(readings) for axis in range(3))


def _is_northern_hemisphere(ctx) -> bool:
    """From the GPS latitude if there is a fix, otherwise ask the pilot."""
    gps = ctx.vehicle.latest("GPS_RAW_INT")
    if gps is not None and gps.fix_type >= GPS_FIX_TYPE_3D_FIX and hasattr(gps, "lat"):
        # latitude in degrees: positive = north of the equator
        return gps.lat / DEGREES_E7 >= 0

    return ctx.confirm("No GPS fix. Is the vehicle in the NORTHERN hemisphere? (Cancel = southern)")


def action_probe_compass_orientation(ctx):
    """Find COMPASS_ORIENTn of the compass used first (external) from two headings: facing magnetic north, then south.

    With the orientation at None the compass reports in its own axes. The Earth's field points north and (in the
    northern hemisphere) down, so facing north the reading is F = H*forward + Z*down, facing south
    S = -H*forward + Z*down (both in sensor axes). Hence F - S is the vehicle's forward axis and F + S its down axis
    (up in the southern hemisphere, where Z < 0). The rotation mapping both onto the vehicle axes is the answer.
    """
    slot = primary_compass_slot(ctx)
    orient = orientation_name(ctx)
    if not is_external(ctx, slot):
        raise VehicleError(
            "The compass used first is internal: it follows the board orientation, no " f"{orient} needed."
        )

    original = ctx.param_value(orient)
    if not ctx.confirm("Start the compass orientation probe (two reboots, about a minute)?"):
        ctx.log("Cancelled - no changes made.")
        return

    northern = _is_northern_hemisphere(ctx)
    with restore_on_failure(ctx, {orient: original}):
        ctx.log("Resetting the compass orientation to None and rebooting ...")
        ctx.set_param(orient, 0)
        action_reboot_flight_controller(ctx)
        ctx.pause(
            "Place the vehicle LEVEL with its front pointing to MAGNETIC NORTH, away from metal, then press Continue."
        )
        facing_north = _average_compass(ctx)
        ctx.pause("Now turn it around: front pointing to MAGNETIC SOUTH, still level, then press Continue.")
        facing_south = _average_compass(ctx)
        # Difference = 2H x forward axis; sum = 2Z x down axis (Z < 0 in the southern hemisphere: flip it).
        forward = tuple(n - s for n, s in zip(facing_north, facing_south))
        down = tuple((n + s) * (1 if northern else -1) for n, s in zip(facing_north, facing_south))
        # |F - S| / 2 = H
        horizontal = sum(c * c for c in forward) ** 0.5 / 2
        # |F + S| / 2 = |Z|
        vertical = sum(c * c for c in down) ** 0.5 / 2
        if horizontal <= 0 or vertical / horizontal < MIN_VERTICAL_TO_HORIZONTAL:
            raise VehicleError(
                "The magnetic field is almost horizontal here (near the magnetic equator) - the "
                "orientation can't be determined this way; use the rotate-by-hand calibration."
            )

        name, confidence = best_matching_rotation([n for n, _ in ORIENTATION_CHOICES], down, forward)
        if name is None or confidence < MIN_ORIENTATION_CONFIDENCE:
            raise VehicleError(
                f"No mounting fits the readings well (best {name}, {confidence:.0%}) - check "
                "the headings, keep away from metal, and try again."
            )
        value = dict(ORIENTATION_CHOICES)[name]
        keep = ctx.confirm(f"Detected compass orientation '{name}' ({confidence:.0%}). Apply it and reboot?")
        ctx.set_param(orient, value if keep else original)
        action_reboot_flight_controller(ctx)

    if keep:
        ctx.set_values(COMPASS_ORIENT=value)
        ctx.log("Compass orientation applied.")
    else:
        ctx.log("Discarded - restored the original setting.")


# [s] the rotate-by-hand compass calibration must finish within this
COMPASS_CALIBRATION_TIMEOUT = 600

# MAG_CAL_PROGRESS.cal_mask: one bit per compass (a uint8)
CAL_MASK_BITS = 8
# COMPASS_CAL_FIT's range (ArduPilot: 4 = very strict .. 32 = relaxed)
COMPASS_CAL_FIT_MIN = 4
COMPASS_CAL_FIT_MAX = 32

# [%] compass progress is logged every 5 %, not every message
PROGRESS_LOG_STEP = 5


def _describe_mag_report(report) -> str:
    """One log line for a MAG_CAL_REPORT."""
    outcome = "SUCCESS" if report.cal_status == MAG_CAL_SUCCESS else f"FAILED (status {report.cal_status})"
    return f"  compass {report.compass_id}: {outcome}, fitness {report.fitness:.1f} (lower is better)"


def action_calibrate_compass(ctx):
    """Onboard compass calibration while the pilot rotates the vehicle by hand; fails if any compass fails."""
    vehicle = ctx.vehicle
    current = vehicle.get("COMPASS_CAL_FIT")
    if current is not None:
        # the limit shown in the form (typed text), also when it wasn't applied yet; unreadable text keeps the current
        limit = ctx.number("COMPASS_CAL_FIT", current)
        if not COMPASS_CAL_FIT_MIN <= limit <= COMPASS_CAL_FIT_MAX:
            raise VehicleError(
                f"Calibration fitness limit must be {COMPASS_CAL_FIT_MIN}..{COMPASS_CAL_FIT_MAX}, " f"not {limit:g}."
            )

        if limit != current:
            ctx.set_param("COMPASS_CAL_FIT", limit)
            ctx.log(f"Calibration fitness limit set to {limit:g} for this calibration.")
    vehicle.clear_inbox(INBOX_MAG_CAL_PROGRESS)
    vehicle.clear_inbox(INBOX_MAG_CAL_REPORT)

    # param3 = 1: save automatically
    result = vehicle.command_long(MAV_CMD_DO_START_MAG_CAL, 0, 0, 1, timeout=5)
    if result not in ACCEPTED_RESULTS:
        raise VehicleError(f"Vehicle refused compass calibration (result {result}). {vehicle.last_text()}")
    ctx.log(
        "Calibration running: hold the vehicle in the air, away from metal/magnets, and rotate it slowly so that "
        "EVERY side points down at some point. Press Cancel to abort."
    )
    # the compasses calibrating in this attempt (MAG_CAL_PROGRESS.cal_mask), those that reported progress, and the
    # report of each finished one
    calibrating, running, reports = set(), set(), {}
    last_logged_percent = -PROGRESS_LOG_STEP
    try:
        start = now()
        while now() - start < COMPASS_CALIBRATION_TIMEOUT:
            ctx.check()
            progress = vehicle.pop_inbox(INBOX_MAG_CAL_PROGRESS)
            while progress is not None:
                # cal_mask has one bit per compass being calibrated
                calibrating |= {bit for bit in range(CAL_MASK_BITS) if progress.cal_mask & (1 << bit)}
                running.add(progress.compass_id)
                if progress.completion_pct >= last_logged_percent + PROGRESS_LOG_STEP:
                    last_logged_percent = progress.completion_pct
                    ctx.log(f"  compass {progress.compass_id}: {progress.completion_pct}%")
                progress = vehicle.pop_inbox(INBOX_MAG_CAL_PROGRESS)

            report = vehicle.pop_inbox(INBOX_MAG_CAL_REPORT)
            while report is not None:
                # ArduPilot repeats a finished compass's report until the next calibration starts (it is part of a
                # telemetry stream): one per compass - and only from compasses that ran in this attempt, so a
                # report still repeating from the previous attempt doesn't end this one.
                if report.compass_id in running and report.compass_id not in reports:
                    reports[report.compass_id] = report
                    ctx.log(_describe_mag_report(report))
                report = vehicle.pop_inbox(INBOX_MAG_CAL_REPORT)

            if calibrating and calibrating <= set(reports):
                failed = sorted(compass for compass, report in reports.items() if report.cal_status != MAG_CAL_SUCCESS)
                if failed:
                    which = "compass " if len(failed) == 1 else "compasses "
                    raise VehicleError(
                        f"Compass calibration failed for {which}{', '.join(map(str, failed))} - "
                        "rotate more slowly, away from metal, and try again. If the fitness stays "
                        "above the limit, relax 'Calibration fitness limit' (COMPASS_CAL_FIT, now "
                        f"{vehicle.get('COMPASS_CAL_FIT', 0):g}) - or check for interference."
                    )
                ctx.changes.mark_reboot_needed("compass calibration")
                ctx.log("Compass calibration finished (offsets auto-saved). A reboot is recommended.")
                return
            ctx.sleep(CALIBRATION_POLL_SECONDS)
        raise VehicleError("Compass calibration timed out.")
    # cancelled, timed out or failed: don't leave the calibration running on the flight controller
    except (Cancelled, VehicleError):
        vehicle.command_long(MAV_CMD_DO_CANCEL_MAG_CAL, 0, wait=False)
        raise


def action_large_vehicle_magcal(ctx):
    """One-shot compass calibration from a known true heading (World Magnetic Model) - no rotating needed."""
    vehicle = ctx.vehicle
    gps = vehicle.latest("GPS_RAW_INT")
    if gps is None or gps.fix_type < GPS_FIX_TYPE_3D_FIX:
        raise VehicleError(
            "Large Vehicle MagCal needs a GPS fix (3D or better) - it looks up magnetic declination "
            "for the current location. Get a GPS fix first (go outside, wait for satellites)."
        )

    heading = ctx.number("magcal_heading", -1)
    # a compass heading in degrees
    if not 0 <= heading < 360:
        raise VehicleError("Enter the vehicle's current heading relative to true north (0-359 deg) first.")

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


# radians (COMPASS_DEC) -> degrees shown in the form
DEGREES_PER_RADIAN = math.degrees(1)

COMPASS_USE_CHOICES = [("Used", 1), ("Not used", 0)]


def orientation_name(ctx) -> str:
    """COMPASS_ORIENTn of the compass ArduPilot uses first (the one RAW_IMU reports)."""
    return slot_name("COMPASS_ORIENT", primary_compass_slot(ctx))


def _primary_is_external(ctx) -> bool:
    """Only an external compass has its own orientation."""
    return is_external(ctx, primary_compass_slot(ctx))


def needs_manual_declination(ctx) -> bool:
    """ArduPilot looks the declination up from its position (COMPASS_AUTODEC): without a GPS there is none, and with
    AUTODEC off it is never looked up - then COMPASS_DEC must be entered."""
    return not has("has_gps")(ctx) or ctx.number("COMPASS_AUTODEC", ctx.param_int("COMPASS_AUTODEC", 1)) == 0


def describe_compasses(ctx) -> str:
    """One line per compass found: slot, internal/external, which is used first."""
    lines = []
    for slot in detected_slots(ctx):
        priority = priority_of_slot(ctx, slot)
        place = "external" if is_external(ctx, slot) else "internal"
        order = f"priority {priority}" if priority else "no priority (not used)"
        first = " - used first" if slot == primary_compass_slot(ctx) else ""
        lines.append(f"Compass {slot}: {place}, {order}{first}")
    return "\n".join(lines) or "none found"


def compass_use_fields() -> list:
    """'Use compass n' for every compass found (COMPASS_USEk of its priority k)."""
    # (a compass without priority isn't shown; its name only keeps the field resolvable)
    return [
        param_field(
            lambda ctx, slot=slot: use_name(ctx, slot) or slot_name("COMPASS_USE", slot),
            f"Use compass {slot}",
            key=f"compass_use{slot}",
            kind="choice",
            choices=COMPASS_USE_CHOICES,
            when=lambda ctx, slot=slot: slot in detected_slots(ctx) and use_name(ctx, slot) is not None,
        )
        for slot in COMPASS_SLOTS
    ]


def check_compass_use(ctx) -> Optional[str]:
    """At least one compass must stay in use."""
    keys = [f"compass_use{slot}" for slot in detected_slots(ctx) if use_name(ctx, slot) is not None]
    if keys and not any(int(ctx.number(key, 1)) for key in keys):
        return "At least one compass must stay in use - or untick 'Vehicle has a compass' on the Vehicle type " "step."
    return None


# ---- compass-motor interference (CompassMot) -------------------------------------------------------------------------
# Compass-motor interference compensation ("CompassMot").
#
# Current through the motors, the ESC and the power wires adds a magnetic field that the compass sees on top of the
# Earth's. ArduPilot can subtract it: COMPASS_MOTCT = 2 ("Use Current") adds COMPASS_MOTn_X/Y/Z [mGauss/A] times the
# battery current to the compass reading (AP_Compass_Backend::correct_field).
#
# ArduPilot measures these values itself only on Copter (ArduCopter/compassmot.cpp, MAV_CMD_PREFLIGHT_CALIBRATION
# param6 = 1); Rover doesn't implement that command. So the wizard measures the same way: the motors run at a few power
# levels with the wheels lifted, and the change of the compass field per amp of extra battery current, negated, is the
# compensation. "Use Throttle" (COMPASS_MOTCT = 1) has no effect on Rover: only Copter feeds the throttle to the
# compass (Compass::set_throttle), so the current-based type is the one to use - which needs a current sensor.

COMPASS_MOTCT_CHOICES = [
    ("Disabled", COMPASS_MOTCT_DISABLED),
    ("Use current (measured below)", COMPASS_MOTCT_CURRENT),
    ("Use throttle (no effect on Rover)", COMPASS_MOTCT_THROTTLE),
]

# [%] highest motor power of the measurement (the pilot can change it), and its limits
DEFAULT_COMPASSMOT_POWER_PCT = 50
COMPASSMOT_POWER_MIN_PCT = 10
COMPASSMOT_POWER_MAX_PCT = 100
# powered levels per motor, evenly up to the highest power - each between two motor-off levels
COMPASSMOT_POWER_LEVELS = 3
# [s] each level: let current and field settle, then average them
LEVEL_SETTLE_SECONDS = 1.5
LEVEL_MEASURE_SECONDS = 3.0
# [s] the motor test of a level runs this much longer than the level, so it never lapses in between
LEVEL_TEST_OVERLAP_SECONDS = 1.0
# [s] telemetry older than this is not used
TELEMETRY_FRESH_SECONDS = 1.0

# [A] below this much extra current the field change is mostly compass noise (ArduPilot's own minimum: 3 A at full
# throttle; lifted wheels draw less, but the levels are averaged over seconds)
MIN_CURRENT_RISE_AMPS = 1.0
# COMPASS_MOT_* to 0.1 mGauss/A
COMPASS_MOT_DECIMALS = 1
# [%] interference above this share of the Earth's field (at the measured current) is worth fixing at the source;
# ArduPilot's guidance for CompassMot: below 30 % is acceptable
STRONG_INTERFERENCE_PCT = 30

# x, y, z
AXES = range(3)
# % -> fraction
PERCENT = 100


def measures_current(ctx) -> bool:
    """The battery monitor reports a current (SYS_STATUS.current_battery is -1 without one) - what COMPASS_MOTCT 2
    needs. Any monitor type counts (analog with a current pin, SMBus, DroneCAN, ...)."""
    status = ctx.vehicle.latest("SYS_STATUS")
    return status is not None and status.current_battery >= 0


def motor_compensation_name(axis: str):
    """COMPASS_MOTn_<axis> of the primary compass (for a field: resolved when shown)."""
    return lambda ctx: f"{slot_name('COMPASS_MOT', primary_compass_slot(ctx))}_{axis}"


def power_sequence(max_percent: float) -> list:
    """Motor powers [%] of one run: off, then each level followed by off again (0, 17, 0, 33, 0, 50, 0 for 50 %)."""
    sequence = [0]
    for level in range(1, COMPASSMOT_POWER_LEVELS + 1):
        sequence += [round(max_percent * level / COMPASSMOT_POWER_LEVELS), 0]
    return sequence


@dataclass
class PowerLevel:
    """Averages over one level of a run."""

    percent: int
    amps: float
    # compass field (x, y, z) [mGauss]
    field: tuple


def fit_interference(runs: list) -> tuple:
    """Field change per amp (x, y, z) [mGauss/A] and the largest extra current [A] over all runs.

    Each powered level is compared with the mean of the motor-off levels on either side, so a slow drift of the
    field or of the idle current cancels out; the slope through the origin is the least-squares fit of field change
    against current rise. (None, 0) without any current rise.
    """
    products = [0.0 for _ in AXES]
    squares = 0.0
    largest_rise = 0.0
    for run in runs:
        for before, level, after in zip(run[0::2], run[1::2], run[2::2]):
            rise = level.amps - (before.amps + after.amps) / 2
            for axis in AXES:
                products[axis] += rise * (level.field[axis] - (before.field[axis] + after.field[axis]) / 2)
            squares += rise * rise
            largest_rise = max(largest_rise, rise)

    if squares <= 0:
        return None, 0.0
    return tuple(product / squares for product in products), largest_rise


def _mean(values) -> float:
    """Arithmetic mean of a non-empty sequence."""
    return sum(values) / len(values)


def _mean_vector(vectors) -> tuple:
    """Component-wise mean of (x, y, z) vectors."""
    return tuple(_mean([vector[axis] for vector in vectors]) for axis in AXES)


def _length(vector) -> float:
    """Length of an (x, y, z) vector."""
    return math.hypot(*vector)


def _measure_level(ctx, instance: int, percent: int) -> PowerLevel:
    """Run the motor at `percent`, wait for it to settle, then average battery current and compass field."""
    vehicle = ctx.vehicle
    result = vehicle.motor_test(
        instance, percent, LEVEL_SETTLE_SECONDS + LEVEL_MEASURE_SECONDS + LEVEL_TEST_OVERLAP_SECONDS
    )
    if result not in ACCEPTED_RESULTS:
        raise VehicleError(f"Motor test refused (result {result}): {MOTOR_TEST_REQUIREMENTS} {vehicle.last_text()}")
    ctx.sleep(LEVEL_SETTLE_SECONDS)

    amps, fields = [], []
    start = now()
    while now() - start < LEVEL_MEASURE_SECONDS:
        status = vehicle.latest("SYS_STATUS", max_age=TELEMETRY_FRESH_SECONDS)
        imu = vehicle.latest("RAW_IMU", max_age=TELEMETRY_FRESH_SECONDS)
        if status is not None and status.current_battery >= 0:
            amps.append(status.current_battery / SYS_STATUS_CENTIAMPS_PER_AMP)
        if imu is not None:
            fields.append((imu.xmag, imu.ymag, imu.zmag))
        ctx.sleep(SENSOR_SAMPLE_SPACING)

    if not amps or len(fields) < IMU_MIN_SAMPLES:
        raise VehicleError("No battery current or compass (RAW_IMU) telemetry during the measurement.")
    return PowerLevel(percent, _mean(amps), _mean_vector(fields))


def _motors(ctx) -> tuple:
    """(motor-test instance, output function) of every driving motor."""
    if ctx.is_skid():
        return (
            (MOTOR_TEST_INSTANCE_THROTTLE_LEFT, SERVO_FUNCTION_THROTTLE_LEFT),
            (MOTOR_TEST_INSTANCE_THROTTLE_RIGHT, SERVO_FUNCTION_THROTTLE_RIGHT),
        )
    return ((MOTOR_TEST_INSTANCE_THROTTLE, SERVO_FUNCTION_THROTTLE),)


def _run_motors(ctx, motors: tuple, max_percent: float) -> list:
    """One run per motor through power_sequence(); the compass reports the uncompensated field meanwhile."""
    runs = []
    started = []
    original_type = ctx.param_int("COMPASS_MOTCT")
    try:
        ctx.set_param("COMPASS_MOTCT", COMPASS_MOTCT_DISABLED)
        with arming_checks_skipped(ctx.vehicle):
            for instance, _ in motors:
                started.append(instance)
                run = []
                for percent in power_sequence(max_percent):
                    level = _measure_level(ctx, instance, percent)
                    x, y, z = level.field
                    ctx.log(
                        f"{MOTOR_NAMES[instance]} {percent:3d}%: {level.amps:5.1f} A, compass "
                        f"({x:.0f}, {y:.0f}, {z:.0f}) mGauss"
                    )
                    run.append(level)
                runs.append(run)
    finally:
        # stop the wheels now, not when the test lapses; then the compensation type as it was
        for instance in started:
            ctx.vehicle.motor_test(instance, 0, MOTOR_TEST_STOP_SECONDS)
        ctx.set_param("COMPASS_MOTCT", original_type)
    return runs


def action_measure_compass_motor_interference(ctx):
    """Measure the compass field per amp of motor current and propose COMPASS_MOTCT = 2 with COMPASS_MOTn_X/Y/Z."""
    if not measures_current(ctx):
        raise VehicleError(
            "The battery monitor reports no current - the compensation needs a current sensor "
            "(set it up on the Battery monitor step)."
        )

    max_percent = ctx.number("compassmot_power", DEFAULT_COMPASSMOT_POWER_PCT)
    if not COMPASSMOT_POWER_MIN_PCT <= max_percent <= COMPASSMOT_POWER_MAX_PCT:
        raise VehicleError(f"Motor power must be {COMPASSMOT_POWER_MIN_PCT}..{COMPASSMOT_POWER_MAX_PCT} %.")

    motors = _motors(ctx)
    if not all(ctx.output_with_function(function) for _, function in motors):
        raise VehicleError("Assign the motor output(s) on the Motor output step first.")

    if not ctx.confirm("Driven wheels off the ground, battery connected? Start the compass-motor measurement?"):
        ctx.log("Cancelled - no changes made.")
        return

    runs = _run_motors(ctx, motors, max_percent)
    per_amp, rise = fit_interference(runs)
    if per_amp is None or rise < MIN_CURRENT_RISE_AMPS:
        raise VehicleError(
            f"The motors drew only {rise:.1f} A more than at rest - too little to measure the "
            "interference reliably. Raise the motor power and try again."
        )
    # the Earth's field: the mean of every motor-off level
    resting = [level.field for run in runs for level in run[0::2]]
    earth = _length(_mean_vector(resting))
    strength = _length(per_amp)

    share = strength * rise / earth * PERCENT if earth else 0.0
    ctx.log(
        f"Interference: {strength:.1f} mGauss per A - {share:.0f} % of the Earth's field at {rise:.1f} A "
        "(more while driving: the wheels are unloaded now)."
    )
    if share > STRONG_INTERFERENCE_PCT:
        ctx.log(
            "That is strong: move the compass further from motors, ESC and battery wires (twist power-wire "
            "pairs) - compensation only corrects part of it."
        )
    compensation = [round(-value, COMPASS_MOT_DECIMALS) for value in per_amp]
    ctx.set_values(
        COMPASS_MOTCT=COMPASS_MOTCT_CURRENT, mot_x=compensation[0], mot_y=compensation[1], mot_z=compensation[2]
    )
    ctx.log(
        f"Proposed: COMPASS_MOTCT = 2 (use current), compensation {compensation} mGauss/A for "
        f"{slot_name('COMPASS_MOT', primary_compass_slot(ctx))}. Press 'Apply' to write it."
    )
    _note_other_compasses(ctx)


def _note_other_compasses(ctx) -> None:
    """Only the first compass is measured; say so if another one carries compensation values."""
    primary = primary_compass_slot(ctx)
    for slot in COMPASS_SLOTS:
        names = [f"{slot_name('COMPASS_MOT', slot)}_{axis}" for axis in "XYZ"]
        if slot != primary and any(ctx.param_value(name) for name in names):
            ctx.log(
                f"Note: {slot_name('COMPASS_MOT', slot)}_X/Y/Z (another compass) keep their values - only the "
                "compass ArduPilot uses first is measured."
            )


def motor_compensation_fields() -> list:
    """The step's fields: compensation type, measurement power and the primary compass's X/Y/Z values (only with a
    current sensor)."""
    return [
        param_field(
            "COMPASS_MOTCT",
            "Motor interference compensation",
            kind="choice",
            choices=COMPASS_MOTCT_CHOICES,
            when=measures_current,
        ),
        value_field(
            "compassmot_power",
            "Motor power for the measurement",
            unit="%",
            default=DEFAULT_COMPASSMOT_POWER_PCT,
            min_value=COMPASSMOT_POWER_MIN_PCT,
            max_value=COMPASSMOT_POWER_MAX_PCT,
            when=measures_current,
        ),
    ] + [
        param_field(
            motor_compensation_name(axis),
            f"Motor compensation {axis}",
            key=f"mot_{axis.lower()}",
            unit="mGauss/A",
            when=measures_current,
        )
        for axis in "XYZ"
    ]


STEP = Step(
    "compass",
    "Compass calibration",
    [value_field("compass_list", "Compasses found", kind="display", default=describe_compasses)]
    + compass_use_fields()
    + [
        param_field(
            "COMPASS_AUTODEC", "Automatic declination", kind="choice", choices=DISABLED_ENABLED_CHOICES, refresh=True
        ),
        param_field(
            "COMPASS_DEC",
            "Declination (east positive)",
            unit="deg",
            scale=DEGREES_PER_RADIAN,
            min_value=-180,
            max_value=180,
            when=needs_manual_declination,
        ),
        param_field(
            "COMPASS_AUTO_ROT", "Detect orientation during calibration", kind="choice", choices=COMPASS_AUTO_ROT_CHOICES
        ),
        param_field(
            orientation_name,
            "External compass orientation",
            key="COMPASS_ORIENT",
            kind="choice",
            choices=ORIENTATION_CHOICES,
            reboot=True,
            when=_primary_is_external,
            inline_actions=[Action("Probe", action_probe_compass_orientation, armed=False, is_probe=True)],
        ),
        param_field(
            "COMPASS_CAL_FIT",
            "Calibration fitness limit",
            min_value=COMPASS_CAL_FIT_MIN,
            max_value=COMPASS_CAL_FIT_MAX,
            when=lambda ctx: not ctx.store.get("heavy_vehicle", False),
        ),
        value_field(
            "magcal_heading",
            "Current heading (true north)",
            unit="deg",
            min_value=0,
            max_value=359.9,
            when=lambda ctx: ctx.store.get("heavy_vehicle", False),
        ),
    ]
    + motor_compensation_fields(),
    [
        Action(
            "Start compass calibration (rotate by hand)",
            action_calibrate_compass,
            armed=False,
            when=lambda ctx: not ctx.store.get("heavy_vehicle", False),
        ),
        Action(
            "Large Vehicle MagCal (from heading)",
            action_large_vehicle_magcal,
            armed=False,
            when=lambda ctx: ctx.store.get("heavy_vehicle", False),
        ),
        Action(
            "Measure compass-motor interference",
            action_measure_compass_motor_interference,
            armed=False,
            when=measures_current,
        ),
    ],
    applies=has("has_compass"),
    check=check_compass_use,
)
