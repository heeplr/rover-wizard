"""rover_wizard.motion.ekf_sensor_errors - how well the wheel encoders and the compass agree with the GPS on straight
runs, as EKF3 settings (EK3_WENC_VERR, EK3_YAW_M_NSE), and what the EKF's own test ratios say.

The runs are driven straight, one out and one back: a compass error that depends on the direction only shows with
both directions. Only steady parts count (moving faster than MIN_GPS_SPEED, hardly turning, speed hardly changing):
the GPS reports its velocity a little late, which would look like an error while the speed changes.

Pure functions only: no vehicle access, no UI, no clock.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from ..vehicle.link import DriveSample

# [m/s] slower than this, GPS speed and course are mostly noise
MIN_GPS_SPEED = 0.5
# [rad/s] turning faster than this (5 deg/s) isn't straight
MAX_STRAIGHT_YAW_RATE = math.radians(5)
# [m/s] the (averaged) GPS speed may change by at most this much from the STEADY_SECONDS before a sample to the
# STEADY_SECONDS after it
MAX_SPEED_CHANGE = 0.1
STEADY_SECONDS = 0.5
# fewer readings than this give no result
MIN_READINGS = 20
# proposals are this much above the measured error (some margin for days with worse conditions)
ERROR_MARGIN = 1.5

# encoders reading more than 3 % high or low: the wheel radius is off, which isn't noise
MAX_ENCODER_BIAS = 0.03
# EK3_WENC_VERR [m/s]: ArduPilot's range is 0.01 .. 1; below 0.05 the wizard wouldn't trust its own measurement
WENC_VERR_RANGE = (0.05, 1.0)

# [deg] a compass heading error larger than this on average (per direction, or between them) is a calibration,
# mounting or declination problem - not noise
MAX_COMPASS_BIAS_DEG = 10.0
# EK3_YAW_M_NSE [rad]: 0.05 is the least ArduPilot itself uses for a heading (MAX(_yawNoise, 0.05)), 1.0 its maximum
YAW_M_NSE_RANGE = (0.05, 1.0)

# an EKF test ratio above this deserves a look; at 1.0 the EKF rejects the sensor's data
EKF_RATIO_WARNING = 0.5
EKF_RATIO_REJECTED = 1.0
HALF_CIRCLE_RAD = math.pi


@dataclass(frozen=True)
class EncoderErrors:
    """How the wheel encoders' speed compares with the GPS speed."""

    # [m/s] standard deviation of encoder minus GPS speed, without the GPS's own reported inaccuracy
    noise: float
    # encoders' speed / GPS speed - 1 (positive: the encoders read high)
    bias: float
    readings: int
    # proposed EK3_WENC_VERR [m/s], or None (see warning)
    proposal: Optional[float]
    warning: str = ""


@dataclass(frozen=True)
class CompassErrors:
    """How the compass heading compares with the GPS course."""

    # [deg] root mean square of the heading errors (their average included: the EKF sees that as error too)
    rms: float
    # [deg] average error on each run (positive: the compass reads clockwise of the course)
    biases: tuple
    readings: int
    # proposed EK3_YAW_M_NSE [rad], or None (see warning)
    proposal: Optional[float]
    warning: str = ""


def steady_samples(samples: list[DriveSample]) -> list[DriveSample]:
    """The samples of a run that are moving straight forward at a steady speed (see the module docstring)."""
    moving = [
        sample
        for sample in samples
        if (sample.gps_speed or 0.0) >= MIN_GPS_SPEED
        and sample.yaw_rate is not None
        and abs(sample.yaw_rate) <= MAX_STRAIGHT_YAW_RATE
        and (sample.forward_speed is None or sample.forward_speed > 0)
    ]
    steady = []
    for sample in moving:
        before = [
            other.gps_speed
            for other in samples
            if 0 < sample.time - other.time <= STEADY_SECONDS and other.gps_speed is not None
        ]
        after = [
            other.gps_speed
            for other in samples
            if 0 < other.time - sample.time <= STEADY_SECONDS and other.gps_speed is not None
        ]
        # averages, so the GPS's noise doesn't count as a change of speed
        if before and after and abs(statistics.fmean(after) - statistics.fmean(before)) <= MAX_SPEED_CHANGE:
            steady.append(sample)
    return steady


def encoder_errors(runs: list[list[DriveSample]]) -> Optional[EncoderErrors]:
    """Encoder speed (the mean of all wheels) between consecutive encoder reports against the GPS speed; None
    without enough steady readings."""
    differences, gps_speeds, accuracies = [], [], []
    for samples in runs:
        steady = {id(sample) for sample in steady_samples(samples)}
        previous = None
        for sample in samples:
            if sample.wheel_distances is None or sample.wheel_time is None:
                continue
            if previous is not None and sample.wheel_time > previous.wheel_time:
                if id(sample) in steady and id(previous) in steady:
                    travelled = statistics.fmean(sample.wheel_distances) - statistics.fmean(previous.wheel_distances)
                    speed = travelled / (sample.wheel_time - previous.wheel_time)
                    differences.append(speed - sample.gps_speed)
                    gps_speeds.append(sample.gps_speed)
                    if sample.gps_speed_accuracy:
                        accuracies.append(sample.gps_speed_accuracy)
            if previous is None or sample.wheel_time > previous.wheel_time:
                previous = sample
    if len(differences) < MIN_READINGS:
        return None

    bias = statistics.fmean(differences) / statistics.fmean(gps_speeds)
    spread = statistics.pstdev(differences)
    gps_noise = math.sqrt(statistics.fmean(accuracy**2 for accuracy in accuracies)) if accuracies else 0.0
    noise = math.sqrt(max(spread**2 - gps_noise**2, 0.0))
    if abs(bias) > MAX_ENCODER_BIAS:
        return EncoderErrors(
            noise,
            bias,
            len(differences),
            None,
            f"the wheel encoders read {bias:+.0%} compared with the GPS: the wheel radius (WENC_RADIUS) is off - "
            "repeat the wheel-encoder calibration (phase 1), then measure again",
        )
    return EncoderErrors(noise, bias, len(differences), _clamped(noise * ERROR_MARGIN, WENC_VERR_RANGE))


def compass_heading(sample: DriveSample, declination: float) -> Optional[float]:
    """The compass's true heading [rad] from its field (tilt-compensated, plus the declination), or None."""
    if sample.magnetic_field is None:
        return None
    x, y, z = sample.magnetic_field
    roll, pitch = sample.roll or 0.0, sample.pitch or 0.0
    # the field's horizontal components (the vehicle's x axis forward, y right, z down)
    north = x * math.cos(pitch) + y * math.sin(roll) * math.sin(pitch) + z * math.cos(roll) * math.sin(pitch)
    east = y * math.cos(roll) - z * math.sin(roll)
    return math.atan2(-east, north) + declination


def compass_errors(runs: list[list[DriveSample]], declination: float) -> Optional[CompassErrors]:
    """Compass heading against the GPS course on each run; None without enough steady readings."""
    per_run = []
    for samples in runs:
        errors = []
        for sample in steady_samples(samples):
            heading = compass_heading(sample, declination)
            if heading is not None and sample.gps_course is not None:
                errors.append(_wrapped(heading - sample.gps_course))
        per_run.append(errors)
    every = [error for errors in per_run for error in errors]
    if len(every) < MIN_READINGS:
        return None

    rms = math.sqrt(statistics.fmean(error**2 for error in every))
    biases = tuple(math.degrees(statistics.fmean(errors)) for errors in per_run if errors)
    result = (math.degrees(rms), biases, len(every))
    worst = max(abs(bias) for bias in biases)
    if worst > MAX_COMPASS_BIAS_DEG or max(biases) - min(biases) > MAX_COMPASS_BIAS_DEG:
        shown = " / ".join(f"{bias:+.0f}" for bias in biases)
        return CompassErrors(
            *result,
            None,
            f"the compass heading is off by {shown} deg on the runs: check the compass calibration, its orientation "
            "(COMPASS_ORIENT) and the declination; with an error that changes with the direction, calibrate again "
            "away from metal",
        )
    return CompassErrors(*result, _clamped(rms * ERROR_MARGIN, YAW_M_NSE_RANGE))


def ekf_health(samples: list[DriveSample]) -> tuple:
    """(the highest test ratios as EkfRatios-like (velocity, position, compass) or None, explanations of high ones)."""
    reports = [sample.ekf_ratios for sample in samples if sample.ekf_ratios is not None]
    if not reports:
        return None, []
    highest = (
        max(report.velocity for report in reports),
        max(report.position for report in reports),
        max(report.compass for report in reports),
    )
    causes = (
        (
            "velocity",
            "GPS velocity and the accelerometers disagree: vibration (mounting of the flight controller) or "
            "the GPS antenna's position (GPS_POS settings)",
        ),
        ("position", "the GPS position jumps: poor reception, or the antenna near metal or electronics"),
        (
            "compass",
            "the compass disagrees with the GPS and gyro: motor interference (compass-motor compensation on "
            "the compass step), a poor calibration or a wrong COMPASS_ORIENT",
        ),
    )
    notes = []
    for (name, cause), ratio in zip(causes, highest):
        if ratio > EKF_RATIO_WARNING:
            rejected = " - the EKF rejected its data" if ratio >= EKF_RATIO_REJECTED else ""
            notes.append(f"EKF {name} test ratio up to {ratio:.2f}{rejected}: {cause}.")
    return highest, notes


def _clamped(value: float, limits: tuple) -> float:
    """The value within the limits, rounded to 0.01."""
    return round(max(limits[0], min(limits[1], value)), 2)


def _wrapped(angle: float) -> float:
    """An angle in -pi..pi."""
    return (angle + HALF_CIRCLE_RAD) % (2 * HALF_CIRCLE_RAD) - HALF_CIRCLE_RAD
