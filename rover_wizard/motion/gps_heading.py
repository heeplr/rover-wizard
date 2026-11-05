"""rover_wizard.motion.gps_heading - the vehicle's true heading from a straight run, for a compass calibration.

A single GPS can't tell which way a standing vehicle points, but while it drives straight forward its course over
ground is its heading. The gyro (independent of the compass) then follows any turn up to the moment the vehicle
stands still again: each GPS course reading plus the turn since then is an estimate of the final heading, and their
circular mean is the result. Their spread shows how straight the run was. Sideways drift (a boat in a current or wind)
makes the course differ from the heading: the result is only as good as the run.

Pure functions only: no vehicle access, no UI, no clock.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from ..util.errors import VehicleError

if TYPE_CHECKING:
    from ..vehicle.link import DriveSample

# [m/s] slower than this, a GPS course is mostly noise
MIN_GPS_SPEED = 0.5
# at least this many course readings while moving (a few seconds of driving)
MIN_COURSE_READINGS = 10
# [deg] the course estimates may spread this much (one standard deviation): more means the run wasn't straight
MAX_COURSE_SPREAD_DEG = 5.0
# [m/s] at the end the vehicle must stand still: the calibration needs the heading it has then
STOPPED_SPEED = 0.1
FULL_CIRCLE_DEG = 360.0


@dataclass(frozen=True)
class RunHeading:
    """The heading a straight run ended with."""

    # [deg, 0..360, 0 = true north]
    heading: float
    # [deg] standard deviation of the estimates
    spread: float
    # [m] distance driven while the course was read
    distance: float


def heading_after_straight_run(samples: list[DriveSample]) -> RunHeading:
    """The true heading the vehicle stands at after a straight run (see the module docstring); raises VehicleError
    when the run can't give one."""
    if not samples:
        raise VehicleError("Nothing recorded.")
    last = samples[-1]
    if last.forward_speed is None or abs(last.forward_speed) > STOPPED_SPEED:
        raise VehicleError("The vehicle hadn't stopped at the end: stop it and keep it still, then try again.")

    # the turn [rad] from each sample to the end, integrated from the gyro backwards
    turned_after = [0.0] * len(samples)
    for index in range(len(samples) - 2, -1, -1):
        earlier, later = samples[index], samples[index + 1]
        rate = later.yaw_rate or 0.0
        turned_after[index] = turned_after[index + 1] + rate * max(0.0, later.time - earlier.time)

    estimates = []
    distance = 0.0
    for (sample, turned), following in zip(zip(samples, turned_after), [*samples[1:], None]):
        if sample.gps_course is None or (sample.gps_speed or 0.0) < MIN_GPS_SPEED:
            continue
        # a reversing vehicle's course is opposite its heading: only forward driving counts
        if sample.forward_speed is not None and sample.forward_speed < 0:
            continue
        estimates.append(sample.gps_course + turned)
        if following is not None:
            distance += sample.gps_speed * max(0.0, following.time - sample.time)

    if len(estimates) < MIN_COURSE_READINGS:
        raise VehicleError(
            f"Too little straight driving faster than {MIN_GPS_SPEED:g} m/s with a GPS course "
            f"({len(estimates)} readings): drive a little faster (more throttle) or farther."
        )

    heading, spread = _circular_mean_and_spread(estimates)
    if spread > MAX_COURSE_SPREAD_DEG:
        raise VehicleError(
            f"The course varied by ±{spread:.0f} deg: the run wasn't straight enough (or the GPS is poor). "
            "Drive straighter, on open ground with good GPS reception, and try again."
        )
    return RunHeading(heading=heading, spread=spread, distance=distance)


def _circular_mean_and_spread(angles: list) -> tuple:
    """Mean [deg, 0..360] and circular standard deviation [deg] of angles [rad]."""
    sine = sum(math.sin(angle) for angle in angles) / len(angles)
    cosine = sum(math.cos(angle) for angle in angles) / len(angles)
    length = min(1.0, math.hypot(sine, cosine))
    # circular standard deviation: sqrt(-2 ln R)
    spread = math.degrees(math.sqrt(-2 * math.log(length))) if length > 0 else math.inf
    return math.degrees(math.atan2(sine, cosine)) % FULL_CIRCLE_DEG, spread
