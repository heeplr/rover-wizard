"""rover_wizard.motion.vibration - how strongly the accelerometers vibrated during a run, judged like ArduPilot's
"Measuring Vibration" guide: levels below 30 m/s^2 are normally fine, above 60 m/s^2 nearly always a problem, and any
clipping (an accelerometer at the end of its range) means vibration overwhelms the sensor. Every filter and the EKF
work with what the accelerometers deliver: vibration is fixed at the mounting, not with a setting.

Pure functions only: no vehicle access, no UI, no clock.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from ..vehicle.link import DriveSample

# [m/s^2] ArduPilot's guide: normally acceptable below this ...
VIBRATION_ACCEPTABLE = 30.0
# ... and nearly always a problem above this
VIBRATION_PROBLEM = 60.0


@dataclass(frozen=True)
class VibrationCheck:
    """The highest vibration level of a run, how often an accelerometer clipped during it, and what that means."""

    # [m/s^2] highest level of any axis
    highest: float
    # clipping events during the run (all accelerometers)
    clipping: int
    # explanations of a problem (empty: all fine)
    notes: tuple


def check_vibration(samples: list[DriveSample]) -> Optional[VibrationCheck]:
    """The run's vibration, or None without VIBRATION reports."""
    reports = [sample.vibration for sample in samples if sample.vibration is not None]
    if not reports:
        return None
    highest = max(max(report.levels) for report in reports)
    # the clipping counters only count up (since boot): what was added during the run
    clipping = sum(max(0, last - first) for first, last in zip(reports[0].clipping, reports[-1].clipping))
    notes = []
    if highest > VIBRATION_ACCEPTABLE:
        severity = "nearly always causes problems" if highest > VIBRATION_PROBLEM else "may cause problems"
        notes.append(
            f"Vibration up to {highest:.0f} m/s^2 {severity} (below {VIBRATION_ACCEPTABLE:.0f} is normally fine): "
            "mount the flight controller softer (vibration-damping mount or foam), balance or isolate motors and "
            "wheels, and check for loose parts."
        )
    if clipping:
        notes.append(
            f"The accelerometers clipped {clipping} times: vibration overwhelms them - the EKF can't trust them then. "
            "Fix the mounting first."
        )
    return VibrationCheck(highest, clipping, tuple(notes))
