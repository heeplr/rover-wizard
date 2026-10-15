"""rover_wizard.vehicle.rc_scaling - conversions between RC pulse widths and normalised stick positions.

ArduPilot maps each RC channel with its RCn_MIN / RCn_TRIM / RCn_MAX / RCn_REVERSED parameters. The two functions
here are exact inverses of each other: reading a channel (pwm_to_stick) and faking one for an RC override
(stick_to_pwm) must agree, or the wizard would drive with a different stick position than it thinks it sends.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..ardupilot_defs import RC_PWM_NOMINAL_MAX, RC_PWM_NOMINAL_MIN, RC_PWM_NOMINAL_TRIM


@dataclass(frozen=True)
class RcChannelCalibration:
    """The four RC calibration parameters of one channel."""

    # RCn_MIN [us]
    minimum: float = RC_PWM_NOMINAL_MIN
    # RCn_TRIM [us] - stick centre
    trim: float = RC_PWM_NOMINAL_TRIM
    # RCn_MAX [us]
    maximum: float = RC_PWM_NOMINAL_MAX
    # RCn_REVERSED
    reversed: bool = False


def pwm_to_stick(pwm: float, calibration: RcChannelCalibration) -> float:
    """Normalised stick position -1..+1 for a received pulse width."""
    # Above trim, scale by the upper half-range; below trim, by the lower half-range (they can differ).
    # max(1, ...) guards against a mis-calibrated channel with zero range.
    if pwm >= calibration.trim:
        position = (pwm - calibration.trim) / max(1.0, calibration.maximum - calibration.trim)
    else:
        position = (pwm - calibration.trim) / max(1.0, calibration.trim - calibration.minimum)
    # A reversed channel means "high pulse = negative stick".
    if calibration.reversed:
        position = -position
    # Pulses beyond the calibrated end points still count as full deflection.
    return max(-1.0, min(1.0, position))


def stick_to_pwm(position: float, calibration: RcChannelCalibration) -> int:
    """Pulse width to send so the flight controller reads the given stick position (inverse of pwm_to_stick)."""
    position = max(-1.0, min(1.0, position))
    # Undo the reversal first (pwm_to_stick applies it last).
    if calibration.reversed:
        position = -position
    # Scale into the upper or lower half-range around trim - the same split pwm_to_stick uses.
    if position >= 0:
        pwm = calibration.trim + position * (calibration.maximum - calibration.trim)
    else:
        pwm = calibration.trim + position * (calibration.trim - calibration.minimum)
    return round(pwm)
