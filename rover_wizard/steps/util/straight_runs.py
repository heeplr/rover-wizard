"""rover_wizard.steps.util.straight_runs - a slow straight run with the steering centred (or driven by the pilot),
recorded with the GPS reporting faster: for measurements that compare sensors with the GPS course and speed."""

from __future__ import annotations

from ...mavlink_defs import MAVLINK_MSG_ID_EKF_STATUS_REPORT, MAVLINK_MSG_ID_GPS_RAW_INT, MAVLINK_MSG_ID_VIBRATION
from ...motion.driving import ramp_up
from .driving import DEFAULT_RUNWAY_M, RUNWAY_USE_FRACTION

# [%] slow and steady, but faster than walking pace (a GPS course needs some speed)
DEFAULT_STRAIGHT_THROTTLE_PCT = 30
# [s] straight driving, then the throttle is cut ...
STRAIGHT_SECONDS = 8.0
# ... and the run records until the vehicle has stopped (at most this long)
STOP_SECONDS = 6.0
# throttle ramps up over this time (no jolt)
RAMP_SECONDS = 1.5
# [Hz] during the runs: GPS, EKF and vibration reports (normally 2 Hz, vibration not at all)
FAST_RATES_HZ = {MAVLINK_MSG_ID_GPS_RAW_INT: 10, MAVLINK_MSG_ID_EKF_STATUS_REPORT: 10, MAVLINK_MSG_ID_VIBRATION: 5}


def fast_gps_reports(vehicle):
    """Context: GPS, EKF and vibration reports at FAST_RATES_HZ; afterwards their usual rates again."""
    return vehicle.faster_reports(FAST_RATES_HZ)


def run_limit(ctx) -> float:
    """[m] how far a straight run may go: the share of the runway straight runs use."""
    return ctx.number("runway", DEFAULT_RUNWAY_M) * RUNWAY_USE_FRACTION


def straight_run(ctx, automatic: bool, throttle_pct: float, manual_prompt: str) -> list:
    """One straight run, recorded until the vehicle stands still: driven by the wizard (throttle_pct, steering
    centred, ends at the runway limit at the latest) or by the pilot (manual_prompt, until Continue)."""
    if not automatic:
        return ctx.drive.record_manual(manual_prompt)
    # % -> 0..1
    throttle = throttle_pct / 100
    return ctx.drive.run_open_loop(
        lambda t: (0.0, ramp_up(throttle, t, RAMP_SECONDS)),
        # the stop condition below ends the straight part; this only bounds the whole run
        STRAIGHT_SECONDS + STOP_SECONDS,
        limit_m=run_limit(ctx),
        stop_if=lambda recorded: recorded[-1].run_time >= STRAIGHT_SECONDS,
        tail=STOP_SECONDS,
    )
