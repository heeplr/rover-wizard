"""rover_wizard.steps.util.straight_runs - a slow straight run with the steering centred (or driven by the pilot),
recorded with the GPS reporting faster: for measurements that compare sensors with the GPS course and speed."""

from __future__ import annotations

import contextlib

from ...mavlink_defs import MAVLINK_MSG_ID_EKF_STATUS_REPORT, MAVLINK_MSG_ID_GPS_RAW_INT
from ...motion.driving import ramp_up
from ...vehicle.link import STREAMS
from .driving import DEFAULT_RUNWAY_M, RUNWAY_USE_FRACTION

# [%] slow and steady, but faster than walking pace (a GPS course needs some speed)
DEFAULT_STRAIGHT_THROTTLE_PCT = 30
# [s] straight driving, then the throttle is cut ...
STRAIGHT_SECONDS = 8.0
# ... and the run records until the vehicle has stopped (at most this long)
STOP_SECONDS = 6.0
# throttle ramps up over this time (no jolt)
RAMP_SECONDS = 1.5
# [Hz] during the runs: GPS and EKF reports (normally 2 Hz)
FAST_RATES_HZ = {MAVLINK_MSG_ID_GPS_RAW_INT: 10, MAVLINK_MSG_ID_EKF_STATUS_REPORT: 10}


@contextlib.contextmanager
def fast_gps_reports(vehicle):
    """While inside: GPS and EKF reports at FAST_RATES_HZ; afterwards their usual rates again."""
    for message_id, rate in FAST_RATES_HZ.items():
        vehicle.set_message_rate(message_id, rate)
    try:
        yield
    finally:
        usual = dict(STREAMS)
        for message_id in FAST_RATES_HZ:
            vehicle.set_message_rate(message_id, usual[message_id])


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
