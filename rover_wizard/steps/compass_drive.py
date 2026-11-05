"""rover_wizard.steps.compass_drive - the "Compass from a straight run" step (phase 2).

For vehicles too heavy to rotate by hand whose GPS can't measure heading: the vehicle drives straight ahead, the GPS
course gives its true heading (motion.gps_heading), and Large Vehicle MagCal calibrates the compass with it.
"""

from __future__ import annotations

from ..mavlink_defs import MAVLINK_MSG_ID_GPS_RAW_INT
from ..model import Action, Step, value_field
from ..motion.driving import ramp_up
from ..motion.gps_heading import heading_after_straight_run
from ..vehicle.link import STREAMS
from .util.driving import DEFAULT_RUNWAY_M, RUNWAY_USE_FRACTION, who_drives_field
from .util.heading_calibration import calibrate_from_heading, require_gps_fix
from .util.rover_hardware import has, is_motorised

# [%] slow and steady, but faster than walking pace (a GPS course needs some speed)
DEFAULT_THROTTLE_PCT = 30
# [s] straight driving, then the throttle is cut ...
STRAIGHT_SECONDS = 8.0
# ... and the run records until the vehicle has stopped (at most this long)
STOP_SECONDS = 6.0
# throttle ramps up over this time (no jolt)
RAMP_SECONDS = 1.5
# [Hz] GPS reports during the run (normally 2 Hz)
GPS_RATE_HZ = 10


def applies(ctx) -> bool:
    """A heavy vehicle with compass, GPS and motor, whose GPS can't measure heading (else the compass step's
    "From GPS" button does it)."""
    return (
        ctx.store.get("heavy_vehicle", False)
        and has("has_compass")(ctx)
        and has("has_gps")(ctx)
        and is_motorised(ctx)
        and not ctx.vehicle.can_measure_gps_heading()
    )


def action_calibrate_from_straight_run(ctx):
    """Drive (or have the pilot drive) straight, take the heading from the GPS course, calibrate the compass."""
    require_gps_fix(ctx.vehicle)
    limit = ctx.number("runway", DEFAULT_RUNWAY_M) * RUNWAY_USE_FRACTION
    ctx.vehicle.set_message_rate(MAVLINK_MSG_ID_GPS_RAW_INT, GPS_RATE_HZ)
    try:
        if ctx.value("how_compass", "auto") == "auto":
            # % -> 0..1
            throttle = ctx.number("compass_thr", DEFAULT_THROTTLE_PCT) / 100
            ctx.drive.safety_brief(
                f"The vehicle drives slowly straight ahead with the steering centred (max {limit:.0f} m) and stops."
            )
            samples = ctx.drive.run_open_loop(
                lambda t: (0.0, ramp_up(throttle, t, RAMP_SECONDS)),
                # the stop condition below ends the straight part; this only bounds the whole run
                STRAIGHT_SECONDS + STOP_SECONDS,
                limit_m=limit,
                stop_if=lambda recorded: recorded[-1].run_time >= STRAIGHT_SECONDS,
                tail=STOP_SECONDS,
            )
        else:
            samples = ctx.drive.record_manual(ctx.prompt("compass_drive.drive_straight"))
    finally:
        ctx.vehicle.set_message_rate(MAVLINK_MSG_ID_GPS_RAW_INT, dict(STREAMS)[MAVLINK_MSG_ID_GPS_RAW_INT])

    run = heading_after_straight_run(samples)
    ctx.log(f"Heading {run.heading:.1f} deg true (course spread ±{run.spread:.1f} deg over {run.distance:.0f} m)")
    if not ctx.confirm(
        f"The vehicle points to {run.heading:.0f} deg (true north). Calibrate the compass with this heading now? "
        "Keep the vehicle still."
    ):
        ctx.log("Cancelled - no changes made.")
        return
    calibrate_from_heading(ctx, run.heading)


STEP = Step(
    "compass_drive",
    "Compass from a straight run",
    [
        who_drives_field("how_compass"),
        value_field(
            "compass_thr",
            "Throttle for the straight run",
            unit="%",
            default=DEFAULT_THROTTLE_PCT,
            min_value=1,
            max_value=100,
        ),
    ],
    [Action("Drive straight & calibrate compass", action_calibrate_from_straight_run, armed=True)],
    applies=applies,
)
