"""rover_wizard.steps.util.ekf_measurement - the EKF part of the "Closed-loop verification & fine tuning" step: two
straight runs (out and back) measure how well the wheel encoders and the compass agree with the GPS, propose
EK3_WENC_VERR and EK3_YAW_M_NSE (motion.ekf_sensor_errors), and report the EKF's test ratios."""

from __future__ import annotations

from ...model import Action, heading_field, param_field, value_field
from ...motion.ekf_sensor_errors import compass_errors, ekf_health, encoder_errors
from ..ekf_sources import EKF_SOURCE_COMPASS, EKF_YAW_GPS_COMPASS_FALLBACK
from .heading_calibration import require_gps_fix
from .rover_hardware import has
from .straight_runs import DEFAULT_STRAIGHT_THROTTLE_PCT, fast_gps_reports, run_limit, straight_run

# EK3_SRC1_YAW values that use the compass (ArduPilot falls back to it when the GPS has no heading)
COMPASS_YAW_SOURCES = (EKF_SOURCE_COMPASS, EKF_YAW_GPS_COMPASS_FALLBACK)
# out and back
RUNS = 2


def measures_encoders(ctx) -> bool:
    """Wheel encoders and a GPS to compare them with."""
    return has("has_wenc")(ctx) and has("has_gps")(ctx)


def measures_compass(ctx) -> bool:
    """A compass the EKF uses for its heading, and a GPS to compare it with."""
    return (
        has("has_compass")(ctx)
        and has("has_gps")(ctx)
        and ctx.param_int("EK3_SRC1_YAW", EKF_SOURCE_COMPASS) in COMPASS_YAW_SOURCES
    )


def measures_anything(ctx) -> bool:
    """Whether the EKF measurement has something to measure."""
    return measures_encoders(ctx) or measures_compass(ctx)


def log_ekf_health(ctx, samples: list, always: bool = False) -> None:
    """Explain high EKF test ratios of a run (and, with always, the highest ratios even when all is well)."""
    highest, notes = ekf_health(samples)
    if highest is None:
        return
    if always:
        velocity, position, compass = highest
        ctx.log(
            f"EKF test ratios (highest): velocity {velocity:.2f}, position {position:.2f}, compass {compass:.2f}"
            + ("" if notes else " - all fine")
        )
    for note in notes:
        ctx.log(f"  {note}")


def action_measure_ekf_sensor_errors(ctx):
    """Two straight runs, out and back; propose EK3_WENC_VERR and EK3_YAW_M_NSE from how the sensors agree with the
    GPS, and report the EKF's test ratios."""
    require_gps_fix(ctx.vehicle)
    automatic = ctx.value("how_verify", "auto") == "auto"
    if automatic:
        ctx.drive.safety_brief(
            f"Two slow straight runs with the steering centred (max {run_limit(ctx):.0f} m each): one out and - "
            "after you have turned the vehicle around - one back."
        )
    runs = []
    with fast_gps_reports(ctx.vehicle):
        for number in range(1, RUNS + 1):
            if number > 1:
                ctx.pause(ctx.prompt("verify.ekf_turn_around"))
            runs.append(
                straight_run(
                    ctx,
                    automatic,
                    ctx.number("ekf_thr", DEFAULT_STRAIGHT_THROTTLE_PCT),
                    ctx.prompt("verify.ekf_drive_straight"),
                )
            )

    if measures_encoders(ctx):
        _report_encoders(ctx, runs)
    if measures_compass(ctx):
        _report_compass(ctx, runs)
    log_ekf_health(ctx, [sample for samples in runs for sample in samples], always=True)
    ctx.log("Review the proposed values and press Apply.")


def _report_encoders(ctx, runs: list) -> None:
    """Log the wheel encoders' error and propose EK3_WENC_VERR."""
    result = encoder_errors(runs)
    if result is None:
        ctx.log(
            "Wheel encoders: too little steady straight driving with encoder and GPS data - drive faster or farther."
        )
        return
    ctx.log(
        f"Wheel encoders: speed error ±{result.noise:.2f} m/s, reading {result.bias:+.1%} compared with the GPS "
        f"({result.readings} readings)"
        + (f" -> EK3_WENC_VERR {result.proposal:g} m/s" if result.proposal is not None else "")
    )
    if result.proposal is None:
        ctx.log(f"  WARNING: {result.warning}.")
        return
    ctx.set_values(EK3_WENC_VERR=result.proposal)


def _report_compass(ctx, runs: list) -> None:
    """Log the compass heading's error and propose EK3_YAW_M_NSE."""
    result = compass_errors(runs, ctx.param_value("COMPASS_DEC", 0.0) or 0.0)
    if result is None:
        ctx.log("Compass: too little steady straight driving with compass and GPS data - drive faster or farther.")
        return
    biases = " / ".join(f"{bias:+.1f}" for bias in result.biases)
    ctx.log(
        f"Compass: heading error {result.rms:.1f} deg RMS (average {biases} deg on the runs, {result.readings} "
        "readings)" + (f" -> EK3_YAW_M_NSE {result.proposal:g} rad" if result.proposal is not None else "")
    )
    if result.proposal is None:
        ctx.log(f"  WARNING: {result.warning}.")
        return
    ctx.set_values(EK3_YAW_M_NSE=result.proposal)


def ekf_fields() -> list:
    """The step's fields for the EKF measurement (shown only when there is something to measure)."""
    return [
        heading_field("ekf_heading", "EKF sensor errors", when=measures_anything),
        value_field(
            "ekf_thr",
            "Throttle for the straight runs",
            unit="%",
            default=DEFAULT_STRAIGHT_THROTTLE_PCT,
            min_value=1,
            max_value=100,
            when=measures_anything,
        ),
        param_field("EK3_WENC_VERR", "Wheel encoder speed error", unit="m/s", when=measures_encoders),
        param_field("EK3_YAW_M_NSE", "Compass heading noise", unit="rad", when=measures_compass),
        heading_field("controller_heading", "Controllers", when=measures_anything),
    ]


EKF_ACTION = Action(
    "Measure EKF sensor errors (straight runs)", action_measure_ekf_sensor_errors, armed=True, when=measures_anything
)
