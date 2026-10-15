"""rover_wizard.steps.util.probing - probes and calibrations: reading sensors, and putting changed parameters back
afterwards."""

from __future__ import annotations

import contextlib
from .actions import action_reboot_flight_controller

# 'Show ...' buttons log live readings once a second this long
LIVE_DISPLAY_SECONDS = 10


IMU_MIN_SAMPLES = 5


# below this, no candidate explains the readings well enough
MIN_ORIENTATION_CONFIDENCE = 0.6


def snapshot(ctx, *parameters) -> dict:
    """Current values of the given parameters (names or tuples of alternative names) that exist on the vehicle."""
    return {names: ctx.param_value(names) for names in parameters if ctx.vehicle.resolve(names)}


def restore(ctx, originals: dict) -> None:
    """Write the saved values back and reboot so they take effect."""
    for names, value in originals.items():
        ctx.set_param(names, value)
    action_reboot_flight_controller(ctx)


@contextlib.contextmanager
def restore_on_failure(ctx, originals: dict):
    """Put every parameter in `originals` back (and reboot) if the block exits with any exception, then re-raise."""
    try:
        yield
    except BaseException:
        # Best effort: even if restoring fails too, the original exception is the one the pilot must see.
        with contextlib.suppress(Exception):
            ctx.log("Restoring the original settings ...")
            restore(ctx, originals)
        raise


# [s] between IMU/compass readings while averaging
SENSOR_SAMPLE_SPACING = 0.1


# how often the calibrations check for progress
CALIBRATION_POLL_SECONDS = 0.2
