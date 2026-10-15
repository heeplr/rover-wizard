"""Rotate-by-hand compass calibration: ArduPilot repeats each finished compass's MAG_CAL_REPORT until the next
calibration starts (it is part of a telemetry stream) - each compass must count once, and only in its own attempt."""

from types import SimpleNamespace

import pytest

from rover_wizard.util.errors import VehicleError
from rover_wizard.mavlink_defs import MAG_CAL_FAILED, MAV_CMD_DO_START_MAG_CAL
from rover_wizard.steps.compass import action_calibrate_compass
from rover_wizard.vehicle.link import INBOX_MAG_CAL_REPORT

from util.helpers import BY_ID, enter, make


@pytest.fixture
def wizard():
    """A simulated vehicle with two compasses, logging into a list."""
    vehicle, ctx = make()
    vehicle.calibrating_compasses = 2
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    yield vehicle, ctx, log
    vehicle.disconnect()


def report_lines(log):
    """The log lines describing a compass's report."""
    return [line for line in log if "fitness" in line]


def test_both_compasses_succeed_each_report_logged_once(wizard):
    vehicle, ctx, log = wizard
    action_calibrate_compass(ctx)
    assert len(report_lines(log)) == 2 and "Compass calibration finished" in log[-1]


def test_a_failed_compass_is_named_once_after_all_have_finished(wizard):
    vehicle, ctx, log = wizard
    # compass 0 fails early; its report repeats while compass 1 is still calibrating
    vehicle.compass_calibration_statuses = {0: MAG_CAL_FAILED}
    with pytest.raises(VehicleError, match=r"failed for compass 0 - "):
        action_calibrate_compass(ctx)
    assert len(report_lines(log)) == 2, "the result waits for compass 1 too"


def test_repeated_failed_attempts_report_the_same(wizard):
    vehicle, ctx, log = wizard
    vehicle.compass_calibration_statuses = {0: MAG_CAL_FAILED}
    messages = []
    for _ in range(3):
        with pytest.raises(VehicleError) as failure:
            action_calibrate_compass(ctx)
        messages.append(str(failure.value))
    assert len(set(messages)) == 1 and "compass 0 - " in messages[0]


def test_a_report_left_over_from_the_previous_attempt_is_ignored(wizard):
    vehicle, ctx, log = wizard
    send = vehicle.command_long

    def starting_with_a_leftover(command, *params, **options):
        """Start the calibration - then a failed report of the previous attempt is still on its way."""
        result = send(command, *params, **options)
        if command == MAV_CMD_DO_START_MAG_CAL:
            vehicle._push_inbox(
                INBOX_MAG_CAL_REPORT, SimpleNamespace(compass_id=0, cal_status=MAG_CAL_FAILED, fitness=99.0)
            )
        return result

    vehicle.command_long = starting_with_a_leftover
    action_calibrate_compass(ctx)
    assert "Compass calibration finished" in log[-1]


# fitness of a borderline compass: above the default limit (16), within the relaxed one (32)
BORDERLINE_FITNESS = 20.0


def test_relaxing_the_fitness_limit_lets_a_borderline_compass_pass(wizard):
    vehicle, ctx, log = wizard
    vehicle.calibrating_compasses = 1
    vehicle.compass_fitness = {0: BORDERLINE_FITNESS}
    enter(ctx, "compass")
    with pytest.raises(VehicleError, match=r"COMPASS_CAL_FIT, now 16\)"):
        action_calibrate_compass(ctx)
    # typed into the form (the form keeps it as text), not applied: the calibration uses it anyway
    ctx.set_field_value("COMPASS_CAL_FIT", "32")
    action_calibrate_compass(ctx)
    assert vehicle.get("COMPASS_CAL_FIT") == 32.0 and "Compass calibration finished" in log[-1]
    assert "Calibration fitness limit set to 32 for this calibration." in log


@pytest.mark.parametrize(
    "typed, outcome",
    [("24,5", 24.5), ("abc", 16.0), ("50", None)],
    ids=["decimal comma", "not a number keeps the limit", "out of range refused"],
)
def test_typed_fitness_limit(wizard, typed, outcome):
    vehicle, ctx, log = wizard
    enter(ctx, "compass")
    ctx.set_field_value("COMPASS_CAL_FIT", typed)
    if outcome is None:
        with pytest.raises(VehicleError, match="must be 4..32, not 50"):
            action_calibrate_compass(ctx)
        assert vehicle.get("COMPASS_CAL_FIT") == 16.0, "nothing written"
    else:
        action_calibrate_compass(ctx)
        assert vehicle.get("COMPASS_CAL_FIT") == outcome


def test_other_compasses_get_twice_the_limit(wizard):
    vehicle, ctx, log = wizard
    # compass 1's limit is 2 x 16: a fitness of 20 passes
    vehicle.compass_fitness = {1: BORDERLINE_FITNESS}
    action_calibrate_compass(ctx)
    assert "Compass calibration finished" in log[-1]


def test_fitness_limit_is_offered_only_for_the_rotate_by_hand_calibration(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "compass")
    field = next(f for f in BY_ID["compass"].fields if f.key == "COMPASS_CAL_FIT")
    assert ctx.is_visible(field)
    ctx.store.set("heavy_vehicle", True)
    enter(ctx, "compass")
    assert not ctx.is_visible(field)
