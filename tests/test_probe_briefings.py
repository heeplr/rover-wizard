"""Every probe explains the pilot's tasks and what to watch out for before it starts; its first question is the
go-ahead, and saying no changes nothing."""

import pytest

from util.helpers import BY_ID, enter, make
from rover_wizard.ui.step_catalog import STEPS

PROBES = [
    (step.id, field.key, action)
    for step in STEPS
    for field in step.fields
    for action in (field.inline_actions or [])
    if action.label == "Probe"
]


@pytest.fixture
def wizard():
    """A simulated vehicle; the log goes into a list, and every question is answered 'no' and recorded."""
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)

    def say_no(message):
        """Record the question in the log, answer no."""
        log.append("?? " + message)
        return False

    ctx.confirm = say_no
    yield vehicle, ctx, log
    vehicle.disconnect()


def probe(step_id, field_key):
    """The Probe button of a field."""
    return next(action for step, key, action in PROBES if (step, key) == (step_id, field_key))


def test_all_seven_probes_found():
    assert {(step, key) for step, key, _ in PROBES} == {
        ("motor_output", "MOT_THR_MIN"),
        ("orientation", "AHRS_ORIENTATION"),
        ("gps", "gps_type"),
        ("compass", "COMPASS_ORIENT"),
        ("distance_sensors", "rng_type"),
        ("distance_sensors", "prx_type"),
        ("gimbal", "gimbal_type"),
    }


@pytest.mark.parametrize("step_id, field_key", [(step, key) for step, key, _ in PROBES])
def test_every_probe_has_a_briefing(step_id, field_key):
    briefing = probe(step_id, field_key).briefing
    assert briefing.startswith("━━") and "What happens:" in briefing
    assert "Your tasks:" in briefing and "Take care:" in briefing


def test_briefing_comes_before_the_go_ahead_and_no_changes_nothing(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "orientation")
    original = vehicle.get("AHRS_ORIENTATION")
    ctx.run_action(probe("orientation", "AHRS_ORIENTATION"))
    briefing_line = next(index for index, line in enumerate(log) if "NOSE (front) points straight DOWN" in line)
    question_line = next(index for index, line in enumerate(log) if line.startswith("?? Start the board orientation"))
    assert briefing_line < question_line
    assert log[-1] == "Cancelled - no changes made." and vehicle.get("AHRS_ORIENTATION") == original


def test_minimum_throttle_probe_waits_for_lifted_wheels(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "motor_output")
    ctx.run_action(probe("motor_output", "MOT_THR_MIN"))
    assert any(line.startswith("?? Driven wheels off the ground") for line in log)
    assert not vehicle.motor_tests, "no motor power before the pilot said the wheels are lifted"


def test_actions_without_briefing_log_nothing_extra(wizard):
    vehicle, ctx, log = wizard
    action = BY_ID["predrive"].actions[0]
    assert not action.briefing
    ctx.run_action(type(action)("x", lambda c: c.log("ran")))
    assert log == ["ran"]
