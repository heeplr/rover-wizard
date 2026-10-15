"""Protection before the first drive: ESC type / ATC_BRAKE / MOT_REV_DELAY (Motor output), the geofence (Failsafes &
arming) and the command timeouts the wizard's runs rely on (Pre-drive checks)."""

import pytest

from rover_wizard.steps.motor_output import ESC_BRAKE, ESC_FORWARD_ONLY, ESC_REVERSE
from rover_wizard.steps.predrive import action_run_predrive_checks
from rover_wizard.ui.param_docs import validation_error

from util.helpers import BY_ID, apply, enter, make

# the simulator's throttle output (front-wheel steering) and its pulse range
THROTTLE_OUTPUT = 3
SERVO_MIN, SERVO_MAX = 1100, 1900


@pytest.fixture
def wizard():
    """A simulated front-wheel-steering vehicle; the log goes into a list."""
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    yield vehicle, ctx, log
    vehicle.disconnect()


def visible(ctx, step_id):
    """Keys of the step's fields shown for the current form values."""
    return {field.key for field in BY_ID[step_id].fields if ctx.is_visible(field)}


def apply_esc(ctx, esc):
    """Choose the ESC type on the Motor output step and Apply."""
    enter(ctx, "motor_output")
    ctx.set_field_value("esc_dir", esc)
    apply(ctx, "motor_output")


# ---- Motor output ------------------------------------------------------------------------------------------------------


def test_reversible_esc_is_the_default_and_shows_brake_and_delay(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "motor_output")
    assert ctx.value("esc_dir") == ESC_REVERSE
    assert {"ATC_BRAKE", "MOT_REV_DELAY"} <= visible(ctx, "motor_output")


def test_forward_only_esc_moves_neutral_to_the_bottom_and_disables_braking(wizard):
    vehicle, ctx, log = wizard
    apply_esc(ctx, ESC_FORWARD_ONLY)
    assert vehicle.get(f"SERVO{THROTTLE_OUTPUT}_TRIM") == SERVO_MIN and vehicle.get("ATC_BRAKE") == 0
    assert any("Forward-only ESC" in line for line in log)
    # remembered, and braking / reversing settings no longer offered
    enter(ctx, "motor_output")
    assert ctx.value("esc_dir") == ESC_FORWARD_ONLY
    assert not {"ATC_BRAKE", "MOT_REV_DELAY"} & visible(ctx, "motor_output")


def test_back_to_a_reversible_esc_puts_neutral_in_the_middle(wizard):
    vehicle, ctx, log = wizard
    apply_esc(ctx, ESC_FORWARD_ONLY)
    apply_esc(ctx, ESC_BRAKE)
    assert vehicle.get(f"SERVO{THROTTLE_OUTPUT}_TRIM") == (SERVO_MIN + SERVO_MAX) / 2
    assert any("check the ESC's neutral" in line for line in log)


def test_brake_and_reversal_delay_are_written(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "motor_output")
    ctx.set_field_value("ATC_BRAKE", 0)
    ctx.set_field_value("MOT_REV_DELAY", "0.3")
    apply(ctx, "motor_output")
    assert (vehicle.get("ATC_BRAKE"), vehicle.get("MOT_REV_DELAY")) == (0, pytest.approx(0.3))
    # the neutral pulse of a reversible ESC is left alone
    assert vehicle.get(f"SERVO{THROTTLE_OUTPUT}_TRIM") == 1500


def test_zero_reversal_delay_means_off_and_is_accepted():
    # ArduPilot documents 0.1 .. 1 s, but 0 (the default) switches the delay off
    assert validation_error("MOT_REV_DELAY", 0) == ""
    assert "outside" in validation_error("MOT_REV_DELAY", 0.05)


# ---- Failsafes & arming: geofence -------------------------------------------------------------------------------------


def test_fence_settings_only_shown_when_enabled_and_written(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "failsafe")
    assert "FENCE_RADIUS" not in visible(ctx, "failsafe")
    ctx.set_field_value("FENCE_ENABLE", 1)
    # the form re-renders (the pilot's edit is kept)
    ctx.enter_step(BY_ID["failsafe"], reload=False)
    assert {"fence_circle", "fence_zones", "FENCE_RADIUS", "FENCE_ACTION"} <= visible(ctx, "failsafe")
    ctx.set_field_value("fence_circle", True)
    ctx.set_field_value("fence_zones", False)
    ctx.set_field_value("FENCE_RADIUS", "50")
    ctx.set_field_value("FENCE_ACTION", 2)
    apply(ctx, "failsafe")
    assert [vehicle.get(name) for name in ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_RADIUS", "FENCE_ACTION")] == [
        1,
        2,
        50,
        2,
    ]


# ---- Pre-drive checks: command timeouts ---------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "override_time, problem", [(3, None), (-1, "never expires"), (0, "switched off"), (30, "too long")]
)
def test_rc_override_timeout_is_checked(wizard, override_time, problem):
    vehicle, ctx, log = wizard
    vehicle.set_param("RC_OVERRIDE_TIME", override_time)
    action_run_predrive_checks(ctx)
    line = next(line for line in log if "RC override expires" in line or "RC_OVERRIDE_TIME" in line)
    assert line.startswith("OK") if problem is None else (line.startswith("FAIL") and problem in line)


@pytest.mark.parametrize("timeout, is_ok", [(3, True), (0.1, False)])
def test_guided_timeout_is_checked(wizard, timeout, is_ok):
    vehicle, ctx, log = wizard
    vehicle.set_param("GUID_TIMEOUT", timeout)
    action_run_predrive_checks(ctx)
    line = next(line for line in log if "Guided mode stops" in line or "GUID_TIMEOUT" in line)
    assert line.startswith("OK" if is_ok else "FAIL")
