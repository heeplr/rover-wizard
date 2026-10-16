"""Compass-motor interference compensation on the Compass step: shown only with a current sensor, measured with the
motor test (wheels lifted), proposed as COMPASS_MOTCT = 2 plus COMPASS_MOT_X/Y/Z, written by Apply."""

import pytest

from rover_wizard.steps.compass import (
    PowerLevel,
    action_measure_compass_motor_interference,
    fit_interference,
    power_sequence,
    primary_compass_slot,
)
from rover_wizard.vehicle.simulator import SimVehicle

from util.helpers import BY_ID, apply, enter, make, wait_until

# the simulator's motor field at the compass [mGauss/A]
TRUE_INTERFERENCE = SimVehicle.TRUE_MOTOR_INTERFERENCE
# [mGauss/A] the measurement must match it this closely (RAW_IMU reports whole mGauss)
TOLERANCE = 1.0
COMPENSATION_KEYS = ("mot_x", "mot_y", "mot_z")


def make_wizard(skid=False):
    """A simulated vehicle; the log goes into a list, and every question is answered with yes."""
    vehicle, ctx = make(skid=skid)
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    ctx.confirm = lambda message: True
    assert wait_until(lambda: vehicle.latest("SYS_STATUS") is not None and vehicle.latest("RAW_IMU") is not None)
    enter(ctx, "compass")
    return vehicle, ctx, log


@pytest.fixture
def wizard():
    """Front-wheel steering."""
    vehicle, ctx, log = make_wizard()
    yield vehicle, ctx, log
    vehicle.disconnect()


def visible(ctx):
    """Keys of the compass step's fields shown now."""
    return {field.key for field in BY_ID["compass"].fields if ctx.is_visible(field)}


def proposed(ctx):
    """The proposed compensation (x, y, z)."""
    return tuple(float(ctx.value(key)) for key in COMPENSATION_KEYS)


# ---- the fit ----------------------------------------------------------------------------------------------------------


def test_power_levels_alternate_with_motor_off():
    assert power_sequence(50) == [0, 17, 0, 33, 0, 50, 0]


def test_fit_ignores_drift_and_idle_current():
    # field drifts by 1 mGauss per level and the idle current by 0.1 A; the motor adds (10, -5, 2) mGauss per A
    levels = []
    for index, (percent, motor_amps) in enumerate([(0, 0), (20, 2), (0, 0), (40, 4), (0, 0)]):
        amps = 1.0 + 0.1 * index + motor_amps
        levels.append(
            PowerLevel(percent, amps, (100 + index + 10 * motor_amps, 50 - 5 * motor_amps, 300 + 2 * motor_amps))
        )
    per_amp, rise = fit_interference([levels])
    assert per_amp == pytest.approx((10, -5, 2)) and rise == pytest.approx(4)


def test_fit_without_current_rise_has_no_result():
    flat = [PowerLevel(percent, 1.0, (100, 0, 300)) for percent in (0, 20, 0)]
    assert fit_interference([flat]) == (None, 0.0)


# ---- the step ---------------------------------------------------------------------------------------------------------


def test_shown_only_with_a_current_sensor(wizard):
    vehicle, ctx, log = wizard
    assert {"COMPASS_MOTCT", "compassmot_power", "mot_x", "mot_y", "mot_z"} <= visible(ctx)
    vehicle.reports_current = False
    assert wait_until(lambda: vehicle.latest("SYS_STATUS").current_battery == -1)
    assert not {"COMPASS_MOTCT", "compassmot_power", "mot_x"} & visible(ctx)
    action = next(action for action in BY_ID["compass"].actions if "compass-motor" in action.label)
    assert not ctx.is_visible(action)


def test_measurement_proposes_the_compensation_and_apply_writes_it(wizard):
    vehicle, ctx, log = wizard
    action_measure_compass_motor_interference(ctx)
    assert ctx.value("COMPASS_MOTCT") == 2
    assert proposed(ctx) == pytest.approx(tuple(-value for value in TRUE_INTERFERENCE), abs=TOLERANCE)
    assert any("Interference:" in line for line in log)
    # nothing written yet; the type is back to what it was, the motors are off
    assert vehicle.get("COMPASS_MOTCT") == 0 and vehicle.get("COMPASS_MOT_X") == 0
    assert vehicle.motor_tests[-1][1] == 0
    apply(ctx, "compass")
    assert vehicle.get("COMPASS_MOTCT") == 2
    assert [vehicle.get(f"COMPASS_MOT_{axis}") for axis in "XYZ"] == list(proposed(ctx))


def test_compensated_compass_no_longer_moves_with_the_motor(wizard):
    vehicle, ctx, log = wizard
    action_measure_compass_motor_interference(ctx)
    apply(ctx, "compass")
    # measuring again with the compensation in place: it is switched off while measuring, so the same result
    action_measure_compass_motor_interference(ctx)
    assert proposed(ctx) == pytest.approx(tuple(-value for value in TRUE_INTERFERENCE), abs=TOLERANCE)
    assert vehicle.get("COMPASS_MOTCT") == 2, "the compensation type is restored after the run"


def test_skid_steering_runs_both_motors():
    vehicle, ctx, log = make_wizard(skid=True)
    try:
        action_measure_compass_motor_interference(ctx)
        assert any(line.startswith("LEFT motor") for line in log) and any(
            line.startswith("RIGHT motor") for line in log
        )
        assert proposed(ctx) == pytest.approx(tuple(-value for value in TRUE_INTERFERENCE), abs=TOLERANCE)
    finally:
        vehicle.disconnect()


def test_too_little_current_gives_no_result(wizard):
    vehicle, ctx, log = wizard
    # 10 % of the simulated motor's 8 A: less than 1 A extra
    ctx.set_field_value("compassmot_power", "10")
    with pytest.raises(Exception, match="too little to measure"):
        action_measure_compass_motor_interference(ctx)
    assert vehicle.get("COMPASS_MOTCT") == 0 and ctx.value("COMPASS_MOTCT") == 0


def test_no_go_ahead_changes_nothing(wizard):
    vehicle, ctx, log = wizard
    ctx.confirm = lambda message: False
    action_measure_compass_motor_interference(ctx)
    assert log[-1] == "Cancelled - no changes made." and not vehicle.motor_tests


def test_compensation_goes_to_the_compass_used_first(wizard):
    vehicle, ctx, log = wizard
    assert primary_compass_slot(ctx) == 1
    vehicle._params.update(COMPASS_DEV_ID2=12345, COMPASS_PRIO1_ID=12345)
    field = next(field for field in BY_ID["compass"].fields if field.key == "mot_x")
    assert primary_compass_slot(ctx) == 2 and field.names(ctx) == ("COMPASS_MOT2_X",)


@pytest.mark.parametrize("power", ["5", "101"])
def test_motor_power_outside_the_allowed_range_is_refused(wizard, power):
    vehicle, ctx, log = wizard
    ctx.set_field_value("compassmot_power", power)
    with pytest.raises(Exception, match="Motor power must be"):
        action_measure_compass_motor_interference(ctx)
    assert not vehicle.motor_tests


def test_unassigned_motor_output_is_refused(wizard):
    vehicle, ctx, log = wizard
    vehicle.set_param("SERVO3_FUNCTION", 0)
    with pytest.raises(Exception, match="Assign the motor output"):
        action_measure_compass_motor_interference(ctx)
    assert not vehicle.motor_tests


def test_no_current_sensor_is_refused(wizard):
    vehicle, ctx, log = wizard
    vehicle.reports_current = False
    assert wait_until(lambda: vehicle.latest("SYS_STATUS").current_battery == -1)
    with pytest.raises(Exception, match="needs a current sensor"):
        action_measure_compass_motor_interference(ctx)
