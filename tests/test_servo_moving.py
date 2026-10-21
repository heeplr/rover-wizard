"""Moving outputs by hand: Test L/R, MIN/TRIM/MAX +/- and the servo panel.

The steering servo must move with the vehicle disarmed and without ArduPilot's motor test (which would arm it);
motor outputs use the motor test, which arms the vehicle only while it runs.
"""

import threading

import pytest

from rover_wizard.util import timebase
from rover_wizard.util.errors import VehicleError
from rover_wizard.context import Ctx
from rover_wizard.rover_defs import (
    MODES,
    MOTOR_TEST_INSTANCE_THROTTLE,
    SERVO_FUNCTION_GROUND_STEERING,
    SERVO_FUNCTION_THROTTLE,
)
from rover_wizard.motion.servo_test import PANEL_STEERING_HOLD_SECONDS, STEERING_HOLD_SECONDS
from rover_wizard.steps.motor_output import DEADZONE_STEP_SECONDS, action_find_throttle_deadzone

from util.helpers import BY_ID, TEST_TIME_SCALE, act, enter, make, wait_until

# The simulator's front-wheel layout:
STEERING_OUTPUT = 1
THROTTLE_OUTPUT = 3
# no function assigned
FREE_OUTPUT = 5
# 1 s of simulated time margin = 200 ms real time
SLOWER_TIME_SCALE = 5.0


@pytest.fixture
def wizard():
    """A front-wheel-steering simulated vehicle and its wizard context; disconnected afterwards."""
    vehicle, ctx = make()
    yield vehicle, ctx
    ctx.servos.release()
    vehicle.disconnect()


@pytest.fixture
def slower_clock():
    """Simulated time at SLOWER_TIME_SCALE instead of the usual 20x: a test that compares two waits of a few
    seconds then has a margin a busy test machine (e.g. a CI runner) can't eat up."""
    timebase.set_time_scale(min(SLOWER_TIME_SCALE, TEST_TIME_SCALE))
    yield
    timebase.set_time_scale(TEST_TIME_SCALE)


def steering_pwm(vehicle):
    """Pulse width the steering output currently sends."""
    return vehicle.servo_raw()[STEERING_OUTPUT - 1]


def test_panel_shows_at_least_eight_outputs(wizard):
    vehicle, ctx = wizard
    assert ctx.servos.output_count() == 8


def test_free_output_moves_with_set_servo(wizard):
    vehicle, ctx = wizard
    ctx.log(ctx.servos.nudge(FREE_OUTPUT, +1))
    assert wait_until(lambda: vehicle.servo_raw()[FREE_OUTPUT - 1] == 1525), vehicle.servo_raw()


def test_steering_panel_moves_servo_without_arming(wizard):
    vehicle, ctx = wizard
    ctx.log(ctx.servos.nudge(STEERING_OUTPUT, +1))
    ctx.log(ctx.servos.nudge(STEERING_OUTPUT, +1))
    # Two 10 % steps right: TRIM 1500 + 20 % of (MAX 1900 - TRIM 1500) = 1580.
    assert wait_until(lambda: steering_pwm(vehicle) == 1580), steering_pwm(vehicle)
    assert not vehicle.armed and vehicle.motor_tests == []


@pytest.mark.parametrize("label, expected_pwm", [("Test L", 1420), ("Test R", 1580)])
def test_steering_test_buttons_move_servo_without_arming(wizard, label, expected_pwm):
    vehicle, ctx = wizard
    enter(ctx, "steering_output")
    ctx.set_field_value("test_pct", 20)
    assert act(ctx, "steering_output", label)
    # 20 % of the half range (400 us) = 80 us left or right of TRIM 1500.
    assert wait_until(lambda: steering_pwm(vehicle) == expected_pwm), steering_pwm(vehicle)
    assert not vehicle.armed and vehicle.motor_tests == [], "the steering test must not use the motor test"


def test_steering_hold_ends_by_itself(wizard):
    vehicle, ctx = wizard
    # timed from the command: noticing the 1900 us late (a busy test machine) must not shorten the measured hold
    start = timebase.now()
    ctx.servos.hold_steering(1.0)
    assert wait_until(lambda: steering_pwm(vehicle) == 1900)
    assert wait_until(lambda: steering_pwm(vehicle) == 1500, timeout=STEERING_HOLD_SECONDS + 5)
    assert timebase.now() - start >= STEERING_HOLD_SECONDS - 0.5


def test_trim_plus_writes_and_centres_servo(wizard):
    vehicle, ctx = wizard
    enter(ctx, "steering_output")
    ctx.servos.nudge_endpoint(SERVO_FUNCTION_GROUND_STEERING, "TRIM", "trim_steering", +1)
    assert vehicle.get("SERVO1_TRIM") == 1510
    assert wait_until(lambda: steering_pwm(vehicle) == 1510)
    assert not vehicle.armed and vehicle.motor_tests == []


@pytest.mark.parametrize("reversed_output", [0, 1])
def test_max_minus_moves_servo_to_the_new_max(wizard, reversed_output):
    vehicle, ctx = wizard
    vehicle.set_param("SERVO1_REVERSED", reversed_output)
    enter(ctx, "steering_output")
    ctx.servos.nudge_endpoint(SERVO_FUNCTION_GROUND_STEERING, "MAX", "max_steering", -1)
    assert vehicle.get("SERVO1_MAX") == 1890
    # Whichever stick direction reaches the MAX end on this (possibly reversed) output, the pulse must be the new MAX.
    assert wait_until(lambda: steering_pwm(vehicle) == 1890), steering_pwm(vehicle)


def test_motor_endpoint_changes_value_but_does_not_drive(wizard):
    vehicle, ctx = wizard
    enter(ctx, "motor_output")
    ctx.servos.nudge_endpoint(SERVO_FUNCTION_THROTTLE, "MAX", "max_throttle", -1)
    assert vehicle.get("SERVO3_MAX") == 1890 and vehicle.motor_tests == [] and not vehicle.armed


def test_motor_panel_motor_test_arms_only_while_running(wizard):
    vehicle, ctx = wizard
    ctx.log(ctx.servos.nudge(THROTTLE_OUTPUT, +1))
    assert vehicle.motor_tests == [(1, 5)] and vehicle.armed, "the motor test arms the vehicle while it runs"
    # a second click during our own test is allowed
    ctx.log(ctx.servos.nudge(THROTTLE_OUTPUT, +1))
    assert vehicle.motor_tests == [(1, 5), (1, 10)]
    assert wait_until(lambda: not vehicle.armed), "the vehicle must be disarmed when the test ends"


def test_motor_test_refused_when_pilot_armed(wizard):
    vehicle, ctx = wizard
    vehicle.arm()
    with pytest.raises(VehicleError, match="disarm"):
        ctx.servos.nudge(THROTTLE_OUTPUT, +1)
    assert vehicle.motor_tests == []


def test_steering_hold_refused_when_armed(wizard):
    vehicle, ctx = wizard
    vehicle.arm()
    with pytest.raises(VehicleError, match="Disarm"):
        ctx.servos.hold_steering(0.5)


def test_output_with_a_boot_time_function_is_refused(wizard):
    vehicle, ctx = wizard
    # GPIO: ArduPilot doesn't move it on command, and its pin is only set up at boot - no function swap
    vehicle.set_param("SERVO6_FUNCTION", -1)
    with pytest.raises(VehicleError, match="function -1: .*only set up when the flight controller boots"):
        ctx.servos.nudge(6, +1)
    assert vehicle.get("SERVO6_FUNCTION") == -1


@pytest.mark.parametrize("function", [28, 140], ids=["gripper", "RCIN1Scaled"])
def test_functions_ardupilot_moves_directly(wizard, function):
    vehicle, ctx = wizard
    vehicle.set_param("SERVO6_FUNCTION", function)
    assert ctx.servos.nudge(6, +1).startswith("SERVO6: ")
    assert vehicle.get("SERVO6_FUNCTION") == function, "no function swap needed"


def test_stop_releases_steering_hold(wizard):
    vehicle, ctx = wizard
    ctx.servos.hold_steering(-1.0)
    assert wait_until(lambda: steering_pwm(vehicle) == 1100)
    ctx.emergency()
    assert wait_until(lambda: steering_pwm(vehicle) == 1500), "STOP must hand the steering back to the transmitter"


def test_deadzone_probe_keeps_one_motor_test_running_and_stops_the_wheel(wizard):
    vehicle, ctx = wizard
    sent = []
    original = vehicle.motor_test

    def recording_motor_test(instance, percent, seconds=2.0):
        """Record every motor test the probe sends, then let the simulator run it."""
        sent.append((instance, percent, seconds))
        return original(instance, percent, seconds)

    vehicle.motor_test = recording_motor_test
    enter(ctx, "motor_output")
    # wheels lifted: yes, start
    ctx.confirm = lambda message: True
    threading.Thread(target=lambda: (wait_until(lambda: len(sent) >= 3), ctx.press_continue()), daemon=True).start()
    action_find_throttle_deadzone(ctx)
    ramp = sent[:-1]
    stop = sent[-1]
    assert all(
        seconds > DEADZONE_STEP_SECONDS for _, _, seconds in ramp
    ), "a lapsing test would disarm/re-arm each step"
    assert stop[1] == 0 and stop[2] < 0.5, "the probe must stop the wheel when it ends"


def test_deadzone_probe_logs_power_and_pulse_width(wizard):
    vehicle, ctx = wizard
    messages = []
    ctx.set_output(log=messages.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    enter(ctx, "motor_output")
    # wheels lifted: yes, start
    ctx.confirm = lambda message: True

    def steps():
        return [line for line in messages if line.startswith("Motor power")]

    threading.Thread(target=lambda: (wait_until(lambda: len(steps()) >= 3), ctx.press_continue()), daemon=True).start()
    action_find_throttle_deadzone(ctx)
    # TRIM 1500 + percent of (MAX 1900 - TRIM 1500): 4 us per percent (MOT_THR_MIN is 0 during the probe)
    assert steps()[:2] == ["Motor power 1% - SERVO3: 1504 us", "Motor power 2% - SERVO3: 1508 us"]


def test_motor_test_buttons_are_not_blocked_by_their_own_arming():
    buttons = [
        action
        for field in BY_ID["motor_output"].fields
        for action in (field.inline_actions or [])
        if action.label.startswith("Test")
    ]
    assert buttons and all(action.armed is None for action in buttons)


def test_panel_log_shows_the_reported_pulse_width(wizard):
    vehicle, ctx = wizard
    # Steering stick +10 %: TRIM 1500 + 10 % of (MAX 1900 - TRIM 1500) = 1540 us, as the output reports it.
    assert ctx.servos.nudge(STEERING_OUTPUT, +1).startswith("SERVO1: 1540 us (steering held at +10%")
    # Throttle motor test +5 %: 1500 + 5 % of 400 = 1520 us.
    assert ctx.servos.nudge(THROTTLE_OUTPUT, +1).startswith("SERVO3: 1520 us (throttle motor test +5%")
    assert ctx.servos.nudge(FREE_OUTPUT, +1) == "SERVO5: 1525 us"


def test_panel_clicks_add_up_while_the_hold_lasts(wizard, slower_clock):
    vehicle, ctx = wizard
    # clicks 2 s apart: longer than the Test buttons' hold, shorter than the panel's
    assert STEERING_HOLD_SECONDS < 2.0 < PANEL_STEERING_HOLD_SECONDS
    first_click = timebase.now()
    assert "+10%" in ctx.servos.nudge(STEERING_OUTPUT, +1)
    # 2 s after the first click started, however long it took to report its pulse width
    timebase.sleep(first_click + 2.0 - timebase.now())
    assert "+20%" in ctx.servos.nudge(STEERING_OUTPUT, +1)
    # after the hold the steering is released and the next click starts from the centre again
    timebase.sleep(PANEL_STEERING_HOLD_SECONDS + 1.0)
    assert "+10%" in ctx.servos.nudge(STEERING_OUTPUT, +1)


def checks_at_command_time(vehicle, name):
    """Record the arming-check parameter at the moment each motor test command is sent."""
    seen = []
    send = vehicle.motor_test

    def spying(*args):
        """Note the parameter, then send the command."""
        seen.append(vehicle.get(name))
        return send(*args)

    vehicle.motor_test = spying
    return seen


def test_simulator_refuses_a_motor_test_while_an_arming_check_fails(wizard):
    vehicle, ctx = wizard
    vehicle.failing_arming_check = "Arm: Compass not calibrated"
    # without the wizard's bypass (MAV_RESULT_FAILED = 4)
    assert vehicle.motor_test(1, 15, 2.0) == 4 and "Compass not calibrated" in vehicle.last_text()


@pytest.mark.parametrize(
    "name, enabled, skip_all",
    [("ARMING_CHECK", 1, 0), ("ARMING_SKIPCHK", 0, -1)],
    ids=["older firmware", "current firmware"],
)
def test_motor_test_skips_the_arming_checks_only_while_it_starts(wizard, name, enabled, skip_all):
    vehicle, ctx = wizard
    if not vehicle.has_parameter(name):
        # current firmware: ARMING_SKIPCHK instead of ARMING_CHECK
        vehicle._store_param(name, float(enabled), 2, 0, 0)
    vehicle.failing_arming_check = "Arm: Compass not calibrated"
    seen = checks_at_command_time(vehicle, name)
    ctx.servos.run_motor_test(MOTOR_TEST_INSTANCE_THROTTLE, 15)
    assert seen == [skip_all], "all checks off when the command is sent"
    assert vehicle.get(name) == enabled, "restored right afterwards"
    assert vehicle.motor_tests, "the motor test ran"


def test_arming_checks_are_restored_when_the_motor_test_is_refused_anyway(wizard):
    vehicle, ctx = wizard
    # refused for another reason (MAV_RESULT_FAILED = 4)
    vehicle.motor_test = lambda instance, percent, seconds: 4
    vehicle.last_text = lambda: "FC says: Motor Test: Safety switch"
    with pytest.raises(VehicleError, match="safety switch pressed") as refusal:
        ctx.servos.run_motor_test(MOTOR_TEST_INSTANCE_THROTTLE, 15)
    assert "FC says: Motor Test: Safety switch" in str(refusal.value)
    assert vehicle.get("ARMING_CHECK") == 1


def test_deadzone_probe_works_despite_failing_arming_checks(wizard):
    vehicle, ctx = wizard
    vehicle.failing_arming_check = "Arm: Radio failsafe on"
    enter(ctx, "motor_output")
    # wheels lifted: yes, start
    ctx.confirm = lambda message: True
    threading.Thread(
        target=lambda: (wait_until(lambda: len(vehicle.motor_tests) >= 3), ctx.press_continue()), daemon=True
    ).start()
    action_find_throttle_deadzone(ctx)
    assert len(vehicle.motor_tests) >= 3 and vehicle.get("ARMING_CHECK") == 1


# ---- function swap: outputs whose function ArduPilot resolves at runtime (e.g. a gimbal servo) -------------------

GIMBAL_OUTPUT = 6
MOUNT_TILT = 7


@pytest.fixture
def gimbal(wizard):
    """A tilt servo of a gimbal on SERVO6; the log goes into a list."""
    vehicle, ctx = wizard
    vehicle.set_param(f"SERVO{GIMBAL_OUTPUT}_FUNCTION", MOUNT_TILT)
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    # the simulator has picked up the function (it re-reads them once a second, like Rover)
    timebase.sleep(1.5)
    return vehicle, ctx, log


def test_gimbal_output_moves_and_gets_its_function_back(gimbal):
    vehicle, ctx, log = gimbal
    trim = vehicle.get(f"SERVO{GIMBAL_OUTPUT}_TRIM")
    first = ctx.servos.nudge(GIMBAL_OUTPUT, +1)
    assert first == (
        f"SERVO{GIMBAL_OUTPUT}: {int(trim) + 25} us (mount tilt paused, its function returns "
        f"{PANEL_STEERING_HOLD_SECONDS:g} s after the last click)"
    )
    # the wizard waited until ArduPilot stopped driving the output with the gimbal function
    assert vehicle._servo_output_pwm(GIMBAL_OUTPUT) == int(trim) + 25
    assert vehicle.get(f"SERVO{GIMBAL_OUTPUT}_FUNCTION") == 0
    assert ctx.store.get("swapped_servo_functions") == {str(GIMBAL_OUTPUT): MOUNT_TILT}
    # further clicks step on, without switching again
    assert ctx.servos.nudge(GIMBAL_OUTPUT, +1).startswith(f"SERVO{GIMBAL_OUTPUT}: {int(trim) + 50} us (its function")
    # the hold ends: the gimbal function is back
    # (the restore writes the function first, then forgets the swap and logs it: wait for that last step)
    assert wait_until(lambda: f"SERVO{GIMBAL_OUTPUT}: back to mount tilt." in log)
    assert (
        vehicle.get(f"SERVO{GIMBAL_OUTPUT}_FUNCTION") == MOUNT_TILT and ctx.store.get("swapped_servo_functions") == {}
    )
    # nothing for "Show all changes" or "Revert": the swap isn't a setup change
    assert f"SERVO{GIMBAL_OUTPUT}_FUNCTION" not in [name for name, _, _ in ctx.changes.items()]


def test_stop_gives_the_function_back_at_once(gimbal):
    vehicle, ctx, log = gimbal
    ctx.servos.nudge(GIMBAL_OUTPUT, -1)
    ctx.emergency()
    # (in the background: the UI thread must not wait for the parameter write)
    assert wait_until(lambda: vehicle.get(f"SERVO{GIMBAL_OUTPUT}_FUNCTION") == MOUNT_TILT)


def test_a_failed_restore_is_retried_on_the_next_start(gimbal):
    vehicle, ctx, log = gimbal
    ctx.servos.nudge(GIMBAL_OUTPUT, +1)
    write = vehicle.set_param

    def lost_link(name, value):
        """The link is gone when the function should be restored."""
        raise VehicleError("No confirmation from the vehicle")

    vehicle.set_param = lost_link
    ctx.servos.restore_functions()
    assert any("Could not give SERVO6 its function 7 back" in line for line in log)
    assert ctx.store.get("swapped_servo_functions") == {str(GIMBAL_OUTPUT): MOUNT_TILT}
    # the wizard starts again (new context on the same vehicle and store): once its log is connected, the
    # function is restored and that is logged
    vehicle.set_param = write
    restarted, restart_log = Ctx(vehicle, ctx.store), []
    assert vehicle.get(f"SERVO{GIMBAL_OUTPUT}_FUNCTION") == 0, "nothing happens before the log is connected"
    restarted.set_output(log=restart_log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    assert vehicle.get(f"SERVO{GIMBAL_OUTPUT}_FUNCTION") == MOUNT_TILT
    assert ctx.store.get("swapped_servo_functions") == {}
    assert restart_log[0] == "Restoring servo output functions the wizard had switched off: SERVO6"


def test_a_redirected_log_does_not_end_a_running_swap(gimbal):
    vehicle, ctx, log = gimbal
    ctx.servos.nudge(GIMBAL_OUTPUT, +1)
    # a dialog (e.g. a probe) redirects the log while the swap is active
    with ctx.redirect_output(log=log.append, on_wait=lambda waiting: None):
        assert vehicle.get(f"SERVO{GIMBAL_OUTPUT}_FUNCTION") == 0


# ---- flight mode: a failsafe may have switched to HOLD, where Rover centres the steering --------------------------


def test_steering_test_switches_from_hold_to_manual(wizard):
    vehicle, ctx = wizard
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    vehicle.custom_mode = MODES["HOLD"]
    ctx.servos.nudge(STEERING_OUTPUT, +1)
    assert vehicle.custom_mode == MODES["MANUAL"]
    assert "Switched to MANUAL mode for the steering test (was HOLD)." in log
    assert wait_until(lambda: steering_pwm(vehicle) > 1500)


def test_failsafe_hold_during_a_steering_hold_is_undone_by_the_next_click(wizard):
    vehicle, ctx = wizard
    ctx.servos.nudge(STEERING_OUTPUT, +1)
    assert wait_until(lambda: steering_pwm(vehicle) > 1500)
    # a failsafe switches to Hold: the steering centres
    vehicle.custom_mode = MODES["HOLD"]
    assert wait_until(lambda: steering_pwm(vehicle) == 1500)
    ctx.servos.nudge(STEERING_OUTPUT, +1)
    assert vehicle.custom_mode == MODES["MANUAL"]
    assert wait_until(lambda: steering_pwm(vehicle) > 1500)


def test_motor_test_and_plain_outputs_work_in_hold_without_a_mode_change(wizard):
    vehicle, ctx = wizard
    vehicle.custom_mode = MODES["HOLD"]
    # ArduPilot runs the motor test instead of the mode's output (Rover::set_servos); DO_SET_SERVO ignores the mode
    ctx.servos.nudge(THROTTLE_OUTPUT, +1)
    assert vehicle.motor_tests
    assert ctx.servos.nudge(FREE_OUTPUT, +1).startswith(f"SERVO{FREE_OUTPUT}: ")
    assert vehicle.custom_mode == MODES["HOLD"]
