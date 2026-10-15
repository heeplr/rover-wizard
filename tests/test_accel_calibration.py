"""6-position accelerometer calibration: ArduPilot repeats each position request every second until it is
confirmed (AP_AccelCal::update) - the pilot must still be asked only once per position. ArduPilot can't be told to
stop it: a cancel says so, and starting again continues at the position waited for."""

import pytest

from rover_wizard.util import timebase
from rover_wizard.util.errors import Cancelled
from rover_wizard.mavlink_defs import MAV_CMD_ACCELCAL_VEHICLE_POS
from rover_wizard.steps.accel import ACCEL_POSITION_COUNT, STILL_RUNNING_NOTE, action_calibrate_accelerometer

from util.helpers import make

# a pilot takes a few seconds per position - meanwhile ArduPilot repeats its request several times
PILOT_SECONDS_PER_POSITION = 3.0


def calibrate(vehicle, ctx):
    """Run the calibration with a slow simulated pilot; the questions asked and the log."""
    questions, log = [], []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    ctx.pause = lambda message, during=None: (questions.append(message), timebase.sleep(PILOT_SECONDS_PER_POSITION))
    action_calibrate_accelerometer(ctx)
    return questions, log


def test_one_question_per_position_and_progress():
    vehicle, ctx = make()
    questions, log = calibrate(vehicle, ctx)
    assert len(questions) == ACCEL_POSITION_COUNT
    assert [line for line in log if line.endswith("done.")] == [
        f"Position {number} of {ACCEL_POSITION_COUNT} done." for number in range(1, ACCEL_POSITION_COUNT + 1)
    ]
    assert log[-1] == "Accelerometer calibration SUCCESSFUL."
    vehicle.disconnect()


def test_a_lost_confirmation_is_repeated_without_asking_again():
    vehicle, ctx = make()
    send = vehicle.command_long
    lost = []

    def losing_the_first_confirmation(command, *params, **options):
        """The first position confirmation never reaches the flight controller."""
        if command == MAV_CMD_ACCELCAL_VEHICLE_POS and not lost:
            lost.append(params[0])
            return None
        return send(command, *params, **options)

    vehicle.command_long = losing_the_first_confirmation
    questions, log = calibrate(vehicle, ctx)
    assert lost and len(questions) == ACCEL_POSITION_COUNT
    assert log[-1] == "Accelerometer calibration SUCCESSFUL."
    vehicle.disconnect()


def test_cancel_says_the_calibration_goes_on_and_a_restart_continues_there():
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    questions = []

    def pilot_cancels_at_the_third_position(message, during=None):
        """Confirms two positions, then presses Cancel."""
        questions.append(message)
        if len(questions) == 3:
            raise Cancelled()
        timebase.sleep(PILOT_SECONDS_PER_POSITION)

    ctx.pause = pilot_cancels_at_the_third_position
    with pytest.raises(Cancelled):
        action_calibrate_accelerometer(ctx)
    assert log[-1] == STILL_RUNNING_NOTE
    cancelled_at = questions[-1]
    questions_again, log = calibrate(vehicle, ctx)
    assert questions_again[0] == cancelled_at and len(questions_again) == ACCEL_POSITION_COUNT - 2
    assert log[-1] == "Accelerometer calibration SUCCESSFUL."
    vehicle.disconnect()
