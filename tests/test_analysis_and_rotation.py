"""Pure maths: drive-test analysis and board-orientation matching."""

import math

import pytest

from rover_wizard.motion.analysis import analyze_top_speed_run
from rover_wizard.motion.analysis import step_response_metrics
from rover_wizard.motion.analysis import suggest_pi_gains
from rover_wizard.util.rotation import ORIENTATION_CHOICES
from rover_wizard.util.rotation import best_matching_rotation
from rover_wizard.util.rotation import gravity_direction_from_accelerometer
from rover_wizard.util.rotation import rotation_matrix_for_name
from rover_wizard.util.rotation import transform
from rover_wizard.util.rotation import transpose
from rover_wizard.vehicle.link import DriveSample

from util.helpers import check


def test_analysis():
    """Check the measurement maths (top-speed detection, step-response metrics, P/I suggestions) against known inputs."""
    samples = []
    speed = 0.0
    for index in range(400):
        time_s = index * 0.05
        # first-order approach to 3 m/s
        speed += (3.0 - speed) / 0.8 * 0.05
        samples.append(
            DriveSample(
                time=time_s,
                forward_speed=speed,
                yaw_rate=0.0,
                steering_input=0.0,
                throttle_input=1.0 if time_s < 15 else 0.0,
            )
        )
    result = analyze_top_speed_run(samples)
    check("top speed ~3", result and abs(result.top_speed - 3.0) < 0.1, result and f"{result.top_speed:.2f}")
    response = step_response_metrics(
        [i * 0.1 for i in range(100)], [min(2.0, i * 0.1 * 2) * 1.2 for i in range(100)], 2.0, 0.0, 9.9
    )
    check("step metrics overshoot detected", response and response.overshoot > 0.1)
    suggestion = suggest_pi_gains(response, 1.0, 1.0, 3.0)
    check(
        "PI suggestion lowers gains on overshoot",
        suggestion.p_gain < 1.0 and suggestion.i_gain < 1.0,
        suggestion.reason,
    )


def test_rotation_maths():
    """Rotation formulas against ArduPilot's, and recovery of every mounting from realistic accelerometer data."""

    def close(a, b):
        return all(abs(x - y) < 1e-9 for x, y in zip(a, b))

    check("Roll180 negates y and z", close(transform(rotation_matrix_for_name("Roll180"), (1, 2, 3)), (1, -2, -3)))
    half_sqrt2 = 2**0.5 / 2
    check(
        "Yaw45 maps x to (x-y, x+y)/sqrt2",
        close(transform(rotation_matrix_for_name("Yaw45"), (1, 0, 0)), (half_sqrt2, half_sqrt2, 0)),
    )
    vector = (0.3, -0.5, 0.8)
    roll_then_yaw = transform(rotation_matrix_for_name("Yaw45"), transform(rotation_matrix_for_name("Roll180"), vector))
    check(
        "compound name applies roll first, then yaw",
        close(transform(rotation_matrix_for_name("Yaw45Roll180"), vector), roll_then_yaw),
    )
    recovered = 0
    for true_name, _ in ORIENTATION_CHOICES:
        to_board = transpose(rotation_matrix_for_name(true_name))
        # What a board mounted this way reads (specific force = -gravity) level and nose-down:
        level = tuple(-c for c in transform(to_board, (0.0, 0.0, 1.0)))
        nose_down = tuple(-c for c in transform(to_board, (1.0, 0.0, 0.0)))
        name, _ = best_matching_rotation(
            [n for n, _ in ORIENTATION_CHOICES],
            gravity_direction_from_accelerometer(level),
            gravity_direction_from_accelerometer(nose_down),
        )
        recovered += name == true_name
    check(
        "every orientation recovered from realistic readings",
        recovered == len(ORIENTATION_CHOICES),
        f"{recovered}/{len(ORIENTATION_CHOICES)}",
    )


# ArduPilot's own code for two combined rotations (AP_Math/vector3.cpp, Vector3::rotate), applied by hand to
# (1, 2, 3) with h = sqrt(2)/2:
#   ROTATION_ROLL_90_YAW_45:  roll 90 -> (1, -3, 2), then yaw 45 -> (h*(1+3), h*(1-3), 2)
#   ROTATION_ROLL_180_YAW_45: (h*(x+y), h*(x-y), -z) = (3h, -h, -3)
H = math.sqrt(2) / 2


@pytest.mark.parametrize("name, expected", [("Yaw45Roll90", (4 * H, -2 * H, 2.0)), ("Yaw45Roll180", (3 * H, -H, -3.0))])
def test_combined_rotations_match_ardupilot(name, expected):
    assert transform(rotation_matrix_for_name(name), (1.0, 2.0, 3.0)) == pytest.approx(expected)


def test_top_speed_run_without_full_throttle():
    # a manual run where the pilot never reached full throttle: a result, not a crash
    from rover_wizard.vehicle.link import DriveSample

    samples = [DriveSample(time=t / 10, forward_speed=min(1.0, t / 20), throttle_input=0.3) for t in range(60)]
    assert analyze_top_speed_run(samples).speed_was_steady is False


def test_step_cut_short_before_its_end_gives_no_metrics():
    # the run stopped (distance limit) before the last part of the step: no steady state to judge
    times = [t / 10 for t in range(30)]
    assert step_response_metrics(times, [1.0] * 30, target=1.0, step_on=0.0, step_off=10.0) is None
