"""Compass calibration from a GPS heading: a two-antenna GPS's own heading (compass step, "From GPS"), and the
heading of a straight run with a normal GPS (phase-2 step "Compass from a straight run")."""

import math
import random
import threading
import time

import pytest

from rover_wizard.motion.gps_heading import heading_after_straight_run
from rover_wizard.mavlink_defs import MAVLINK_MSG_ID_GPS_RAW_INT
from rover_wizard.steps.compass_drive import applies
from rover_wizard.util import timebase
from rover_wizard.util.errors import VehicleError
from rover_wizard.vehicle.link import STREAMS, DriveSample

from util.helpers import BY_ID, act, check, enter, make, wait_until

HZ = 20


def run(courses_deg, speeds, yaw_rates_deg=None, stopped=True):
    """Samples of a run: GPS course [deg] and speed per sample, gyro turn rate [deg/s] (default 0)."""
    yaw_rates_deg = yaw_rates_deg or [0.0] * len(courses_deg)
    samples = [
        DriveSample(
            time=index / HZ,
            forward_speed=speed,
            yaw_rate=math.radians(rate),
            gps_course=None if course is None else math.radians(course),
            gps_speed=speed,
        )
        for index, (course, speed, rate) in enumerate(zip(courses_deg, speeds, yaw_rates_deg))
    ]
    if stopped:
        samples.append(DriveSample(time=len(samples) / HZ, forward_speed=0.0, yaw_rate=0.0))
    return samples


# ---------------------------------------------------------------------------------------------- the maths
def test_straight_run_gives_its_course():
    noise = random.Random(3)
    result = heading_after_straight_run(run([120 + noise.gauss(0, 2) for _ in range(100)], [1.0] * 100))
    assert result.heading == pytest.approx(120, abs=1)
    assert result.spread < 3 and result.distance == pytest.approx(5, abs=0.2)


def test_a_turn_while_stopping_is_followed_by_the_gyro():
    # straight at 120 deg, then while slowing down the vehicle turns right by 10 deg (10 deg/s for 1 s)
    courses = [120.0] * 100 + [None] * 20
    speeds = [1.0] * 100 + [0.3] * 20
    rates = [0.0] * 100 + [10.0] * 20
    assert heading_after_straight_run(run(courses, speeds, rates)).heading == pytest.approx(130, abs=0.6)


def test_headings_around_north_average_correctly():
    courses = [358.0, 2.0] * 50
    heading = heading_after_straight_run(run(courses, [1.0] * 100)).heading
    assert min(heading, 360 - heading) < 0.5


@pytest.mark.parametrize(
    "courses, speeds, stopped, message",
    [
        ([90 + index * 0.4 for index in range(100)], [1.0] * 100, True, "wasn't straight"),
        ([90.0] * 100, [0.3] * 100, True, "Too little straight driving"),
        ([90.0] * 100, [1.0] * 100, False, "hadn't stopped"),
        # reversing: its course is opposite the heading, so it doesn't count
        ([270.0] * 100, [-1.0] * 100, True, "Too little straight driving"),
    ],
    ids=["curved", "too slow", "not stopped", "reversing"],
)
def test_runs_that_give_no_heading_are_refused(courses, speeds, stopped, message):
    samples = run(courses, [abs(speed) for speed in speeds], stopped=stopped)
    for sample, speed in zip(samples, speeds):
        sample.forward_speed = speed
    with pytest.raises(VehicleError, match=message):
        heading_after_straight_run(samples)


# ---------------------------------------------------------------------------------------------- the vehicle
@pytest.fixture
def heavy():
    """A heavy (not rotatable) simulated rover with GPS, facing 68.8 deg; disconnected afterwards."""
    veh, ctx = make()
    ctx.store.set("heavy_vehicle", True)
    ctx.store.set("has_gps", True)
    ctx.store.set("has_compass", True)
    veh.heading = 1.2
    yield veh, ctx
    veh.disconnect()


def test_the_step_is_for_heavy_vehicles_whose_gps_measures_no_heading(heavy):
    veh, ctx = heavy
    assert applies(ctx)
    veh.gps_measures_heading = True
    assert wait_until(lambda: not applies(ctx), timeout=5)
    ctx.store.set("heavy_vehicle", False)
    veh.gps_measures_heading = False
    assert not applies(ctx)


def test_from_gps_fills_the_heading_of_a_two_antenna_gps(heavy):
    veh, ctx = heavy
    enter(ctx, "compass")
    (button,) = next(field for field in BY_ID["compass"].fields if field.key == "magcal_heading").inline_actions
    check("hidden for a normal GPS", not button.when(ctx))
    veh.gps_measures_heading = True
    assert wait_until(lambda: button.when(ctx), timeout=5)
    assert act(ctx, "compass", "From GPS")
    assert ctx.values["magcal_heading"] == pytest.approx(math.degrees(1.2), abs=0.1)
    # configured for heading, but no heading right now (e.g. no fix on the second antenna)
    veh.gps_heading_available = False
    time.sleep(0.2)
    with pytest.raises(VehicleError, match="no heading right now"):
        button.fn(ctx)


def straight_run_setup(heavy, answer=True):
    """Arm the vehicle and answer the confirmation; returns the log lines."""
    veh, ctx = heavy
    lines = []
    ctx.set_output(log=lines.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    ctx.pause = lambda message, during=None: None
    ctx.confirm = lambda message: answer
    ctx.values.update(runway=40)
    enter(ctx, "compass_drive")
    veh.arm()
    return lines


def test_automatic_straight_run_calibrates_with_the_true_heading(heavy):
    veh, ctx = heavy
    lines = straight_run_setup(heavy)
    check("action ran", act(ctx, "compass_drive", "Drive straight"), "\n".join(lines))
    check("one calibration", len(veh.fixed_yaw_calibrations) == 1, veh.fixed_yaw_calibrations)
    check(
        "with the heading the vehicle stands at",
        veh.fixed_yaw_calibrations[0] == pytest.approx(math.degrees(veh.heading) % 360, abs=2),
        (veh.fixed_yaw_calibrations, math.degrees(veh.heading)),
    )
    assert veh.message_rates[MAVLINK_MSG_ID_GPS_RAW_INT] == dict(STREAMS)[MAVLINK_MSG_ID_GPS_RAW_INT]


def test_declining_the_heading_changes_nothing(heavy):
    veh, ctx = heavy
    straight_run_setup(heavy, answer=False)
    assert act(ctx, "compass_drive", "Drive straight")
    assert veh.fixed_yaw_calibrations == []


def test_the_pilot_can_drive_the_straight_run(heavy):
    veh, ctx = heavy
    straight_run_setup(heavy)
    ctx.values["how_compass"] = "manual"

    def pilot():
        """Drive straight for 6 s at speed, stop, wait until still, press Continue."""
        wait_until(lambda: ctx.waiting)
        veh.pilot_sticks = (0.0, 0.4)
        wait_until(lambda: veh.speed > 0.6)
        # 6 s on the wizard's clock: the same drive whatever the test's time scale (5x on macOS, 20x elsewhere)
        start = timebase.now()
        while timebase.now() - start < 6:
            time.sleep(0.01)
        veh.pilot_sticks = (0.0, 0.0)
        wait_until(lambda: abs(veh.speed) < 0.02)
        time.sleep(0.1)
        ctx.press_continue()

    threading.Thread(target=pilot, daemon=True).start()
    assert act(ctx, "compass_drive", "Drive straight")
    assert veh.fixed_yaw_calibrations == [pytest.approx(math.degrees(veh.heading) % 360, abs=2)]


def test_no_gps_fix_no_run(heavy):
    veh, ctx = heavy
    straight_run_setup(heavy)
    veh.gps_fix_type = 1
    time.sleep(0.2)
    assert not act(ctx, "compass_drive", "Drive straight")
    assert veh.distance_travelled == 0
