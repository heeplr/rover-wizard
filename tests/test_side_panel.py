"""Side panel: compass tape, accelerometer and gyro bars - their text, the vehicle readings and when they show."""

import math

import pytest

from rover_wizard.steps.util.rover_hardware import is_accelerometer_fitted, is_gyro_fitted
from rover_wizard.ui.monitor import (
    BAR_LEVELS,
    CELLS_PER_CIRCLE,
    COMPASS_TAPE_WIDTH,
    COMPASS_TICK_DEGREES,
    DEGREES_PER_CELL,
    accel_bars_text,
    bar_for_signed,
    bar_row,
    compass_tape_text,
    gyro_bars_text,
)

from util.helpers import make, wait_until

CENTRE = COMPASS_TAPE_WIDTH // 2


def tape(heading):
    """The tape line and the indicator line for a heading."""
    return compass_tape_text(heading).split("\n")


def column(degrees, heading):
    """Where an angle is drawn, in cells right (+) or left (-) of the middle column, for a heading."""
    offset = round(degrees / DEGREES_PER_CELL) - round(heading / DEGREES_PER_CELL)
    # the shorter way round the circle
    return (offset + CELLS_PER_CIRCLE // 2) % CELLS_PER_CIRCLE - CELLS_PER_CIRCLE // 2


# less than half a cell off a compass point still shows that point in the middle
SLIGHTLY_OFF = 0.4 * DEGREES_PER_CELL


@pytest.mark.parametrize(
    "heading, letter", [(0, "N"), (90, "E"), (180, "S"), (270, "W"), (360 - SLIGHTLY_OFF, "N"), (SLIGHTLY_OFF, "N")]
)
def test_heading_is_centred_under_the_indicator(heading, letter):
    line, indicator = tape(heading)
    assert len(line) == COMPASS_TAPE_WIDTH and line[CENTRE] == letter
    assert indicator == " " * CENTRE + "▲"


def test_tape_wraps_around_north():
    line, _ = tape(350)
    north, northeast = CENTRE + column(360, 350), CENTRE + column(45, 350)
    # north is to the right of 350 degrees; two-letter points cover the cell after them
    assert column(360, 350) > 0 and line[north] == "N"
    assert line[northeast : northeast + 2] == "NE"


def test_a_tick_every_10_degrees_and_dots_between():
    line, _ = tape(350)
    for degrees in range(0, 360, COMPASS_TICK_DEGREES):
        position = CENTRE + column(degrees, 350)
        # on the tape and not covered by a compass point's letters
        if 0 <= position < COMPASS_TAPE_WIDTH and line[position] not in "NESW":
            assert line[position] == "╷", degrees
    assert set(line) <= set("╷.NESW") and line.count(".") > line.count("╷")


def test_no_heading_data():
    assert "no ATTITUDE" in compass_tape_text(None)


@pytest.mark.parametrize(
    "value, bar",
    [(0.0, "▅"), (-2.0, "▁"), (2.0, "█"), (-9.0, "▁"), (9.0, "█"), (-1.0, "▃")],
    ids=[
        "zero = half height like a centred RC stick",
        "full negative",
        "full positive",
        "clamped low",
        "clamped high",
        "half negative",
    ],
)
def test_signed_bar_levels(value, bar):
    assert bar_for_signed(value, 2.0) == bar


def test_accel_and_gyro_look_like_the_rc_bars():
    assert accel_bars_text((0.0, -1.0, 2.0)) == " X  Y  Z \n ▅  ▃  █ "
    assert gyro_bars_text((-180.0, 0.0, 999.0)) == " X  Y  Z \n ▁  ▅  █ "
    # the RC/servo bars are unchanged by the shared code: 1000 / 1500 / 2000 us
    assert bar_row([1000, 1500, 2000]) == f" {BAR_LEVELS[0]}  {BAR_LEVELS[4]}  {BAR_LEVELS[-1]} "
    assert "no SCALED_IMU" in accel_bars_text(None)


def test_vehicle_heading_and_imu_from_telemetry():
    vehicle, ctx = make()
    # the simulated rover stands still, pointing 30 degrees west of north
    vehicle.heading = math.radians(-30)
    assert wait_until(lambda: vehicle.heading_degrees() is not None and abs(vehicle.heading_degrees() - 330) < 1)
    assert wait_until(lambda: vehicle.imu() is not None)
    accel, gyro = vehicle.imu()
    # at rest: 1 g of gravity (on whichever axis the simulated board is mounted), no rotation
    assert math.dist(accel, (0.0, 0.0, 0.0)) == pytest.approx(1.0, abs=0.02)
    assert gyro == pytest.approx((0.0, 0.0, 0.0), abs=1.0)
    vehicle.disconnect()


def test_sensors_count_as_fitted_when_detected():
    vehicle, ctx = make()
    assert is_accelerometer_fitted(ctx) and is_gyro_fitted(ctx)
    vehicle.set_param("INS_GYR_ID", 0)
    assert is_accelerometer_fitted(ctx) and not is_gyro_fitted(ctx)
    vehicle.disconnect()
