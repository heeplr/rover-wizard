"""How the link turns raw telemetry into what the drive tests measure (DriveSample) and into stick positions.

The simulator has its own sample(), so the real link's decoding - used with every real vehicle - is checked here
with stored messages, no connection needed."""

import math
from types import SimpleNamespace

import pytest

from rover_wizard.vehicle.link import Vehicle

# the vehicle heads east (yaw = 90 degrees)
EAST = math.pi / 2


def vehicle_with(**messages):
    """An unconnected Vehicle that has just received these messages (type -> fields)."""
    vehicle = Vehicle()
    vehicle._params.update(RCMAP_ROLL=1, RCMAP_THROTTLE=3)
    for channel in (1, 3):
        vehicle._params.update({f"RC{channel}_MIN": 1100, f"RC{channel}_TRIM": 1500, f"RC{channel}_MAX": 1900})
    for msg_type, fields in messages.items():
        vehicle._store_message(msg_type, SimpleNamespace(**fields))
    return vehicle


def rc(steering_pwm, throttle_pwm, count=8):
    """RC_CHANNELS fields with steering on channel 1 and throttle on channel 3."""
    fields = {f"chan{channel}_raw": 1500 for channel in range(1, 19)}
    fields.update(chancount=count, chan1_raw=steering_pwm, chan3_raw=throttle_pwm)
    return fields


def test_speed_along_the_heading_from_the_position_estimate():
    vehicle = vehicle_with(
        ATTITUDE={"yaw": EAST, "yawspeed": 0.3},
        LOCAL_POSITION_NED={"x": 12.0, "y": 34.0, "vx": 0.0, "vy": 2.0},
        RC_CHANNELS=rc(1700, 1900),
    )
    sample = vehicle.sample()
    assert sample.forward_speed == pytest.approx(2.0)
    assert (sample.yaw_rate, sample.north, sample.east) == (0.3, 12.0, 34.0)
    assert (sample.steering_input, sample.throttle_input) == pytest.approx((0.5, 1.0))
    assert sample.is_usable()


def test_driving_backwards_shows_as_negative_speed():
    vehicle = vehicle_with(
        ATTITUDE={"yaw": EAST, "yawspeed": 0.0},
        LOCAL_POSITION_NED={"x": 0.0, "y": 0.0, "vx": 0.0, "vy": -1.5},
        RC_CHANNELS=rc(1500, 1100),
    )
    assert vehicle.sample().forward_speed == pytest.approx(-1.5)


@pytest.mark.parametrize("throttle_pwm, expected", [(1900, 1.2), (1100, -1.2)], ids=["forward", "reversing"])
def test_without_a_position_estimate_the_ground_speed_is_signed_by_the_throttle(throttle_pwm, expected):
    # VFR_HUD's ground speed has no sign: the throttle stick tells whether the vehicle reverses
    vehicle = vehicle_with(
        ATTITUDE={"yaw": 0.0, "yawspeed": 0.1}, VFR_HUD={"groundspeed": 1.2}, RC_CHANNELS=rc(1500, throttle_pwm)
    )
    sample = vehicle.sample()
    assert sample.forward_speed == pytest.approx(expected)
    assert sample.north is None


def test_no_telemetry_gives_an_unusable_sample():
    sample = vehicle_with().sample()
    assert (sample.steering_input, sample.throttle_input, sample.forward_speed, sample.yaw_rate) == (None,) * 4
    assert not sample.is_usable()


def test_channels_the_receiver_does_not_deliver_have_no_stick_position():
    # a 2-channel receiver: no channel 3; and a channel reported as "unused" (0) / UINT16_MAX
    assert vehicle_with(RC_CHANNELS=rc(1700, 1900, count=2)).rc_inputs() == (pytest.approx(0.5), None)
    vehicle = vehicle_with()
    assert vehicle.rc_norm(1, 0) is None and vehicle.rc_norm(1, 65535) is None


def test_last_text_finds_the_newest_matching_message():
    vehicle = vehicle_with()
    for text in ("PreArm: Battery low", "Mode change failed", "PreArm: Compass not calibrated"):
        vehicle._add_status_text(text)
    assert vehicle.last_text("PreArm") == "FC says: PreArm: Compass not calibrated"
    assert vehicle.last_text("EKF") == ""
