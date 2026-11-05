"""The verify step's controller tuning against the simulator: noisy gyro, ArduPilot's own limits, saturation."""

import pytest

from rover_wizard.rover_defs import GCS_PID_MASK_STEERING

from util.helpers import act, check, enter, make

# [rad/s] a gyro on a vehicle bumping over grass (the simulator's default is 0.01)
BUMPY_YAW_RATE_NOISE = 0.08


def tuning_run(skid=False, rounds=6, r_step=30, noise=BUMPY_YAW_RATE_NOISE, **params):
    """Run the steering-rate tuning; returns (vehicle, its log lines). params are set on the vehicle first."""
    veh, ctx = make(skid=skid)
    lines = []
    ctx.set_output(log=lines.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    ctx.pause = lambda message, during=None: None
    veh.yaw_rate_noise = noise
    for name, value in {"TURN_RADIUS": 1.5, **params}.items():
        veh.set_param(name, value)
    ctx.values.update(circle_d=40, runway=40)
    enter(ctx, "verify")
    ctx.values.update(rounds=rounds, r_step=r_step, v_step=1.0)
    veh.arm()
    check("tuning ran", act(ctx, "verify", "STEERING-RATE"), "\n".join(lines))
    return veh, lines


def rounds_of(lines):
    """The log lines of the tuning rounds."""
    return [line for line in lines if line.startswith("Round")]


@pytest.fixture
def vehicles():
    """Vehicles a test created, disconnected afterwards."""
    created = []
    yield created
    for vehicle in created:
        vehicle.disconnect()


def test_steering_tuning_settles_despite_a_noisy_gyro(vehicles):
    veh, lines = tuning_run()
    vehicles.append(veh)
    rounds = rounds_of(lines)
    check(
        "settled within the rounds",
        "tracking is good" in rounds[-1] or "changes have become small" in rounds[-1],
        "\n".join(rounds),
    )
    # the simulated car turns at speed x steering / 1.5 m: at 1 m/s it needs 1.5 steering per rad/s
    check("FF corrected to what the car needs", 1.3 < veh.get("ATC_STR_RAT_FF") < 1.7, veh.get("ATC_STR_RAT_FF"))
    check("controller reports switched off again", veh.get_int("GCS_PID_MASK") == 0)


def test_a_target_out_of_reach_stops_tuning(vehicles):
    # without TURN_RADIUS the wizard doesn't cap the request: 80 deg/s at 1 m/s needs more than full lock
    veh, lines = tuning_run(r_step=80, TURN_RADIUS=0, rounds=10)
    vehicles.append(veh)
    rounds = rounds_of(lines)
    check("stopped at the limit", len(rounds) < 10 and "at its limit" in rounds[-1], "\n".join(rounds))
    # once FF asks for full lock, more P or I can't help: they stay as they were
    check("P and I untouched", (veh.get("ATC_STR_RAT_P"), veh.get("ATC_STR_RAT_I")) == (0.2, 0.2))


def test_ardupilots_own_limit_is_named_and_tuned_against(vehicles):
    # skid steering pivots on the spot: only ATC_STR_RAT_MAX caps the 40 deg/s request
    veh, lines = tuning_run(skid=True, r_step=40, ATC_STR_RAT_MAX=25)
    vehicles.append(veh)
    check(
        "the limit is named",
        any(
            "ArduPilot aimed for 25 instead of 40 deg/s (limited by ATC_STR_RAT_MAX = 25 deg/s)" in line
            for line in lines
        ),
        "\n".join(lines),
    )
    check("aimed at the limit", "aimed at 25 deg/s" in rounds_of(lines)[0], rounds_of(lines)[0])


def test_without_gcs_pid_mask_the_requested_target_is_used(vehicles):
    veh, ctx = make()
    vehicles.append(veh)
    del veh._params["GCS_PID_MASK"]
    lines = []
    ctx.set_output(log=lines.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    ctx.pause = lambda message, during=None: None
    veh.set_param("TURN_RADIUS", 1.5)
    ctx.values.update(circle_d=40, runway=40)
    enter(ctx, "verify")
    ctx.values.update(rounds=2, r_step=30, v_step=1.0)
    veh.arm()
    assert act(ctx, "verify", "STEERING-RATE")
    assert any("No GCS_PID_MASK" in line for line in lines)
    assert len(rounds_of(lines)) == 2


def test_gcs_pid_mask_bits_of_the_pilot_are_kept(vehicles):
    veh, lines = tuning_run(rounds=1, GCS_PID_MASK=8)
    vehicles.append(veh)
    assert veh.get_int("GCS_PID_MASK") == 8
    assert not 8 & GCS_PID_MASK_STEERING
