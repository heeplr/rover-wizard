"""How much the EKF can trust the wheel encoders and the compass: the maths on straight runs, and the verify step's
"Measure EKF sensor errors" against the simulator."""

import math
import random

import pytest

from rover_wizard.mavlink_defs import (
    MAVLINK_MSG_ID_EKF_STATUS_REPORT,
    MAVLINK_MSG_ID_GPS_RAW_INT,
    MAVLINK_MSG_ID_VIBRATION,
)
from rover_wizard.motion.ekf_sensor_errors import (
    compass_errors,
    compass_heading,
    ekf_health,
    encoder_errors,
)
from rover_wizard.steps.util.ekf_measurement import measures_anything, measures_compass, measures_encoders
from rover_wizard.vehicle.link import STREAMS, DriveSample, EkfRatios

from util.helpers import BY_ID, act, check, enter, make

HZ = 10
# [mGauss] horizontal and vertical earth field
NORTH, DOWN = 200.0, 400.0


def field_for(heading_deg):
    """The calibrated compass's field of a level vehicle heading this way (magnetic)."""
    heading = math.radians(heading_deg)
    return (NORTH * math.cos(heading), -NORTH * math.sin(heading), DOWN)


def straight_run(
    course_deg, speed=1.0, encoder_scale=1.0, encoder_noise=0.0, compass_error_deg=0.0, compass_noise_deg=0.0, seed=1
):
    """A steady straight run of 10 s with GPS, encoders and compass."""
    noise = random.Random(seed)
    samples, distance = [], 0.0
    for index in range(10 * HZ):
        time = index / HZ
        distance += (speed * encoder_scale + noise.gauss(0, encoder_noise)) / HZ
        compass = course_deg + compass_error_deg + noise.gauss(0, compass_noise_deg)
        samples.append(
            DriveSample(
                time=time,
                forward_speed=speed,
                yaw_rate=0.0,
                gps_course=math.radians(course_deg),
                gps_speed=speed,
                wheel_distances=(distance, distance),
                wheel_time=time,
                magnetic_field=field_for(compass),
                roll=0.0,
                pitch=0.0,
            )
        )
    return samples


# ---------------------------------------------------------------------------------------------- the maths
def test_compass_heading_from_the_field_with_tilt_and_declination():
    sample = DriveSample(time=0.0, magnetic_field=field_for(30), roll=0.0, pitch=0.0)
    assert math.degrees(compass_heading(sample, math.radians(3))) == pytest.approx(33)
    # the same field seen by a vehicle pitched up 10 deg: tilt compensation gives the same heading
    pitch = math.radians(10)
    x, y, z = field_for(30)
    tilted = DriveSample(
        time=0.0,
        magnetic_field=(x * math.cos(pitch) - z * math.sin(pitch), y, x * math.sin(pitch) + z * math.cos(pitch)),
        roll=0.0,
        pitch=pitch,
    )
    assert math.degrees(compass_heading(tilted, 0.0)) == pytest.approx(30, abs=0.01)


def test_encoder_noise_becomes_the_proposal():
    runs = [straight_run(40, encoder_noise=0.1), straight_run(220, encoder_noise=0.1, seed=2)]
    result = encoder_errors(runs)
    check("noise measured", 0.08 < result.noise < 0.12, result.noise)
    assert result.bias == pytest.approx(0, abs=0.01)
    assert result.proposal == pytest.approx(round(result.noise * 1.5, 2))


def test_the_gps_accuracy_is_taken_out_and_the_proposal_kept_sane():
    runs = [straight_run(40, encoder_noise=0.02)]
    for sample in runs[0]:
        sample.gps_speed_accuracy = 0.3
    result = encoder_errors(runs)
    assert result.noise == 0 and result.proposal == 0.05


def test_encoders_reading_high_warn_about_the_wheel_radius():
    result = encoder_errors([straight_run(40, encoder_scale=1.05)])
    assert result.proposal is None and "+5%" in result.warning and "WENC_RADIUS" in result.warning


def test_compass_noise_becomes_the_proposal():
    runs = [straight_run(40, compass_noise_deg=3), straight_run(220, compass_noise_deg=3, seed=2)]
    result = compass_errors(runs, 0.0)
    check("RMS error", 2.5 < result.rms < 3.5, result.rms)
    assert result.proposal == pytest.approx(round(math.radians(result.rms) * 1.5, 2))


def test_a_good_compass_gets_the_lowest_sane_value():
    assert compass_errors([straight_run(40, compass_noise_deg=0.5)], 0.0).proposal == 0.05


@pytest.mark.parametrize("errors", [(12.0, 12.0), (6.0, -6.0)], ids=["offset", "depends on the direction"])
def test_a_compass_heading_error_gives_a_warning_instead(errors):
    runs = [straight_run(40, compass_error_deg=errors[0]), straight_run(220, compass_error_deg=errors[1])]
    result = compass_errors(runs, 0.0)
    assert result.proposal is None and "COMPASS_ORIENT" in result.warning
    assert result.biases == pytest.approx(errors, abs=0.1)


def test_turning_and_slow_parts_are_left_out():
    samples = straight_run(40)
    for sample in samples[:45]:
        sample.yaw_rate = 0.3
    for sample in samples[45:90]:
        sample.gps_speed = 0.3
    assert encoder_errors([samples]) is None
    assert compass_errors([samples], 0.0) is None


def test_ekf_health_explains_high_ratios():
    samples = [DriveSample(time=0.0, ekf_ratios=EkfRatios(0.1, 0.2, 0.3))]
    assert ekf_health(samples) == ((0.1, 0.2, 0.3), [])
    samples.append(DriveSample(time=0.1, ekf_ratios=EkfRatios(0.6, 0.2, 1.2)))
    highest, notes = ekf_health(samples)
    assert highest == (0.6, 0.2, 1.2)
    assert len(notes) == 2 and "vibration" in notes[0] and "rejected" in notes[1] and "compass" in notes[1]
    assert ekf_health([DriveSample(time=0.0)]) == (None, [])


# ---------------------------------------------------------------------------------------------- the vehicle
@pytest.fixture
def rover():
    """A simulated rover with GPS, compass and wheel encoders (correctly calibrated), facing 57 deg."""
    veh, ctx = make()
    for key in ("has_gps", "has_compass", "has_wenc"):
        ctx.store.set(key, True)
    for name in ("WENC_RADIUS", "WENC2_RADIUS"):
        veh.set_param(name, veh.TRUE_WHEEL_RADIUS)
    veh.heading = 1.0
    lines, prompts = [], []
    ctx.set_output(log=lines.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)

    def turn_around(message, during=None):
        """The pilot reads the safety brief, and turns the vehicle around between the runs."""
        if message.startswith("Turn the vehicle around"):
            prompts.append(message)
            veh.heading += math.pi

    ctx.pause = turn_around
    ctx.values.update(runway=40)
    yield veh, ctx, lines, prompts
    veh.disconnect()


def run_measurement(ctx):
    """Arm and press the button."""
    enter(ctx, "verify")
    ctx.vehicle.arm()
    return act(ctx, "verify", "EKF sensor errors")


def test_both_values_are_proposed_from_the_same_two_runs(rover):
    veh, ctx, lines, prompts = rover
    check("ran", run_measurement(ctx), "\n".join(lines))
    check("asked to turn around once", len(prompts) == 1, prompts)
    # simulator: encoder speed noise 0.05 m/s, compass noise 2 deg
    check("WENC_VERR proposed", 0.05 <= ctx.values["EK3_WENC_VERR"] <= 0.12, ctx.values["EK3_WENC_VERR"])
    check("YAW_M_NSE proposed", 0.05 <= ctx.values["EK3_YAW_M_NSE"] <= 0.1, ctx.values["EK3_YAW_M_NSE"])
    check("nothing written yet", (veh.get("EK3_WENC_VERR"), veh.get("EK3_YAW_M_NSE")) == (0.1, 0.5))
    check("EKF ratios logged", any(line.startswith("EKF test ratios") for line in lines), "\n".join(lines))
    check("vibration logged", any(line.startswith("Vibration (highest)") and "fine" in line for line in lines))
    usual = dict(STREAMS)
    for message_id in (MAVLINK_MSG_ID_GPS_RAW_INT, MAVLINK_MSG_ID_EKF_STATUS_REPORT, MAVLINK_MSG_ID_VIBRATION):
        assert veh.message_rates[message_id] == usual.get(message_id, 0)


def test_wrong_wheel_radius_and_compass_errors_are_reported_not_proposed(rover):
    veh, ctx, lines, prompts = rover
    for name in ("WENC_RADIUS", "WENC2_RADIUS"):
        veh.set_param(name, 0.09)
    veh.compass_heading_dependent_deg = 15
    assert run_measurement(ctx)
    warnings = [line for line in lines if "WARNING" in line]
    check("two warnings", len(warnings) == 2, "\n".join(lines))
    assert "WENC_RADIUS" in warnings[0] and "COMPASS_ORIENT" in warnings[1]
    assert (ctx.values["EK3_WENC_VERR"], ctx.values["EK3_YAW_M_NSE"]) == (0.1, 0.5)


def test_a_compass_the_ekf_rejects_is_explained(rover):
    veh, ctx, lines, prompts = rover
    veh.set_param("EK3_YAW_M_NSE", 0.05)
    veh.compass_offset_deg = 12
    assert run_measurement(ctx)
    assert any("compass test ratio" in line and "rejected" in line for line in lines), "\n".join(lines)


def test_what_is_measured_follows_the_hardware(rover):
    veh, ctx, lines, prompts = rover
    assert measures_encoders(ctx) and measures_compass(ctx)
    ctx.store.set("has_wenc", False)
    assert not measures_encoders(ctx) and measures_anything(ctx)
    # the EKF takes its heading from a GPS only: the compass doesn't matter to it
    veh.set_param("EK3_SRC1_YAW", 2)
    assert not measures_anything(ctx)
    step = BY_ID["verify"]
    shown = [field.key for field in step.fields if field.when is None or field.when(ctx)]
    assert not {"ekf_heading", "ekf_thr", "EK3_WENC_VERR", "EK3_YAW_M_NSE"} & set(shown)
    assert not next(action for action in step.actions if "EKF" in action.label).when(ctx)
