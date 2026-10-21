"""The wizard's MAVLink layer against a fake autopilot over real pymavlink/UDP: what goes over the wire."""

import itertools

import pytest

pytest.importorskip("pymavlink")

from pymavlink.dialects.v20 import ardupilotmega as mavlink  # noqa: E402

from rover_wizard.util import timebase  # noqa: E402
from rover_wizard.context import Ctx  # noqa: E402
from rover_wizard.util.errors import VehicleError  # noqa: E402
from rover_wizard.rover_defs import MODES  # noqa: E402
from rover_wizard.mavlink_defs import GUIDED_VELOCITY_YAW_RATE_TYPEMASK  # noqa: E402
from rover_wizard.session.store import Store  # noqa: E402
from rover_wizard.vehicle.link import (  # noqa: E402
    INBOX_ACCEL_CALIBRATION,
    INBOX_MAG_CAL_PROGRESS,
    PARAM_SET_ATTEMPTS,
    STREAMS,
    Vehicle,
    _as_float32,
    _param_file_value,
)

from util.fake_autopilot import RC_CHANNEL_COUNT, FakeAutopilot  # noqa: E402
from util.helpers import TEST_TIME_SCALE  # noqa: E402

PORTS = itertools.count(14610)


@pytest.fixture(autouse=True)
def real_time():
    """The fake autopilot sends in real time: the link's freshness limits and timeouts must run in real time too
    (fast-forwarded, a 1 s freshness limit is 50 ms - shorter than the fake's 100 ms telemetry period)."""
    timebase.set_time_scale(1.0)
    yield
    timebase.set_time_scale(TEST_TIME_SCALE)


@pytest.fixture
def link(tmp_path):
    """A connected Vehicle and the fake autopilot it talks to."""
    port = next(PORTS)
    autopilot = FakeAutopilot(port)
    vehicle = Vehicle()
    vehicle.connect(f"udpin:127.0.0.1:{port}", timeout=10)
    yield vehicle, autopilot
    vehicle.disconnect()
    autopilot.close()


def test_connect_reads_identity_version_and_parameters(link):
    vehicle, autopilot = link
    assert (vehicle.system_id, vehicle.vehicle_type, vehicle.version) == (1, mavlink.MAV_TYPE_GROUND_ROVER, (4, 5, 7))
    assert vehicle.parameter_count == len(autopilot.params)
    assert vehicle.gcs_system_id == 255, "RC override is only accepted from SYSID_MYGCS"


def test_every_stream_requested_at_its_rate(link):
    vehicle, autopilot = link
    wanted = {message_id: int(1e6 / rate) for message_id, rate in STREAMS}
    assert autopilot.wait_for(lambda: len(autopilot.commands(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)) >= len(wanted))
    sent = {int(c.param1): int(c.param2) for c in autopilot.commands(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)}
    assert sent == wanted


def test_incomplete_parameter_download_is_reported(link):
    vehicle, autopilot = link
    assert vehicle.parameter_download_warning() == ""
    autopilot.withheld = set(autopilot.names[:2])
    vehicle.fetch_params(timeout=1.0)
    assert f"2 of {len(autopilot.names)} parameters could not be downloaded" in vehicle.parameter_download_warning()


def test_set_param_keeps_the_parameter_type(link):
    vehicle, autopilot = link
    assert vehicle.set_param("SERVO1_FUNCTION", 70) == 70
    written = autopilot.messages("PARAM_SET")[-1]
    assert (written.param_id, written.param_value, written.param_type) == (
        "SERVO1_FUNCTION",
        70,
        mavlink.MAV_PARAM_TYPE_INT16,
    )
    assert (written.target_system, written.target_component) == (1, 1)


def test_unknown_parameter_is_refused_without_sending(link):
    vehicle, autopilot = link
    with pytest.raises(VehicleError):
        vehicle.set_param("NO_SUCH_PARAM", 1)
    assert not [m for m in autopilot.messages("PARAM_SET") if m.param_id == "NO_SUCH_PARAM"]


def test_motor_test_parameters(link):
    vehicle, autopilot = link
    vehicle.motor_test(3, -20, 2.0)
    test = autopilot.commands(mavlink.MAV_CMD_DO_MOTOR_TEST)[-1]
    # instance, throttle type (percent), value, timeout [s], motor count, order
    assert (test.param1, test.param2, test.param3, test.param4, test.param5, test.param6) == (
        3,
        mavlink.MOTOR_TEST_THROTTLE_PERCENT,
        -20,
        2,
        1,
        mavlink.MOTOR_TEST_ORDER_DEFAULT,
    )


def test_rc_override_values_and_release(link):
    vehicle, autopilot = link
    vehicle.rc_override({1: 1600, 3: 1400})
    assert autopilot.wait_for(lambda: autopilot.messages("RC_CHANNELS_OVERRIDE"))
    override = autopilot.messages("RC_CHANNELS_OVERRIDE")[-1]
    assert (override.target_system, override.chan1_raw, override.chan2_raw, override.chan3_raw) == (1, 1600, 0, 1400)
    vehicle.rc_release()
    assert autopilot.wait_for(lambda: autopilot.messages("RC_CHANNELS_OVERRIDE")[-1].chan1_raw == 0)


def test_guided_target_frame_mask_and_values(link):
    vehicle, autopilot = link
    vehicle.guided_cmd(1.5, 0.3)
    assert autopilot.wait_for(lambda: autopilot.messages("SET_POSITION_TARGET_LOCAL_NED"))
    target = autopilot.messages("SET_POSITION_TARGET_LOCAL_NED")[-1]
    assert target.coordinate_frame == mavlink.MAV_FRAME_BODY_NED
    assert target.type_mask == GUIDED_VELOCITY_YAW_RATE_TYPEMASK
    assert (target.vx, target.vy, target.yaw_rate) == pytest.approx((1.5, 0.0, 0.3))


def test_set_mode_arm_and_disarm_follow_the_heartbeat(link):
    vehicle, autopilot = link
    vehicle.set_mode("GUIDED")
    assert vehicle.custom_mode == MODES["GUIDED"]
    assert autopilot.commands(mavlink.MAV_CMD_DO_SET_MODE)[-1].param2 == MODES["GUIDED"]
    vehicle.arm()
    assert vehicle.armed
    vehicle.disarm(force=True)
    assert autopilot.commands(mavlink.MAV_CMD_COMPONENT_ARM_DISARM)[-1].param2 == 21196
    assert autopilot.wait_for(lambda: not vehicle.armed)


def test_servo_mount_and_emergency_commands(link):
    vehicle, autopilot = link
    vehicle.set_servo_pwm(5, 1600)
    set_servo = autopilot.commands(mavlink.MAV_CMD_DO_SET_SERVO)[-1]
    assert (set_servo.param1, set_servo.param2) == (5, 1600)
    vehicle.set_relay(1, True)
    relay = autopilot.commands(mavlink.MAV_CMD_DO_SET_RELAY)[-1]
    assert (relay.param1, relay.param2) == (0, 1), "DO_SET_RELAY numbers relays from 0"
    vehicle.point_gimbal(2, -30)
    pointing = autopilot.commands(mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)[-1]
    assert (pointing.param1, pointing.param2, pointing.param7) == (-30, 0, 2), "pitch, yaw, gimbal device 2"
    vehicle.gripper(mavlink.GRIPPER_ACTION_GRAB)
    grip = autopilot.commands(mavlink.MAV_CMD_DO_GRIPPER)[-1]
    assert (grip.param1, grip.param2) == (1, mavlink.GRIPPER_ACTION_GRAB)
    vehicle.set_ekf_source_set(2)
    assert autopilot.commands(mavlink.MAV_CMD_SET_EKF_SOURCE_SET)[-1].param1 == 2
    vehicle.mount_control(-30)
    mount = autopilot.commands(mavlink.MAV_CMD_DO_MOUNT_CONTROL)[-1]
    assert (mount.param1, mount.param7) == (-30, mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
    vehicle.emergency_stop()
    assert autopilot.wait_for(
        lambda: getattr(autopilot.last_command(mavlink.MAV_CMD_DO_SET_MODE), "param2", None) == MODES["HOLD"]
    )
    assert autopilot.wait_for(
        lambda: getattr(autopilot.last_command(mavlink.MAV_CMD_COMPONENT_ARM_DISARM), "param2", None) == 21196
    )


def test_telemetry_decoding(link):
    vehicle, autopilot = link
    assert autopilot.wait_for(lambda: vehicle.rc_raw() is not None and vehicle.servo_raw() is not None)
    assert len(vehicle.rc_raw()) == RC_CHANNEL_COUNT, "channel count comes from RC_CHANNELS.chancount"
    assert vehicle.servo_raw()[:3] == [1500, 0, 1500] and len(vehicle.servo_raw()) == 16
    assert [(m.id, m.current_distance) for m in vehicle.distance_sensors()] == [(0, 123)]


def test_steering_test_sends_override_on_the_steering_channel_only(link, tmp_path):
    vehicle, autopilot = link
    ctx = Ctx(vehicle, Store(vehicle.system_id, str(tmp_path)))
    ctx.set_output(log=lambda text: None, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    ctx.servos.hold_steering(0.5)
    try:
        # Stick +0.5 on RC1 (TRIM 1500, MAX 1900) = 1700 us; throttle RC3 held at its trim; Manual mode; not armed.
        assert autopilot.wait_for(
            lambda: autopilot.messages("RC_CHANNELS_OVERRIDE")
            and autopilot.messages("RC_CHANNELS_OVERRIDE")[-1].chan1_raw == 1700
        )
        override = autopilot.messages("RC_CHANNELS_OVERRIDE")[-1]
        assert override.chan3_raw == 1500
        assert autopilot.commands(mavlink.MAV_CMD_DO_SET_MODE)[-1].param2 == MODES["MANUAL"]
        assert not autopilot.commands(mavlink.MAV_CMD_DO_MOTOR_TEST), "no motor test for the steering servo"
        assert not autopilot.commands(mavlink.MAV_CMD_COMPONENT_ARM_DISARM) and not vehicle.armed
    finally:
        ctx.servos.release()
    assert autopilot.wait_for(lambda: autopilot.messages("RC_CHANNELS_OVERRIDE")[-1].chan1_raw == 0)


# Fields of each telemetry message the simulator publishes; they must match the real message definitions,
# otherwise the wizard would be tested against telemetry the real vehicle never sends.
SIMULATED_FIELDS = {
    "SYS_STATUS": ["voltage_battery", "current_battery"],
    "GPS_RAW_INT": ["fix_type", "satellites_visible", "eph", "lat"],
    "GPS2_RAW": ["fix_type", "satellites_visible", "eph"],
    "RAW_IMU": ["xmag", "ymag", "zmag"],
    "RPM": ["rpm1", "rpm2"],
    "EKF_STATUS_REPORT": ["flags"],
    "WHEEL_DISTANCE": ["count", "distance"],
    "SCALED_IMU": ["xacc", "yacc", "zacc"],
    "SERVO_OUTPUT_RAW": [f"servo{n}_raw" for n in range(1, 17)],
    "RC_CHANNELS": ["chancount"] + [f"chan{n}_raw" for n in range(1, 19)],
    "DISTANCE_SENSOR": ["id", "orientation", "current_distance"],
    "MAG_CAL_PROGRESS": ["cal_mask", "compass_id", "completion_pct"],
    "MAG_CAL_REPORT": ["compass_id", "cal_status", "fitness"],
    "ATTITUDE": ["yaw", "yawspeed"],
    "LOCAL_POSITION_NED": ["x", "y", "vx", "vy"],
    "VFR_HUD": ["groundspeed"],
    "AUTOPILOT_VERSION": ["flight_sw_version"],
    "HEARTBEAT": ["type", "autopilot", "base_mode", "custom_mode"],
}


@pytest.mark.parametrize("msg_type", sorted(SIMULATED_FIELDS))
def test_simulated_telemetry_uses_real_field_names(msg_type):
    message_class = mavlink.mavlink_map[getattr(mavlink, f"MAVLINK_MSG_ID_{msg_type}")]
    missing = [field for field in SIMULATED_FIELDS[msg_type] if field not in message_class.fieldnames]
    assert not missing, f"{msg_type} has no field(s) {missing}"


def test_newer_firmware_ground_station_id_is_used_for_rc_override():
    # Current ArduPilot renamed SYSID_MYGCS to MAV_GCS_SYSID; RC overrides from any other id are ignored.
    port = next(PORTS)
    autopilot = FakeAutopilot(port, renamed={"SYSID_MYGCS": ("MAV_GCS_SYSID", 254)})
    vehicle = Vehicle()
    vehicle.connect(f"udpin:127.0.0.1:{port}", timeout=10)
    try:
        assert vehicle.gcs_system_id == 254
        vehicle.rc_override({1: 1600})
        assert autopilot.wait_for(lambda: autopilot.messages("RC_CHANNELS_OVERRIDE"))
        assert autopilot.messages("RC_CHANNELS_OVERRIDE")[-1].get_srcSystem() == 254
    finally:
        vehicle.disconnect()
        autopilot.close()


def test_wizard_sends_a_ground_station_heartbeat(link):
    # ArduPilot's GCS failsafe (FS_GCS_ENABLE) counts the GCS as present only from its heartbeats (and RC overrides)
    vehicle, autopilot = link

    def heartbeats():
        return autopilot.messages("HEARTBEAT")

    assert autopilot.wait_for(lambda: len(heartbeats()) >= 2, timeout=5)
    beat = heartbeats()[-1]
    assert (beat.type, beat.autopilot, beat.get_srcSystem()) == (
        mavlink.MAV_TYPE_GCS,
        mavlink.MAV_AUTOPILOT_INVALID,
        vehicle.gcs_system_id,
    )


def _sender(autopilot, component):
    """A MAVLink encoder writing to the fake autopilot's socket as another component of the vehicle (its own
    sequence numbers, so the autopilot's telemetry thread is not disturbed)."""
    return mavlink.MAVLink(autopilot.link, srcSystem=autopilot.link.source_system, srcComponent=component)


def test_long_flight_controller_text_arriving_in_pieces_is_joined(link):
    vehicle, autopilot = link
    autopilot_sender = _sender(autopilot, vehicle.component_id)
    marker = vehicle.status_text_marker()
    # ArduPilot (GCS::send_textv): 50 characters per STATUSTEXT, same id, a shorter last piece
    text = "PreArm: " + "a reason longer than one STATUSTEXT message can carry, " * 2
    for sequence, start in enumerate(range(0, len(text), 50)):
        autopilot_sender.statustext_send(4, text[start : start + 50].encode(), id=7, chunk_seq=sequence)
    assert autopilot.wait_for(lambda: vehicle.status_texts_since(marker) == [text], timeout=5)


def test_text_with_a_lost_last_piece_is_shown_as_far_as_it_arrived(link):
    vehicle, autopilot = link
    marker = vehicle.status_text_marker()
    _sender(autopilot, vehicle.component_id).statustext_send(4, b"x" * 50, id=9, chunk_seq=0)
    assert autopilot.wait_for(lambda: vehicle.status_texts_since(marker) == ["x" * 50], timeout=5)


def test_other_components_of_the_vehicle_do_not_mix_in(link):
    vehicle, autopilot = link
    # e.g. a MAVLink gimbal: its own heartbeat (armed flag set), parameters and texts
    gimbal = _sender(autopilot, mavlink.MAV_COMP_ID_GIMBAL)
    gimbal.heartbeat_send(
        mavlink.MAV_TYPE_GIMBAL,
        mavlink.MAV_AUTOPILOT_INVALID,
        mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
        99,
        mavlink.MAV_STATE_ACTIVE,
    )
    gimbal.param_value_send(b"GMB_OWN_PARAM", 1.0, mavlink.MAV_PARAM_TYPE_REAL32, 1, 0)
    gimbal.statustext_send(6, b"Gimbal ready")
    assert autopilot.wait_for(lambda: "Gimbal ready" in vehicle.status_texts_since(0), timeout=5)
    assert not vehicle.has_parameter("GMB_OWN_PARAM")
    assert not vehicle.armed and vehicle.custom_mode != 99


def test_a_new_text_before_the_last_piece_of_the_previous_one(link):
    vehicle, autopilot = link
    marker = vehicle.status_text_marker()
    sender = _sender(autopilot, vehicle.component_id)
    # text 11 loses its last piece; text 12 follows in one message
    sender.statustext_send(4, b"y" * 50, id=11, chunk_seq=0)
    sender.statustext_send(4, b"PreArm: next text", id=12, chunk_seq=0)
    assert autopilot.wait_for(lambda: vehicle.status_texts_since(marker) == ["y" * 50, "PreArm: next text"], timeout=5)


def test_messages_from_another_vehicle_are_ignored(link):
    vehicle, autopilot = link
    other = mavlink.MAVLink(autopilot.link, srcSystem=2, srcComponent=1)
    other.heartbeat_send(
        mavlink.MAV_TYPE_GROUND_ROVER,
        mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
        99,
        mavlink.MAV_STATE_ACTIVE,
    )
    other.statustext_send(6, b"I am vehicle 2")
    # a message of our vehicle sent afterwards: when it has arrived, the other ones have been handled
    _sender(autopilot, vehicle.component_id).statustext_send(6, b"I am vehicle 1")
    assert autopilot.wait_for(lambda: "I am vehicle 1" in vehicle.status_texts_since(0), timeout=5)
    assert "I am vehicle 2" not in vehicle.status_texts_since(0)
    assert not vehicle.armed and vehicle.custom_mode != 99


def test_calibration_requests_and_progress_go_to_their_inboxes(link):
    vehicle, autopilot = link
    sender = _sender(autopilot, vehicle.component_id)
    # the accelerometer calibration asks for position 3 (on its right side)
    sender.command_long_send(255, 0, mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS, 0, 3, 0, 0, 0, 0, 0, 0)
    sender.mag_cal_progress_send(0, 1, mavlink.MAG_CAL_RUNNING_STEP_TWO, 1, 40, [0] * 10, 0, 0, 0)
    assert autopilot.wait_for(lambda: vehicle._inbox[INBOX_MAG_CAL_PROGRESS], timeout=5)
    assert vehicle.pop_inbox(INBOX_ACCEL_CALIBRATION) == 3
    assert vehicle.pop_inbox(INBOX_MAG_CAL_PROGRESS).completion_pct == 40


def test_parameters_lost_from_the_list_are_asked_for_one_by_one(link):
    vehicle, autopilot = link
    lost = set(range(3, 6))
    autopilot.lost_from_list = {autopilot.names[index] for index in lost}
    progress = []
    vehicle.fetch_params(progress=lambda received, total: progress.append((received, total)), timeout=20)
    assert vehicle.parameter_count == len(autopilot.names) and vehicle.parameter_download_warning() == ""
    assert lost <= {message.param_index for message in autopilot.messages("PARAM_REQUEST_READ")}
    assert progress[-1] == (len(autopilot.names), len(autopilot.names))


def test_no_parameters_at_all_is_an_error(link):
    vehicle, autopilot = link
    autopilot.withheld = set(autopilot.names)
    with pytest.raises(VehicleError, match="Could not download the parameter list"):
        vehicle.fetch_params(timeout=1.0)


# the next tests wait for answers that never come: a faster clock keeps them short (the fake's telemetry period of
# 100 ms real time is then 1 s - still fresh for the link)
FAST_TIME_SCALE = 10.0


def test_unconfirmed_parameter_write_is_an_error(link):
    vehicle, autopilot = link
    timebase.set_time_scale(FAST_TIME_SCALE)
    autopilot.withheld = {"CRUISE_SPEED"}
    with pytest.raises(VehicleError, match="No confirmation from the vehicle when setting CRUISE_SPEED"):
        vehicle.set_param("CRUISE_SPEED", 3.0)
    assert len([m for m in autopilot.messages("PARAM_SET") if m.param_id == "CRUISE_SPEED"]) == PARAM_SET_ATTEMPTS


def test_refused_arming_names_the_pre_arm_reason(link):
    vehicle, autopilot = link
    timebase.set_time_scale(FAST_TIME_SCALE)
    autopilot.refused_commands[mavlink.MAV_CMD_COMPONENT_ARM_DISARM] = "PreArm: Throttle below failsafe"
    with pytest.raises(VehicleError, match="Arming refused. FC says: PreArm: Throttle below failsafe"):
        vehicle.arm()
    assert not vehicle.armed


def test_refused_mode_change_names_the_reason(link):
    vehicle, autopilot = link
    timebase.set_time_scale(FAST_TIME_SCALE)
    autopilot.refused_commands[mavlink.MAV_CMD_DO_SET_MODE] = "Mode change to GUIDED failed: requires position"
    with pytest.raises(VehicleError, match="Could not switch to GUIDED mode. FC says: Mode change to GUIDED failed"):
        vehicle.set_mode("GUIDED")


def test_version_from_the_banner_when_autopilot_version_is_not_sent(link):
    vehicle, autopilot = link
    timebase.set_time_scale(FAST_TIME_SCALE)
    # old firmware: no AUTOPILOT_VERSION on request, but the boot banner names the version
    autopilot.refused_commands[mavlink.MAV_CMD_REQUEST_MESSAGE] = "ArduRover V4.2.3 (2172cfb3)"
    vehicle.read_version()
    assert vehicle.version == (4, 2, 3) and vehicle.version_str() == "4.2.3"


# ---- parameter backup (.param file) -----------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "value, text",
    [(0.0145, "0.0145"), (0.2, "0.2"), (1036369, "1036369"), (-1, "-1"), (16777216, "16777216"), (1e-07, "1e-07")],
)
def test_backup_values_are_short_and_exact(value, text):
    stored = _as_float32(value)
    assert _param_file_value(stored) == text
    assert _as_float32(float(text)) == stored


def test_backup_gives_every_float32_back():
    # values whose 6 significant digits are not enough
    for value in (0.123456789, 3.14159274, 1234.56789, 7.0000005):
        stored = _as_float32(value)
        assert _as_float32(float(_param_file_value(stored))) == stored
