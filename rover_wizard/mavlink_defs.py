"""rover_wizard.mavlink_defs - MAVLink protocol definitions the wizard uses.

Commands, message ids, enumeration values and field units, each under its official name from MAVLink's common.xml /
ardupilotmega.xml (as pymavlink names them: MAVLINK_MSG_ID_* for message ids, MAV_* and others for enumerations).
tests/test_protocol_constants.py checks every value against pymavlink. Defined here rather than imported from
pymavlink, so the wizard's demo mode runs without pymavlink installed.
"""

from __future__ import annotations

# ---- Commands (MAV_CMD, sent with COMMAND_LONG) --------------------------------------------------------------------
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_PREFLIGHT_CALIBRATION = 241
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
MAV_CMD_DO_MOTOR_TEST = 209
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE = 512
# ArduPilot-specific, ardupilotmega.xml
MAV_CMD_DO_START_MAG_CAL = 42424
# ArduPilot-specific, ardupilotmega.xml
MAV_CMD_DO_CANCEL_MAG_CAL = 42426
# ArduPilot-specific, ardupilotmega.xml - "Large Vehicle MagCal": one-shot compass calibration from a known heading, no
# rotation needed
MAV_CMD_FIXED_MAG_CAL_YAW = 42006
# ArduPilot-specific, ardupilotmega.xml
MAV_CMD_ACCELCAL_VEHICLE_POS = 42429
# param1 = output number, param2 = pulse width [us]
MAV_CMD_DO_SET_SERVO = 183
# param1 pitch, param2 roll, param3 yaw [deg], param7 = mount mode
MAV_CMD_DO_MOUNT_CONTROL = 205
# param1 = relay instance (0 = RELAY1), param2 = 1 on / 0 off
MAV_CMD_DO_SET_RELAY = 181
# param1 pitch, param2 yaw [deg], param7 gimbal device id (1, 2, ...)
MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000
# param1 gripper number, param2 GRIPPER_ACTIONS
MAV_CMD_DO_GRIPPER = 211
# param1 = source set 1..3
MAV_CMD_SET_EKF_SOURCE_SET = 42007
# COMMAND_LONG carries param1..param7
COMMAND_LONG_PARAMS = 7


# ---- Command parameter values --------------------------------------------------------------------------------------
# MOTOR_TEST_THROTTLE_TYPE enum: throttle given as a percentage
MOTOR_TEST_THROTTLE_PERCENT = 0
# MOTOR_TEST_ORDER enum: use the board's default test order
MOTOR_TEST_ORDER_DEFAULT = 0
# ArduPilot's documented "magic number" that must be passed as MAV_CMD_COMPONENT_ARM_DISARM's
# param2 to force an arm/disarm that bypasses the normal safety checks.
ARM_DISARM_FORCE_MAGIC_NUMBER = 21196
# ACCELCAL_VEHICLE_POS enum (ardupilotmega.xml): which way up the vehicle should be placed
# during the six-position accelerometer calibration, plus its two special "meta" results.
ACCELCAL_VEHICLE_POS_LEVEL = 1
ACCELCAL_VEHICLE_POS_LEFT = 2
ACCELCAL_VEHICLE_POS_RIGHT = 3
ACCELCAL_VEHICLE_POS_NOSEDOWN = 4
ACCELCAL_VEHICLE_POS_NOSEUP = 5
ACCELCAL_VEHICLE_POS_BACK = 6
ACCELCAL_VEHICLE_POS_SUCCESS = 16777215
ACCELCAL_VEHICLE_POS_FAILED = 16777216
# MAV_CMD_PREFLIGHT_CALIBRATION has no separate formal enum for its parameters; these are
# the documented meanings of the specific parameter values this wizard sends.
# param1 = 1: calibrate gyros
PREFLIGHT_CALIBRATION_GYRO = 1
# param5 = 1: run the full 6-position accelerometer calibration
PREFLIGHT_CALIBRATION_ACCEL = 1
# param5 = 2: set the level/trim reference only
PREFLIGHT_CALIBRATION_LEVEL = 2
# the mount follows angles sent over MAVLink
MAV_MOUNT_MODE_MAVLINK_TARGETING = 2
GRIPPER_ACTION_RELEASE = 0
GRIPPER_ACTION_GRAB = 1


# ---- Command results and calibration status ------------------------------------------------------------------------
# COMMAND_ACK result: command accepted
MAV_RESULT_ACCEPTED = 0
# COMMAND_ACK result: firmware/vehicle doesn't support this command
MAV_RESULT_UNSUPPORTED = 3
# COMMAND_ACK result: accepted and still running (final result follows)
MAV_RESULT_IN_PROGRESS = 5
# a command the vehicle took on: done, or running (long commands such as calibrations answer IN_PROGRESS first)
ACCEPTED_RESULTS = (MAV_RESULT_ACCEPTED, MAV_RESULT_IN_PROGRESS)
# MAG_CAL_STATUS enum (MAG_CAL_REPORT.cal_status): the one value that means the calibration succeeded.
MAG_CAL_SUCCESS = 4
MAG_CAL_FAILED = 5
# MAV_RESULT value for a command that was understood but could not be executed.
MAV_RESULT_FAILED = 4


# ---- Message ids ---------------------------------------------------------------------------------------------------
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_GPS_RAW_INT = 24
MAVLINK_MSG_ID_ATTITUDE = 30
# xacc/yacc/zacc in mg - used by the board-orientation auto-probe
MAVLINK_MSG_ID_SCALED_IMU = 26
MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT = 62
MAVLINK_MSG_ID_RC_CHANNELS = 65
MAVLINK_MSG_ID_VFR_HUD = 74
MAVLINK_MSG_ID_AUTOPILOT_VERSION = 148
# ArduPilot-specific, ardupilotmega.xml
MAVLINK_MSG_ID_EKF_STATUS_REPORT = 193
# ArduPilot-specific, ardupilotmega.xml
MAVLINK_MSG_ID_WHEEL_DISTANCE = 9000
# servo1_raw .. servo16_raw [us]
MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 36
# rangefinder / proximity-sector distances [cm]
MAVLINK_MSG_ID_DISTANCE_SENSOR = 132
# xmag/ymag/zmag [mGauss] of the first compass
MAVLINK_MSG_ID_RAW_IMU = 27
# the second receiver's fix and satellites
MAVLINK_MSG_ID_GPS2_RAW = 124
# rpm1, rpm2 of the first two RPM sensors
MAVLINK_MSG_ID_RPM = 226
# which relays are set up (present) and switched on, one bit per relay (bit 0 = relay 1); firmware 4.5+
MAVLINK_MSG_ID_RELAY_STATUS = 376


# ---- Enumerations in messages --------------------------------------------------------------------------------------
# HEARTBEAT.autopilot: this is ArduPilot firmware
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
# HEARTBEAT.type: wheeled rover
MAV_TYPE_GROUND_ROVER = 10
# HEARTBEAT.type: boat
MAV_TYPE_SURFACE_BOAT = 11
# the wizard's own HEARTBEAT: a ground station, not an autopilot, up and running
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4
# DO_SET_MODE param1 / HEARTBEAT.base_mode bit
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
# HEARTBEAT.base_mode bit: vehicle is armed
MAV_MODE_FLAG_SAFETY_ARMED = 128
# PARAM_SET type used when a parameter's type isn't known
MAV_PARAM_TYPE_REAL32 = 9
# PARAM_VALUE.param_type of a whole-number parameter (ArduPilot's AP_Int32)
MAV_PARAM_TYPE_INT32 = 6
# REQUEST_DATA_STREAM: the legacy "all streams" group
MAV_DATA_STREAM_ALL = 0
# SET_POSITION_TARGET_LOCAL_NED frame: velocities are forward/right/down
MAV_FRAME_BODY_NED = 8
# EKF_STATUS_FLAGS bits (EKF_STATUS_REPORT.flags, ardupilotmega.xml) used by the pre-drive check.
# attitude estimate is good
EKF_ATTITUDE = 1
# horizontal velocity estimate is good
EKF_VELOCITY_HORIZ = 2
# relative horizontal position estimate is good
EKF_POS_HORIZ_REL = 8
# absolute horizontal position estimate is good
EKF_POS_HORIZ_ABS = 16
# GPS_FIX_TYPE enum (GPS_RAW_INT.fix_type) and the human-readable names the wizard shows for it.
GPS_FIX_TYPE_NO_FIX = 1
GPS_FIX_TYPE_3D_FIX = 3
GPS_FIX_TYPE_NAMES = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}
# STATUSTEXT severities (MAV_SEVERITY): lower is more severe; ERROR covers EMERGENCY..ERROR (0..3)
MAV_SEVERITY_ERROR = 3
MAV_SEVERITY_WARNING = 4
MAV_SEVERITY_INFO = 6


# ---- SET_POSITION_TARGET type masks --------------------------------------------------------------------------------
# POSITION_TARGET_TYPEMASK bits (each one = "ignore this field of SET_POSITION_TARGET_LOCAL_NED")
POSITION_TARGET_TYPEMASK_X_IGNORE = 1
POSITION_TARGET_TYPEMASK_Y_IGNORE = 2
POSITION_TARGET_TYPEMASK_Z_IGNORE = 4
POSITION_TARGET_TYPEMASK_VZ_IGNORE = 32
POSITION_TARGET_TYPEMASK_AX_IGNORE = 64
POSITION_TARGET_TYPEMASK_AY_IGNORE = 128
POSITION_TARGET_TYPEMASK_AZ_IGNORE = 256
POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1024
# Guided Rover command: use only forward/right velocity (vx, vy) and yaw rate; ignore position, vz, acceleration, yaw.
GUIDED_VELOCITY_YAW_RATE_TYPEMASK = (
    POSITION_TARGET_TYPEMASK_X_IGNORE
    | POSITION_TARGET_TYPEMASK_Y_IGNORE
    | POSITION_TARGET_TYPEMASK_Z_IGNORE
    | POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | POSITION_TARGET_TYPEMASK_AX_IGNORE
    | POSITION_TARGET_TYPEMASK_AY_IGNORE
    # == 1511
    | POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | POSITION_TARGET_TYPEMASK_YAW_IGNORE
)


# ---- Units and special values of message fields --------------------------------------------------------------------
# RC_CHANNELS: a channel value of UINT16_MAX means "unused"
UINT16_MAX = 0xFFFF
# PARAM_VALUE.param_index when the index isn't meaningful (e.g. a single param_set confirmation, not part of a list)
PARAM_INDEX_UNKNOWN = 65535
# GPS_RAW_INT.eph is HDOP multiplied by 100
GPS_RAW_INT_EPH_SCALE = 100
# Unit scales of SYS_STATUS fields.
# voltage_battery is in mV
SYS_STATUS_MILLIVOLTS_PER_VOLT = 1000
# current_battery is in cA (-1 = not measured)
SYS_STATUS_CENTIAMPS_PER_AMP = 100
# SYS_STATUS.onboard_control_sensors_* bit of the barometer(s)
MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8
# SCALED_PRESSURE.temperature is in centi-degrees Celsius
SCALED_PRESSURE_CENTIDEGREES_PER_DEGREE = 100
# SCALED_IMU acceleration unit: milli-g.
MILLI_G_PER_G = 1000
# SCALED_IMU rotation-rate unit: milli-radians per second.
MILLIRAD_PER_RAD = 1000
# RC_CHANNELS carries chan1_raw .. chan18_raw (chancount says how many are used)
RC_CHANNELS_MAX = 18
