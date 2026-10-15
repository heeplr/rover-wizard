"""rover_wizard.vehicle.simulator - a simulated Rover with the same interface as Vehicle, for --demo and the self-test.

The simulator hides a "true" vehicle the wizard has to discover: top speed 3 m/s, front-wheel turn radius 1.5 m,
wheel radius 0.10 m, a board mounted as Yaw45Roll90, battery voltage 12.4 V, and (with wired_wrong=True) inverted
throttle and steering wiring. Telemetry follows ArduPilot's conventions, including the accelerometer sign
(specific force: a level board reads z = -1 g), so the wizard's maths is tested against realistic data.
"""

from __future__ import annotations

import math
import random
import threading
import time
from types import SimpleNamespace

from ..util import timebase
from ..util.errors import VehicleError
from ..ardupilot_defs import (
    ARMING_CHECK_NONE,
    ARMING_SKIPCHK_ALL,
    AUX_FUNCTION_REFRESH_SECONDS,
    COMPASS_MOTCT_CURRENT,
    FS_ACTION_NOTHING,
    FS_OPTIONS_ACTIVE_IN_HOLD,
    MODES,
    MOTOR_TEST_INSTANCE_STEERING,
    MOTOR_TEST_INSTANCE_THROTTLE,
    MOTOR_TEST_INSTANCE_THROTTLE_LEFT,
    MOTOR_TEST_INSTANCE_THROTTLE_RIGHT,
    RANGEFINDER_INSTANCES,
    RC_PWM_NOMINAL_TRIM,
    RELAYS,
    RELAY_FUNCTION_RELAY,
    RPM_SENSORS,
    SERIAL_PROTOCOL_GIMBAL,
    SERIAL_PROTOCOL_LIDAR360,
    SERIAL_PROTOCOL_MAVLINK2,
    SERIAL_PROTOCOL_RANGEFINDER,
    SERVO_FUNCTIONS_DIRECTLY_SETTABLE,
    SERVO_FUNCTION_GROUND_STEERING,
    SERVO_FUNCTION_RCIN_BASE,
    SERVO_FUNCTION_RC_PASSTHRU,
    SERVO_FUNCTION_THROTTLE,
    SERVO_FUNCTION_THROTTLE_LEFT,
    SERVO_FUNCTION_THROTTLE_RIGHT,
    SERVO_OUTPUTS,
)
from ..mavlink_defs import (
    ACCELCAL_VEHICLE_POS_BACK,
    ACCELCAL_VEHICLE_POS_LEVEL,
    ACCELCAL_VEHICLE_POS_SUCCESS,
    COMMAND_LONG_PARAMS,
    EKF_ATTITUDE,
    EKF_POS_HORIZ_ABS,
    EKF_POS_HORIZ_REL,
    EKF_VELOCITY_HORIZ,
    GRIPPER_ACTION_GRAB,
    MAG_CAL_FAILED,
    MAG_CAL_SUCCESS,
    MAV_CMD_ACCELCAL_VEHICLE_POS,
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
    MAV_CMD_DO_GRIPPER,
    MAV_CMD_DO_MOTOR_TEST,
    MAV_CMD_DO_MOUNT_CONTROL,
    MAV_CMD_DO_SET_MODE,
    MAV_CMD_DO_SET_RELAY,
    MAV_CMD_DO_SET_SERVO,
    MAV_CMD_DO_CANCEL_MAG_CAL,
    MAV_CMD_DO_START_MAG_CAL,
    MAV_CMD_FIXED_MAG_CAL_YAW,
    MAV_CMD_PREFLIGHT_CALIBRATION,
    MAV_CMD_SET_EKF_SOURCE_SET,
    MAV_RESULT_ACCEPTED,
    MAV_PARAM_TYPE_INT32,
    MAV_PARAM_TYPE_REAL32,
    MAV_RESULT_FAILED,
    MAV_TYPE_GROUND_ROVER,
    MILLIRAD_PER_RAD,
    MILLI_G_PER_G,
    PARAM_INDEX_UNKNOWN,
    PREFLIGHT_CALIBRATION_ACCEL,
    SYS_STATUS_CENTIAMPS_PER_AMP,
    SYS_STATUS_MILLIVOLTS_PER_VOLT,
)
from .rc_scaling import pwm_to_stick, stick_to_pwm
from ..util.rotation import ORIENTATION_NAME_BY_VALUE, rotation_matrix_for_name, transform, transpose
from .link import (
    GCS_SOURCE_SYSTEM,
    DriveSample,
    INBOX_ACCEL_CALIBRATION,
    INBOX_MAG_CAL_PROGRESS,
    INBOX_MAG_CAL_REPORT,
    MODE_CHANGE_TIMEOUT,
    Vehicle,
)

# TEMP1..TEMP3 (ArduPilot has more; the wizard sets up three)
SIMULATED_TEMPERATURE_SENSORS = range(1, 4)
# [s] an override not refreshed for this long is dropped
RC_OVERRIDE_TIMEOUT = 3.0
# [s] integration step
PHYSICS_STEP = 0.02
# [s] real time between telemetry updates
LOOP_PERIOD = 0.01
# [m/s] standard deviation of the speed noise
SPEED_NOISE = 0.015
# [rad/s] standard deviation of the yaw-rate noise
YAW_RATE_NOISE = 0.01
# the simulated motor needs 8 % before it moves
MOTOR_DEADZONE = 0.08
# [s] coasting slows down faster than accelerating speeds up
DECELERATION_TIME_CONSTANT = 0.5
# [rad/s] physical limit for the front-wheel-steering model
MAX_YAW_RATE_ACKERMANN = 4.0
# what the simulated distance sensors read
SIMULATED_DISTANCE_CM = 150
# ids used for the proximity sensor's eight sectors
PROXIMITY_SENSOR_ID_START = 10
FUNCTION_BY_TEST_INSTANCE = {
    MOTOR_TEST_INSTANCE_STEERING: SERVO_FUNCTION_GROUND_STEERING,
    MOTOR_TEST_INSTANCE_THROTTLE: SERVO_FUNCTION_THROTTLE,
    MOTOR_TEST_INSTANCE_THROTTLE_LEFT: SERVO_FUNCTION_THROTTLE_LEFT,
    MOTOR_TEST_INSTANCE_THROTTLE_RIGHT: SERVO_FUNCTION_THROTTLE_RIGHT,
}


def _vehicle_frame_gravity(pitch_deg: float) -> tuple:
    """Unit gravity direction in the vehicle frame when the vehicle is pitched nose-down by `pitch_deg`.

    Level (0 deg) gravity points along +z (down); nose straight down (90 deg) it points along +x (forward).
    In between it rotates in the x-z plane: (sin p, 0, cos p).
    """
    radians = math.radians(pitch_deg)
    # Pitching nose-down by p rotates "down" from +z towards +x: x = sin p, z = cos p, y unchanged (0).
    return (math.sin(radians), 0.0, math.cos(radians))


class SimVehicle(Vehicle):
    """A physics-simulated Rover implementing Vehicle's interface (see the module docstring)."""

    # [m/s]
    TOP_SPEED = 3.0
    # [s]
    SPEED_TIME_CONSTANT = 0.8
    # [m]
    MIN_TURN_RADIUS = 1.5
    # [rad/s]
    MAX_PIVOT_RATE = 3.0
    # [s]
    YAW_TIME_CONSTANT = 0.25
    TRUE_WHEEL_RADIUS = 0.10
    TRUE_BATTERY_VOLTS = 12.4
    TRUE_VOLT_MULT = 10.1
    TRUE_BOARD_ORIENTATION = "Yaw45Roll90"
    # how the simulated external compass is really mounted
    TRUE_COMPASS_ORIENTATION = "Yaw90"
    # [mGauss] horizontal field, towards magnetic north
    EARTH_FIELD_NORTH = 200.0
    # [mGauss] vertical field (downwards in the northern hemisphere)
    EARTH_FIELD_DOWN = 400.0
    # northern hemisphere
    LATITUDE_DEG = 48.1
    # the firmware the simulator pretends to run
    FIRMWARE_VERSION = (4, 5, 7)
    # the simulated vehicle's MAVLink system id
    SYSTEM_ID = 1
    # SERIAL1..7 as on a typical Pixhawk Rover: two telemetry ports (MAVLink2, 57600), two GPS ports, three unused.
    # (protocol, baud)
    SERIAL_SETUP = [(2, 57), (2, 57), (5, 230), (5, 38), (-1, 57), (-1, 57), (-1, 57)]
    # [%] simulated compass-calibration progress reports ...
    COMPASS_PROGRESS_STEPS = (10, 40, 70, 100)
    # ... this far apart
    CALIBRATION_STEP_SECONDS = 0.5
    # [s] a finished compass calibration's MAG_CAL_REPORT repeats this often (part of a telemetry stream)
    MAG_CAL_REPORT_INTERVAL_SECONDS = 0.5
    # [mGauss] a compass's calibration fitness unless a test sets another one
    DEFAULT_COMPASS_FITNESS = 6.5
    # [s] AP_AccelCal repeats an unconfirmed position request this often (AP_ACCELCAL_POSITION_REQUEST_INTERVAL_MS)
    ACCEL_REQUEST_INTERVAL_SECONDS = 1.0
    # [s] collecting the samples of one confirmed position
    ACCEL_SAMPLE_SECONDS = 2.0
    # [cA] 3 A at standstill, more when driving
    STANDSTILL_CURRENT_CA = 300
    # [cA] extra current of one motor at 100 % motor test (wheels lifted)
    MOTOR_TEST_FULL_POWER_CA = 800
    # [mGauss/A] field of the motor current at the compass (in its reported axes)
    TRUE_MOTOR_INTERFERENCE = (12.0, -6.0, 25.0)

    def __init__(self, skid: bool = False, wired_wrong: bool = False) -> None:
        """Create the simulated vehicle; wired_wrong=True inverts throttle and steering like a fresh real build."""
        super().__init__()
        self.skid = skid
        self.version = self.FIRMWARE_VERSION
        self.vehicle_type = MAV_TYPE_GROUND_ROVER
        self.system_id = self.SYSTEM_ID
        self.custom_mode = MODES["HOLD"]
        self.armed = False
        # stands in for the pymavlink link (gcs_system_id reads it)
        self.master = SimpleNamespace(source_system=GCS_SOURCE_SYSTEM)
        # Physical state: local north/east position [m], heading [rad], forward speed [m/s], turn rate [rad/s].
        self.north = self.east = self.heading = self.speed = self.yaw_rate = 0.0
        self.distance_travelled = 0.0
        # What the "pilot" is doing (tests set these): sticks, a forced raw RC picture, and the vehicle's tilt.
        self.pilot_sticks = (0.0, 0.0)
        self.raw_rc_override = None
        self.held_pitch_deg = 0.0
        # the pilot's transmitter (tests switch it off to trigger the radio failsafe)
        self.transmitter_on = True
        # radio failsafe (Rover failsafe.cpp): since when RC is missing, and whether the failsafe has acted
        self._radio_lost_since = None
        self._radio_failsafe_triggered = False
        # Physical wiring errors the wizard has to find and fix (independent of SERVOx_REVERSED).
        self.throttle_wired_backwards = self.steering_wired_backwards = wired_wrong
        # GPS behaviour: None = a receiver that Auto detects; otherwise the only GPS_TYPE that finds it.
        self.true_gps_type = None
        # GPS_FIX_TYPE the receiver reports (3 = 3D fix)
        self.gps_fix_type = 3
        # MAG_CAL_REPORT.cal_status the calibrations end with
        self.compass_calibration_status = MAG_CAL_SUCCESS
        # how many compasses calibrate, and results differing from compass_calibration_status by compass id
        self.calibrating_compasses = 1
        self.compass_calibration_statuses = {}
        # compass id -> calibration fitness [mGauss]: fails above the limit (see _compass_fitness_status)
        self.compass_fitness = {}
        # compass id -> report of a finished calibration, repeated until the next one starts
        self.finished_mag_reports = {}
        self._mag_reports_sent_at = 0.0
        # channels the simulated receiver delivers (chancount)
        self.rc_channel_count = 12
        # Hidden peripherals: they only answer when configured with exactly these settings (None = not fitted).
        # (type, serial port or 0 for I2C) of rangefinder 1, or (instance "1".."A", type, port)
        self.true_rangefinder = None
        # SERIALx_BAUD a serial hidden rangefinder needs (None = any rate)
        self.true_rangefinder_baud = None
        # (type, serial port, SERIALx_BAUD) of proximity sensor 1, or (instance, type, port, baud)
        self.true_proximity = None
        # MNT1_TYPE that makes the gimbal move
        self.true_gimbal_type = None
        # accelerometer calibration (like AP_AccelCal): the position waited for, when it was last requested, and
        # the position whose samples are being collected with the time collecting ends
        self._accel_cal_waiting_for = None
        self._accel_cal_requested_at = None
        self._accel_cal_collecting = None
        # output -> SERVOx_FUNCTION as last picked up: like Rover, the outputs' functions are re-read once a second
        self._applied_functions: dict = {}
        self._functions_applied_at = None
        # an arming check that fails (e.g. "Arm: Compass not calibrated") while arming checks are enabled
        self.failing_arming_check = None
        # (serial port, SERIALx_BAUD) the hidden gimbal is wired to (None = no wiring check)
        self.true_gimbal_serial = None
        # where the simulated gimbal points [deg]
        self.gimbal_pitch = 0.0
        # output -> pulse width set by DO_SET_SERVO / motor tests
        self._servo_pwm: dict = {}
        # like ArduPilot, a motor test arms the vehicle until it ends
        self._motor_test_until = None
        # motor-test instance -> power [%] while the test runs
        self._motor_test_power: dict = {}
        # False: the battery monitor measures no current (SYS_STATUS.current_battery = -1)
        self.reports_current = True
        # relay 1..6 -> on/off, as last switched with DO_SET_RELAY
        self.relay_states: dict = {}
        # magnetic heading the pilot holds the vehicle at (0 = north)
        self.held_heading_deg = 0.0
        # GPS_TYPE that finds a second receiver (None = not fitted)
        self.true_gps2_type = None
        # MNT2_TYPE that makes a second gimbal move
        self.true_gimbal2_type = None
        # gimbal instance -> pitch it was pointed to
        self.gimbal_pitches: dict = {}
        # "grab" / "release" after DO_GRIPPER
        self.gripper_state = None
        # source set chosen with SET_EKF_SOURCE_SET
        self.ekf_source_set = 1
        self.motor_tests: list = []
        self._override_sticks = None
        self._override_time = 0.0
        self._guided_target = (0.0, 0.0)
        self._guided_integral = [0.0, 0.0]
        self._running = False
        self._build_params()

    # ------------------------------------------------------------------ parameters
    def _build_params(self) -> None:
        """A realistic starting parameter set (ArduPilot defaults for everything the wizard touches)."""
        params = {}
        for output in SERVO_OUTPUTS:
            params.update(
                {
                    f"SERVO{output}_FUNCTION": 0,
                    f"SERVO{output}_MIN": 1100,
                    f"SERVO{output}_MAX": 1900,
                    f"SERVO{output}_TRIM": 1500,
                    f"SERVO{output}_REVERSED": 0,
                    f"RC{output}_MIN": 1100,
                    f"RC{output}_MAX": 1900,
                    f"RC{output}_TRIM": 1500,
                    f"RC{output}_REVERSED": 0,
                    f"RC{output}_OPTION": 0,
                }
            )

        if self.skid:
            params["SERVO1_FUNCTION"] = SERVO_FUNCTION_THROTTLE_LEFT
            params["SERVO3_FUNCTION"] = SERVO_FUNCTION_THROTTLE_RIGHT
        else:
            params["SERVO1_FUNCTION"] = SERVO_FUNCTION_GROUND_STEERING
            params["SERVO3_FUNCTION"] = SERVO_FUNCTION_THROTTLE

        params.update(
            RCMAP_ROLL=1,
            RCMAP_THROTTLE=3,
            MODE_CH=8,
            PILOT_STEER_TYPE=0,
            FRAME_CLASS=1,
            MOT_PWM_TYPE=0,
            MOT_PWM_FREQ=16,
            MOT_THR_MIN=0,
            MOT_THR_MAX=100,
            MOT_SLEWRATE=100,
            MOT_STR_THR_MIX=0.5,
            MOT_REV_DELAY=0,
            ATC_BRAKE=1,
            RC_OVERRIDE_TIME=3,
            GUID_TIMEOUT=3,
            FENCE_ENABLE=0,
            FENCE_TYPE=6,
            FENCE_RADIUS=300,
            FENCE_ACTION=1,
            AHRS_ORIENTATION=0,
            GPS1_TYPE=1,
            GPS1_POS_X=0,
            GPS1_POS_Y=0,
            GPS1_POS_Z=0,
            COMPASS_AUTODEC=1,
            COMPASS_ORIENT=0,
            SYSID_MYGCS=255,
            INS_GYRO_FILTER=20,
            SCR_ENABLE=0,
            ARMING_CHECK=1,
            FS_ACTION=2,
            FS_TIMEOUT=5,
            FS_OPTIONS=0,
            FS_THR_ENABLE=1,
            FS_THR_VALUE=910,
            FS_GCS_ENABLE=0,
            FS_CRASH_CHECK=0,
            FS_EKF_ACTION=1,
            FS_EKF_THRESH=0.8,
            BATT_MONITOR=4,
            BATT_VOLT_PIN=13,
            BATT_CURR_PIN=14,
            BATT_VOLT_MULT=10.0,
            BATT_AMP_PERVLT=17.0,
            BATT_AMP_OFFSET=0,
            BATT_CAPACITY=5000,
            BATT_LOW_VOLT=10.5,
            BATT_CRT_VOLT=10.0,
            BATT_FS_LOW_ACT=0,
            BATT_FS_CRT_ACT=0,
            BATT_LOW_TIMER=10,
            BATT_FS_VOLTSRC=0,
            BATT_LOW_MAH=0,
            BATT_CRT_MAH=0,
            BATT_ARM_VOLT=0,
            SPEED_MAX=0,
            CRUISE_SPEED=2,
            CRUISE_THROTTLE=50,
            TURN_RADIUS=0.9,
            ACRO_TURN_RATE=180,
            ATC_ACCEL_MAX=1,
            ATC_DECEL_MAX=0,
            ATC_STR_RAT_FF=0.2,
            ATC_STR_RAT_P=0.2,
            ATC_STR_RAT_I=0.2,
            ATC_STR_RAT_D=0,
            ATC_STR_RAT_FLTT=20,
            ATC_STR_RAT_FLTD=5,
            ATC_STR_RAT_MAX=120,
            ATC_STR_ACC_MAX=120,
            ATC_TURN_MAX_G=0.6,
            ATC_SPEED_P=0.2,
            ATC_SPEED_I=0.2,
            ATC_SPEED_D=0,
            WP_SPEED=2,
            WP_RADIUS=2,
            WP_OVERSHOOT=2,
            RTL_SPEED=0,
            NAVL1_PERIOD=10,
            NAVL1_DAMPING=0.75,
            WP_PIVOT_ANGLE=60,
            WP_PIVOT_RATE=90,
            WENC_TYPE=1,
            WENC_CPR=1000,
            WENC_RADIUS=0.09,
            WENC_POS_X=0,
            WENC_POS_Y=0,
            WENC_PINA=54,
            WENC_PINB=55,
            WENC2_TYPE=1,
            WENC2_CPR=1000,
            WENC2_RADIUS=0.09,
            WENC2_POS_X=0,
            WENC2_POS_Y=0,
            WENC2_PINA=52,
            WENC2_PINB=53,
            COMPASS_ENABLE=1,
            AVOID_ENABLE=3,
            AVOID_MARGIN=2,
            AVOID_BEHAVE=0,
            AVOID_BACKUP_SPD=0.5,
            AVOID_ACCEL_MAX=3,
            MNT1_TYPE=0,
            MNT1_PITCH_MIN=-90,
            MNT1_PITCH_MAX=20,
            MNT1_YAW_MIN=-180,
            MNT1_YAW_MAX=180,
            MODE1=0,
            MODE2=0,
            MODE3=0,
            MODE4=0,
            MODE5=0,
            MODE6=0,
            GPS2_TYPE=0,
            GPS2_POS_X=0,
            GPS2_POS_Y=0,
            GPS2_POS_Z=0,
            GPS_AUTO_SWITCH=1,
            GPS_PRIMARY=0,
            COMPASS_AUTO_ROT=2,
            COMPASS_CAL_FIT=16,
            COMPASS_EXTERNAL=1,
            COMPASS_DEV_ID=97539,
            COMPASS_PRIO1_ID=97539,
            COMPASS_DEV_ID2=0,
            COMPASS_DEV_ID3=0,
            COMPASS_PRIO2_ID=0,
            COMPASS_PRIO3_ID=0,
            COMPASS_EXTERN2=0,
            COMPASS_EXTERN3=0,
            COMPASS_ORIENT2=0,
            COMPASS_ORIENT3=0,
            COMPASS_USE=1,
            COMPASS_USE2=1,
            COMPASS_USE3=1,
            COMPASS_DEC=0,
            COMPASS_MOTCT=0,
            COMPASS_MOT_X=0.0,
            COMPASS_MOT_Y=0.0,
            COMPASS_MOT_Z=0.0,
            INS_ACC_ID=2753028,
            INS_GYR_ID=2752772,
            MNT2_TYPE=0,
            MNT2_PITCH_MIN=-90,
            MNT2_PITCH_MAX=20,
            MNT2_YAW_MIN=-180,
            MNT2_YAW_MAX=180,
            GRIP_ENABLE=0,
            GRIP_TYPE=0,
            GRIP_GRAB=1900,
            GRIP_RELEASE=1100,
            GRIP_NEUTRAL=1500,
            GRIP_AUTOCLOSE=0,
            GRIP_REGRAB=0,
            TEMP_LOG=0,
            LOG_BACKEND_TYPE=1,
            LOG_BITMASK=65535,
            LOG_DISARMED=0,
            LOG_FILE_DSRMROT=0,
            LOG_DARM_RATEMAX=0,
            LOG_FILE_BUFSIZE=200,
            LOG_FILE_MB_FREE=500,
        )

        # Two rangefinder and two proximity-sensor instances (ArduPilot has 10 and 5).
        for instance in "12":
            params.update(
                {
                    f"RNGFND{instance}_TYPE": 0,
                    f"RNGFND{instance}_ADDR": 0,
                    f"RNGFND{instance}_ORIENT": 0,
                    f"RNGFND{instance}_MIN": 0.2,
                    f"RNGFND{instance}_MAX": 7,
                    f"RNGFND{instance}_POS_X": 0,
                    f"RNGFND{instance}_POS_Y": 0,
                    f"RNGFND{instance}_POS_Z": 0,
                }
            )
            params.update(
                {
                    f"PRX{instance}_TYPE": 0,
                    f"PRX{instance}_ORIENT": 0,
                    f"PRX{instance}_YAW_CORR": 0,
                    f"PRX{instance}_MIN": 0,
                    f"PRX{instance}_MAX": 0,
                }
            )

        for sensor in SIMULATED_TEMPERATURE_SENSORS:
            params.update({f"TEMP{sensor}_TYPE": 0, f"TEMP{sensor}_BUS": 0, f"TEMP{sensor}_ADDR": 0})

        for sensor in RPM_SENSORS:
            params.update(
                {
                    f"RPM{sensor}_TYPE": 0,
                    f"RPM{sensor}_PIN": -1,
                    f"RPM{sensor}_SCALING": 1,
                    f"RPM{sensor}_MIN": 10,
                    f"RPM{sensor}_MAX": 100000,
                    f"RPM{sensor}_MIN_QUAL": 0.5,
                    f"RPM{sensor}_ESC_MASK": 0,
                }
            )

        # Standard firmware: no DID_ (Remote ID) parameters - those need a firmware built with OpenDroneID.
        params.update(
            SPRAY_ENABLE=0,
            SPRAY_PUMP_RATE=10,
            SPRAY_PUMP_MIN=0,
            SPRAY_SPEED_MIN=100,
            SPRAY_SPINNER=1300,
            FLOW_TYPE=0,
            FLOW_FXSCALER=0,
            FLOW_FYSCALER=0,
            FLOW_ORIENT_YAW=0,
            FLOW_POS_X=0,
            FLOW_POS_Y=0,
            FLOW_POS_Z=0,
            FLOW_ADDR=0,
            VISO_TYPE=0,
            VISO_POS_X=0,
            VISO_POS_Y=0,
            VISO_POS_Z=0,
            BCN_TYPE=0,
            BCN_LATITUDE=0,
            BCN_LONGITUDE=0,
            BCN_ALT=0,
            BCN_ORIENT_YAW=0,
            EK3_SRC1_POSXY=3,
            EK3_SRC1_VELXY=3,
            EK3_SRC1_POSZ=1,
            EK3_SRC1_VELZ=3,
            EK3_SRC1_YAW=1,
            OSD_TYPE=0,
            OSD_CHAN=0,
            OSD_SW_METHOD=0,
            OSD_UNITS=0,
            OSD_FONT=0,
            OSD_W_RSSI=30,
            OSD_W_NSAT=9,
            OSD_MSG_TIME=10,
            GEN_TYPE=0,
            GEN_OPTIONS=0,
            EK3_SRC_OPTIONS=1,
            AHRS_EKF_TYPE=3,
            EK3_ENABLE=1,
            EK3_IMU_MASK=3,
            EK3_GPS_CHECK=31,
            EK3_CHECK_SCALE=100,
            EK3_WENC_VERR=0.1,
            EK3_FLOW_DELAY=10,
            EK3_VIS_VERR_MIN=0.1,
            EK3_VIS_VERR_MAX=0.9,
            EK3_BCN_DELAY=50,
        )

        # ArduPilot's defaults for the extra source sets: all "None"
        for source_set in (2, 3):
            params.update({f"EK3_SRC{source_set}_{axis}": 0 for axis in ("POSXY", "VELXY", "POSZ", "VELZ", "YAW")})

        # firmware 4.5+ relay parameters
        for relay in RELAYS:
            params.update(
                {
                    f"RELAY{relay}_FUNCTION": 0,
                    f"RELAY{relay}_PIN": -1,
                    f"RELAY{relay}_DEFAULT": 0,
                    f"RELAY{relay}_INVERTED": 0,
                }
            )

        for port, (protocol, baud) in enumerate(self.SERIAL_SETUP, 1):
            params[f"SERIAL{port}_PROTOCOL"] = protocol
            params[f"SERIAL{port}_BAUD"] = baud

        for index, (name, value) in enumerate(params.items()):
            integer = any(marker in name for marker in ("FUNCTION", "REVERSED", "TYPE"))
            self._store_param(
                name, float(value), MAV_PARAM_TYPE_INT32 if integer else MAV_PARAM_TYPE_REAL32, index, len(params)
            )

    def set_param(self, name: str, value: float) -> float:
        """Write a parameter directly (no link round-trip in simulation)."""
        if not self.has_parameter(name):
            raise VehicleError(f"Parameter {name} does not exist on this firmware.")

        self._params[name] = float(value)
        return self._params[name]

    # ------------------------------------------------------------------ connection
    def connect(
        self,
        connection_string: str = "sim",
        baud_rate: int = 0,
        timeout: float = 0,
        progress=None,
        link_only: bool = False,
    ) -> None:
        """Same signature as Vehicle.connect(); starts the simulation."""
        self.start()

    def start(self) -> None:
        """Start the physics/telemetry thread and publish the initial telemetry."""
        self._running = True
        self._publish_telemetry(timebase.now())
        threading.Thread(target=self._loop, daemon=True).start()
        self._add_status_text(f"ArduRover V{'.'.join(map(str, self.FIRMWARE_VERSION))} (simulated)")

    def disconnect(self) -> None:
        """Stop the simulation thread."""
        self._running = False

    def reboot(self, progress=None) -> None:
        """A reboot disarms and stops all outputs/sensors; parameters persist (like real EEPROM storage)."""
        self.armed = False
        if self.get_int("SCR_ENABLE") and not self.has_parameter("SCR_HEAP_SIZE"):
            # Like ArduPilot: scripting's other parameters exist only after a reboot with SCR_ENABLE set.
            extra = {"SCR_HEAP_SIZE": 100000, "SCR_VM_I_COUNT": 10000, "SCR_DEBUG_OPTS": 0, "SCR_DIR_DISABLE": 0}
            # SCR_USER1..SCR_USER6
            extra.update({f"SCR_USER{number}": 0 for number in range(1, 7)})
            for name, value in extra.items():
                self._store_param(name, float(value), MAV_PARAM_TYPE_INT32, PARAM_INDEX_UNKNOWN, self._param_total)

        self._distance_sensors.clear()
        self._servo_pwm.clear()
        timebase.sleep(1.0)

    def setup_streams(self) -> None:
        """Nothing to request - the simulator always publishes everything."""

    def fetch_params(self, progress=None, timeout: float = 0) -> None:
        """Nothing to download - the parameters are already in memory."""

    def read_param(self, name: str):
        """The parameter's value, or None if the simulated vehicle doesn't have it."""
        return self._params.get(name)

    def read_version(self) -> None:
        """The version is fixed at construction."""

    # ------------------------------------------------------------------ commands
    def command_long(self, command: int, *params: float, wait: bool = True, timeout: float = 3.0):
        """Emulate the commands the wizard sends: mode, arm/disarm, motor test, calibrations."""
        # missing params are 0
        params = list(params) + [0] * (COMMAND_LONG_PARAMS - len(params))
        if command == MAV_CMD_DO_SET_MODE:
            self.custom_mode = int(params[1])
        elif command == MAV_CMD_COMPONENT_ARM_DISARM:
            self.armed = bool(params[0])
        elif command == MAV_CMD_DO_MOTOR_TEST:
            # the test arms the vehicle, which runs the arming checks (Rover/motor_test.cpp)
            if not self.armed and self.failing_arming_check and not self._are_arming_checks_skipped():
                self._add_status_text(self.failing_arming_check)
                return MAV_RESULT_FAILED

            self.motor_tests.append((int(params[0]), params[2]))
            self._motor_test_power[int(params[0])] = params[2]
            # Rover/motor_test.cpp: the test arms the vehicle if needed and disarms it when it times out.
            self.armed = True
            # param4 = timeout [s]
            self._motor_test_until = timebase.now() + params[3]
            output = self._output_with_function(FUNCTION_BY_TEST_INSTANCE.get(int(params[0]), -1))
            if output:
                self._servo_pwm[output] = self._pwm_for_percent(output, params[2])
        elif command == MAV_CMD_DO_SET_SERVO:
            output = int(params[0])
            pwm = int(params[1])
            if self.get_int(f"SERVO{output}_FUNCTION") not in SERVO_FUNCTIONS_DIRECTLY_SETTABLE:
                self._add_status_text(f"ServoRelayEvent: Channel {output} is already in use")
                return MAV_RESULT_FAILED
            self._servo_pwm[output] = pwm
        elif command == MAV_CMD_DO_SET_RELAY:
            # the command numbers relays from 0
            relay = int(params[0]) + 1
            if (
                self.get_int(f"RELAY{relay}_FUNCTION") != RELAY_FUNCTION_RELAY
                or self.get_int(f"RELAY{relay}_PIN", -1) < 0
            ):
                # ArduPilot refuses relays without function and pin
                return MAV_RESULT_FAILED
            self.relay_states[relay] = bool(params[1])
        elif command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
            # param7 = gimbal device id (0 = the primary one)
            instance = int(params[6]) or 1
            true_type = self.true_gimbal_type if instance == 1 else self.true_gimbal2_type
            if true_type is not None and self.get_int(f"MNT{instance}_TYPE") == true_type and self._is_gimbal_wired():
                self.gimbal_pitches[instance] = params[0]
                if instance == 1:
                    self.gimbal_pitch = params[0]
        elif command == MAV_CMD_SET_EKF_SOURCE_SET:
            self.ekf_source_set = int(params[0])
        elif command == MAV_CMD_DO_GRIPPER:
            if not self.get_int("GRIP_ENABLE") or not self.get_int("GRIP_TYPE"):
                return MAV_RESULT_FAILED
            self.gripper_state = "grab" if int(params[1]) == GRIPPER_ACTION_GRAB else "release"
        elif command == MAV_CMD_DO_MOUNT_CONTROL:
            if (
                self.true_gimbal_type is not None
                and self.get_int("MNT1_TYPE") == self.true_gimbal_type
                and self._is_gimbal_wired()
            ):
                self.gimbal_pitch = params[0]
        # param5 selects the accelerometer calibration.
        elif command == MAV_CMD_PREFLIGHT_CALIBRATION and params[4] == PREFLIGHT_CALIBRATION_ACCEL:
            # like AP_AccelCal::start: ignored (still accepted) while a calibration runs - it goes on where it is
            if self._accel_cal_waiting_for is None and self._accel_cal_collecting is None:
                self.clear_inbox(INBOX_ACCEL_CALIBRATION)
                self._accel_cal_waiting_for = ACCELCAL_VEHICLE_POS_LEVEL
                self._accel_cal_requested_at = None
        elif command == MAV_CMD_ACCELCAL_VEHICLE_POS:
            # like AP_AccelCal::gcs_vehicle_position: only the position being waited for is accepted
            if int(params[0]) != self._accel_cal_waiting_for:
                return MAV_RESULT_FAILED
            self._accel_cal_collecting = (self._accel_cal_waiting_for, timebase.now() + self.ACCEL_SAMPLE_SECONDS)
            self._accel_cal_waiting_for = None
        elif command == MAV_CMD_DO_START_MAG_CAL:
            # a new calibration resets the calibrators: their old reports stop
            self.finished_mag_reports = {}
            threading.Thread(target=self._simulate_compass_calibration, daemon=True).start()
        elif command == MAV_CMD_DO_CANCEL_MAG_CAL:
            self.finished_mag_reports = {}
        elif command == MAV_CMD_FIXED_MAG_CAL_YAW:
            # Large Vehicle MagCal sets and saves the offsets right away; it sends no MAG_CAL_REPORT
            pass

        return MAV_RESULT_ACCEPTED

    def _pwm_for_percent(self, output: int, percent: float) -> int:
        """Pulse width a motor test at `percent` produces on an output (its MIN/TRIM/MAX, reversal respected)."""
        low, trim, high = (self.get(f"SERVO{output}_{part}") for part in ("MIN", "TRIM", "MAX"))
        # percent -> fraction of full output, mirrored for a reversed output
        fraction = percent / 100 * (-1 if self.get(f"SERVO{output}_REVERSED") else 1)
        # Above zero scale into TRIM..MAX, below zero into MIN..TRIM (the two halves may differ in size).
        return int(trim + fraction * ((high - trim) if fraction >= 0 else (trim - low)))

    def _compass_fitness_status(self, compass: int, fitness: float) -> int:
        """Like ArduPilot: fails above COMPASS_CAL_FIT - doubled unless it is the first compass and external."""
        limit = self.get("COMPASS_CAL_FIT")
        if not (compass == 0 and self.get_int("COMPASS_EXTERNAL")):
            limit *= 2

        if fitness > limit:
            return MAG_CAL_FAILED

        return self.compass_calibration_status

    def _simulate_compass_calibration(self) -> None:
        """Progress for each calibrating compass (later ones finish later), then each one's report, repeated."""
        mask = (1 << self.calibrating_compasses) - 1
        steps = len(self.COMPASS_PROGRESS_STEPS)
        # compass k lags k steps behind compass 0
        for tick in range(steps + self.calibrating_compasses - 1):
            timebase.sleep(self.CALIBRATION_STEP_SECONDS)
            for compass in range(self.calibrating_compasses):
                step = tick - compass
                if 0 <= step < steps:
                    self._push_inbox(
                        INBOX_MAG_CAL_PROGRESS,
                        SimpleNamespace(
                            cal_mask=mask, compass_id=compass, completion_pct=self.COMPASS_PROGRESS_STEPS[step]
                        ),
                    )

                if step == steps - 1:
                    # finished: its report repeats with the telemetry stream until the next calibration starts
                    fitness = self.compass_fitness.get(compass, self.DEFAULT_COMPASS_FITNESS)
                    status = self.compass_calibration_statuses.get(
                        compass, self._compass_fitness_status(compass, fitness)
                    )
                    self.finished_mag_reports[compass] = SimpleNamespace(
                        compass_id=compass, cal_status=status, fitness=fitness
                    )

    def set_mode(self, name: str, timeout: float = MODE_CHANGE_TIMEOUT) -> None:
        """Mode changes are instantaneous."""
        self.custom_mode = MODES[name]

    def arm(self) -> None:
        """Arming always succeeds."""
        self.armed = True

    def disarm(self, force: bool = False) -> None:
        """Disarming is instantaneous."""
        self.armed = False

    def rc_override(self, pwm_by_channel: dict) -> None:
        """Remember the overridden sticks (converted back from pulse widths, as ArduPilot would read them)."""
        if not pwm_by_channel:
            self._override_sticks = None
            return
        steering_channel, throttle_channel = self.rc_mapping()

        def stick(channel):
            """Stick position the autopilot reads from the overridden pulse width of one channel."""
            return pwm_to_stick(pwm_by_channel.get(channel, RC_PWM_NOMINAL_TRIM), self.rc_calibration(channel))

        self._override_sticks = (stick(steering_channel), stick(throttle_channel))
        self._override_time = timebase.now()

    def guided_cmd(self, speed: float, yaw_rate: float) -> None:
        """Remember the Guided target for the simulated speed/steering controllers."""
        self._guided_target = (speed, yaw_rate)

    def emergency_stop(self) -> None:
        """STOP: release the override, zero the target, Hold, disarm."""
        self._override_sticks = None
        self._guided_target = (0.0, 0.0)
        self.custom_mode = MODES["HOLD"]
        self.armed = False

    # ------------------------------------------------------------------ test helpers
    def roll_by_hand(self, meters: float) -> None:
        """Simulate the pilot pushing the vehicle forward (for the wheel-encoder calibration)."""
        self.distance_travelled += meters

    # ------------------------------------------------------------------ telemetry
    def _current_sticks(self) -> tuple:
        """(steering, throttle) in effect: a fresh RC override wins over the pilot's sticks."""
        if self._override_sticks is not None and timebase.now() - self._override_time < RC_OVERRIDE_TIMEOUT:
            return self._override_sticks

        return self.pilot_sticks

    def rc_raw(self):
        """Raw pulse widths of the receiver's channels, derived from the sticks in effect (or a test's forced picture)."""
        if self.raw_rc_override is not None:
            return list(self.raw_rc_override)

        # every channel the receiver delivers, centred ...
        raw = [RC_PWM_NOMINAL_TRIM] * self.rc_channel_count
        # ... except steering and throttle, whose pulse widths come from the current sticks.
        for channel, position in zip(self.rc_mapping(), self._current_sticks()):
            raw[channel - 1] = stick_to_pwm(position, self.rc_calibration(channel))

        return raw

    def sample(self) -> DriveSample:
        """A drive sample with measurement noise; pilot inputs are only meaningful in Manual/Acro."""
        manual = self.custom_mode in (MODES["MANUAL"], MODES["ACRO"])
        steering, throttle = self._current_sticks() if manual else (0.0, 0.0)
        return DriveSample(
            time=timebase.now(),
            forward_speed=self.speed + random.gauss(0, SPEED_NOISE),
            yaw_rate=self.yaw_rate + random.gauss(0, YAW_RATE_NOISE),
            steering_input=steering,
            throttle_input=throttle,
            north=self.north,
            east=self.east,
        )

    # ------------------------------------------------------------------ physics
    def _output_with_function(self, function: int):
        """The SERVO output currently assigned `function`, or None."""
        for output in SERVO_OUTPUTS:
            if self.get_int(f"SERVO{output}_FUNCTION") == function:
                return output

        return None

    def _output_sign(self, function: int, wired_backwards: bool) -> int:
        """+1 or -1: the combined effect of physical wiring and the output's SERVOx_REVERSED setting."""
        output = self._output_with_function(function)
        reversed_setting = bool(self.get(f"SERVO{output}_REVERSED", 0)) if output else False
        # Two inversions cancel out: that is exactly how the wizard's "Reverse output" fixes bad wiring.
        return (-1 if wired_backwards else 1) * (-1 if reversed_setting else 1)

    def _commanded_sticks(self, step: float) -> tuple:
        """(steering, throttle) -1..1 the autopilot outputs this step: sticks, or the Guided-mode controllers."""
        if not self.armed or self.custom_mode == MODES["HOLD"]:
            # a stopped controller forgets its integrators
            self._guided_integral = [0.0, 0.0]
            return 0.0, 0.0

        if self.custom_mode != MODES["GUIDED"]:
            return self._current_sticks()

        target_speed, target_yaw_rate = self._guided_target
        # Speed feed-forward: CRUISE_THROTTLE at CRUISE_SPEED means throttle per m/s = cruise throttle / cruise speed.
        cruise_speed = self.get("CRUISE_SPEED")
        # CRUISE_THROTTLE is in percent
        cruise_throttle = self.get("CRUISE_THROTTLE") / 100.0
        throttle_per_speed = cruise_throttle / cruise_speed if cruise_speed > 0 else 1 / self.TOP_SPEED
        speed_error = target_speed - self.speed
        yaw_error = target_yaw_rate - self.yaw_rate
        # Integrators: accumulate I-gain x error x time, clamped to +-1 (anti-windup).
        self._guided_integral[0] = max(
            -1, min(1, self._guided_integral[0] + self.get("ATC_SPEED_I") * speed_error * step)
        )
        self._guided_integral[1] = max(
            -1, min(1, self._guided_integral[1] + self.get("ATC_STR_RAT_I") * yaw_error * step)
        )
        # Output = feed-forward x target + P x error + integrator (the structure of ArduPilot's controllers).
        throttle = throttle_per_speed * target_speed + self.get("ATC_SPEED_P") * speed_error + self._guided_integral[0]
        steering = (
            self.get("ATC_STR_RAT_FF") * target_yaw_rate
            + self.get("ATC_STR_RAT_P") * yaw_error
            + self._guided_integral[1]
        )
        return max(-1, min(1, steering)), max(-1, min(1, throttle))

    def _physics_step(self, step: float) -> None:
        """Advance the vehicle by `step` seconds: outputs -> motor effort -> target speeds -> lagged speeds -> position."""
        steering, throttle = self._commanded_sticks(step)
        # MOT_THR_MIN is in percent
        configured_deadzone = self.get("MOT_THR_MIN") / 100.0

        def autopilot_output(command):
            """ArduPilot adds MOT_THR_MIN: any non-zero command starts at the configured dead-zone."""
            # practically zero throttle
            if abs(command) <= 1e-3:
                return 0.0
            # Map |command| 0..1 onto dead-zone..1 (linear), keeping the command's sign.
            return math.copysign(configured_deadzone + abs(command) * (1 - configured_deadzone), command)

        def motor_effort(output):
            """The real motor ignores the first 8 % and scales the rest back to 0..1."""
            # Subtract the motor's own dead-zone (never below 0), rescale the rest to 0..1, keep the sign.
            return math.copysign(max(0.0, abs(output) - MOTOR_DEADZONE) / (1 - MOTOR_DEADZONE), output)

        if self.skid:
            # Differential mixing: left = throttle + steering, right = throttle - steering (clamped to +-1).
            left_output = max(-1, min(1, throttle + steering))
            right_output = max(-1, min(1, throttle - steering))
            left = motor_effort(autopilot_output(left_output)) * self._output_sign(
                SERVO_FUNCTION_THROTTLE_LEFT, self.throttle_wired_backwards
            )
            right = motor_effort(autopilot_output(right_output)) * self._output_sign(
                SERVO_FUNCTION_THROTTLE_RIGHT, self.throttle_wired_backwards
            )
            # The physical left motor sits on the lower channel number; assigning the functions the other way round
            # (or crossed steering wiring) swaps which side is actually driven.
            left_channel = self._output_with_function(SERVO_FUNCTION_THROTTLE_LEFT)
            right_channel = self._output_with_function(SERVO_FUNCTION_THROTTLE_RIGHT)
            if left_channel and right_channel and left_channel > right_channel:
                left, right = right, left

            if self.steering_wired_backwards:
                left, right = right, left
            # Speed = mean of both sides; turn rate proportional to their difference.
            target_speed = self.TOP_SPEED * (left + right) / 2
            target_yaw_rate = self.MAX_PIVOT_RATE * (left - right) / 2
        else:
            target_speed = (
                self.TOP_SPEED
                * motor_effort(autopilot_output(throttle))
                * self._output_sign(SERVO_FUNCTION_THROTTLE, self.throttle_wired_backwards)
            )
            steering *= self._output_sign(SERVO_FUNCTION_GROUND_STEERING, self.steering_wired_backwards)
            # A car turns at yaw rate = speed / radius; full lock gives the minimum radius.
            target_yaw_rate = max(
                -MAX_YAW_RATE_ACKERMANN, min(MAX_YAW_RATE_ACKERMANN, self.speed * steering / self.MIN_TURN_RADIUS)
            )
        if not self.armed or self.custom_mode == MODES["HOLD"]:
            target_speed = 0.0
        # First-order lags: close the fraction step/tau of the remaining difference each step (tau = time constant).
        speeding_up = abs(target_speed) > abs(self.speed)
        speed_tau = self.SPEED_TIME_CONSTANT if speeding_up else DECELERATION_TIME_CONSTANT
        self.speed += (target_speed - self.speed) / speed_tau * step
        self.yaw_rate += (target_yaw_rate - self.yaw_rate) / self.YAW_TIME_CONSTANT * step
        # Dead reckoning: heading integrates the turn rate; position integrates speed along the heading.
        self.heading += self.yaw_rate * step
        self.north += self.speed * math.cos(self.heading) * step
        self.east += self.speed * math.sin(self.heading) * step
        self.distance_travelled += self.speed * step

    def _loop(self) -> None:
        """Background thread: integrate the physics in fixed steps and publish telemetry."""
        last = timebase.now()
        while self._running:
            # real-time pacing; the physics itself runs on the (scaled) wizard clock
            time.sleep(LOOP_PERIOD)
            current = timebase.now()
            # Integrate the elapsed wizard time in PHYSICS_STEP pieces (capped at 1 s after e.g. a debugger pause).
            # Both computed from the OLD `last`, so they must stay one simultaneous assignment.
            remaining, last = min(current - last, 1.0), current
            # integrate until (practically) no time is left
            while remaining > 1e-6:
                step = min(remaining, PHYSICS_STEP)
                self._physics_step(step)
                remaining -= step
            self._publish_telemetry(current)

    def _advance_accel_calibration(self, timestamp: float) -> None:
        """Like AP_AccelCal::update: repeat the position request until confirmed, then collect, then move on."""
        if self._accel_cal_collecting is not None:
            position, done_at = self._accel_cal_collecting
            if timestamp >= done_at:
                self._accel_cal_collecting = None
                if position < ACCELCAL_VEHICLE_POS_BACK:
                    self._accel_cal_waiting_for, self._accel_cal_requested_at = position + 1, None
                else:
                    self._push_inbox(INBOX_ACCEL_CALIBRATION, ACCELCAL_VEHICLE_POS_SUCCESS)

        if self._accel_cal_waiting_for is not None and (
            self._accel_cal_requested_at is None
            or timestamp - self._accel_cal_requested_at >= self.ACCEL_REQUEST_INTERVAL_SECONDS
        ):
            self._accel_cal_requested_at = timestamp
            self._push_inbox(INBOX_ACCEL_CALIBRATION, self._accel_cal_waiting_for)

    def _check_radio_failsafe(self, timestamp: float) -> None:
        """Rover::radio_failsafe_check / failsafe_trigger for the radio: without RC (transmitter off and no fresh RC
        override) the failsafe acts after FS_TIMEOUT - but not in RTL, nor in Hold unless FS_OPTIONS bit 0 is set;
        "Radio Failsafe Cleared" only follows a failsafe that has acted."""
        override_fresh = self._override_sticks is not None and timestamp - self._override_time < RC_OVERRIDE_TIMEOUT
        radio_lost = bool(self.get("FS_THR_ENABLE", 0)) and not self.transmitter_on and not override_fresh
        if not radio_lost:
            if self._radio_failsafe_triggered:
                self._add_status_text("Radio Failsafe Cleared")
            self._radio_lost_since = None
            self._radio_failsafe_triggered = False
            return

        if self._radio_lost_since is None:
            self._radio_lost_since = timestamp
        ignored_here = self.custom_mode == MODES["RTL"] or (
            self.custom_mode == MODES["HOLD"] and not self.get_int("FS_OPTIONS") & FS_OPTIONS_ACTIVE_IN_HOLD
        )

        if (
            not self._radio_failsafe_triggered
            and not ignored_here
            and timestamp - self._radio_lost_since > self.get("FS_TIMEOUT", 0)
        ):
            self._radio_failsafe_triggered = True
            self._add_status_text("Radio Failsafe")
            # ArduPilot clears RC overrides; every action other than "nothing" ends in Hold here (no RTL navigation)
            self._override_sticks = None
            if self.get_int("FS_ACTION") != FS_ACTION_NOTHING:
                self.custom_mode = MODES["HOLD"]

    def _publish_telemetry(self, timestamp: float) -> None:
        """Battery, wheel encoders, IMU, GPS, EKF and heartbeat as ArduPilot would report them."""
        self._advance_accel_calibration(timestamp)
        if self._functions_applied_at is None or timestamp - self._functions_applied_at >= AUX_FUNCTION_REFRESH_SECONDS:
            self._functions_applied_at = timestamp
            self._applied_functions = {output: self.get_int(f"SERVO{output}_FUNCTION") for output in SERVO_OUTPUTS}

        if self.finished_mag_reports and timestamp - self._mag_reports_sent_at >= self.MAG_CAL_REPORT_INTERVAL_SECONDS:
            self._mag_reports_sent_at = timestamp
            # copies: other threads (commands, compass calibration) add entries meanwhile
            for report in list(self.finished_mag_reports.values()):
                self._push_inbox(INBOX_MAG_CAL_REPORT, report)

        self.last_heartbeat_time = timestamp
        if self._motor_test_until is not None and timestamp > self._motor_test_until:
            self._motor_test_until = None
            self._motor_test_power = {}
            # motor_test_stop() disarms
            self.armed = False
            self._servo_pwm = {
                output: pwm
                for output, pwm in list(self._servo_pwm.items())
                if self.get_int(f"SERVO{output}_FUNCTION") in SERVO_FUNCTIONS_DIRECTLY_SETTABLE
            }

        # The voltage reading scales with BATT_VOLT_MULT; it is correct when it equals the true multiplier.
        reported_volts = self.TRUE_BATTERY_VOLTS * self.get("BATT_VOLT_MULT") / self.TRUE_VOLT_MULT
        self._store_message(
            "SYS_STATUS",
            SimpleNamespace(
                voltage_battery=int(reported_volts * SYS_STATUS_MILLIVOLTS_PER_VOLT),
                current_battery=(
                    round(self._battery_amps() * SYS_STATUS_CENTIAMPS_PER_AMP) if self.reports_current else -1
                ),
            ),
            timestamp,
        )

        # Encoders count revolutions; the autopilot turns them into distance with its (possibly wrong) radius.
        def reported_distance(radius):
            """Distance an encoder reports when its configured radius is `radius`."""
            # revolutions = true distance / true radius; reported distance = revolutions x configured radius.
            return self.distance_travelled * radius / self.TRUE_WHEEL_RADIUS

        # Only encoders whose radius parameter exists are reported (boards/firmware may have fewer).
        distances = [
            reported_distance(self.get(name)) for name in ("WENC_RADIUS", "WENC2_RADIUS") if self.has_parameter(name)
        ]
        self._store_message("WHEEL_DISTANCE", SimpleNamespace(count=len(distances), distance=distances), timestamp)
        self._store_message("SCALED_IMU", self._scaled_imu(), timestamp)
        # heading wrapped to -pi..pi, as ArduPilot reports ATTITUDE.yaw
        yaw = math.atan2(math.sin(self.heading), math.cos(self.heading))
        self._store_message(
            "ATTITUDE", SimpleNamespace(roll=0.0, pitch=0.0, yaw=yaw, yawspeed=self.yaw_rate), timestamp
        )
        self._store_message("RAW_IMU", self._raw_imu_compass(), timestamp)
        if any(self.get_int(f"RPM{sensor}_TYPE") for sensor in (1, 2)):
            # A configured sensor reports a steady 1200 RPM; an unconfigured one reports -1 (like ArduPilot).
            rpm1, rpm2 = (1200.0 if self.get_int(f"RPM{sensor}_TYPE") else -1.0 for sensor in (1, 2))
            self._store_message("RPM", SimpleNamespace(rpm1=rpm1, rpm2=rpm2), timestamp)

        if self.true_gps2_type is not None and self.get_int("GPS2_TYPE") == self.true_gps2_type:
            self._store_message("GPS2_RAW", SimpleNamespace(fix_type=3, satellites_visible=10, eph=110), timestamp)

        receiver_answers = self.true_gps_type is None or self.get_int("GPS1_TYPE", 1) == self.true_gps_type
        if receiver_answers:
            self._store_message(
                "GPS_RAW_INT",
                SimpleNamespace(
                    fix_type=self.gps_fix_type,
                    satellites_visible=12,
                    eph=90,
                    # GPS_RAW_INT.lat is in degE7
                    lat=int(self.LATITUDE_DEG * 1e7),
                ),
                timestamp,
            )
        else:
            # no receiver talking -> no GPS message at all
            self._messages.pop("GPS_RAW_INT", None)

        self._store_message(
            "SERVO_OUTPUT_RAW",
            SimpleNamespace(**{f"servo{output}_raw": self._servo_output_pwm(output) for output in SERVO_OUTPUTS}),
            timestamp,
        )
        self._publish_distance_sensors(timestamp)
        self._check_radio_failsafe(timestamp)
        # RELAY_STATUS: bit n-1 = relay n; present = function Relay (AP_Relay::send_relay_status)
        self._store_message(
            "RELAY_STATUS",
            SimpleNamespace(
                present=sum(
                    1 << (relay - 1)
                    for relay in RELAYS
                    if self.get_int(f"RELAY{relay}_FUNCTION") == RELAY_FUNCTION_RELAY
                ),
                on=sum(1 << (relay - 1) for relay, on in list(self.relay_states.items()) if on),
            ),
            timestamp,
        )
        self._store_message(
            "EKF_STATUS_REPORT",
            SimpleNamespace(flags=EKF_ATTITUDE | EKF_VELOCITY_HORIZ | EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS),
            timestamp,
        )

    def _servo_output_pwm(self, output: int) -> int:
        """What an output currently sends: steering follows the steering stick even while disarmed (as in Rover);
        motor tests / DO_SET_SERVO values where active; TRIM for other assigned outputs; 0 for unused ones."""
        function = self.get_int(f"SERVO{output}_FUNCTION")
        if output in self._servo_pwm:
            # An output just switched to a settable function (e.g. "none") is still driven by its old function
            # until the next once-a-second refresh; a function just switched back on doesn't drive it yet either.
            applied = self._applied_functions.get(output, function)
            if function in SERVO_FUNCTIONS_DIRECTLY_SETTABLE and applied not in SERVO_FUNCTIONS_DIRECTLY_SETTABLE:
                return self.get_int(f"SERVO{output}_TRIM", RC_PWM_NOMINAL_TRIM)
            return self._servo_pwm[output]

        if function == SERVO_FUNCTION_GROUND_STEERING and self.custom_mode in (MODES["MANUAL"], MODES["ACRO"]):
            # stick -> MIN..TRIM..MAX
            return self._pwm_for_percent(output, self._current_sticks()[0] * 100)

        rc_channel = (
            output
            if function == SERVO_FUNCTION_RC_PASSTHRU
            # RCIN1..16
            else (
                function - SERVO_FUNCTION_RCIN_BASE
                # last RCIN16
                if SERVO_FUNCTION_RCIN_BASE < function <= SERVO_FUNCTION_RCIN_BASE + 16
                else None
            )
        )
        # the output passes that RC channel's pulse through
        if rc_channel is not None:
            raw = self.rc_raw()
            return raw[rc_channel - 1] if rc_channel <= len(raw) else 0

        # GPIO (-1) / unused: no pulse
        return self.get_int(f"SERVO{output}_TRIM", RC_PWM_NOMINAL_TRIM) if function > 0 else 0

    def _are_arming_checks_skipped(self) -> bool:
        """All arming checks off: ARMING_SKIPCHK skips all (current firmware) or ARMING_CHECK performs none."""
        if self.has_parameter("ARMING_SKIPCHK"):
            return self.get_int("ARMING_SKIPCHK") == ARMING_SKIPCHK_ALL

        return self.get_int("ARMING_CHECK", 1) == ARMING_CHECK_NONE

    def _is_gimbal_wired(self) -> bool:
        """The hidden gimbal's port is set to a gimbal protocol (serial gimbal or MAVLink2) at the gimbal's baud rate."""
        if self.true_gimbal_serial is None:
            return True

        port, baud = self.true_gimbal_serial
        return any(
            self._is_serial_port_configured(port, protocol, baud)
            for protocol in (SERIAL_PROTOCOL_GIMBAL, SERIAL_PROTOCOL_MAVLINK2)
        )

    def _is_serial_port_configured(self, port: int, protocol: int, baud=None) -> bool:
        """Whether a serial port is set to `protocol` (and, if given, to `baud`)."""
        return self.get_int(f"SERIAL{port}_PROTOCOL", -1) == protocol and (
            baud is None or self.get_int(f"SERIAL{port}_BAUD") == baud
        )

    def _publish_distance_sensors(self, timestamp: float) -> None:
        """DISTANCE_SENSOR from the hidden rangefinder / proximity sensor, but only if they are configured correctly."""
        if self.true_rangefinder is not None:
            # (type, port) means instance 1; (instance, type, port) another one
            instance, kind, port = (("1",) + tuple(self.true_rangefinder))[-3:]
            if self.get_int(f"RNGFND{instance}_TYPE") == kind and (
                port == 0
                or self._is_serial_port_configured(port, SERIAL_PROTOCOL_RANGEFINDER, self.true_rangefinder_baud)
            ):
                orientation = self.get_int(f"RNGFND{instance}_ORIENT")
                self._store_distance_sensor(
                    SimpleNamespace(
                        id=RANGEFINDER_INSTANCES.index(instance),
                        orientation=orientation,
                        current_distance=SIMULATED_DISTANCE_CM,
                    ),
                    timestamp,
                )

        if self.true_proximity is not None:
            # (type, port, baud) means instance 1; (instance, type, port, baud) another one
            instance, kind, port, baud = ((1,) + tuple(self.true_proximity))[-4:]
            if self.get_int(f"PRX{instance}_TYPE") == kind and self._is_serial_port_configured(
                port, SERIAL_PROTOCOL_LIDAR360, baud
            ):
                # one reading per 45-degree sector, like ArduPilot's proximity reports
                for sector in range(8):
                    self._store_distance_sensor(
                        SimpleNamespace(
                            id=PROXIMITY_SENSOR_ID_START + sector,
                            orientation=sector,
                            current_distance=SIMULATED_DISTANCE_CM * 2,
                        ),
                        timestamp,
                    )

    def _raw_imu_compass(self) -> SimpleNamespace:
        """RAW_IMU magnetic field of the external compass for the held heading, its true mounting and COMPASS_ORIENT."""
        heading = math.radians(self.held_heading_deg)
        # 1. Earth's field in the vehicle frame (level): north component seen from the heading, down component.
        field_vehicle = (
            self.EARTH_FIELD_NORTH * math.cos(heading),
            -self.EARTH_FIELD_NORTH * math.sin(heading),
            self.EARTH_FIELD_DOWN,
        )
        # 2. Into the compass's own axes: undo its true mounting (inverse of a rotation = its transpose).
        field_sensor = transform(transpose(rotation_matrix_for_name(self.TRUE_COMPASS_ORIENTATION)), field_vehicle)
        # 3. ArduPilot rotates an external compass's readings by COMPASS_ORIENT before reporting them.
        configured = ORIENTATION_NAME_BY_VALUE.get(self.get_int("COMPASS_ORIENT"), "None")
        field = transform(rotation_matrix_for_name(configured), field_sensor)
        # 4. The motor current's own field, minus ArduPilot's compensation (COMPASS_MOTCT 2: COMPASS_MOT x current).
        motor_amps = self._motor_amps()
        compensated = self.get_int("COMPASS_MOTCT") == COMPASS_MOTCT_CURRENT
        compensation = [
            self.get(f"COMPASS_MOT_{axis}", 0.0) * self._battery_amps() if compensated else 0.0 for axis in "XYZ"
        ]
        x, y, z = (
            int(component + interference * motor_amps + correction)
            for component, interference, correction in zip(field, self.TRUE_MOTOR_INTERFERENCE, compensation)
        )
        return SimpleNamespace(xmag=x, ymag=y, zmag=z)

    def _motor_amps(self) -> float:
        """Current of the motors running in a motor test [A]."""
        # each motor: its share of the full-power current
        return (
            # share = percent / 100
            sum(abs(percent) / 100 * self.MOTOR_TEST_FULL_POWER_CA for percent in list(self._motor_test_power.values()))
            / SYS_STATUS_CENTIAMPS_PER_AMP
        )

    def _battery_amps(self) -> float:
        """Battery current [A]: standstill, more when driving, plus motor-test motors."""
        return self.STANDSTILL_CURRENT_CA * (1 + abs(self.speed)) / SYS_STATUS_CENTIAMPS_PER_AMP + self._motor_amps()

    def _scaled_imu(self) -> SimpleNamespace:
        """SCALED_IMU for the current tilt, the true board mounting and the configured AHRS_ORIENTATION."""
        # 1. Gravity in the vehicle frame for how the pilot is holding the vehicle.
        gravity_vehicle = _vehicle_frame_gravity(self.held_pitch_deg)
        # 2. Into the board's own axes: undo the true mounting (inverse of a rotation = its transpose).
        gravity_board = transform(transpose(rotation_matrix_for_name(self.TRUE_BOARD_ORIENTATION)), gravity_vehicle)
        # 3. ArduPilot rotates board readings by the configured AHRS_ORIENTATION before reporting them.
        configured = ORIENTATION_NAME_BY_VALUE.get(self.get_int("AHRS_ORIENTATION"), "None")
        gravity_reported = transform(rotation_matrix_for_name(configured), gravity_board)
        # 4. An accelerometer measures specific force, which at rest is -gravity (a level board reads z = -1 g).
        x, y, z = (int(-component * MILLI_G_PER_G) for component in gravity_reported)
        # 5. Rotation rates [mrad/s]: the rover only turns around its vertical axis.
        return SimpleNamespace(xacc=x, yacc=y, zacc=z, xgyro=0, ygyro=0, zgyro=int(self.yaw_rate * MILLIRAD_PER_RAD))
