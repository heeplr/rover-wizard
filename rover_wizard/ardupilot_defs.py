"""rover_wizard.ardupilot_defs - ArduPilot Rover firmware definitions the wizard uses.

Rover's flight-mode numbers and values of its parameters (SERVOx_FUNCTION, RCx_OPTION, SERIALx_PROTOCOL, ...), each
under a name following ArduPilot's own. tests/test_param_docs.py checks the parameter values against ArduPilot's
parameter metadata, tests/test_protocol_constants.py the flight modes against pymavlink.
"""

from __future__ import annotations

# ---- Rover flight modes (HEARTBEAT.custom_mode) --------------------------------------------------------------------
# ArduPilot Rover flight-mode numbers (HEARTBEAT.custom_mode / MODEx parameters).
MODES = {
    "MANUAL": 0,
    "ACRO": 1,
    "STEERING": 3,
    "HOLD": 4,
    "LOITER": 5,
    "FOLLOW": 6,
    "SIMPLE": 7,
    "DOCK": 8,
    "CIRCLE": 9,
    "AUTO": 10,
    "RTL": 11,
    "SMART_RTL": 12,
    "GUIDED": 15,
}
MODE_NAMES = {v: k for k, v in MODES.items()}


# ---- Servo outputs (SERVOx_FUNCTION) -------------------------------------------------------------------------------
# ArduPilot SERVOx_FUNCTION values (the parameter's own enum, not MAVLink) for the outputs a Rover uses.
SERVO_FUNCTION_GROUND_STEERING = 26
SERVO_FUNCTION_THROTTLE = 70
SERVO_FUNCTION_THROTTLE_LEFT = 73
SERVO_FUNCTION_THROTTLE_RIGHT = 74
# SERVO_OUTPUT_RAW carries servo1_raw .. servo16_raw
SERVO_OUTPUTS_MAX = 16
# SERVO1..SERVO16
SERVO_OUTPUTS = range(1, SERVO_OUTPUTS_MAX + 1)
# RC1..RC16: the channels with RCx_ parameters (RCIN1..RCIN16 as servo functions)
RC_CHANNELS = range(1, 17)
# SERIAL1..SERIAL8 (SERIAL0 is the USB port)
SERIAL_PORTS = range(1, 9)
# RPM1..RPM4
RPM_SENSORS = range(1, 5)
# SERVOx_FUNCTION values ArduPilot lets MAV_CMD_DO_SET_SERVO move (others answer "Channel x is already in use"):
# AP_ServoRelayEvents::do_set_servo - none, RCPassThru, sprayer pump/spinner, gripper, RCIN1-16, RCIN1Scaled-16.
SERVO_FUNCTION_DISABLED = 0
SERVO_FUNCTIONS_DIRECTLY_SETTABLE = frozenset([0, 1, 22, 23, 28] + list(range(51, 67)) + list(range(140, 156)))
# Functions whose code looks up their outputs on every use (AP_Mount_Servo, AP_Camera_Servo), so switching an output
# to "none" and back works without a reboot - SERVOx_FUNCTION's RebootRequired covers functions set up only at boot
# (motors, GPIO, ...). Name for messages.
SERVO_FUNCTIONS_RESOLVED_AT_RUNTIME = {
    6: "mount pan",
    7: "mount tilt",
    8: "mount roll",
    10: "camera trigger",
    12: "mount 2 pan",
    13: "mount 2 tilt",
    14: "mount 2 roll",
}
# [s] Rover re-reads SERVOx_FUNCTION once a second (Rover::one_second_loop -> SRV_Channels::enable_aux_servos)
AUX_FUNCTION_REFRESH_SECONDS = 1.0
# output used as a digital pin (relay) - firmware 4.2 and later
SERVO_FUNCTION_GPIO = -1
# output follows the RC input with the same number
SERVO_FUNCTION_RC_PASSTHRU = 1
# SERVOx_FUNCTION 51..66 = RCIN1..RCIN16: output follows that RC channel
SERVO_FUNCTION_RCIN_BASE = 50
SERVO_FUNCTION_GRIPPER = 28


# ---- RC inputs (pulse range, RCx_OPTION) ---------------------------------------------------------------------------
# RCx_OPTION value that makes an RC switch an input for Lua scripts; ArduPilot's rover-quicktune.lua uses it.
RC_OPTION_SCRIPTING_1 = 300
# Standard hobby-RC pulse range (microseconds), used to scale the live RC bargraph.
RC_PWM_NOMINAL_MIN = 1000
RC_PWM_NOMINAL_MAX = 2000
# RCx_OPTION values that make an RC switch toggle relay 1..6 (ArduPilot RC_Channel parameter values).
RC_OPTION_RELAY = {1: 28, 2: 34, 3: 35, 4: 36, 5: 66, 6: 67}
RC_OPTION_GRIPPER = 19
# switch low / middle / high = source set 1 / 2 / 3
RC_OPTION_EKF_SOURCE_SET = 90
# [us] centre of the nominal RC pulse range
RC_PWM_NOMINAL_TRIM = 1500


# ---- Compass ---------------------------------------------------------------------------------------------------------
# COMPASS_MOTCT: no motor interference compensation / per throttle (Copter only) / per amp of battery current
COMPASS_MOTCT_DISABLED = 0
COMPASS_MOTCT_THROTTLE = 1
COMPASS_MOTCT_CURRENT = 2


# ---- Relays --------------------------------------------------------------------------------------------------------
RELAY_COUNT = 6
# RELAY1..RELAY6
RELAYS = range(1, RELAY_COUNT + 1)

# ---- Failsafes (Rover failsafe.cpp) ---------------------------------------------------------------------------------
# FS_OPTIONS bit 0: failsafes also act (and are reported) in Hold mode - otherwise Hold and RTL ignore them
FS_OPTIONS_ACTIVE_IN_HOLD = 1
# FS_ACTION 0: a failsafe changes no mode
FS_ACTION_NOTHING = 0
# RELAYn_FUNCTION (firmware 4.5+): 1 = a plain relay (switched by DO_SET_RELAY, RC switch, ...)
RELAY_FUNCTION_RELAY = 1
# the pin parameter of relay 1..6 before firmware 4.5 (no RELAYn_FUNCTION yet; pin -1 = not set up)
LEGACY_RELAY_PIN = {1: "RELAY_PIN"} | {relay: f"RELAY_PIN{relay}" for relay in range(2, RELAY_COUNT + 1)}


# ---- Serial ports (SERIALx_PROTOCOL) -------------------------------------------------------------------------------
# SERIALx_PROTOCOL values for peripherals.
SERIAL_PROTOCOL_MAVLINK2 = 2
# SToRM32 serial / Siyi / Viewpro gimbals
SERIAL_PROTOCOL_GIMBAL = 8
SERIAL_PROTOCOL_RANGEFINDER = 9
# "Lidar360": ArduPilot's protocol for proximity sensors such as 360-degree lidars
SERIAL_PROTOCOL_LIDAR360 = 11
SERIAL_PROTOCOL_NONE = -1
SERIAL_PROTOCOL_GPS = 5
SERIAL_PROTOCOL_BEACON = 13
SERIAL_PROTOCOL_GENERATOR = 30


# ---- Motor test instances (Rover's meaning of MAV_CMD_DO_MOTOR_TEST param1) ----------------------------------------
# ArduPilot Rover motor-test "instance" numbers for MAV_CMD_DO_MOTOR_TEST param1 (AP_MotorsUGV motor_test_order).
# "A" in ground-station motor-test pages
MOTOR_TEST_INSTANCE_THROTTLE = 1
# "B"
MOTOR_TEST_INSTANCE_STEERING = 2
# "C"
MOTOR_TEST_INSTANCE_THROTTLE_LEFT = 3
# "D"
MOTOR_TEST_INSTANCE_THROTTLE_RIGHT = 4


# ---- Rangefinders ----------------------------------------------------------------------------------------------------
# Rangefinder instances: RNGFND1..RNGFND9 and RNGFNDA.
RANGEFINDER_INSTANCES = "123456789A"


# ---- Arming checks ---------------------------------------------------------------------------------------------------
# ARMING_SKIPCHK (current firmware) lists the checks to skip, one bit each (AP_Arming::check_enabled): all bits set
# skips every check. Older firmware's ARMING_CHECK lists the checks to perform: 0 performs none.
ARMING_SKIPCHK_ALL = -1
ARMING_CHECK_NONE = 0


# ---- Barometers ------------------------------------------------------------------------------------------------------
# BARO1_DEVID .. BARO3_DEVID: the barometers found at boot, in the order of SCALED_PRESSURE, _PRESSURE2, _PRESSURE3
BARO_INSTANCES = (1, 2, 3)
# A device ID packs (AP_HAL/Device.h, DeviceStructure): bits 0-2 bus type, 3-7 bus number, 8-15 address on the bus,
# 16-23 the driver's device type.
DEVID_BUS_TYPE_BITS = 3
DEVID_BUS_BITS = 5
DEVID_BYTE_BITS = 8
# AP_HAL::Device::BusType
DEVICE_BUS_TYPES = {1: "I2C", 2: "SPI", 3: "DroneCAN", 4: "SITL", 5: "MSP", 6: "serial", 7: "WSPI"}
DEVICE_BUS_TYPE_I2C = 1
DEVICE_BUS_TYPE_SPI = 2
DEVICE_BUS_TYPE_DRONECAN = 3
DEVICE_BUS_TYPE_MSP = 5
# AP_Baro_Backend::DevTypes
BARO_DEVICE_TYPES = {
    0x01: "SITL",
    0x02: "BMP085",
    0x03: "BMP280",
    0x04: "BMP388",
    0x05: "DPS280",
    0x06: "DPS310",
    0x07: "FBM320",
    0x08: "ICM20789",
    0x09: "KellerLD",
    0x0A: "LPS2XH",
    0x0B: "MS5611",
    0x0C: "SPL06",
    0x0D: "DroneCAN",
    0x0E: "MSP",
    0x0F: "ICP101XX",
    0x10: "ICP201XX",
    0x11: "MS5607",
    0x12: "MS5837-30BA",
    0x13: "MS5637",
    0x14: "BMP390",
    0x15: "BMP581",
    0x16: "SPA06",
    0x17: "AUAV",
    0x18: "MS5837-02BA",
}
BARO_DEVICE_TYPE_MS5611 = 0x0B
BARO_DEVICE_TYPE_BMP280 = 0x03
# BARO_PROBE_EXT bits Rover looks for at boot (AP_Baro::_probe_i2c_barometers; Keller and MS5837 only on Sub):
# I2C sensors on the external buses (or the one BARO_EXT_BUS names), MSP from an MSP serial device
BARO_PROBE_EXT_BITS = [
    (1 << 0, "BMP085"),
    (1 << 1, "BMP280"),
    (1 << 2, "MS5611"),
    (1 << 3, "MS5607"),
    (1 << 4, "MS5637"),
    (1 << 5, "FBM320"),
    (1 << 6, "DPS280"),
    (1 << 7, "LPS25H"),
    (1 << 10, "BMP388 / BMP390"),
    (1 << 11, "SPL06 / SPA06"),
    (1 << 12, "MSP (MSP serial device)"),
    (1 << 13, "BMP581"),
    (1 << 14, "AUAV"),
]
BARO_PROBE_EXT_BMP280 = 1 << 1
# BARO_EXT_BUS = -1: search every external I2C bus
BARO_EXT_BUS_ALL = -1
