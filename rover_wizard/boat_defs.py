"""rover_wizard.boat_defs - ArduPilot values only boats and sailing vehicles use (sails, wind vane, Torqeedo motors,
depth sounders, loiter), checked against ArduPilot's source; rover_defs has the rest."""

from __future__ import annotations

# ---- Sails (Rover/sailboat.cpp, AR_Motors/AP_MotorsUGV.cpp) ---------------------------------------------------------
# SERVOx_FUNCTION of the sail actuators (SRV_Channel.h): one kind per vehicle
SERVO_FUNCTION_MAINSAIL = 89
SERVO_FUNCTION_WINGSAIL_ELEVATOR = 128
SERVO_FUNCTION_MAST_ROTATION = 137
SAIL_FUNCTIONS = (SERVO_FUNCTION_MAINSAIL, SERVO_FUNCTION_WINGSAIL_ELEVATOR, SERVO_FUNCTION_MAST_ROTATION)
# MAV_CMD_DO_MOTOR_TEST instance that moves the sail actuators (AP_MotorsUGV::MOTOR_TEST_MAINSAIL); the percentage
# is the sail output: a sail winch (range output) 0 = sheeted in .. 100 = let out, a wing sail elevator or mast
# rotation (angle output) -100 .. +100 with 0 = centred
MOTOR_TEST_INSTANCE_MAINSAIL = 5
SAIL_TEST_PERCENT_MAX = 100
# the sail outputs AP_MotorsUGV sets up as an angle (-100..100) instead of a range (0..100)
ANGLE_SAIL_FUNCTIONS = (SERVO_FUNCTION_WINGSAIL_ELEVATOR, SERVO_FUNCTION_MAST_ROTATION)

# RCx_OPTION values for sailing (RC_Channel.h)
RC_OPTION_SAILBOAT_TACK = 63
RC_OPTION_SAILBOAT_MOTOR_3POS = 74
RC_OPTION_WIND_VANE_DIRECTION_OFFSET = 97
RC_OPTION_MAINSAIL = 207

# ---- Wind vane (AP_WindVane) ----------------------------------------------------------------------------------------
# WNDVN_TYPE values
WIND_VANE_TYPE_NONE = 0
WIND_VANE_TYPE_HOME_HEADING = 1
WIND_VANE_TYPE_RC_OFFSET_HOME_HEADING = 2
WIND_VANE_TYPE_ANALOG = 3
WIND_VANE_TYPE_NMEA = 4
# WNDVN_SPEED_TYPE values
WIND_SPEED_TYPE_NONE = 0
WIND_SPEED_TYPE_AIRSPEED = 1
WIND_SPEED_TYPE_MODERN_DEVICES = 2
WIND_SPEED_TYPE_RPM = 3
WIND_SPEED_TYPE_NMEA = 4
# WNDVN_CAL: start a calibration (the flight controller resets it to 0 when done)
WIND_VANE_CALIBRATE_DIRECTION = 1
WIND_VANE_CALIBRATE_SPEED = 2
# [s] AP_WindVane_Analog: a direction calibration records the voltage range for 30 s
WIND_VANE_DIRECTION_CALIBRATION_SECONDS = 30
# SERIALx_PROTOCOL of an NMEA 0183 wind sensor (AP_SerialManager SerialProtocol_WindVane)
SERIAL_PROTOCOL_WIND_VANE = 21
# NAMED_VALUE_FLOAT names of the apparent wind (AP_WindVane::send_wind; WIND carries the true wind)
APPARENT_WIND_SPEED_NAME = "AppWndSpd"
APPARENT_WIND_DIRECTION_NAME = "AppWndDir"

# ---- Torqeedo electric motors (AP_Torqeedo) ------------------------------------------------------------------------
# SERIALx_PROTOCOL of a Torqeedo (SerialProtocol_Torqeedo); the driver always talks at 19200 baud (TORQEEDO_SERIAL_BAUD)
SERIAL_PROTOCOL_TORQEEDO = 39
TORQEEDO_SERIAL_BAUD = 19
# BATTx_MONITOR: the Torqeedo battery or motor reports voltage and current
BATTERY_MONITOR_TORQEEDO = 23
RC_OPTION_TORQEEDO_CLEAR_ERROR = 156

# ---- Loiter (Rover/Parameters.cpp) -----------------------------------------------------------------------------------
# MIS_DONE_BEHAVE: what Auto does after the last mission item; boats default to loitering (Rover::init_ardupilot)
MISSION_DONE_LOITER = 1
