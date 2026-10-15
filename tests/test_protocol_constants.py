"""Every protocol number in rover_wizard.mavlink_defs (and Rover's flight modes in rover_wizard.ardupilot_defs)
against pymavlink's official definitions."""

import pytest

pytest.importorskip("pymavlink")

from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as mavlink  # noqa: E402

import rover_wizard.ardupilot_defs as ardupilot_defs  # noqa: E402
import rover_wizard.mavlink_defs as mavlink_defs  # noqa: E402

CONSTANTS = {name: getattr(mavlink_defs, name) for name in dir(mavlink_defs) if name.isupper()}
# Names in mavlink_defs that pymavlink doesn't define: derived values, units of message fields and parameter
# conventions of commands (all documented in MAVLink's message/command descriptions, not as enum entries).
NOT_IN_PYMAVLINK = {
    "GUIDED_VELOCITY_YAW_RATE_TYPEMASK",
    "GPS_FIX_TYPE_NAMES",
    "COMMAND_LONG_PARAMS",
    "ARM_DISARM_FORCE_MAGIC_NUMBER",
    "PREFLIGHT_CALIBRATION_ACCEL",
    "PREFLIGHT_CALIBRATION_GYRO",
    "PREFLIGHT_CALIBRATION_LEVEL",
    "GPS_RAW_INT_EPH_SCALE",
    "SYS_STATUS_CENTIAMPS_PER_AMP",
    "SYS_STATUS_MILLIVOLTS_PER_VOLT",
    "MILLI_G_PER_G",
    "MILLIRAD_PER_RAD",
    "PARAM_INDEX_UNKNOWN",
    "RC_CHANNELS_MAX",
    "UINT16_MAX",
    "ACCEPTED_RESULTS",
}


@pytest.mark.parametrize("name", sorted(n for n in CONSTANTS if hasattr(mavlink, n)))
def test_value_matches_mavlink(name):
    assert CONSTANTS[name] == getattr(mavlink, name)


def test_every_mavlink_name_is_known():
    # Anything else pymavlink doesn't know is a typo (its value was never checked) or not MAVLink at all.
    unknown = {name for name in CONSTANTS if not hasattr(mavlink, name)}
    assert unknown <= NOT_IN_PYMAVLINK, sorted(unknown - NOT_IN_PYMAVLINK)


def test_rover_flight_modes_match_ardupilot():
    for name, number in ardupilot_defs.MODES.items():
        # pymavlink maps number -> name
        assert mavutil.mode_mapping_rover[number] == name


def test_guided_typemask_ignores_all_but_velocity_and_yaw_rate():
    ignored = {
        getattr(mavlink, f"POSITION_TARGET_TYPEMASK_{axis}_IGNORE")
        for axis in ("X", "Y", "Z", "VZ", "AX", "AY", "AZ", "YAW")
    }
    assert mavlink_defs.GUIDED_VELOCITY_YAW_RATE_TYPEMASK == sum(ignored)
    used = {getattr(mavlink, f"POSITION_TARGET_TYPEMASK_{axis}_IGNORE") for axis in ("VX", "VY", "YAW_RATE")}
    assert not mavlink_defs.GUIDED_VELOCITY_YAW_RATE_TYPEMASK & sum(used)
