"""rover_wizard.steps.log_settings - the "Logging" step."""

from __future__ import annotations

from .util.choices import DISABLED_ENABLED_CHOICES
from ..model import Step, bit_field, param_field

# Values from AP_Logger and Rover's LOG_BITMASK definition.
LOG_BACKEND_BITS = [(1, "Log to SD card file"), (2, "Stream log over MAVLink"), (4, "Log to onboard flash chip")]

LOG_BITMASK_BITS = [
    (1, "Fast attitude"),
    (2, "Medium attitude"),
    (4, "GPS"),
    (8, "System performance"),
    (16, "Throttle"),
    (32, "Navigation tuning"),
    (128, "IMU"),
    (256, "Mission commands"),
    (512, "Battery monitor"),
    (1024, "Rangefinder"),
    (2048, "Compass"),
    (4096, "Camera"),
    (8192, "Steering"),
    (16384, "RC input/output"),
    (524288, "Raw IMU"),
    (1048576, "Video stabilization"),
    (2097152, "Optical flow"),
]

LOG_DISARMED_CHOICES = [("Only while armed", 0), ("Always", 1), ("Always, except on USB power", 2)]


def logging_fields() -> list:
    """Where to log, what to log, when to log."""
    fields = [bit_field("LOG_BACKEND_TYPE", bit, label, key=f"log_backend_{bit}") for bit, label in LOG_BACKEND_BITS]
    fields += [bit_field("LOG_BITMASK", bit, f"Log: {label}", key=f"log_bit_{bit}") for bit, label in LOG_BITMASK_BITS]
    fields += [
        param_field("LOG_DISARMED", "Log while disarmed", kind="choice", choices=LOG_DISARMED_CHOICES),
        param_field(
            "LOG_FILE_DSRMROT", "New log file after each disarm", kind="choice", choices=DISABLED_ENABLED_CHOICES
        ),
        param_field("LOG_DARM_RATEMAX", "Maximum log rate while disarmed", unit="Hz"),
        param_field("LOG_FILE_BUFSIZE", "Log file buffer size", kind="int", unit="kB"),
        param_field("LOG_FILE_MB_FREE", "Keep this much space free on the card", kind="int", unit="MB"),
    ]
    return fields


STEP = Step("log_settings", "Logging", logging_fields(), [])
