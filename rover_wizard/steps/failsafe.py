"""rover_wizard.steps.failsafe - the "Failsafes & arming" step."""

from __future__ import annotations

from .util.rover_hardware import has
from ..model import Step, bit_field, heading_field, param_field

FAILSAFE_ACTION_CHOICES = [
    ("Nothing", 0),
    ("RTL", 1),
    ("Hold", 2),
    ("SmartRTL or RTL", 3),
    ("SmartRTL or Hold", 4),
    ("Terminate", 5),
]

# Arming checks: the same bit positions in ARMING_CHECK and ARMING_SKIPCHK (AP_Arming.cpp).
ARMING_CHECK_BITS = [
    (2, "Barometer"),
    (4, "Compass"),
    (8, "GPS lock"),
    (16, "INS (accelerometers & gyros)"),
    (32, "Parameters"),
    (64, "RC channels"),
    (128, "Board voltage"),
    (256, "Battery level"),
    (1024, "Logging available"),
    (2048, "Hardware safety switch"),
    (4096, "GPS configuration"),
    (8192, "System"),
    (16384, "Mission"),
    (32768, "Rangefinder"),
    (65536, "Camera"),
    (131072, "AuxAuth"),
    (262144, "Visual odometry"),
    (524288, "FFT"),
]

# FENCE_ACTION on Rover (AC_Fence.cpp, @Values{Rover})
FENCE_ACTION_CHOICES = [
    ("Report only", 0),
    ("RTL or Hold", 1),
    ("Hold", 2),
    ("SmartRTL or RTL or Hold", 3),
    ("SmartRTL or Hold", 4),
    ("Terminate", 5),
    ("Loiter or Hold", 6),
]
# FENCE_TYPE bits on Rover
FENCE_TYPE_CIRCLE = 2
FENCE_TYPE_ZONES = 4

# ARMING_CHECK bit 0 = all checks (older firmware)
ARMING_CHECK_ALL = 1


def _is_fence_enabled(ctx) -> bool:
    """Whether the fence is switched on in the form."""
    return ctx.number("FENCE_ENABLE", 0) == 1


def fence_fields() -> list:
    """Geofence: a circle around home (and/or uploaded zones) that switches to the chosen mode when crossed."""
    return [
        heading_field("fence_heading", "Geofence", when=lambda ctx: ctx.vehicle.resolve("FENCE_ENABLE") is not None),
        param_field("FENCE_ENABLE", "Fence", kind="choice", choices=[("Off", 0), ("On", 1)], refresh=True),
        bit_field("FENCE_TYPE", FENCE_TYPE_CIRCLE, "Circle around home", key="fence_circle", when=_is_fence_enabled),
        bit_field(
            "FENCE_TYPE",
            FENCE_TYPE_ZONES,
            "Uploaded zones (inclusion / exclusion)",
            key="fence_zones",
            when=_is_fence_enabled,
        ),
        param_field("FENCE_RADIUS", "Circle radius", unit="m", min_value=30, when=_is_fence_enabled),
        param_field(
            "FENCE_ACTION", "When crossed", kind="choice", choices=FENCE_ACTION_CHOICES, when=_is_fence_enabled
        ),
    ]


def arming_check_fields() -> list:
    """A heading, then one 'performed' checkbox per arming check, for whichever of the two parameters the firmware
    has (help texts: wizard_text.json, failsafe.arm_heading and failsafe.arm_skip_<bit> / arm_check_<bit>)."""
    fields = [
        heading_field(
            "arm_heading",
            "Arming checks",
            when=lambda ctx: ctx.vehicle.resolve(("ARMING_SKIPCHK", "ARMING_CHECK")) is not None,
        )
    ]
    for bit, label in ARMING_CHECK_BITS:
        fields.append(bit_field("ARMING_SKIPCHK", bit, label, key=f"arm_skip_{bit}", bit_skips=True))
        fields.append(bit_field("ARMING_CHECK", bit, label, key=f"arm_check_{bit}", bit_all=ARMING_CHECK_ALL))
    return fields


STEP = Step(
    "failsafe",
    "Failsafes & arming",
    [
        param_field("FS_ACTION", "Failsafe action", kind="choice", choices=FAILSAFE_ACTION_CHOICES),
        param_field(("FS_GCS_TIMEOUT", "FS_TIMEOUT"), "Failsafe timeout", unit="s"),
        param_field(
            "FS_THR_ENABLE",
            "Throttle (radio) failsafe",
            when=has("has_rc"),
            kind="choice",
            choices=[("Disabled", 0), ("Enabled", 1), ("Enabled, continue mission in Auto", 2)],
        ),
        param_field("FS_THR_VALUE", "Throttle failsafe PWM", kind="int", unit="us", when=has("has_rc")),
        param_field(
            "FS_GCS_ENABLE",
            "GCS failsafe",
            kind="choice",
            choices=[("Disabled", 0), ("Enabled", 1), ("Enabled, always", 2)],
        ),
        param_field(
            "FS_CRASH_CHECK",
            "Crash check",
            kind="choice",
            choices=[("Disabled", 0), ("Hold", 1), ("Hold and disarm", 2)],
        ),
        param_field(
            "FS_EKF_ACTION", "EKF failsafe", kind="choice", choices=[("Disabled", 0), ("Hold", 1), ("Report only", 2)]
        ),
        param_field("FS_EKF_THRESH", "EKF failsafe threshold"),
    ]
    + fence_fields()
    + arming_check_fields(),
    [],
)
