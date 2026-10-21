"""rover_wizard.steps.ekf_sources - the "Position sources (EKF)" step."""

from __future__ import annotations

from ..util.errors import VehicleError
from ..rover_defs import RC_OPTION_EKF_SOURCE_SET
from ..mavlink_defs import ACCEPTED_RESULTS
from .util.outputs import RC_SWITCH_CHOICES, assign_rc_switch, rc_channel_with_option
from .util.choices import DISABLED_ENABLED_CHOICES
from .util.rover_hardware import GPS_TYPE_PARAMETERS, has
from ..model import Action, Step, bit_field, heading_field, param_field, value_field

# EK3_SRCn_* values (AP_NavEKF_Source.cpp)
EKF_SOURCE_NONE = 0
EKF_SOURCE_COMPASS = 1
EKF_SOURCE_GPS = 3
# heading: GPS (dual antenna), GPS with compass fallback, and the GSF yaw estimator (GPS velocity + IMU, no compass)
EKF_YAW_GPS = 2
EKF_YAW_GPS_COMPASS_FALLBACK = 3
EKF_YAW_GSF = 8

EKF_POSXY_CHOICES = [
    ("None", EKF_SOURCE_NONE),
    ("GPS", EKF_SOURCE_GPS),
    ("Beacon", 4),
    ("ExternalNav (visual odometry)", 6),
]

EKF_VELXY_CHOICES = [
    ("None", EKF_SOURCE_NONE),
    ("GPS", EKF_SOURCE_GPS),
    ("Beacon", 4),
    ("Optical flow", 5),
    ("ExternalNav (visual odometry)", 6),
    ("Wheel encoders", 7),
]

EKF_POSZ_CHOICES = [
    ("None", EKF_SOURCE_NONE),
    ("Barometer", 1),
    ("Rangefinder", 2),
    ("GPS", EKF_SOURCE_GPS),
    ("Beacon", 4),
    ("ExternalNav (visual odometry)", 6),
]

EKF_VELZ_CHOICES = [
    ("None", EKF_SOURCE_NONE),
    ("GPS", EKF_SOURCE_GPS),
    ("Beacon", 4),
    ("ExternalNav (visual odometry)", 6),
]

EKF_YAW_CHOICES = [
    ("None", EKF_SOURCE_NONE),
    ("Compass", EKF_SOURCE_COMPASS),
    ("GPS (dual antenna / moving baseline)", EKF_YAW_GPS),
    ("GPS, compass as fallback", EKF_YAW_GPS_COMPASS_FALLBACK),
    ("ExternalNav (visual odometry)", 6),
    ("GSF estimator (GPS + IMU, no compass)", EKF_YAW_GSF),
]

# AHRS_EKF_TYPE: EKF3 is the only estimator in standard builds since 4.5 (EKF2 is compiled out, slated for removal)
AHRS_EKF_TYPE_CHOICES = [("EKF3", 3)]

# GPS1_TYPE / GPS2_TYPE values of receivers that measure heading with two antennas
GPS_TYPES_WITH_HEADING = (17, 18, 22, 23, 25, 26)

# EK3_IMU_MASK bits: one EKF instance per IMU (INS_ACCn_ID set = IMU n detected)
IMU_COUNT = 3


EKF_SOURCE_SET_CHOICES = [("Source set 1", 1), ("Source set 2", 2), ("Source set 3", 3)]


def selected_source_set(ctx) -> int:
    """The EKF source set (1..3) the step's fields currently describe."""
    return int(ctx.values.get("ekf_source_set") or 1)


def source_parameter(suffix: str):
    """EK3_SRCn_<suffix> of the selected source set."""
    return lambda ctx: f"EK3_SRC{selected_source_set(ctx)}_{suffix}"


def source_switch_channel(ctx) -> int:
    """RC channel whose switch selects the source set (0 = none)."""
    return rc_channel_with_option(ctx, RC_OPTION_EKF_SOURCE_SET)


def apply_position_sources(ctx):
    """The RC switch that changes between the source sets."""
    assign_rc_switch(ctx, RC_OPTION_EKF_SOURCE_SET, int(ctx.number("ekf_source_rc_channel", 0)))


def action_use_source_set(ctx):
    """Switch the EKF to the selected source set now (apply its settings first)."""
    source_set = selected_source_set(ctx)
    result = ctx.vehicle.set_ekf_source_set(source_set)
    if result not in ACCEPTED_RESULTS:
        raise VehicleError(f"Source set {source_set} refused (result {result}). {ctx.vehicle.last_text()}")
    ctx.log(f"EKF now uses source set {source_set}.")


def has_gps_heading(ctx) -> bool:
    """A receiver that measures heading (dual antenna / moving baseline) is configured."""
    return any(ctx.param_int(names) in GPS_TYPES_WITH_HEADING for names in GPS_TYPE_PARAMETERS)


def proposed_heading_source(ctx):
    """Heading source to propose for source set 1, with the reason - or (None, "") if the current one fits."""
    current = ctx.param_int("EK3_SRC1_YAW", EKF_SOURCE_COMPASS)
    if current != EKF_SOURCE_COMPASS:
        return None, ""
    if has_gps_heading(ctx):
        return EKF_YAW_GPS_COMPASS_FALLBACK, "a GPS that measures heading is configured"
    if not has("has_compass")(ctx) and has("has_gps")(ctx):
        return EKF_YAW_GSF, "the vehicle has no compass"
    return None, ""


def propose_heading_source(ctx):
    """When the step opens: propose a better heading source for set 1 (written only by Apply)."""
    proposal, reason = proposed_heading_source(ctx)
    if proposal is None or selected_source_set(ctx) != 1 or ctx.values.get("src_yaw") != EKF_SOURCE_COMPASS:
        return
    ctx.set_values(src_yaw=proposal)
    label = dict((value, name) for name, value in EKF_YAW_CHOICES)[proposal]
    ctx.log(f"Heading from: proposing '{label}' because {reason} - press Apply to use it.")


def has_position_aid(ctx) -> bool:
    """The vehicle has a sensor besides GPS the EKF can use for position or speed."""
    return any(has(key)(ctx) for key in ("has_wenc", "has_optical_flow", "has_visual_odometry", "has_beacons"))


def imu_bit_field(number: int):
    """EK3_IMU_MASK checkbox for IMU `number`, shown when ArduPilot detected that IMU."""
    id_parameter = "INS_ACC_ID" if number == 1 else f"INS_ACC{number}_ID"
    return bit_field(
        "EK3_IMU_MASK",
        1 << (number - 1),
        f"EKF instance for IMU {number}",
        key=f"ekf_imu{number}",
        when=lambda ctx: bool(ctx.param_int(id_parameter)),
    )


STEP = Step(
    "ekf_sources",
    "Position sources (EKF)",
    [
        value_field(
            "ekf_source_set",
            "Source set to configure",
            kind="choice",
            choices=EKF_SOURCE_SET_CHOICES,
            default=lambda ctx: ctx.values.get("ekf_source_set") or 1,
            reload_on_change=True,
        ),
        param_field(
            source_parameter("POSXY"),
            "Horizontal position from",
            key="src_posxy",
            kind="choice",
            choices=EKF_POSXY_CHOICES,
            reboot=True,
        ),
        param_field(
            source_parameter("VELXY"),
            "Horizontal velocity from",
            key="src_velxy",
            kind="choice",
            choices=EKF_VELXY_CHOICES,
            reboot=True,
        ),
        param_field(
            source_parameter("POSZ"),
            "Vertical position from",
            key="src_posz",
            kind="choice",
            choices=EKF_POSZ_CHOICES,
            reboot=True,
        ),
        param_field(
            source_parameter("VELZ"),
            "Vertical velocity from",
            key="src_velz",
            kind="choice",
            choices=EKF_VELZ_CHOICES,
            reboot=True,
        ),
        param_field(
            source_parameter("YAW"), "Heading from", key="src_yaw", kind="choice", choices=EKF_YAW_CHOICES, reboot=True
        ),
        value_field(
            "ekf_source_rc_channel",
            "Source set chosen by RC channel",
            kind="choice",
            choices=RC_SWITCH_CHOICES,
            default=source_switch_channel,
        ),
        bit_field("EK3_SRC_OPTIONS", 1, "Fuse velocities from all sources", key="src_fuse_all_velocities"),
        heading_field("ekf_estimator", "Estimator"),
        param_field("AHRS_EKF_TYPE", "Estimator used", kind="choice", choices=AHRS_EKF_TYPE_CHOICES, reboot=True),
        param_field("EK3_ENABLE", "EKF3", kind="choice", choices=DISABLED_ENABLED_CHOICES, reboot=True),
    ]
    + [imu_bit_field(number) for number in range(1, IMU_COUNT + 1)]
    + [
        heading_field("ekf_sensors", "Sensor settings", when=has_position_aid),
        param_field("EK3_WENC_VERR", "Wheel encoder speed error", unit="m/s", when=has("has_wenc")),
        param_field(
            "EK3_FLOW_DELAY", "Optical flow delay", kind="int", unit="ms", reboot=True, when=has("has_optical_flow")
        ),
        param_field(
            "EK3_VIS_VERR_MIN", "Visual odometry speed error (min)", unit="m/s", when=has("has_visual_odometry")
        ),
        param_field(
            "EK3_VIS_VERR_MAX", "Visual odometry speed error (max)", unit="m/s", when=has("has_visual_odometry")
        ),
        param_field("EK3_BCN_DELAY", "Beacon delay", kind="int", unit="ms", reboot=True, when=has("has_beacons")),
    ],
    [Action("Use this source set now", action_use_source_set)],
    on_apply=apply_position_sources,
    on_enter=propose_heading_source,
)
