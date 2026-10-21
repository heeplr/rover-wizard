"""rover_wizard.steps.gimbal - the "Camera gimbals (mounts)" step."""

from __future__ import annotations

from ..util.errors import VehicleError
from ..rover_defs import SERIAL_PROTOCOL_GIMBAL, SERIAL_PROTOCOL_MAVLINK2
from .util.choices import COUNT_ONE_TWO_CHOICES
from .util.ports import (
    DEFAULT_SERIAL_BAUD,
    baud_label,
    baud_rates_to_try,
    ensure_port_free,
    SERIAL_PORT_CHOICES,
    _serial_snapshot,
    chosen_serial_port,
    configure_serial,
    configure_serial_keeping_baud,
    port_with_protocol,
)
from .util.actions import action_reboot_flight_controller
from .util.rover_hardware import has
from .util.probing import restore, restore_on_failure, snapshot
from ..model import Action, Step, param_field, value_field

MOUNT_TYPE_CHOICES = [
    ("None", 0),
    ("Servo", 1),
    ("SToRM32 MAVLink", 4),
    ("SToRM32 serial", 5),
    ("MAVLink gimbal (Gremsy ...)", 6),
    ("Brushless PWM", 7),
    ("Siyi", 8),
    ("Viewpro", 11),
]

# need SERIALx_PROTOCOL = Gimbal
MOUNT_SERIAL_TYPES = (5, 8, 11)

# need SERIALx_PROTOCOL = MAVLink2
MOUNT_MAVLINK_TYPES = (4, 6)

# [deg] "look down" for the gimbal test
GIMBAL_TEST_PITCH = -30

# 57600: the usual rate of MAVLink telemetry ports, often kept for MAVLink gimbals
MAVLINK_GIMBAL_BAUD = 57

# (MNTx_TYPE, serial protocol, name) of the gimbals the probe tries
GIMBAL_CANDIDATES = [
    (8, SERIAL_PROTOCOL_GIMBAL, "Siyi"),
    (5, SERIAL_PROTOCOL_GIMBAL, "SToRM32 serial"),
    (11, SERIAL_PROTOCOL_GIMBAL, "Viewpro"),
    (6, SERIAL_PROTOCOL_MAVLINK2, "MAVLink gimbal (Gremsy ...)"),
    (4, SERIAL_PROTOCOL_MAVLINK2, "SToRM32 MAVLink"),
]

# the single mount's parameter before firmware 4.3
LEGACY_MOUNT_TYPE = "MNT_TYPE"


def detected_gimbal_count(ctx) -> int:
    """2 if a second mount is configured on the vehicle, else 1."""
    return 2 if ctx.param_int("MNT2_TYPE") else 1


def gimbal_count(ctx) -> int:
    """Number of gimbals chosen in the form (the vehicle's configuration until chosen)."""
    return int(ctx.values.get("gimbal_count") or detected_gimbal_count(ctx))


def selected_gimbal(ctx) -> int:
    """The gimbal (1 or 2) the step's fields currently describe."""
    return min(int(ctx.values.get("gimbal") or 1), gimbal_count(ctx))


def mount_type_names(ctx) -> tuple:
    """Type parameter of the selected gimbal (older firmware has a single MNT_TYPE)."""
    gimbal = selected_gimbal(ctx)
    return (f"MNT{gimbal}_TYPE", LEGACY_MOUNT_TYPE) if gimbal == 1 else (f"MNT{gimbal}_TYPE",)


def mount_limit_names(suffix: str):
    """A limit parameter (e.g. PITCH_MIN) of the selected gimbal."""
    return lambda ctx: f"MNT{selected_gimbal(ctx)}_{suffix}"


def apply_gimbal(ctx):
    """Serial and MAVLink gimbals need their serial port set to the matching protocol."""
    if gimbal_count(ctx) == 1:
        ctx.set_param("MNT2_TYPE", 0, reboot=True, required=False)

    kind = int(ctx.number("gimbal_type", 0))
    protocol = (
        SERIAL_PROTOCOL_GIMBAL
        if kind in MOUNT_SERIAL_TYPES
        else SERIAL_PROTOCOL_MAVLINK2 if kind in MOUNT_MAVLINK_TYPES else None
    )
    if protocol is not None:
        port = chosen_serial_port(ctx, "gimbal_port")
        if not port:
            raise VehicleError("This gimbal type needs the serial port it is connected to.")
        ensure_port_free(ctx, port, protocol)
        configure_serial_keeping_baud(ctx, port, protocol, DEFAULT_SERIAL_BAUD)


def _tilt_and_ask(ctx, question: str) -> bool:
    """Tilt the gimbal down, ask the pilot whether it moved, and level it again."""
    ctx.vehicle.point_gimbal(selected_gimbal(ctx), GIMBAL_TEST_PITCH)
    try:
        return ctx.confirm(question)
    finally:
        ctx.vehicle.point_gimbal(selected_gimbal(ctx), 0)


def action_test_gimbal(ctx):
    """Tilt the gimbal down and back so the pilot can see it responds."""
    moved = _tilt_and_ask(ctx, f"The gimbal was told to tilt {abs(GIMBAL_TEST_PITCH)} deg down. Did it move?")
    ctx.log("Gimbal responds." if moved else "Gimbal did not move - check type, wiring and serial port.")


def _ensure_probe_port_free(ctx, port: int) -> None:
    """The probe switches the port between the gimbal and MAVLink2 protocols: refuse ports other devices use.

    A MAVLink2 port counts as the gimbal's own only if the selected gimbal already is a MAVLink gimbal - otherwise
    it is most likely the telemetry link the wizard may be talking through.
    """
    current = ctx.param_int(f"SERIAL{port}_PROTOCOL", -1)
    own_mavlink_port = (
        current == SERIAL_PROTOCOL_MAVLINK2 and ctx.param_int(mount_type_names(ctx)) in MOUNT_MAVLINK_TYPES
    )
    if current not in (-1, 0, SERIAL_PROTOCOL_GIMBAL) and not own_mavlink_port:
        raise VehicleError(
            f"SERIAL{port} is used for protocol {current} (MAVLink2 = 2 is usually the telemetry link) - "
            "choose the port the gimbal is wired to, or free it on the Serial ports step."
        )


def action_probe_gimbal(ctx):
    """Try the serial and MAVLink gimbal types on the chosen port; the pilot confirms which one moves the gimbal."""
    port = chosen_serial_port(ctx, "gimbal_port")
    if not port:
        raise VehicleError("Choose the serial port the gimbal is connected to first (servo gimbals need no probe).")
    _ensure_probe_port_free(ctx, port)

    # every gimbal type at each baud rate: the port's current one, then 115200 and 57600, then other common ones
    attempts = [
        (kind, protocol, baud, f"{name} at {baud_label(baud)} baud")
        for baud in baud_rates_to_try(ctx, port, preferred=(DEFAULT_SERIAL_BAUD, MAVLINK_GIMBAL_BAUD))
        for kind, protocol, name in GIMBAL_CANDIDATES
    ]
    if not ctx.confirm(f"Start the gimbal probe on SERIAL{port} (up to {len(attempts)} attempts, a reboot each)?"):
        ctx.log("Cancelled - no changes made.")
        return

    mount_type = mount_type_names(ctx)
    originals = {**snapshot(ctx, mount_type), **_serial_snapshot(ctx, port)}
    with restore_on_failure(ctx, originals):
        for kind, protocol, baud, name in attempts:
            ctx.log(f"Trying {name} ...")
            ctx.set_param(mount_type, kind, reboot=True)
            configure_serial(ctx, port, protocol, baud)
            action_reboot_flight_controller(ctx)
            if _tilt_and_ask(ctx, f"{name}: did the gimbal tilt down? (Cancel = no, try the next type / baud rate)"):
                ctx.set_values(gimbal_type=kind)
                ctx.log(f"Gimbal set to {name}.")
                return
        ctx.log("No gimbal type kept - restoring the original settings.")
        restore(ctx, originals)


STEP = Step(
    "gimbal",
    "Camera gimbals (mounts)",
    [
        value_field(
            "gimbal_count",
            "Number of gimbals",
            kind="choice",
            choices=COUNT_ONE_TWO_CHOICES,
            default=lambda ctx: ctx.values.get("gimbal_count") or detected_gimbal_count(ctx),
            reload_on_change=True,
        ),
        value_field(
            "gimbal",
            "Gimbal to configure",
            kind="choice",
            choices=[("Gimbal 1", 1), ("Gimbal 2", 2)],
            default=lambda ctx: ctx.values.get("gimbal") or 1,
            reload_on_change=True,
            when=lambda ctx: gimbal_count(ctx) == 2,
        ),
        param_field(
            mount_type_names,
            "Gimbal type",
            key="gimbal_type",
            kind="choice",
            choices=MOUNT_TYPE_CHOICES,
            reboot=True,
            inline_actions=[Action("Probe", action_probe_gimbal, armed=False, is_probe=True)],
        ),
        value_field(
            "gimbal_port",
            "Connected to serial port",
            kind="choice",
            choices=SERIAL_PORT_CHOICES,
            default=port_with_protocol(SERIAL_PROTOCOL_GIMBAL),
        ),
        param_field(mount_limit_names("PITCH_MIN"), "Pitch limit down", key="gimbal_pitch_min", unit="deg"),
        param_field(mount_limit_names("PITCH_MAX"), "Pitch limit up", key="gimbal_pitch_max", unit="deg"),
        param_field(mount_limit_names("YAW_MIN"), "Yaw limit left", key="gimbal_yaw_min", unit="deg"),
        param_field(mount_limit_names("YAW_MAX"), "Yaw limit right", key="gimbal_yaw_max", unit="deg"),
    ],
    [Action("Test gimbal (tilt down and back)", action_test_gimbal)],
    on_apply=apply_gimbal,
    applies=has("has_gimbal"),
)
