"""rover_wizard.steps.vehicle_type - the "Vehicle type" step."""

from __future__ import annotations

from .util.hardware import OPTIONAL_HARDWARE, has
from .util.outputs import ROVER_OUTPUT_FUNCTIONS
from ..model import Step, param_field, value_field
from ..ardupilot_defs import SERVO_OUTPUTS


def hardware_fields() -> list:
    """One checkbox per optional hardware item, starting from the saved answer or the detected state."""
    return [
        value_field(hardware.key, hardware.label, kind="bool", default=has(hardware.key))
        for hardware in OPTIONAL_HARDWARE
    ]


def apply_hardware(ctx) -> None:
    """Save the answers; for hardware the vehicle doesn't have, switch it off in ArduPilot."""
    for hardware in OPTIONAL_HARDWARE:
        present = bool(ctx.value(hardware.key))
        ctx.store.set(hardware.key, present)
        if present:
            continue
        for names, value in hardware.disable:
            ctx.set_param(names, value, reboot=True, required=False)
    if not ctx.store.get("has_rc"):
        ctx.log(
            "No RC receiver: drive from a ground station or joystick, enable FS_GCS_ENABLE, and make sure "
            "ARMING_CHECK doesn't require RC."
        )


OLDEST_FULLY_SUPPORTED_FIRMWARE = (4, 2)


def action_show_vehicle_info(ctx):
    """Fill the 'Vehicle info' display: firmware, ids, steering type and the rover outputs already assigned."""
    vehicle = ctx.vehicle
    lines = [
        f"Firmware: ArduPilot Rover {vehicle.version_str()}  (MAV_TYPE {vehicle.vehicle_type}, "
        f"sysid {vehicle.system_id}, {vehicle.parameter_count} params)",
        f"Steering type in use: {'SKID (differential)' if ctx.is_skid() else 'FRONT-WHEEL (Ackermann)'}",
    ]
    assigned = []
    for output in SERVO_OUTPUTS:
        function = vehicle.get_int(f"SERVO{output}_FUNCTION")
        if function in ROVER_OUTPUT_FUNCTIONS:
            assigned.append(f"SERVO{output}={ROVER_OUTPUT_FUNCTIONS[function]}")
    lines.append("Motor/steering outputs: " + (", ".join(assigned) or "none configured yet"))
    if vehicle.version and vehicle.version < OLDEST_FULLY_SUPPORTED_FIRMWARE:
        lines.append(
            "WARNING: firmware older than 4.2 - some features may be missing "
            "(the wizard hides unavailable parameters)."
        )
    ctx.set_values(vehicle_info="\n".join(lines))


def apply_vehicle(ctx):
    """Remember whether the vehicle is too heavy to rotate by hand and which optional hardware it has."""
    ctx.store.set("heavy_vehicle", bool(ctx.value("heavy_vehicle")))
    apply_hardware(ctx)


STEP = Step(
    "vehicle_type",
    "Vehicle type",
    [
        value_field("vehicle_info", "Vehicle info", kind="display", default="(querying ...)"),
        param_field("FRAME_CLASS", "Frame class", kind="choice", choices=[("Rover", 1), ("Boat", 2)], reboot=True),
        value_field(
            "heavy_vehicle",
            "Vehicle is too heavy to pick up and rotate",
            kind="bool",
            default=lambda ctx: ctx.store.get("heavy_vehicle", False),
        ),
    ]
    + hardware_fields(),
    [],
    on_apply=apply_vehicle,
    on_enter=action_show_vehicle_info,
)
