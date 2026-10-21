"""rover_wizard.steps.vehicle_type - the "Vehicle type" step."""

from __future__ import annotations

from .util.rover_hardware import OPTIONAL_HARDWARE, has
from ..rover_defs import FRAME_CLASS_BOAT, FRAME_CLASS_ROVER
from .util.outputs import ROVER_OUTPUT_FUNCTIONS
from .steering_output import steering_kind
from ..model import Step, param_field, value_field
from ..rover_defs import SERVO_OUTPUTS


def _is_offered_in_form(hardware):
    """Condition of a hardware checkbox: it fits the frame class and the boxes chosen in this form (before Apply)."""

    def is_offered(ctx) -> bool:
        """The checkbox is shown."""
        boat = ctx.form_int("FRAME_CLASS") == FRAME_CLASS_BOAT
        return hardware.is_offered(boat, bool(hardware.needs and ctx.values.get(hardware.needs)))

    return is_offered


def hardware_fields() -> list:
    """One checkbox per optional hardware item, starting from the saved answer or the detected state."""
    return [
        value_field(
            hardware.key,
            hardware.label,
            kind="bool",
            default=has(hardware.key),
            when=_is_offered_in_form(hardware),
            # an item another one needs: (un)ticking it shows/hides that one at once
            refresh=any(other.needs == hardware.key for other in OPTIONAL_HARDWARE),
        )
        for hardware in OPTIONAL_HARDWARE
    ]


def apply_hardware(ctx) -> None:
    """Save the answers; switch on what ArduPilot needs switched on, and off what the vehicle doesn't have.

    Items that don't fit this vehicle (e.g. wheel encoders on a boat) are left alone."""
    for hardware in OPTIONAL_HARDWARE:
        if not _is_offered_in_form(hardware)(ctx):
            continue
        present = bool(ctx.value(hardware.key))
        ctx.store.set(hardware.key, present)
        for names, value in hardware.enable if present else hardware.disable:
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
        f"Steering type in use: {steering_kind(ctx)}",
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
        param_field(
            "FRAME_CLASS",
            "Frame class",
            kind="choice",
            choices=[("Rover", FRAME_CLASS_ROVER), ("Boat", FRAME_CLASS_BOAT)],
            reboot=True,
            refresh=True,
        ),
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
