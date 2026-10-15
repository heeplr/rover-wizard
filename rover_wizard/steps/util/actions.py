"""rover_wizard.steps.util.actions - actions offered on several steps: list of changes, reboot."""

from __future__ import annotations

from ...util.errors import VehicleError
from ...util.values import format_value


def action_show_changes(ctx):
    """List every parameter the wizard changed (original -> latest) and whether a reboot is pending."""
    changes = ctx.changes.items()
    if not changes:
        ctx.log("No parameter has been changed by the wizard yet.")

    for name, original, latest in changes:
        ctx.log(f"{name}: {format_value(original)} -> {format_value(latest)}")

    if ctx.changes.reboot_reasons:
        ctx.log("REBOOT NEEDED because of: " + ", ".join(ctx.changes.reboot_reasons))


def action_reboot_flight_controller(ctx):
    """Reboot the flight controller and reconnect, so pending changes take effect."""
    if ctx.vehicle.armed:
        raise VehicleError("Disarm before rebooting.")
    ctx.log("Rebooting the flight controller and reconnecting ...")
    ctx.vehicle.reboot()
    ctx.changes.reboot_done()
    ctx.log(f"Reconnected (firmware {ctx.vehicle.version_str()}, {ctx.vehicle.parameter_count} parameters).")
    ctx.warn_if_parameters_missing()
