"""rover_wizard.steps.save - the "Save, reboot & finish Phase 1" step."""

from __future__ import annotations

from .util.actions import action_show_changes
from ..model import Action, Step


def action_revert_changes(ctx):
    """After confirmation, restore every parameter the wizard changed to its original value."""
    if not ctx.confirm("Revert EVERY parameter the wizard changed to its original value?"):
        return

    for name, original, _latest in ctx.changes.items():
        ctx.set_param(name, original)
    ctx.changes.forget_all()
    # some of them may only apply after a reboot
    ctx.changes.mark_reboot_needed("reverted parameters")
    ctx.log("All wizard changes reverted.")


STEP = Step(
    "save",
    "Save, reboot & finish Phase 1",
    [],
    [
        Action("Show all changes", action_show_changes),
        Action("Revert ALL wizard changes", action_revert_changes, armed=False),
    ],
)
