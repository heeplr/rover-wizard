"""rover_wizard.steps.final - the "Review & finish" step."""

from __future__ import annotations

from .util.actions import action_show_changes
from ..model import Action, Step

STEP = Step(
    "final",
    "Review & finish",
    [],
    [
        Action("Show all changes", action_show_changes),
    ],
)
