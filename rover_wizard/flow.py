"""rover_wizard.flow - the order in which the pilot may visit steps (independent of any UI).

Rules: steps that don't apply to this vehicle are hidden and skipped; a step can only be opened once every
earlier applicable step is done or skipped; the wizard resumes at the first applicable step not yet finished.
"""

from __future__ import annotations

from .session.store import STEP_DONE, STEP_SKIPPED

STATUS_MARKS = {STEP_DONE: "✔", STEP_SKIPPED: "⏭"}
CURRENT_MARK = "▶"
OPEN_MARK = "·"


class StepFlow:
    """Navigation rules over an ordered list of steps for one wizard context."""

    def __init__(self, steps: list, ctx) -> None:
        """Bind the step list to the context whose store holds the progress."""
        self.steps = steps
        self._ctx = ctx

    def does_apply(self, index: int) -> bool:
        """Whether step `index` is relevant for this vehicle."""
        return self.steps[index].does_apply_to(self._ctx)

    def is_reachable(self, index: int) -> bool:
        """Whether step `index` may be opened: every earlier applicable step is done or skipped."""
        return all(self._ctx.store.is_finished(step.id) for step in self.steps[:index] if step.does_apply_to(self._ctx))

    def first_open_step(self) -> int:
        """Where to resume: the first applicable step not yet finished (0 if everything is finished)."""
        return next(
            (
                index
                for index, step in enumerate(self.steps)
                if step.does_apply_to(self._ctx) and not self._ctx.store.is_finished(step.id)
            ),
            0,
        )

    def neighbour(self, index: int, direction: int) -> int:
        """The next (direction +1) or previous (-1) applicable step; `index` itself if there is none."""
        candidate = index + direction
        while 0 <= candidate < len(self.steps):
            if self.does_apply(candidate):
                return candidate
            candidate += direction
        return index

    def mark(self, index: int, current_index: int) -> str:
        """Sidebar mark for a step: done, skipped, the one being shown, or open."""
        status = self._ctx.store.step_status(self.steps[index].id)
        if status in STATUS_MARKS:
            return STATUS_MARKS[status]
        return CURRENT_MARK if index == current_index else OPEN_MARK
