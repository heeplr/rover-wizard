"""rover_wizard.session.changes - remembers what the wizard changed on the vehicle and whether a reboot is pending.

Used for "Show all changes", "Revert ALL wizard changes" and the "reboot pending" status. The original value of
a parameter is the one from before the wizard *first* touched it, also across sessions (persisted in the Store).
"""

from __future__ import annotations

from .store import Store


class ChangeTracker:
    """Parameter change log (original and latest value) plus the reasons a reboot is still needed."""

    def __init__(self, store: Store) -> None:
        """Load the change log saved by earlier sessions for this vehicle."""
        self._store = store
        self._changes: dict = {name: list(pair) for name, pair in store.get("changes", {}).items()}
        self._reboot_reasons: set = set()

    def record(self, name: str, old_value: float, new_value: float) -> None:
        """Remember a parameter write; the first recorded old value is kept as the original."""
        if name in self._changes:
            self._changes[name][1] = new_value
        else:
            self._changes[name] = [old_value, new_value]

        self._store.set("changes", self._changes)

    def items(self) -> list:
        """[(name, original value, latest value)] sorted by parameter name."""
        return [(name, pair[0], pair[1]) for name, pair in sorted(self._changes.items())]

    def forget_all(self) -> None:
        """Clear the change log (after everything was reverted)."""
        self._changes.clear()
        self._store.set("changes", self._changes)

    def mark_reboot_needed(self, reason: str) -> None:
        """Note that something only takes effect after a reboot (a parameter name or a calibration)."""
        self._reboot_reasons.add(reason)

    def reboot_done(self) -> None:
        """The flight controller was rebooted - nothing is pending any more."""
        self._reboot_reasons.clear()

    @property
    def reboot_reasons(self) -> list:
        """Sorted reasons a reboot is still needed (empty = none)."""
        return sorted(self._reboot_reasons)
