"""rover_wizard.session.activity_log - the activity log's lines as plain text, to save them to a file."""

from __future__ import annotations

from rich.errors import MarkupError
from rich.text import Text


class ActivityLog:
    """Every line written to the activity log, without its markup (colours, bold ...)."""

    def __init__(self) -> None:
        """An empty log."""
        self.lines: list[str] = []

    def add(self, text: str) -> None:
        """Note one line as shown (Rich markup) - stored as plain text."""
        try:
            plain = Text.from_markup(text).plain
        except MarkupError:
            # not valid markup after all: keep it as it is
            plain = text
        self.lines.append(plain)

    def save(self, path: str) -> None:
        """Write all lines so far to a text file (UTF-8)."""
        with open(path, "w", encoding="utf-8") as handle:
            handle.writelines(line + "\n" for line in self.lines)
