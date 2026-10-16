"""rover_wizard.ui.param_screen - every parameter of the vehicle: an alphabetical list to search (Ctrl+P), and a
dialog to edit one raw value, with ArduPilot's description, what the numbers mean and a live reading of the input;
Backup saves every parameter to a file, Restore writes the values of such a file back.

Values are written like the steps write them (MainScreen.run_job -> ctx.set_param): logged, recorded in the change
log (so "Revert ALL wizard changes" covers them) and marking a reboot where ArduPilot needs one.
"""

from __future__ import annotations

import glob
import os
from typing import Optional

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen, Screen
from textual.widgets import Button, DataTable, Footer, Header, Input, Markdown, Select, Static

from ..util.errors import VehicleError
from ..util.values import are_nearly_equal, escape_markup, format_value
from ..vehicle.link import read_param_file
from .markdown import markdown_parser
from .modals import ConfirmScreen
from .param_docs import (
    OFF_VALUES,
    describe,
    display_name,
    is_bitmask,
    is_read_only,
    is_reboot_required,
    meanings,
    reading,
    units,
    validation_error,
    value_range,
)

# columns of the parameter list
COLUMNS = ("Parameter", "Value", "Unit", "R/W", "Description")
# marks a parameter changed in this session
CHANGED_MARK = " *"
# parameter files offered for Restore: the backups in the wizard's data folder
BACKUP_PATTERN = "*.param"
# kept on Restore although writable: a different FORMAT_VERSION makes ArduPilot erase every parameter at the next
# boot; the fence and rally point counts belong to the points stored on the vehicle (written by uploading them)
NOT_RESTORED = ("FORMAT_VERSION", "FENCE_TOTAL", "RALLY_TOTAL")
# parameter names listed in a Restore summary before "..."
NAMES_SHOWN = 8


def _parse(text: str) -> Optional[float]:
    """The number typed in, or None if it isn't one."""
    try:
        return float(text.strip())
    except ValueError:
        return None


def access(name: str) -> str:
    """The R/W column: "R" read-only, "RW" writable (ArduPilot has no write-only parameters)."""
    return "R" if is_read_only(name) else "RW"


def restore_plan(current: dict, saved: dict) -> tuple:
    """What restoring `saved` (name -> value) onto the vehicle's `current` parameters does: (values to write,
    names already equal, differing names kept because they are read-only or NOT_RESTORED, names the vehicle lacks)."""
    writes, equal, kept, missing = {}, [], [], []
    for name, value in sorted(saved.items()):
        if name not in current:
            missing.append(name)
        elif are_nearly_equal(current[name], value):
            equal.append(name)
        elif is_read_only(name) or name in NOT_RESTORED:
            kept.append(name)
        else:
            writes[name] = value

    return writes, equal, kept, missing


def _names(names: list) -> str:
    """The first NAMES_SHOWN names, then "..."."""
    return ", ".join(names[:NAMES_SHOWN]) + (", ..." if len(names) > NAMES_SHOWN else "")


def restore_summary(path: str, plan: tuple) -> str:
    """What Restore will do with the file, for the confirmation and the log."""
    writes, equal, kept, missing = plan
    lines = [f"{os.path.basename(path)}: {len(writes)} value(s) differ from the vehicle, {len(equal)} are equal."]
    if kept:
        lines.append(f"Not restored (read-only or managed by ArduPilot): {_names(kept)}")
    if missing:
        lines.append(
            f"Not on this vehicle: {_names(missing)} - parameters of a feature appear after enabling it and "
            "rebooting; restore again then."
        )
    return "\n".join(lines)


class RestoreScreen(ModalScreen):
    """Choose the parameter file to restore: a backup from the data folder or any path; dismissed with the path or
    None when cancelled."""

    BINDINGS = [("escape", "cancel", "Cancel")]
    DEFAULT_CSS = """
    RestoreScreen { align: center middle; }
    #pr-dialog { width: 90; max-width: 95%; height: auto; border: thick $primary; padding: 0 1; background: $surface; }
    #pr-error { color: $error; height: auto; }
    #pr-buttons { height: auto; margin-top: 1; }
    #pr-buttons Button { margin-right: 2; }
    """

    def __init__(self, folder: str) -> None:
        """The folder holding the backups (newest first)."""
        super().__init__()
        self.backups = sorted(glob.glob(os.path.join(folder, BACKUP_PATTERN)), key=os.path.getmtime, reverse=True)

    def compose(self) -> ComposeResult:
        """The backups, the path field, Restore/Cancel."""
        with Vertical(id="pr-dialog"):
            yield Static("Restore parameters: choose a backup or enter the path of a parameter file.")
            yield Select(
                [(os.path.basename(path), path) for path in self.backups],
                prompt="No backups yet",
                # with backups the newest is selected; without, the prompt shows (Select's "no selection" value
                # is spelled differently across Textual versions: not named here)
                allow_blank=not self.backups,
                disabled=not self.backups,
                id="pr-backups",
            )
            yield Input(self.backups[0] if self.backups else "", placeholder="Path of a .param file", id="pr-path")
            yield Static("", id="pr-error")
            with Horizontal(id="pr-buttons"):
                yield Button("Restore", id="pr-restore", variant="primary")
                yield Button("Cancel", id="pr-cancel")

    def on_select_changed(self, event: Select.Changed) -> None:
        """A backup chosen: its path goes into the field."""
        if isinstance(event.value, str):
            self.query_one("#pr-path", Input).value = str(event.value)

    def on_input_submitted(self, event: Input.Submitted) -> None:
        """Enter in the path field restores."""
        self._choose()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Restore or Cancel."""
        if event.button.id == "pr-restore":
            self._choose()
        else:
            self.dismiss(None)

    def _choose(self) -> None:
        """Close with the path - unless there is no such file (said so)."""
        path = os.path.expanduser(self.query_one("#pr-path", Input).value.strip())
        if not os.path.isfile(path):
            self.query_one("#pr-error", Static).update(f"No such file: {escape_markup(path)}")
            return

        self.dismiss(path)

    def action_cancel(self) -> None:
        """Esc: close without restoring."""
        self.dismiss(None)


class ParameterEditScreen(ModalScreen):
    """Edit one parameter's raw value; dismissed with the new value, or None when cancelled."""

    BINDINGS = [("escape", "cancel", "Cancel")]
    DEFAULT_CSS = """
    ParameterEditScreen { align: center middle; }
    #pe-dialog { width: 90; max-width: 95%; height: auto; max-height: 90%; border: thick $primary; padding: 0 1;
                 background: $surface; }
    #pe-doc { height: auto; max-height: 20; }
    #pe-reading { color: $success; height: auto; }
    #pe-error { color: $error; height: auto; }
    #pe-buttons { height: auto; margin-top: 1; }
    #pe-buttons Button { margin-right: 2; }
    """

    def __init__(self, name: str, value: float) -> None:
        """The parameter to edit and its current value."""
        super().__init__()
        self.parameter = name
        self.value = value

    def _documentation(self) -> str:
        """ArduPilot's description, what the numbers mean, the range and reboot/read-only notes."""
        name = self.parameter
        parts = [describe(name) or f"`{name}` - no description in ArduPilot's documentation."]
        if meanings(name):
            title = "Bits (add the values of all wanted bits):" if is_bitmask(name) else "Values:"
            parts.append(title + "\n\n" + "\n".join(f"- {line}" for line in meanings(name)))

        limits = value_range(name)
        if limits:
            off = f" (or {format_value(OFF_VALUES[name])} = off)" if name in OFF_VALUES else ""
            parts.append(f"Range: {format_value(limits[0])} .. {format_value(limits[1])} {units(name)}".rstrip() + off)

        if is_reboot_required(name):
            parts.append("A change takes effect after a **reboot**.")

        if is_read_only(name):
            parts.append("**Read-only:** ArduPilot reports this value but doesn't accept a new one.")

        return "\n\n".join(parts)

    def compose(self) -> ComposeResult:
        """Documentation, current value, input with live reading, Save/Cancel."""
        with Vertical(id="pe-dialog"):
            with VerticalScroll(id="pe-doc"):
                yield Markdown(self._documentation(), parser_factory=markdown_parser)
            yield Static(f"Current value: {format_value(self.value)}", id="pe-current")
            yield Input(format_value(self.value), id="pe-value", disabled=is_read_only(self.parameter))
            yield Static("", id="pe-reading")
            yield Static("", id="pe-error")
            with Horizontal(id="pe-buttons"):
                yield Button("Save", id="pe-save", variant="primary", disabled=is_read_only(self.parameter))
                yield Button("Cancel", id="pe-cancel")

    def on_mount(self) -> None:
        """Start in the input field, with the reading of the current value."""
        self.query_one("#pe-value", Input).focus()
        self._show_reading(format_value(self.value))

    def _show_reading(self, text: str) -> None:
        """The live reading of the input, and why it can't be saved (if so)."""
        value = _parse(text)
        words = reading(self.parameter, value) if value is not None else ""
        self.query_one("#pe-reading", Static).update(f"= {words}" if words else "")
        self.query_one("#pe-error", Static).update(validation_error(self.parameter, value))

    def on_input_changed(self, event: Input.Changed) -> None:
        """Update the reading while typing."""
        self._show_reading(event.value)

    def on_input_submitted(self, event: Input.Submitted) -> None:
        """Enter in the input field saves."""
        self._save()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Save or Cancel."""
        if event.button.id == "pe-save":
            self._save()
        else:
            self.dismiss(None)

    def _save(self) -> None:
        """Close with the new value - unless it isn't valid (the reason stays shown)."""
        value = _parse(self.query_one("#pe-value", Input).value)
        if validation_error(self.parameter, value):
            self._show_reading(self.query_one("#pe-value", Input).value)
            return

        self.dismiss(value)

    def action_cancel(self) -> None:
        """Esc: close without writing."""
        self.dismiss(None)


class ParameterScreen(Screen):
    """Every parameter, alphabetically; type to filter, Enter or a click to edit, Esc to close.

    Dismissed with True if values were written (so the step behind reloads its values).
    """

    BINDINGS = [Binding("escape", "close", "close")]
    DEFAULT_CSS = """
    #p-search { margin: 0 1; }
    #p-table { height: 1fr; margin: 0 1; }
    #p-buttons { height: auto; margin: 0 1; }
    #p-buttons Button { width: 1fr; }
    #p-status { height: auto; margin: 0 1; color: $text-muted; }
    """

    def __init__(self, main) -> None:
        """The main screen whose vehicle and change log the list works with."""
        super().__init__()
        self.main = main
        self.ctx = main.ctx
        self.written = False
        # a write started here is still running: the list stays open until it is done
        self._writing = False

    def compose(self) -> ComposeResult:
        """Search field, the list, Backup/Restore and a status line."""
        yield Header()
        yield Input(
            placeholder="Search: parameter name or description (Enter or click a row to edit, Esc closes)",
            id="p-search",
        )
        yield DataTable(id="p-table", cursor_type="row", zebra_stripes=True)
        with Horizontal(id="p-buttons"):
            yield Button("💾 Backup", id="p-backup", variant="primary", flat=True)
            yield Button("📂 Restore", id="p-restore", variant="primary", flat=True)
        yield Static("", id="p-status")
        yield Footer()

    def on_mount(self) -> None:
        """Columns, all parameters, cursor in the search field."""
        self.query_one("#p-table", DataTable).add_columns(*COLUMNS)
        self._fill()
        self.query_one("#p-search", Input).focus()

    def _fill(self) -> None:
        """The parameters matching the search, alphabetically; changed ones marked."""
        search = self.query_one("#p-search", Input).value.strip().lower()
        changed = {name for name, _, _ in self.ctx.changes.items()}
        parameters = sorted(self.ctx.vehicle.parameters().items())
        table = self.query_one("#p-table", DataTable)
        table.clear()
        shown = 0
        for name, value in parameters:
            if search and search not in name.lower() and search not in display_name(name).lower():
                continue
            label = name + (CHANGED_MARK if name in changed else "")
            table.add_row(label, format_value(value), units(name), access(name), display_name(name), key=name)
            shown += 1
        self._status(
            f"{shown} of {len(parameters)} parameters" + ("  (* = changed in this session)" if changed else "")
        )

    def _status(self, text: str) -> None:
        """The line under the list."""
        self.query_one("#p-status", Static).update(text)

    def on_input_changed(self, event: Input.Changed) -> None:
        """Filter while typing."""
        self._fill()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        """Enter in the search field: go to the list."""
        self.query_one("#p-table", DataTable).focus()

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        """Enter or a click on a row: edit that parameter."""
        name = event.row_key.value
        self.app.push_screen(
            ParameterEditScreen(name, self.ctx.vehicle.get(name)), callback=lambda value: self._write(name, value)
        )

    def _write(self, name: str, value: Optional[float]) -> None:
        """Write a value from the dialog (None = cancelled), like the steps do."""
        if value is not None:
            reboot = " - takes effect after a reboot" if is_reboot_required(name) else ""
            self._run(
                f"Set {name}",
                lambda: self.ctx.set_param(name, value, reboot=is_reboot_required(name)),
                f"Writing {name} = {format_value(value)} ...",
                f"{name} = {format_value(value)} written{reboot}",
            )

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Backup or Restore."""
        if event.button.id == "p-backup":
            path = self.ctx.store.backup_path()
            self._run(
                "Parameter backup",
                lambda: self._backup(path),
                "Saving the parameters ...",
                f"Saved every parameter to {path}",
                writes=False,
            )
        elif event.button.id == "p-restore":
            self.app.push_screen(RestoreScreen(self.ctx.store.root), callback=self._restore_from)

    def _backup(self, path: str) -> None:
        """Save every parameter to `path` (worker thread)."""
        try:
            self.ctx.vehicle.write_param_file(path)
        except OSError as error:
            raise VehicleError(f"Can't write {path}: {error}") from None
        self.ctx.log(f"Saved full parameter backup: {path}")

    def _restore_from(self, path: Optional[str]) -> None:
        """A file chosen (None = cancelled): show what restoring it does and ask before writing."""
        if path is None:
            return

        try:
            plan = restore_plan(self.ctx.vehicle.parameters(), read_param_file(path))
        except (OSError, UnicodeDecodeError, VehicleError) as error:
            self._status(f"❌ {escape_markup(f'{os.path.basename(path)}: {error}')}")
            return

        summary = restore_summary(path, plan)
        writes = plan[0]
        if not writes:
            self._status(escape_markup(summary) + "\nNothing to write.")
            return

        def answered(yes: bool) -> None:
            """Yes: write the differing values."""
            if yes:
                self._run(
                    f"Restore {os.path.basename(path)}",
                    lambda: self._restore(summary, writes),
                    f"Restoring {len(writes)} value(s) ...",
                    f"{len(writes)} value(s) restored from {os.path.basename(path)}"
                    + (" - reboot to apply them" if any(map(is_reboot_required, writes)) else ""),
                )

        self.app.push_screen(
            ConfirmScreen(escape_markup(f"{summary}\n\nWrite the {len(writes)} differing value(s)?")), answered
        )

    def _restore(self, summary: str, writes: dict) -> None:
        """Write the values of a file that differ from the vehicle's (worker thread)."""
        for line in summary.splitlines():
            self.ctx.log(line)
        for number, (name, value) in enumerate(writes.items(), 1):
            self.main.ui(self._status, f"Restoring {number} / {len(writes)}: {name}")
            self.ctx.set_param(name, value, reboot=is_reboot_required(name))

    def _run(self, label: str, job, running: str, success: str, writes: bool = True) -> None:
        """Run `job` like the steps' actions (logged, one at a time); the status line says how it went."""
        if self.main.busy:
            self._status("⏳ An operation is running - wait for it, then try again.")
            return

        errors = []

        def work() -> None:
            """Run it; a failure is reported here and in the log."""
            try:
                job()
            except VehicleError as error:
                errors.append(str(error))
                self.ctx.log(f"❌ {error}")
            finally:
                self._writing = False

        self._writing = True
        self._status(escape_markup(running))
        self.main.run_job(work, label=label, on_success=lambda: self._done(errors, success, writes))

    def _done(self, errors: list, success: str, writes: bool) -> None:
        """After a job: refresh the list (keeping the search) and say what happened."""
        # even a failed restore may have written some values
        self.written = self.written or writes
        if not self.is_mounted:
            # closed in the moment between the job and this report
            return

        self._fill()
        self._status(f"❌ {escape_markup(errors[0])}" if errors else escape_markup(success))

    def action_close(self) -> None:
        """Esc: back to the wizard - not while a write is running (the wizard must reload its result)."""
        if self._writing:
            self._status("⏳ Writing - a moment, please.")
            return

        self.dismiss(self.written)
