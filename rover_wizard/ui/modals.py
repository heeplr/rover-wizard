"""rover_wizard.ui.modals - pop-up screens: help text, yes/no confirmation, and the live probe dialog."""

from __future__ import annotations

import threading
import traceback
from collections.abc import Callable

from textual.app import ComposeResult
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import Button, Markdown, RichLog, Static

from ..context import Ctx
from ..util.errors import Cancelled, VehicleError
from ..util.values import escape_markup
from .markdown import markdown_parser

# RichLog lays text out at least this wide; its default (78) cuts lines in narrow windows
LOG_MIN_WIDTH = 20


def run_on_ui_thread(screen, ui_thread: int, function: Callable, *args):
    """Run function(*args) on the UI thread for `screen` - safe from worker threads and from the UI thread itself.

    An update arriving once the app has stopped running (a job finishing while the wizard closes) is dropped:
    the screens' widgets are already being removed.
    """
    try:
        # raises (NoActiveAppError, a RuntimeError) once the app has ended
        app = screen.app

        def run_unless_closing():
            """The update - unless the app is shutting down."""
            return function(*args) if app.is_running else None

        if threading.get_ident() == ui_thread:
            return run_unless_closing()

        return app.call_from_thread(run_unless_closing)

    # the app is shutting down; nothing left to update
    except RuntimeError:
        return None


class HelpScreen(ModalScreen):
    """The ❓ popup: shows one field's or step's help text as scrollable Markdown."""

    BINDINGS = [("escape", "close", "Close")]

    def __init__(self, title: str, body: str) -> None:
        """Remember the help title/body to show once mounted."""
        super().__init__()
        self._title = title
        self._body = body

    def compose(self) -> ComposeResult:
        """Lay out the help title, scrollable body and a Close button."""
        with Vertical(id="help-box"):
            yield Static(f"❓  {self._title}", id="help-title")
            with VerticalScroll(id="help-scroll"):
                yield Markdown(self._body, parser_factory=markdown_parser)

            yield Button("Close (Esc)", id="help-close", variant="primary")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Close button pressed: dismiss the help popup."""
        event.stop()
        self.dismiss()

    def action_close(self) -> None:
        """Escape key pressed: dismiss the help popup."""
        self.dismiss()


class ConfirmScreen(ModalScreen):
    """A Yes/No popup used before anything risky (arming, rebooting, reverting changes).

    "No" has the focus and Escape answers "No": a second Enter after the button that opened it must not confirm.
    """

    AUTO_FOCUS = "#no"
    BINDINGS = [("escape", "answer_no", "No")]

    def __init__(self, text: str) -> None:
        """Remember the question to ask."""
        super().__init__()
        self._text = text

    def compose(self) -> ComposeResult:
        """Lay out the question text and the Yes/No buttons."""
        with Vertical(id="confirm-box"):
            yield Static(self._text)
            with Horizontal(classes="frow"):
                yield Button("Yes", id="yes", variant="error")
                yield Button("No", id="no")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Yes/No pressed: close the popup, telling the caller which one."""
        event.stop()
        self.dismiss(event.button.id == "yes")

    def action_answer_no(self) -> None:
        """Escape: the same as No."""
        self.dismiss(False)


class LeaveStepScreen(ModalScreen):
    """Leaving a step whose form holds values not written yet: Apply them, Discard them, or Stay on the step.
    Dismisses with "apply", "discard" or "stay" (Stay has the focus; Escape = Stay)."""

    AUTO_FOCUS = "#stay"
    BINDINGS = [("escape", "stay", "Stay")]

    def __init__(self, labels: list) -> None:
        """Remember the fields that weren't applied."""
        super().__init__()
        self._labels = labels

    def compose(self) -> ComposeResult:
        """The list of fields and the three buttons."""
        with Vertical(id="confirm-box"):
            yield Static(
                "Not applied yet (Apply writes them to the vehicle):\n"
                + "\n".join(f"  • {escape_markup(label)}" for label in self._labels)
            )

            with Horizontal(classes="frow"):
                yield Button("Apply", id="apply", variant="primary")
                yield Button("Discard", id="discard", variant="error")
                yield Button("Stay", id="stay")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Answer with the button's id."""
        event.stop()
        self.dismiss(event.button.id)

    def action_stay(self) -> None:
        """Escape: stay on the step."""
        self.dismiss("stay")


class ProbeScreen(ModalScreen):
    """Dialog for the auto-probes: runs the probe in the background and shows its log and prompts in-dialog.

    While open, the wizard context's output is redirected here (ctx.redirect_output), and Continue/Cancel map to
    the same context methods the main screen uses. The probe itself asks whether to keep its result, so Close
    only leaves the dialog. The dialog dismisses with no value; the caller's callback runs after that.
    """

    def __init__(self, ctx: Ctx, runner: Callable, title: str) -> None:
        """Remember the probe function to run once the dialog is shown."""
        super().__init__()
        self.ctx = ctx
        self.runner = runner
        self.title_text = title
        self.main_thread = None

    def compose(self) -> ComposeResult:
        """Title, scrolling status log, and Continue / Cancel / Close buttons."""
        with Vertical(id="probe-box"):
            yield Static(f"🔎 {self.title_text}")
            yield RichLog(id="probe-status", wrap=True, markup=True, highlight=False, min_width=LOG_MIN_WIDTH)
            with Horizontal(classes="frow"):
                yield Button("Continue ▶", id="probe-cont", variant="warning", disabled=True)
                yield Button("■ Cancel", id="probe-cancel")
                yield Button("Close", id="probe-close", disabled=True)

    def on_mount(self) -> None:
        """Start the probe in a worker thread."""
        self.main_thread = threading.get_ident()
        self.run_worker(self._run, thread=True)

    def _ui(self, function: Callable, *args) -> None:
        """Run function(*args) on the UI thread (safe from the worker thread, dropped once the app is closing)."""
        run_on_ui_thread(self, self.main_thread, function, *args)

    def _log(self, text: str) -> None:
        """Append one line to the dialog's log."""
        self.query_one("#probe-status", RichLog).write(text)

    def _set_waiting(self, waiting: bool) -> None:
        """Enable Continue only while the probe waits for the pilot."""
        self.query_one("#probe-cont", Button).disabled = not waiting

    def _run(self) -> None:
        """Worker thread: run the probe with output redirected into this dialog, then enable Close."""
        ctx = self.ctx
        ctx.reset_for_new_action()
        with ctx.redirect_output(
            log=lambda text: self._ui(self._log, text), on_wait=lambda waiting: self._ui(self._set_waiting, waiting)
        ):
            try:
                self.runner(ctx)
            except Cancelled:
                ctx.log("⏹ cancelled")
            except VehicleError as exc:
                ctx.log(f"❌ {exc}")
            # unexpected bug: show it instead of silently killing the worker
            except Exception:
                ctx.log("❌ internal error:\n" + traceback.format_exc())
            finally:
                ctx.cleanup()
        self._ui(self._finish)

    def _finish(self) -> None:
        """The probe has ended (applied, declined, failed or cancelled): only Close stays active."""
        self.query_one("#probe-cont", Button).disabled = True
        self.query_one("#probe-cancel", Button).disabled = True
        self.query_one("#probe-close", Button).disabled = False

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Continue / Cancel drive the running probe; Close leaves the dialog."""
        event.stop()
        if event.button.id == "probe-cont":
            self.ctx.press_continue()
        elif event.button.id == "probe-cancel":
            self.ctx.request_cancel()
        elif event.button.id == "probe-close":
            self.dismiss()
