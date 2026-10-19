"""rover_wizard.ui.main_screen - the wizard screen: step sidebar, step form, control buttons and activity log.

The screen renders a step (fields, inline and normal action buttons, help buttons), runs actions as background
jobs (one at a time), and shows vehicle status. Navigation rules come from StepFlow; everything that talks to
the vehicle happens through the wizard context (Ctx) in worker threads.
"""

from __future__ import annotations

import threading
import traceback
from typing import Optional
from collections.abc import Callable

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import Screen
from textual.widgets import Button, Checkbox, Footer, Header, Input, Markdown, RichLog, Select, Static

from ..context import Ctx
from ..util.errors import Cancelled, VehicleError
from ..flow import StepFlow
from ..ardupilot_defs import MODE_NAMES, RELAYS, SERVO_OUTPUTS
from ..mavlink_defs import GPS_FIX_TYPE_NAMES, SYS_STATUS_MILLIVOLTS_PER_VOLT
from ..model import Action, Field, parameter_names_text
from ..steps.util.actions import action_reboot_flight_controller
from ..steps.util.hardware import has, is_accelerometer_fitted, is_gyro_fitted
from ..steps.barometer import barometer_in_use
from ..steps.relays import toggle_relay
from .step_catalog import STEPS
from ..session.store import STEP_DONE, STEP_SKIPPED
from ..util.values import escape_markup, format_value
from .param_docs import describe
from .markdown import markdown_parser
from .modals import LOG_MIN_WIDTH, ConfirmScreen, HelpScreen, LeaveStepScreen, ProbeScreen, run_on_ui_thread
from .param_screen import ParameterScreen
from .monitor import (
    accel_bars_text,
    barometer_text,
    compass_tape_text,
    gyro_bars_text,
    number_row,
    rc_bar_text,
    servo_bar_text,
)

STATUS_REFRESH_SECONDS = 0.7
BUSY_MESSAGE = "⏳ An operation is running - wait for it or press Cancel."


class MainScreen(Screen):
    """Sidebar of steps, the current step's form, control buttons and the activity log."""

    # shown in the footer as "^p parameters"
    BINDINGS = [Binding("ctrl+p", "parameters", "parameters")]

    def __init__(self, ctx: Ctx) -> None:
        """Bookkeeping only; the step content is built by show_step() once mounted."""
        super().__init__()
        self.ctx = ctx
        self.flow = StepFlow(STEPS, ctx)
        self.step_index = 0
        # a background job (or probe dialog) is running
        self.busy = False
        self.main_thread = None
        # makes widget ids unique across re-renders of the step box
        self._render_generation = 0
        # show_step() calls still mounting their widgets
        self._steps_being_shown = 0
        # field key -> input widget
        self.widgets: dict = {}
        # widget id -> Field
        self.fields: dict = {}
        # field key -> Field
        self.by_key: dict = {}
        # help button id -> (title, markdown)
        self.help: dict = {}
        # action button id -> Action
        self.actions: dict = {}
        # field key -> values offered by its Select
        self._select_options: dict = {}

    @property
    def focus_chain(self) -> list:
        """Tab order: every field and button first, the ❓ help buttons last (layout stays per row)."""
        chain = list(super().focus_chain)
        help_widgets = [widget for widget in chain if (widget.id or "").startswith("help-")]
        other_widgets = [widget for widget in chain if not (widget.id or "").startswith("help-")]
        return other_widgets + help_widgets

    # ================================================================== layout
    def compose(self) -> ComposeResult:
        """Sidebar (phase headings + one button per step) and the main area."""
        yield Header()
        with Horizontal(id="body"):
            with Vertical(id="left"):
                with VerticalScroll(id="sidebar"):
                    previous_phase = None
                    for index, step in enumerate(STEPS):
                        if step.phase != previous_phase:
                            previous_phase = step.phase
                            yield Static(step.phase.heading, classes="phase")
                        yield Button(
                            self._nav_label(index),
                            id=f"nav-{index}",
                            classes="nav",
                            disabled=not self.flow.is_reachable(index),
                        )
                yield from self._compose_monitor()
            with Vertical(id="main"):
                yield Static("", id="status")
                yield VerticalScroll(id="stepbox")
                with Horizontal(id="nav"):
                    yield Button("◀ Back", id="b-back")
                    yield Button("💾 Apply", id="b-apply", variant="primary")
                    yield Button("Next ▶", id="b-next", variant="success")
                    yield Button("Skip", id="b-skip")
                with Horizontal(id="ctl"):
                    yield Button("Continue ▶", id="b-cont", variant="warning", disabled=True)
                    yield Button("■ Cancel", id="b-cancel")
                    yield Button("Arm", id="b-arm")
                    yield Button("Disarm", id="b-disarm")
                    yield Button("♻ Reboot FC", id="b-reboot")
                    yield Button("🛑 STOP", id="b-stop", variant="error")
                yield RichLog(id="log", wrap=True, markup=True, highlight=False, min_width=LOG_MIN_WIDTH)
        yield Footer()

    def _compose_monitor(self) -> ComposeResult:
        """Live panel under the step list: RC channel bars; per servo output its number, +, - and bar; - when the
        sensor is present - a compass tape, accelerometer / gyro bars and the barometer's reading; and a toggle button
        per relay."""
        with Vertical(id="monitor"):
            yield Static("RC channels", classes="panel-title")
            yield Static("", id="rc-bars")
            yield Static("Servo output", classes="panel-title")
            yield Static("", id="servo-numbers")
            with Horizontal(id="servo-plus"):
                for output in SERVO_OUTPUTS:
                    yield Button("+", id=f"servo-plus-{output}", flat=True, classes="servo-btn-plus")
            with Horizontal(id="servo-minus"):
                for output in SERVO_OUTPUTS:
                    yield Button("-", id=f"servo-minus-{output}", flat=True, classes="servo-btn-minus")
            yield Static("", id="servo-bars")
            yield Static("Compass", classes="panel-title", id="compass-title")
            yield Static("", id="compass-monitor")
            # Accel and Gyro side by side, to save rows
            with Horizontal(id="imu-monitors"):
                with Vertical(classes="monitor-column"):
                    yield Static("Accel", classes="panel-title", id="accel-title")
                    yield Static("", id="accel-monitor")
                with Vertical(classes="monitor-column"):
                    yield Static("Gyro", classes="panel-title", id="gyro-title")
                    yield Static("", id="gyro-monitor")
                # one button per relay set up on the vehicle: its number, on red (off) or green (on)
                with Vertical(classes="monitor-column", id="relay-column"):
                    yield Static("Relays", classes="panel-title", id="relay-title")
                    with Horizontal(id="relay-buttons"):
                        for relay in RELAYS:
                            yield Button(str(relay), id=f"relay-{relay}", flat=True, classes="relay-btn")
            yield Static("Barometer", classes="panel-title", id="baro-title")
            yield Static("", id="baro-monitor")

    async def on_mount(self) -> None:
        """Connect the context's output to this screen and open the first unfinished step."""
        self.main_thread = threading.get_ident()
        self.ctx.set_output(
            log=lambda text: self.ui(self._write_log, text),
            on_wait=lambda waiting: self.ui(self._set_waiting, waiting),
            on_values_changed=lambda: self.ui(self._refresh_fields),
        )
        self.set_interval(STATUS_REFRESH_SECONDS, self._tick)
        self._write_log("Welcome to the rover-wizard. Click ❓ for help on any step or field.")
        self.ctx.warn_if_parameters_missing()
        await self.show_step(self.flow.first_open_step())

    # ================================================================== small helpers
    def ui(self, function: Callable, *args):
        """Run function(*args) on the UI thread (see run_on_ui_thread)."""
        return run_on_ui_thread(self, self.main_thread, function, *args)

    def _write_log(self, text: str) -> None:
        """Append one line to the activity log."""
        self.query_one("#log", RichLog).write(text)

    def _set_waiting(self, waiting: bool) -> None:
        """Enable Continue only while an action waits for the pilot."""
        self.query_one("#b-cont", Button).disabled = not waiting

    def _nav_label(self, index: int) -> str:
        """Sidebar text: status mark + step title."""
        return f"{self.flow.mark(index, self.step_index)} {STEPS[index].title}"

    def _refresh_nav(self) -> None:
        """Update sidebar marks, hide steps that don't apply, disable steps that aren't reachable yet."""
        for index in range(len(STEPS)):
            button = self.query_one(f"#nav-{index}", Button)
            button.display = self.flow.does_apply(index)
            if button.display:
                button.label = self._nav_label(index)
                button.disabled = not self.flow.is_reachable(index)

    def _tick(self) -> None:
        """Periodic refresh: new flight-controller messages, the status bar and the side panel."""
        ctx = self.ctx
        vehicle = self.ctx.vehicle
        # new flight-controller messages into the log
        ctx.flush_texts()
        status = [
            vehicle.link_status(),
            f"FW {vehicle.version_str()}",
            f"sysid {vehicle.system_id}",
            "[red]ARMED[/]" if vehicle.armed else "disarmed",
            f"mode {MODE_NAMES.get(vehicle.custom_mode, vehicle.custom_mode)}",
            f"{'SKID' if ctx.is_skid() else 'FRONT-WHEEL'} steering",
        ]
        gps = vehicle.latest("GPS_RAW_INT")
        if gps is not None:
            status.append(f"GPS {GPS_FIX_TYPE_NAMES.get(gps.fix_type, gps.fix_type)}, {gps.satellites_visible} sats")
        sys_status = vehicle.latest("SYS_STATUS")
        if sys_status is not None:
            status.append(f"{sys_status.voltage_battery / SYS_STATUS_MILLIVOLTS_PER_VOLT:.1f} V")
        if ctx.changes.reboot_reasons:
            status.append("❗ reboot pending")
        self.query_one("#status", Static).update(" | ".join(status))
        # update sidebar monitors for RC channels and servo outputs
        self._update_monitor()

    def _update_monitor(self) -> None:
        """Refresh the side panel: RC bars for every received channel; servo numbers, buttons and bars; compass,
        accelerometer and gyro sections for the sensors the vehicle has."""
        vehicle = self.ctx.vehicle
        self.query_one("#rc-bars", Static).update(rc_bar_text(vehicle.rc_raw()))
        count = self.ctx.servos.output_count()
        self.query_one("#servo-numbers", Static).update(number_row(count))
        for output in SERVO_OUTPUTS:
            for direction in ("plus", "minus"):
                self.query_one(f"#servo-{direction}-{output}", Button).display = output <= count
        self.query_one("#servo-bars", Static).update(servo_bar_text(vehicle.servo_raw(), count))
        imu = vehicle.imu()
        self._show_monitor_section(
            "compass", has("has_compass")(self.ctx), lambda: compass_tape_text(vehicle.heading_degrees())
        )
        self._show_monitor_section("accel", is_accelerometer_fitted(self.ctx), lambda: accel_bars_text(imu and imu[0]))
        self._show_monitor_section("gyro", is_gyro_fitted(self.ctx), lambda: gyro_bars_text(imu and imu[1]))
        barometer = barometer_in_use(self.ctx)
        self._show_monitor_section(
            "baro",
            barometer is not None,
            lambda: barometer_text(vehicle.pressure(barometer), vehicle.barometers_healthy()),
        )
        self._update_relay_buttons(vehicle.relays())

    def _update_relay_buttons(self, relays: dict) -> None:
        """Show a button for each relay set up on the vehicle (green while on, red while off); no relays: no column."""
        self.query_one("#relay-column").display = bool(relays)
        for relay in RELAYS:
            button = self.query_one(f"#relay-{relay}", Button)
            button.display = relay in relays
            button.set_class(relays.get(relay, False), "relay-on")

    def _show_monitor_section(self, name: str, present: bool, text) -> None:
        """Show a side-panel section (title and content) only for a sensor the vehicle has; `text` makes its content."""
        self.query_one(f"#{name}-title", Static).display = present
        content = self.query_one(f"#{name}-monitor", Static)
        content.display = present
        # a section in a column of its own (Accel, Gyro) hides the whole column, so the other moves up to the left
        if content.parent.has_class("monitor-column"):
            content.parent.display = present
        if present:
            content.update(text())

    @staticmethod
    def _input_text(field: Field, value) -> str:
        """Text an Input shows for a value (integers without decimals, numbers compact)."""
        if value is None:
            return ""
        if field.kind == "int":
            try:
                return str(round(float(value)))
            except (TypeError, ValueError):
                return str(value)
        return format_value(value) if isinstance(value, (int, float)) else str(value)

    # ================================================================== step rendering
    async def show_step(self, index: int, reload: bool = True) -> None:
        """Render a step: title, intro, one row per visible field, then its action buttons."""
        index = max(0, min(len(STEPS) - 1, index))
        self.step_index = index
        step = STEPS[index]
        ctx = self.ctx
        ctx.enter_step(step, reload=reload)
        self._render_generation += 1
        self.widgets = {}
        self.fields = {}
        self.by_key = {}
        self.help = {}
        self.actions = {}
        self._select_options = {}
        box = self.query_one("#stepbox", VerticalScroll)
        self._steps_being_shown += 1
        try:
            await self._mount_step(box, step)
        finally:
            self._steps_being_shown -= 1
        # values a job proposed while the widgets were being mounted
        self._refresh_fields()
        box.scroll_home(animate=False)
        self._refresh_nav()
        if step.on_enter is not None:
            self.run_job(lambda: step.on_enter(ctx), "")

    async def _mount_step(self, box: VerticalScroll, step) -> None:
        """Replace the step box's content with the step's title, intro, field rows and action rows."""
        ctx = self.ctx
        await box.remove_children()
        rows = [
            Horizontal(
                Static(f"[dim]{step.phase.label}[/dim]  ·  {step.title}", classes="title"),
                self._help_button(step.title, step.help),
                classes="trow",
            )
        ]
        if step.phase.drives:
            rows.append(
                Static(
                    "[bold]❗ Phase 2 DRIVES the vehicle:[/bold] ARM it outdoors first (Arm button), keep your "
                    "transmitter in hand and the STOP button ready.",
                    classes="warn",
                )
            )
        rows.append(Markdown(step.intro, parser_factory=markdown_parser, classes="intro"))
        # Settings missing on this firmware - unless another field with the same label (the same setting under a
        # different parameter name/unit, e.g. RNGFND1_MAX vs RNGFND1_MAX_CM) is shown instead.
        shown_labels = {field.label for field in step.fields if ctx.is_visible(field)}
        hidden = [
            f"{field.label} ({parameter_names_text(field.names(ctx))})"
            for field in step.fields
            if field.param is not None
            and (field.when is None or field.when(ctx))
            and not ctx.vehicle.resolve(field.names(ctx))
            and field.label not in shown_labels
        ]
        if hidden:
            rows.append(Static("ℹ Not available on this firmware (hidden): " + ", ".join(hidden), classes="dim"))
        rows.extend(self._field_row(field) for field in step.fields if ctx.is_visible(field))
        action_rows = [
            self._action_row(position, action)
            for position, action in enumerate(step.actions)
            if action.when is None or action.when(ctx)
        ]
        if action_rows:
            # grouped, so the CSS can set them apart from the fields (.actions)
            rows.append(Vertical(*action_rows, classes="actions"))
        # The widgets report their starting values as changes while mounting; those aren't the pilot's edits (and
        # could overwrite a value a job proposed meanwhile).
        with self.prevent(Input.Changed, Select.Changed, Checkbox.Changed):
            await box.mount(*rows)

    def _help_button(self, title: str, body: str) -> Button:
        """A ❓ button that opens `body` (markdown) in a HelpScreen."""
        button_id = f"help-{self._render_generation}-{len(self.help)}"
        self.help[button_id] = (title, body)
        return Button("❓", id=button_id, classes="help")

    def _field_widget(self, field: Field, widget_id: str):
        """The input widget for one field, pre-filled with its current value."""
        ctx = self.ctx
        value = ctx.values.get(field.key)
        if field.kind == "choice":
            options = list(field.choices or [])
            if value is not None and all(option_value != value for _, option_value in options):
                # vehicle has a value not in our list
                options.append((f"{format_value(value)} (current)", value))
            if value is None:
                value = options[0][1]
                ctx.set_field_value(field.key, value)
            self._select_options[field.key] = [option_value for _, option_value in options]
            return Select(options, value=value, allow_blank=False, id=widget_id)
        if field.kind == "bool":
            # a Checkbox carries its own label (the row's label column stays empty for it)
            return Checkbox(field.label, value=bool(value), id=widget_id)
        if field.kind == "display":
            return Static(
                escape_markup(str(value)) if value is not None else "(querying ...)",
                id=widget_id,
                classes="display-value",
            )
        return Input(value=self._input_text(field, value), id=widget_id, placeholder=field.unit)

    def _field_row(self, field: Field) -> Horizontal:
        """Label, widget, unit, inline action buttons, current-value hint and ❓ for one field."""
        if field.kind == "heading":
            help_part = self._help_button(field.label, field.help) if field.help else Static("", classes="help")
            return Horizontal(Static(f"{field.label}:", classes="field-heading"), help_part, classes="frow")
        ctx = self.ctx
        widget_id = f"w-{self._render_generation}-{len(self.widgets)}"
        widget = self._field_widget(field, widget_id)
        self.widgets[field.key] = widget
        self.fields[widget_id] = field
        self.by_key[field.key] = field
        name = ctx.vehicle.resolve(field.names(ctx)) if field.param is not None else None
        hint = f"({name} = {format_value(ctx.vehicle.get(name))})" if name else ""
        # A checkbox shows its own label; every other widget gets a label column in front.
        row = [widget] if field.kind == "bool" else [Static(f"{field.label}:", classes="lbl"), widget]
        if field.unit:
            # right after the value it belongs to, before any buttons
            row.append(Static(field.unit, classes="unit"))
        for position, action in enumerate(field.inline_actions or []):
            if action.when is None or action.when(ctx):
                button_id = f"act-{self._render_generation}-f{len(self.widgets)}-{position}"
                self.actions[button_id] = action
                row.append(
                    Button(
                        action.label,
                        id=button_id,
                        classes="inline-act",
                        variant="primary" if "Probe" in action.label else "default",
                    )
                )
        # after the buttons: the hint takes the rest of the row, so the buttons stay next to the input
        row.append(Static(hint, classes="hint"))
        if field.help or name:
            row.append(self._help_button(field.label, self._field_help(field, name)))
        else:
            row.append(Static("", classes="help"))
        return Horizontal(*row, classes="frow-tall" if field.kind == "display" else "frow")

    @staticmethod
    def _field_help(field: Field, name: str) -> str:
        """The wizard's own guidance (if any) followed by ArduPilot's description of the parameter (if documented)."""
        official = describe(name) if name else None
        if official and field.bit is not None:
            official += f"\n\nThis checkbox is the bit with value {field.bit} of `{name}`."
        parts = [part for part in (field.help, official) if part]
        if not parts:
            return f"**{field.label}**\n\nArduPilot parameter: `{name}` (no description in ArduPilot's documentation)."
        return "\n\n---\n\n".join(parts)

    def _action_row(self, position: int, action: Action) -> Horizontal:
        """An action button with a note about the arming state it needs."""
        button_id = f"act-{self._render_generation}-{position}"
        self.actions[button_id] = action
        note = {True: "needs ARMED vehicle", False: "vehicle must be DISARMED"}.get(action.armed, "")
        help_part = self._help_button(action.label, action.help) if action.help else Static("", classes="help")
        return Horizontal(
            Button(action.label, id=button_id, classes="act"), help_part, Static(note, classes="dim"), classes="arow"
        )

    def _refresh_fields(self) -> None:
        """Push values proposed by an action (ctx.set_values) into the visible widgets."""
        # A job can end while show_step() is still mounting a step: its widgets aren't composed yet (a Select has no
        # label to update). show_step() refreshes them itself once they are mounted.
        if self._steps_being_shown:
            return
        # Without prevent(), each widget would report the change back as a pilot edit (_field_changed) and replace
        # the proposed value with its display text.
        with self.prevent(Input.Changed, Select.Changed, Checkbox.Changed):
            for key, widget in self.widgets.items():
                field = self.by_key[key]
                value = self.ctx.values.get(key)
                if value is None:
                    continue
                if isinstance(widget, Input):
                    widget.value = self._input_text(field, value)
                elif isinstance(widget, Select):
                    # a Select only accepts one of its options
                    if value in self._select_options.get(key, ()):
                        widget.value = value
                elif isinstance(widget, Checkbox):
                    widget.value = bool(value)
                elif isinstance(widget, Static):
                    widget.update(escape_markup(str(value)))

    # ================================================================== field edits
    def _field_changed(self, widget_id: Optional[str], value) -> None:
        """A widget changed: store the value; re-render if the field changes which fields are shown."""
        field = self.fields.get(widget_id or "")
        if field is None or value == self.ctx.values.get(field.key):
            return
        self.ctx.set_field_value(field.key, value)
        if field.reload_on_change:
            # other fields now describe another item
            self.call_later(self.show_step, self.step_index, True)
        elif field.refresh:
            self.call_later(self.show_step, self.step_index, False)

    def on_input_changed(self, event: Input.Changed) -> None:
        """Text typed into an Input."""
        self._field_changed(event.input.id, event.value)

    def on_select_changed(self, event: Select.Changed) -> None:
        """A dropdown selection changed."""
        self._field_changed(event.select.id, event.value)

    def on_checkbox_changed(self, event: Checkbox.Changed) -> None:
        """A checkbox was toggled (Checkbox.Changed.checkbox is the widget, per the Textual API)."""
        self._field_changed(event.checkbox.id, event.value)

    # ================================================================== buttons
    async def on_button_pressed(self, event: Button.Pressed) -> None:
        """Route every button: help, prompts, navigation, apply/next/skip, arming, reboot, STOP and actions."""
        event.stop()
        button_id = event.button.id or ""
        ctx = self.ctx
        if button_id.startswith("help-"):
            self.app.push_screen(HelpScreen(*self.help[button_id]))
        elif button_id == "b-cont":
            ctx.press_continue()
        elif button_id == "b-cancel":
            ctx.request_cancel()
            self._write_log("Cancel requested ...")
        elif button_id == "b-stop":
            ctx.emergency()
            self._write_log("🛑 STOP: overrides released, Hold mode + disarm commanded.")
        elif button_id.startswith("nav-"):
            target = int(button_id[len("nav-") :])
            if self.flow.does_apply(target) and self.flow.is_reachable(target) and self._is_idle():
                await self._leave_step(target)
        elif button_id == "b-back":
            if self._is_idle():
                await self._leave_step(self.flow.neighbour(self.step_index, -1))
        elif button_id == "b-apply":
            # Re-render afterwards: enabling a feature (gripper, scripting ...) can make new parameters appear.
            self.run_job(self._apply_current_step, "Apply parameters", on_success=self._reload)
        elif button_id == "b-next":
            self.run_job(lambda: self._apply_current_step(mark_done=True), "Apply & continue", on_success=self._advance)
        elif button_id == "b-skip":
            if self._is_idle():
                step_id = STEPS[self.step_index].id
                await self._leave_step(
                    self.flow.neighbour(self.step_index, +1), lambda: ctx.store.mark_step(step_id, STEP_SKIPPED)
                )
        elif button_id == "b-arm":
            self.app.push_screen(
                ConfirmScreen("ARM the vehicle? Wheels on the ground, area clear, people away."),
                lambda yes: yes and self.run_job(self._arm, "Arm"),
            )
        elif button_id == "b-disarm":
            self.run_job(self._disarm, "Disarm")
        elif button_id == "b-reboot":
            self.app.push_screen(
                ConfirmScreen("Reboot the flight controller now?"),
                lambda yes: yes
                and self.run_job(lambda: action_reboot_flight_controller(ctx), "Reboot", on_success=self._reload),
            )
        elif button_id.startswith("act-"):
            self._start_action(self.actions[button_id])
        elif button_id.startswith("relay-"):
            relay = int(button_id.rsplit("-", 1)[1])
            self.run_job(lambda: toggle_relay(ctx, relay))
        elif button_id.startswith(("servo-plus-", "servo-minus-")):
            output = int(button_id.rsplit("-", 1)[1])
            direction = +1 if button_id.startswith("servo-plus-") else -1
            self.run_job(lambda: ctx.log(self.ctx.servos.nudge(output, direction)))

    def _start_action(self, action: Action) -> None:
        """Run a step action (inline or normal) after checking its arming requirement."""
        ctx = self.ctx
        if action.armed is True and not ctx.vehicle.armed:
            self._write_log("❗ This action needs an ARMED vehicle: use the Arm button first.")
        elif action.armed is False and ctx.vehicle.armed:
            self._write_log("❗ This action needs a DISARMED vehicle: disarm first.")
        elif action.is_probe:
            if self._is_idle():
                # the probe dialog owns the context until it is closed
                self.busy = True
                self.app.push_screen(
                    ProbeScreen(ctx, lambda probe_ctx: probe_ctx.run_action(action), action.label),
                    lambda _result: self._probe_closed(),
                )
        else:
            self.run_job(lambda: ctx.run_action(action), action.label)

    def _probe_closed(self) -> None:
        """The probe dialog was closed: release the busy flag and show the (possibly changed) values."""
        self.busy = False
        self._reload()

    def _is_idle(self) -> bool:
        """True if nothing is running; otherwise tell the pilot to wait."""
        if self.busy:
            self._write_log(BUSY_MESSAGE)
        return not self.busy

    def _arm(self) -> None:
        """Arm (after the pilot confirmed the popup)."""
        self.ctx.vehicle.arm()
        self.ctx.log("Vehicle ARMED.")

    def _disarm(self) -> None:
        """Disarm from the control bar."""
        self.ctx.vehicle.disarm()
        self.ctx.log("Disarm command sent.")

    def _apply_current_step(self, mark_done: bool = False) -> None:
        """Write the current step's changed fields; optionally mark the step done (only if writing succeeded)."""
        ctx = self.ctx
        step = STEPS[self.step_index]
        written = ctx.apply_step(step)
        ctx.log(f"{written} parameter(s) written." if written else "Nothing to write (values already on the vehicle).")
        if mark_done:
            ctx.store.mark_step(step.id, STEP_DONE)

    async def _leave_step(self, target: int, on_leave: Optional[Callable] = None) -> None:
        """Open step `target` (after `on_leave`); if the form holds values not applied yet, first ask whether to
        apply or discard them - or to stay (measurement results are only proposals until Apply)."""
        unapplied = self.ctx.unapplied_fields(STEPS[self.step_index])
        if not unapplied:
            if on_leave:
                on_leave()
            await self.show_step(target)
            return

        def leave() -> None:
            """Go on to the target step."""
            if on_leave:
                on_leave()
            self.call_later(self.show_step, target, True)

        def answered(answer: Optional[str]) -> None:
            """Apply (then leave if it worked), Discard (leave) or Stay."""
            if answer == "apply":
                self.run_job(self._apply_current_step, "Apply parameters", on_success=leave)
            elif answer == "discard":
                self._write_log("Discarded: " + ", ".join(unapplied))
                leave()

        self.app.push_screen(LeaveStepScreen(unapplied), answered)

    def _advance(self) -> None:
        """Open the next applicable step (or say that this was the last one)."""
        following = self.flow.neighbour(self.step_index, +1)
        if following == self.step_index:
            self._write_log("🎉 That was the last step.")
            return
        self.call_later(self.show_step, following, True)

    def _reload(self) -> None:
        """Re-render the current step with fresh values (e.g. after a reboot)."""
        self.call_later(self.show_step, self.step_index, True)

    # ================================================================== background jobs
    def run_job(self, job: Callable, label: str = "", on_success: Optional[Callable] = None) -> None:
        """Run `job` in a worker thread (one at a time); log its outcome; run `on_success` only if it succeeded."""
        if not self._is_idle():
            return
        self.busy = True
        ctx = self.ctx
        ctx.reset_for_new_action()

        def work() -> None:
            """Worker body: run the job, turn exceptions into log lines, clean up, then report back to the UI."""
            succeeded = False
            try:
                if label:
                    ctx.log(f"── {label} ──")
                job()
                succeeded = True
            except Cancelled:
                ctx.log("⏹ cancelled")
            except VehicleError as exc:
                ctx.log(f"❌ {exc}")
            # a bug: show the traceback instead of killing the worker silently
            except Exception:
                ctx.log("❌ internal error:\n" + traceback.format_exc())
            finally:
                ctx.cleanup()
                self.ui(self._job_done, on_success if succeeded else None)

        self.run_worker(work, thread=True)

    def action_parameters(self) -> None:
        """Ctrl+P: the list of all parameters (not while an operation is running)."""
        if self._is_idle():
            self.app.push_screen(ParameterScreen(self), callback=self._parameters_closed)

    def _parameters_closed(self, written: bool) -> None:
        """Back from the parameter list: if values were written, the step shows them (not the old form values)."""
        if written:
            self._reload()

    def _job_done(self, follow_up: Optional[Callable]) -> None:
        """On the UI thread after a job: release the busy flag, refresh the screen, run the follow-up."""
        self.busy = False
        self._set_waiting(False)
        self._refresh_fields()
        self._refresh_nav()
        if follow_up:
            follow_up()
