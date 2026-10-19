"""rover_wizard.context - what a wizard step or action works with (the `ctx` argument everywhere).

Ctx combines five collaborators and adds the pilot interaction between them:
* ctx.vehicle  - the MAVLink link (vehicle/link.py)
* ctx.store    - saved progress for this vehicle (session/store.py)
* ctx.changes  - parameter change log + pending reboot reasons (session/changes.py)
* ctx.drive    - drive-test runs (motion/driving.py)
* ctx.servos   - moving outputs by hand (motion/servo_test.py)
Actions run in worker threads; the UI connects to a Ctx only through set_output() / redirect_output(),
press_continue(), request_cancel() and emergency(), so neither side needs to know the other's internals.
"""

from __future__ import annotations

import contextlib
import threading
from typing import Any, Optional
from collections.abc import Callable

from .util import timebase
from .session.changes import ChangeTracker
from .motion.driving import DriveSession
from .util.errors import Cancelled, VehicleError
from .ardupilot_defs import SERVO_FUNCTION_THROTTLE_LEFT, SERVO_FUNCTION_THROTTLE_RIGHT, SERVO_OUTPUTS
from .mavlink_defs import MAV_SEVERITY_ERROR, MAV_SEVERITY_WARNING
from .model import Field, Step, parameter_names_text
from .motion.servo_test import ServoTester

from .session.store import Store
from .util.values import are_nearly_equal, escape_markup, format_value
from .vehicle.link import Vehicle

# how often a waiting prompt checks for Continue/Cancel
PROMPT_POLL_SECONDS = 0.1
# how often a paused action (ctx.sleep) checks for Cancel / STOP
PAUSE_POLL_SECONDS = 0.05


class Ctx:
    """The wizard context handed to every step hook and action."""

    def __init__(self, vehicle: Vehicle, store: Store) -> None:
        """Create the context for one connected vehicle and its saved progress."""
        self.vehicle = vehicle
        self.store = store
        self.changes = ChangeTracker(store)
        self.drive = DriveSession(vehicle, self)
        self.servos = ServoTester(self)
        # field key -> current form value of the step being shown
        self.values: dict = {}
        # True while an action waits for Continue
        self.waiting = False
        # Cancel or STOP pressed
        self._cancel = threading.Event()
        # STOP pressed (never treated as a mere "no")
        self._emergency = threading.Event()
        # Continue pressed
        self._continue = threading.Event()
        self._status_text_marker = vehicle.status_text_marker()
        self._log: Callable = print
        self._on_wait: Callable = lambda _waiting: None
        self._on_values_changed: Callable = lambda: None

    # ================================================================== UI connection
    def set_output(self, log: Callable, on_wait: Callable, on_values_changed: Callable) -> None:
        """Where log lines go, who is told when Continue becomes (un)available, who refreshes the form."""
        self._log = log
        self._on_wait = on_wait
        self._on_values_changed = on_values_changed
        # with the log in place: undo a servo-function swap a crash or lost link left behind (once)
        self.servos.restore_leftovers()

    @contextlib.contextmanager
    def redirect_output(self, log: Callable, on_wait: Callable):
        """Temporarily send log lines / wait notifications elsewhere (e.g. into a modal), restoring them after."""
        saved = (self._log, self._on_wait, self._on_values_changed)
        self.set_output(log, on_wait, lambda: None)
        try:
            yield
        finally:
            self._log, self._on_wait, self._on_values_changed = saved

    def log(self, message: Any, markup: bool = False) -> None:
        """Write one line to the activity log; it is shown as it is unless markup=True (then the caller escapes any
        text from outside with values.escape_markup)."""
        self._log(str(message) if markup else escape_markup(str(message)))

    def flush_texts(self) -> None:
        """Log flight-controller status messages that arrived since the last call."""
        for severity, text in self.vehicle.status_messages_since(self._status_text_marker):
            self.log(f"FC: [italic {_severity_colour(severity)}]{escape_markup(text)}[/]", markup=True)
        self._status_text_marker = self.vehicle.status_text_marker()

    # ================================================================== form values
    def value(self, key: str, default: Any = None) -> Any:
        """Current form value of a field of the step being shown."""
        return self.values.get(key, default)

    def number(self, key: str, default: float = 0.0) -> float:
        """A field value as float; falls back to a saved setting (e.g. runway), then to `default`."""
        for candidate in (self.values.get(key), self.store.setting(key), default):
            try:
                # accept "1,5" as well as "1.5"
                return float(str(candidate).replace(",", "."))
            except (TypeError, ValueError):
                continue
        return default

    def set_values(self, **values: Any) -> None:
        """Propose new form values (e.g. measurement results) and refresh the form; Apply writes them."""
        self.values.update(values)
        self._on_values_changed()

    def set_field_value(self, key: str, value: Any) -> None:
        """The pilot edited a field (called by the UI)."""
        self.values[key] = value

    # ================================================================== parameters
    def param_value(self, names, default: Any = None) -> Any:
        """Current value of a parameter (or of the first existing of several alternative names)."""
        name = self.vehicle.resolve(names)
        return default if name is None else self.vehicle.get(name)

    def fetch_params(self) -> None:
        """Download all parameters again (e.g. after enabling a feature reveals more); warn if some are missing."""
        self.vehicle.fetch_params()
        self.warn_if_parameters_missing()

    def warn_if_parameters_missing(self) -> None:
        """Log a warning if the last parameter download was incomplete."""
        warning = self.vehicle.parameter_download_warning()
        if warning:
            self.log(warning)

    def param_int(self, names, default: int = 0) -> int:
        """param_value() as int, for types, functions, options and bitmasks."""
        name = self.vehicle.resolve(names)
        return default if name is None else self.vehicle.get_int(name, default)

    def form_int(self, key: str, names=None, default: int = 0) -> int:
        """The choice made in the form for field `key`; until the pilot picks one, the vehicle's value of parameter
        `names` (default: the parameter named like the field)."""
        chosen = self.values.get(key)
        return self.param_int(key if names is None else names, default) if chosen in (None, "") else int(chosen)

    def set_param(self, names, value: float, reboot: bool = False, required: bool = True) -> Optional[float]:
        """Write a parameter if it changes, log it, record it for "revert", and note if a reboot is needed."""
        name = self.vehicle.resolve(names)
        if name is None:
            if required:
                raise VehicleError(f"Parameter {parameter_names_text(names)} is not available on this firmware.")
            self.log(f"i {parameter_names_text(names)}: not available on this firmware, skipped")
            return None
        current = self.vehicle.get(name)
        if current is not None and are_nearly_equal(current, value):
            return current
        stored = self.vehicle.set_param(name, value)
        self.changes.record(name, current, stored)
        self.log(f"OK {name}: {format_value(current)} -> {format_value(stored)}")
        if not are_nearly_equal(stored, value):
            self.log(f"WARNING: vehicle stored {format_value(stored)} instead of {format_value(value)}")
        if reboot:
            self.changes.mark_reboot_needed(name)
        return stored

    def output_with_function(self, function: int) -> int:
        """The first SERVO output that has this SERVOx_FUNCTION (e.g. throttle); 0 = none."""
        return next((output for output in SERVO_OUTPUTS if self.param_int(f"SERVO{output}_FUNCTION") == function), 0)

    def is_skid(self) -> bool:
        """Skid steering? The pilot's saved choice if any, otherwise detected from the output functions."""
        steering_type = self.store.get("steer")
        if steering_type:
            return steering_type == "skid"
        return any(
            self.output_with_function(function)
            for function in (SERVO_FUNCTION_THROTTLE_LEFT, SERVO_FUNCTION_THROTTLE_RIGHT)
        )

    # ================================================================== step forms
    def is_visible(self, field: Field) -> bool:
        """Whether a field is shown: its condition holds and its parameter (if any) exists on this firmware."""
        if field.when is not None and not field.when(self):
            return False
        return field.param is None or self.vehicle.resolve(field.names(self)) is not None

    def initial(self, field: Field) -> Any:
        """Starting value of a field: the vehicle's current parameter value, else the field default."""
        current = None
        if field.param is not None:
            name = self.vehicle.resolve(field.names(self))
            current = self.vehicle.get(name) if name else None
            if current is not None:
                current *= field.scale
        if current is None:
            current = field.default(self) if callable(field.default) else field.default
        if field.kind == "choice" and field.choices:
            # Parameters arrive as floats; return the matching choice's own value (e.g. 1 instead of 1.0).
            for _, choice_value in field.choices:
                if choice_value == current or (
                    isinstance(choice_value, (int, float))
                    and isinstance(current, (int, float))
                    and are_nearly_equal(choice_value, current)
                ):
                    return choice_value
        if field.kind == "int" and current is not None:
            return round(float(current))
        if field.bit is not None:
            return self._is_bit_checked(field, current)
        if field.kind == "bool":
            return bool(current) if current is not None else False
        return current

    @staticmethod
    def _is_bit_checked(field: Field, value) -> bool:
        """Whether a bit field's checkbox is ticked for a parameter value (honouring 'all' and 'skip' semantics)."""
        mask = int(value or 0)
        if field.bit_all is not None and mask & field.bit_all:
            # the "all" bit switches on every item
            return True
        bit_set = bool(mask & field.bit)
        return not bit_set if field.bit_skips else bit_set

    def enter_step(self, step: Step, reload: bool = True) -> None:
        """Load the starting value of every visible field (reload=False keeps values the pilot already edited)."""
        for field in step.fields:
            if self.is_visible(field) and (reload or field.key not in self.values):
                self.values[field.key] = self.initial(field)

    def parse(self, field: Field) -> Any:
        """The field's entered value as the type it represents (None if a number field holds no number)."""
        raw = self.values.get(field.key)
        if field.kind in ("choice", "text", "bool", "display"):
            return raw
        try:
            number = float(str(raw).strip().replace(",", "."))
        except ValueError:
            return None
        return round(number) if field.kind == "int" else number

    def apply_step(self, step: Step) -> int:
        """Write every changed, valid parameter field of a step, then run its on_apply hook; returns the count.
        Nothing is written if the step's check finds a contradiction in the form."""
        problem = step.check(self) if step.check else None
        if problem:
            raise VehicleError(problem + " Nothing was written.")
        written = self._apply_bit_fields(step)
        for field in step.fields:
            if field.param is None or field.bit is not None or not self.is_visible(field):
                continue
            name = self.vehicle.resolve(field.names(self))
            new_value = self.parse(field)
            if new_value is None:
                self.log(f"WARNING: '{field.label}' is not a valid number - skipped")
                continue
            too_small = field.min_value is not None and new_value < field.min_value
            too_large = field.max_value is not None and new_value > field.max_value
            if too_small or too_large:
                self.log(f"WARNING: '{field.label}' must be within {field.min_value}..{field.max_value} - skipped")
                continue
            current = self.vehicle.get(name)
            if current is not None and are_nearly_equal(current * field.scale, new_value):
                continue
            self.set_param(name, new_value / field.scale, reboot=field.reboot)
            written += 1
        if step.on_apply:
            step.on_apply(self)
        return written

    def unapplied_fields(self, step: Step) -> list:
        """Labels of the shown parameter fields whose form value differs from the vehicle's (edited or proposed by a
        measurement, but not written yet)."""
        labels = []
        for field in step.fields:
            if field.param is None or field.key not in self.values or not self.is_visible(field):
                continue
            name = self.vehicle.resolve(field.names(self))
            current = self.vehicle.get(name) if name else None
            if current is None:
                continue
            if field.bit is not None:
                changed = bool(self.values.get(field.key)) != self._is_bit_checked(field, current)
            else:
                new_value = self.parse(field)
                if new_value is None:
                    # not a number: Apply would skip it, but it isn't on the vehicle either
                    changed = True
                elif isinstance(new_value, (bool, int, float)):
                    changed = not are_nearly_equal(current * field.scale, new_value)
                else:
                    changed = False
            if changed:
                labels.append(field.label)
        return labels

    def _apply_bit_fields(self, step: Step) -> int:
        """Write every bitmask parameter whose checkboxes changed (bits without a checkbox are kept)."""
        groups: dict = {}
        for field in step.fields:
            if field.bit is not None and self.is_visible(field):
                groups.setdefault(self.vehicle.resolve(field.names(self)), []).append(field)
        written = 0
        for name, fields in groups.items():
            current = self.vehicle.get_int(name)
            new = current
            all_bit = next((field.bit_all for field in fields if field.bit_all is not None), None)
            if all_bit is not None and current & all_bit:
                # "All" was set: spell it out as the individual bits so single items can be switched off.
                new = (current & ~all_bit) | sum(field.bit for field in fields)
            for field in fields:
                # Set the bit when checked (or, for a skip-list, when unchecked); clear it otherwise.
                wanted = bool(self.values.get(field.key)) != field.bit_skips
                new = new | field.bit if wanted else new & ~field.bit
            if all_bit is not None and all(self.values.get(field.key) for field in fields):
                # everything checked -> the "all" value itself
                new = all_bit
            if new != current:
                self.set_param(name, new, reboot=any(field.reboot for field in fields))
                written += 1
        return written

    # ================================================================== pilot interaction
    def reset_for_new_action(self) -> None:
        """Forget earlier Cancel/STOP presses before a new action starts."""
        self._cancel.clear()
        self._emergency.clear()

    def request_cancel(self) -> None:
        """Cancel pressed: the running action stops at its next check (a pending question counts as 'no')."""
        self._cancel.set()

    def emergency(self) -> None:
        """STOP pressed: abort the action, release driving control, and command Hold + disarm."""
        self._emergency.set()
        self._cancel.set()
        # first what stops the vehicle (fire-and-forget commands), then the open-loop override
        self.vehicle.emergency_stop()
        self.drive.abort()
        # the steering test and swapped servo functions: parameter writes that wait for confirmation - not on the
        # UI thread, which must stay responsive right now
        threading.Thread(target=self.servos.release, daemon=True).start()

    def check(self) -> None:
        """Raise Cancelled if Cancel/STOP was pressed. Long-running actions call this often."""
        if self._cancel.is_set():
            raise Cancelled()

    def sleep(self, seconds: float) -> None:
        """Wait, but stop early (raising Cancelled) if the pilot cancels."""
        deadline = timebase.now() + seconds
        while timebase.now() < deadline:
            self.check()
            timebase.sleep(PAUSE_POLL_SECONDS)

    def run_action(self, action) -> None:
        """Run a step action; its briefing (what the pilot will do, what to watch out for) is logged first, so it
        stands above the action's first question - which is the pilot's go-ahead."""
        if action.briefing:
            for line in action.briefing.split("\n"):
                self.log(line)
        action.fn(self)

    def begin_wait(self, message: str) -> None:
        """Show an instruction and enable Continue."""
        self.log(f">> {message}   (press Continue)")
        self._continue.clear()
        self.waiting = True
        self._on_wait(True)

    def end_wait(self) -> None:
        """Disable Continue again."""
        self.waiting = False
        self._on_wait(False)

    def was_continue_pressed(self) -> bool:
        """Whether Continue was pressed since the last begin_wait()."""
        return self._continue.is_set()

    def press_continue(self) -> None:
        """Continue pressed (called by the UI)."""
        self._continue.set()

    def pause(self, message: str, during: Optional[Callable] = None) -> None:
        """Show an instruction and block until Continue (Cancel/STOP raise); runs `during` while waiting."""
        self.begin_wait(message)
        try:
            while not timebase.wait_for(self._continue, PROMPT_POLL_SECONDS):
                self.check()
                if during:
                    during()
        finally:
            self.end_wait()

    def confirm(self, message: str) -> bool:
        """Yes/no question: Continue = yes, Cancel = no (the action goes on). STOP still aborts (raises Cancelled)."""
        self.begin_wait(message + "  [Continue = yes / Cancel = no]")
        try:
            while not timebase.wait_for(self._continue, PROMPT_POLL_SECONDS):
                if self._emergency.is_set():
                    raise Cancelled()
                if self._cancel.is_set():
                    # this Cancel was the answer, not a request to abort the action
                    self._cancel.clear()
                    return False
            return True
        finally:
            self.end_wait()

    def require_armed(self) -> None:
        """Refuse to continue unless the vehicle is armed (guards every action that drives)."""
        if not self.vehicle.armed:
            raise VehicleError(
                "The vehicle must be ARMED for this action (use the Arm button, outdoors, wheels on the ground)."
            )

    def cleanup(self) -> None:
        """After every action (success, error or cancel): stop driving and clear any open prompt."""
        self.drive.stop()
        self.end_wait()

    def shutdown(self) -> None:
        """The application is closing: stop the running action and release control of the vehicle."""
        self._cancel.set()
        self.drive.abort()
        self.servos.release()


def _severity_colour(severity: int) -> str:
    """Colour of a flight-controller message in the log: errors red, warnings yellow (e.g. PreArm), the rest white."""
    if severity <= MAV_SEVERITY_ERROR:
        return "red"
    if severity == MAV_SEVERITY_WARNING:
        return "yellow"
    return "white"
