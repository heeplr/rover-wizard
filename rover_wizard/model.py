"""rover_wizard.model - the declarative building blocks of the wizard: steps, their form fields and buttons.

A Step is pure data (texts, fields, actions, hooks); the wizard context interprets it and the UI renders it.
Keeping these types free of behaviour means a step's definition can be read without knowing either of them.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional
from collections.abc import Callable


@dataclass
class Field:
    """One form field on a wizard step - either bound to an ArduPilot parameter (built with param_field()) or a plain
    value the wizard computes or asks for (built with value_field()).
    """

    key: str
    label: str
    # str | tuple | callable(ctx) -> str|tuple | None
    param: Any = None
    # float | int | choice | bool | text | display
    kind: str = "float"
    choices: Optional[list] = None
    unit: str = ""
    help: str = ""
    # value or callable(ctx)
    default: Any = None
    reboot: bool = False
    when: Optional[Callable] = None
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    # re-render the step when this choice changes (unsaved edits are kept)
    refresh: bool = False
    # checkbox for ONE bit of a bitmask parameter (several fields share the parameter)
    bit: Optional[int] = None
    # the parameter lists what is skipped/disabled: checked = bit NOT set
    bit_skips: bool = False
    # bit that means "everything" (e.g. ARMING_CHECK 1 = All checks)
    bit_all: Optional[int] = None
    # this field selects WHICH output/relay the other fields edit: changing it re-renders the step with every
    # field reloaded from the vehicle
    reload_on_change: bool = False
    # list[Action]: small buttons shown in this field's own row (e.g. Test up/down)
    inline_actions: Optional[list] = None
    # form value = parameter value x scale (e.g. 57.2958 shows a parameter in radians as degrees)
    scale: float = 1.0

    def names(self, ctx) -> tuple:
        """Resolve this field's ArduPilot parameter name(s) for the current vehicle (the name can depend on skid vs
        front-wheel steering, or on firmware version).
        """
        p = self.param(ctx) if callable(self.param) else self.param
        return () if p is None else ((p,) if isinstance(p, str) else tuple(p))


def param_field(names, label: str, **kwargs) -> Field:
    """Build a field that reads/writes an ArduPilot parameter (or the first of several alternative names for different
    firmware versions).
    """
    key = kwargs.pop("key", None) or (names if isinstance(names, str) else names[0])
    return Field(key=key, label=label, param=names, **kwargs)


def bit_field(names, bit: int, label: str, key: str, **kwargs) -> Field:
    """A checkbox for one bit of a bitmask parameter; several bit fields of one parameter are written together."""
    return Field(key, label, names, kind="bool", bit=bit, **kwargs)


def heading_field(key: str, label: str, **kwargs) -> Field:
    """A title row above a group of fields (no value, no parameter); its help (if any) explains the group."""
    return Field(key=key, label=label, kind="heading", **kwargs)


def value_field(key: str, label: str, **kwargs) -> Field:
    """Build a field that holds a value the wizard computes or asks for, without being tied to one ArduPilot parameter."""
    return Field(key=key, label=label, **kwargs)


@dataclass
class Action:
    """One button shown under a step, e.g. a calibration routine or a measurement run."""

    label: str
    fn: Callable
    help: str = ""
    # True: vehicle must be armed, False: must be disarmed
    armed: Optional[bool] = None
    when: Optional[Callable] = None
    # True: open a live-updating modal instead of running inline (see app.ProbeScreen)
    is_probe: bool = False
    # What the pilot will do and must watch out for, logged before the action starts (probes: from wizard_text.json)
    briefing: str = ""


@dataclass(frozen=True)
class Phase:
    """A group of steps done under the same conditions (rover_wizard.ui.step_catalog lists them and their steps)."""

    number: int
    # "Bench setup (disarmed, indoors OK)"
    title: str
    # its steps drive the vehicle (the step page warns to arm outdoors with the STOP button ready)
    drives: bool
    # the modules of its steps (each defines STEP), in order
    steps: tuple

    @property
    def label(self) -> str:
        """Short name shown above a step's title, e.g. "PHASE 1"."""
        return f"PHASE {self.number}"

    @property
    def heading(self) -> str:
        """Shown above the phase's steps in the step list, e.g. "PHASE 1 - Bench setup (disarmed, indoors OK)"."""
        return f"{self.label} - {self.title}"


@dataclass
class Step:
    """One page of the wizard: an introduction, optional parameter fields, and the buttons (actions) it offers."""

    id: str
    title: str
    fields: list
    actions: list
    applies: Optional[Callable] = None
    # ctx -> None, runs after the step's fields were written
    on_apply: Optional[Callable] = None
    # ctx -> error text or None, checked before anything is written: the form contradicts itself
    check: Optional[Callable] = None
    # ctx -> None, runs in the background whenever the step is shown
    on_enter: Optional[Callable] = None
    # The texts shown with the step; rover_wizard.ui.step_catalog attaches them from ui/wizard_text.json.
    intro: str = ""
    help: str = ""
    # set by rover_wizard.ui.step_catalog, which puts the steps into their phases
    phase: Optional[Phase] = None

    def does_apply_to(self, ctx) -> bool:
        """Whether this step is relevant for the connected vehicle (hidden and skipped otherwise)."""
        return self.applies is None or bool(self.applies(ctx))


def parameter_names_text(names) -> str:
    """A parameter name, or alternative names (differing between firmware versions), for messages: "A or B"."""
    if isinstance(names, str):
        return names
    return " or ".join(names)
