"""rover_wizard.ui.param_docs - ArduPilot's own documentation of each parameter: display name, description, units,
range, named values, bitmask bits, and whether a change needs a reboot or the parameter is read-only.

The texts come word for word from ArduPilot's parameter metadata (tests/util/extract_param_docs.py builds
param_docs.json from apm.pdef.json, which ArduPilot generates from the @DisplayName/@Description/@Units lines
of its source code).
"""

from __future__ import annotations

import json
import os
from functools import lru_cache
from typing import Optional

DOCS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "param_docs.json")


@lru_cache(maxsize=1)
def _all_docs() -> dict:
    """The documentation of every parameter the wizard can show (loaded once)."""
    with open(DOCS_FILE, encoding="utf-8") as docs_file:
        return json.load(docs_file)


def is_documented(name: str) -> bool:
    """Whether ArduPilot documents this parameter."""
    return name in _all_docs()


def describe(name: str) -> Optional[str]:
    """ArduPilot's description of a parameter as Markdown, or None if it isn't documented."""
    entry = _all_docs().get(name)
    if entry is None:
        return None

    units = f" [{entry['units']}]" if entry.get("units") else ""
    return f"**{entry['name']}** - `{name}`{units}\n\n{entry['description']}"


# ArduPilot's bitmask parameters are 32-bit integers: -1 has all bits set
BITMASK_BITS = 32


def display_name(name: str) -> str:
    """ArduPilot's short name of a parameter ("" if undocumented)."""
    return _all_docs().get(name, {}).get("name", "")


def units(name: str) -> str:
    """The parameter's unit ("" if none)."""
    return _all_docs().get(name, {}).get("units", "")


def value_range(name: str) -> Optional[tuple]:
    """(lowest, highest) allowed value, or None if ArduPilot documents no range."""
    limits = _all_docs().get(name, {}).get("range")
    return tuple(limits) if limits else None


def is_read_only(name: str) -> bool:
    """Whether ArduPilot reports the parameter but doesn't accept writes."""
    return bool(_all_docs().get(name, {}).get("readonly"))


def is_reboot_required(name: str) -> bool:
    """Whether a change only takes effect after a reboot."""
    return bool(_all_docs().get(name, {}).get("reboot"))


def is_bitmask(name: str) -> bool:
    """Whether the value is a set of bits."""
    return bool(_all_docs().get(name, {}).get("bitmask"))


def _named_values(name: str) -> dict:
    """value -> name, from ArduPilot's list of named values."""
    return {float(value): label for value, label in _all_docs().get(name, {}).get("values", {}).items()}


def _bits(name: str) -> dict:
    """bit number -> name, from ArduPilot's bitmask description."""
    return {int(bit): label for bit, label in _all_docs().get(name, {}).get("bitmask", {}).items()}


def meanings(name: str) -> list:
    """What the numbers mean, one line each: "0 = Disabled", or for a bitmask "1 = Barometer" (the bit's value)."""
    bits = _bits(name)
    if bits:
        return [f"{1 << bit} = {label}" for bit, label in sorted(bits.items())]

    return [f"{format_number(value)} = {label}" for value, label in sorted(_named_values(name).items())]


def format_number(value: float) -> str:
    """A number without a needless ".0"."""
    return str(int(value)) if float(value).is_integer() else str(value)


def reading(name: str, value: float) -> str:
    """The value in words, using ArduPilot's names: "Enabled", or "Barometer, GPS lock" for a bitmask."""
    bits = _bits(name)
    if bits:
        if not float(value).is_integer():
            return "a bitmask needs a whole number"
        # as ArduPilot stores it: a 32-bit integer (so -1 has all bits set)
        mask = int(value) & ((1 << BITMASK_BITS) - 1)
        if not mask:
            return "nothing selected"
        return ", ".join(bits.get(bit, f"bit {bit}") for bit in range(BITMASK_BITS) if mask & (1 << bit))

    named = _named_values(name)
    if named:
        return named.get(float(value), "not one of the listed values")

    return ""


# Values that switch a feature off but lie outside the documented range (ArduPilot's description says so).
OFF_VALUES = {"MOT_REV_DELAY": 0.0}


def validation_error(name: str, value: Optional[float]) -> str:
    """Why `value` can't be written to the parameter ("" if it can): read-only, not a number, not whole although
    the parameter has named values or bits, or outside ArduPilot's range."""
    if is_read_only(name):
        return "read-only: ArduPilot doesn't accept a new value"

    if value is None:
        return "not a number"

    if meanings(name) and not float(value).is_integer():
        return "needs a whole number"

    limits = value_range(name)
    if limits and not limits[0] <= value <= limits[1] and value != OFF_VALUES.get(name):
        return f"outside ArduPilot's range {format_number(limits[0])} .. {format_number(limits[1])}"

    return ""
