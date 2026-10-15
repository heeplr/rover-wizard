"""rover_wizard.util.values - small helpers for displaying and comparing parameter values."""

from __future__ import annotations

import re
from typing import Any

# parameters are float32 on the vehicle, so exact equality is meaningless
PARAMETER_RELATIVE_TOLERANCE = 1e-5

# what Rich and Textual markup take for a tag: "[" + a lowercase letter, "#", "/" or "@" ... "]"
_MARKUP_TAG = re.compile(r"(\\*)(\[[a-z#/@][^[]*?])")


def escape_markup(text: str) -> str:
    """Text to put into Rich/Textual markup (the log, the status bar) shown as it is.

    Messages from outside - the flight controller, exceptions, file names - can contain something that looks like a
    tag ("[/dev/ttyACM0]"): unescaped, the text disappears or the markup parser raises. Same rule as
    rich.markup.escape / textual.markup.escape, without importing either (the steps also run without the UI).
    """
    escaped = _MARKUP_TAG.sub(lambda match: f"{match[1]}{match[1]}\\{match[2]}", text)
    # a trailing backslash would escape the closing tag that follows the text
    return escaped + "\\" if escaped.endswith("\\") and not escaped.endswith("\\\\") else escaped


def format_value(value: Any) -> str:
    """Short, human-readable text for a value in the log/UI ('-' when there is no value)."""
    if value is None:
        return "-"
    if isinstance(value, str):
        return value
    number = float(value)
    # whole numbers (integer, enum and bitmask parameters) in full, never as 1.03637e+06
    if number.is_integer():
        return str(int(number))
    return f"{number:.6g}"


def are_nearly_equal(a: float, b: float, relative_tolerance: float = PARAMETER_RELATIVE_TOLERANCE) -> bool:
    """Whether two parameter values are equal within float32 precision (so we don't rewrite an unchanged value).

    Whole numbers (integer, enum and bitmask parameters) must match exactly: a relative tolerance on a large
    bitmask would hide a change of one small bit.
    """
    if float(a).is_integer() and float(b).is_integer():
        return int(a) == int(b)
    # Scale the tolerance by the larger magnitude (but at least 1) so it works for both tiny and large values.
    scale = max(1.0, abs(float(a)), abs(float(b)))
    return abs(float(a) - float(b)) <= relative_tolerance * scale
