"""rover_wizard.texts - what the pilot reads, from ui/wizard_text.json, worded for the vehicle at hand.

wizard_text.json holds every text as a land vehicle reads it. ui/wizard_text_water.json holds only the texts that
read differently on a boat, under the same keys; such a text is loaded as a model.Wording, which carries both, and
Ctx.words() picks the one for the connected vehicle. Sections:
* steps   - step id -> {"intro", "help"}
* fields  - "step id.field key" -> help of a field
* actions - "step id.button label" -> help of an action button
* probes  - "step id.field key" / "step id.button label" -> briefing logged before a probe or drive run starts
* prompts - "step id.name" -> a question or instruction an action shows while it runs
"""

from __future__ import annotations

import json
import os
from functools import lru_cache

from .model import Wording

TEXT_DIRECTORY = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui")
TEXT_FILE = os.path.join(TEXT_DIRECTORY, "wizard_text.json")
WATER_TEXT_FILE = os.path.join(TEXT_DIRECTORY, "wizard_text_water.json")


def _read(path: str) -> dict:
    """One text file."""
    with open(path, encoding="utf-8") as text_file:
        return json.load(text_file)


def _with_water(land: dict, water: dict) -> dict:
    """The land texts of one section; those with a water version become Wording (nested dicts: steps)."""
    merged = {}
    for key, text in land.items():
        if isinstance(text, dict):
            merged[key] = _with_water(text, water.get(key, {}))
        else:
            merged[key] = Wording(text, water[key]) if key in water else text
    return merged


@lru_cache(maxsize=1)
def all_texts() -> dict:
    """Every section, loaded once."""
    land, water = _read(TEXT_FILE), _read(WATER_TEXT_FILE)
    return {section: _with_water(entries, water.get(section, {})) for section, entries in land.items()}


def prompt(key: str) -> str:
    """A question or instruction an action shows, e.g. "motor_output.ramp_start"."""
    return all_texts()["prompts"][key]
