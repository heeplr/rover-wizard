"""Every help text ends with a link to the matching page of ArduPilot's Rover documentation; the boat wording
(wizard_text_water.json) only rewords texts that exist for land vehicles."""

import json
import re

from rover_wizard.texts import TEXT_FILE, WATER_TEXT_FILE

DOCS_LINK = re.compile(r"\n\nArduPilot documentation: <https://ardupilot\.org/rover/docs/[\w-]+\.html>$")
# ArduPilot documents the sprayer for Copter and Plane only
WITHOUT_ROVER_PAGE = {"sprayer", "vehicle_type.has_sprayer"}


def _help_entries(texts: dict) -> dict:
    """Step help, field help and action help of one text file (prompts and briefings have no link)."""
    entries = {step: entry["help"] for step, entry in texts.get("steps", {}).items() if "help" in entry}
    return entries | texts.get("fields", {}) | texts.get("actions", {})


def test_help_texts_end_with_a_rover_docs_link():
    for path in (TEXT_FILE, WATER_TEXT_FILE):
        entries = _help_entries(json.load(open(path, encoding="utf-8")))
        unlinked = [
            key for key, text in entries.items() if key not in WITHOUT_ROVER_PAGE and not DOCS_LINK.search(text)
        ]
        assert not unlinked, path


def test_boat_wording_replaces_existing_texts():
    land = json.load(open(TEXT_FILE, encoding="utf-8"))
    water = json.load(open(WATER_TEXT_FILE, encoding="utf-8"))
    for section, entries in water.items():
        for key, text in entries.items():
            assert key in land[section], f"{section}.{key} has no land text"
            if isinstance(text, dict):
                assert set(text) <= set(land[section][key]), f"{section}.{key}"
