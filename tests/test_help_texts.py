"""Every help text ends with a link to the matching page of ArduPilot's Rover documentation."""

import json
import re

from rover_wizard.ui.step_catalog import TEXT_FILE

DOCS_LINK = re.compile(r"\n\nArduPilot documentation: <https://ardupilot\.org/rover/docs/[\w-]+\.html>$")
# ArduPilot documents the sprayer for Copter and Plane only
WITHOUT_ROVER_PAGE = {"sprayer", "vehicle_type.has_sprayer"}


def test_help_texts_end_with_a_rover_docs_link():
    texts = json.load(open(TEXT_FILE, encoding="utf-8"))
    entries = {step: entry["help"] for step, entry in texts["steps"].items()}
    entries |= texts["fields"] | texts["actions"]
    unlinked = [key for key, text in entries.items() if key not in WITHOUT_ROVER_PAGE and not DOCS_LINK.search(text)]
    assert not unlinked
