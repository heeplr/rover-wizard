"""The log and the status bar parse markup: text from outside (flight controller, exceptions) must show as it is."""

import pytest

from rover_wizard.util.values import escape_markup
from rover_wizard.vehicle.link import Vehicle

from util.helpers import make

# looks like a closing tag: unescaped, Rich and Textual raise MarkupError; "[x ...]" would vanish as an unknown style
OUTSIDE_TEXTS = ["PreArm: check [/dev/ttyACM0]", "list [x for x in y]", "[Errno 2] No such file", "plain"]


@pytest.mark.parametrize("text", OUTSIDE_TEXTS)
def test_escaped_text_shows_as_it_is_in_rich_and_textual_markup(text):
    text_module = pytest.importorskip("rich.text")
    content_module = pytest.importorskip("textual.content")
    assert text_module.Text.from_markup(f"[italic]{escape_markup(text)}[/]").plain == text
    assert content_module.Content.from_markup(f"[yellow]{escape_markup(text)}[/]").plain == text


def test_log_lines_and_flight_controller_texts_are_escaped():
    vehicle, ctx = make()
    try:
        log = []
        ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
        ctx.log("Could not open [/dev/ttyACM0]")
        ctx.log("[bold]styled on purpose[/]", markup=True)
        vehicle._add_status_text("PreArm: [/x] odd text")
        ctx.flush_texts()
        assert log == [
            "Could not open \\[/dev/ttyACM0]",
            "[bold]styled on purpose[/]",
            "FC: [italic white]PreArm: \\[/x] odd text[/]",
        ]
    finally:
        vehicle.disconnect()


def test_status_bar_link_error_is_escaped():
    vehicle = Vehicle()
    vehicle.link_error = "reconnecting ... (could not open [/dev/ttyACM0])"
    content_module = pytest.importorskip("textual.content")
    assert "[/dev/ttyACM0]" in content_module.Content.from_markup(vehicle.link_status()).plain
