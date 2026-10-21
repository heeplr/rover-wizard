"""The real UI with a simulated sailing boat: texts, choices and the status line are worded for the water, and the
sail steps show up."""

import asyncio

import pytest

pytest.importorskip("textual")

from rover_wizard.ui.step_catalog import STEPS  # noqa: E402
from rover_wizard.ui.wizard_app import WizardApp  # noqa: E402

from test_ui import SIZE, screen_text, start, until  # noqa: E402
from util.helpers import check  # noqa: E402

INDEX = {step.id: position for position, step in enumerate(STEPS)}


async def boat_scenario():
    """Open the steps whose wording differs on a boat and the sailing steps."""
    app = WizardApp(demo="sailboat")
    async with app.run_test(size=SIZE) as pilot:
        screen, ctx, vehicle = await start(pilot, app)
        # the vehicle info is filled in the background while the step is shown
        check(
            "vehicle info names the boat's steering",
            await until(pilot, lambda: "Steering type in use: RUDDER" in screen_text(app)),
            screen_text(app)[:600],
        )
        check(
            "status line names the boat's steering", await until(pilot, lambda: "RUDDER steering" in screen_text(app))
        )
        check("land-only boxes are hidden", "has_wenc" not in screen.widgets)
        check("boat boxes are offered", {"has_torqeedo", "has_depth_sounder", "has_sails"} <= set(screen.widgets))

        await screen.show_step(INDEX["steering_output"])
        await pilot.pause(0.2)
        text = screen_text(app)
        check("steering type in boat words", "Rudder or steered motor (outboard)" in text, text[:1500])

        await screen.show_step(INDEX["motor_output"])
        await pilot.pause(0.2)
        check("intro in boat words", "in the water, tied up" in screen_text(app))

        await screen.show_step(INDEX["sails"])
        await pilot.pause(0.2)
        text = screen_text(app)
        check("sails step shows its settings", "The sail is moved by" in text and "Ideal angle" in text, text[:1500])
        vehicle.disconnect()


def test_boat_wording_in_the_ui():
    asyncio.run(boat_scenario())
