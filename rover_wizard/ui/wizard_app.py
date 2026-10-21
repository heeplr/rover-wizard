"""rover_wizard.ui.wizard_app - the Textual application: connect screen first, then the wizard screen."""

from __future__ import annotations

import os
from typing import Optional

from textual.app import App
from textual.binding import Binding

from .. import __version__
from ..context import Ctx
from ..session.store import DEFAULT_ROOT, Store
from ..vehicle.demo import demo_vehicle
from ..vehicle.link import DEFAULT_BAUD_RATE, Vehicle
from .connect_screen import ConnectScreen
from .main_screen import MainScreen

# demo progress must never mix with a real vehicle's
DEMO_STORE_ROOT = os.path.join(DEFAULT_ROOT, "demo")
# terminal width below which the compact layout is used
COMPACT_BELOW_COLUMNS = 160


class WizardApp(App):
    """Shows the connect screen, then the main wizard screen once a vehicle is connected."""

    TITLE = "ArduPilot Rover Setup Wizard"
    # the wizard's version; the vehicle's system id is added once connected
    SUB_TITLE = f"v{__version__}"
    # Ctrl+P opens the parameter list instead of Textual's command palette
    ENABLE_COMMAND_PALETTE = False
    # Textual's Ctrl+Q (quit) made visible in the footer, as "^q quit"; it runs action_quit below
    BINDINGS = [Binding("ctrl+q", "quit", "quit", priority=True)]
    # Textual adds the class to the screen by terminal width; narrow terminals get the compact layout below.
    HORIZONTAL_BREAKPOINTS = [(0, "-compact"), (COMPACT_BELOW_COLUMNS, "-roomy")]
    CSS_PATH = "wizard_app.css"

    def __init__(
        self,
        connection_string: str = "",
        baud_rate: int = DEFAULT_BAUD_RATE,
        demo: str = "",
    ) -> None:
        """Startup options from the command line (they pre-fill, or skip, the connect screen); `demo` names a
        simulated vehicle (vehicle.demo.DEMO_VEHICLES), "" connects to a real one."""
        super().__init__()
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.demo = demo
        self.ctx: Optional[Ctx] = None

    def on_mount(self) -> None:
        """Start with the connect screen."""
        self.sub_title = self.SUB_TITLE
        self.push_screen(ConnectScreen())

    def make_vehicle(self) -> Vehicle:
        """A real Vehicle, or the simulator for --demo / --demo-skid / --demo-boat / --demo-sailboat."""
        if self.demo:
            return demo_vehicle(self.demo, wired_wrong=True)
        return Vehicle()

    def connected(self, vehicle: Vehicle) -> None:
        """Connection succeeded: create the wizard context and switch to the wizard screen."""
        store = Store(vehicle.system_id, root=DEMO_STORE_ROOT if self.demo else None)
        self.ctx = Ctx(vehicle, store)
        # replaces the connect screen (nothing leads back to it)
        self.switch_screen(MainScreen(self.ctx))

    async def action_quit(self) -> None:
        """Quit: stop the running action, release control of the vehicle, close the link."""
        if self.ctx is not None:
            self.ctx.shutdown()
            self.ctx.vehicle.disconnect()
        self.exit()
