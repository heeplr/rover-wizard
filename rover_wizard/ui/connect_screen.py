"""rover_wizard.ui.connect_screen - the first screen: where to connect (auto-connects if given on the command line)."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import Horizontal, VerticalScroll
from textual.screen import Screen
from textual.widgets import Button, Footer, Header, Input, Static

from ..util.values import escape_markup
from ..vehicle.link import DEFAULT_BAUD_RATE
from .modals import HelpScreen

CONNECT_HELP = """
**Connection string** (anything `pymavlink` understands):

* USB / serial telemetry: `/dev/ttyACM0` (Linux), `COM5` (Windows), `/dev/tty.usbmodem14101` (macOS) - set the baud rate
  (115200 for USB, 57600 for most telemetry radios).
* UDP from a ground station / MAVProxy: `udpin:0.0.0.0:14550`
* TCP: `tcp:127.0.0.1:5760` (e.g. SITL)

Close other programs (Mission Planner, QGroundControl) that use the same serial port, or forward MAVLink to a UDP port and
connect to that.
"""


class ConnectScreen(Screen):
    """The first screen: enter a MAVLink connection string and connect to the vehicle."""

    def compose(self) -> ComposeResult:
        """Lay out the connection-string/baud-rate form and the Connect button."""
        app = self.app
        yield Header()
        with VerticalScroll(id="connect"):
            yield Static("ArduPilot Rover Setup Wizard", id="c-title")
            yield Static(
                "Connect to the flight controller over MAVLink. The wizard reads and writes parameters "
                "and guides you through setup and tuning."
            )
            with Horizontal(classes="frow"):
                yield Static("Connection string:", classes="lbl")
                yield Input(value=app.connection_string, id="c-conn", placeholder="/dev/ttyACM0 or udpin:0.0.0.0:14550")
                yield Button("❓", id="help-conn", classes="help")
            with Horizontal(classes="frow"):
                yield Static("Baud rate (serial only):", classes="lbl")
                yield Input(value=str(app.baud_rate), id="c-baud")
            yield Button("Connect", id="c-go", variant="success")
            yield Static("", id="c-status")
        yield Footer()

    def on_mount(self) -> None:
        """If a connection string was already given on the command line (--conn/--demo), connect right away
        instead of waiting for the pilot to press Connect. The dialog stays available in case it fails."""
        if self.app.connection_string or self.app.demo:
            self._start_connect(self.app.connection_string, self.app.baud_rate)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Handle the help button or the Connect button being pressed."""
        event.stop()
        if event.button.id == "help-conn":
            self.app.push_screen(HelpScreen("Connection string", CONNECT_HELP))
        elif event.button.id == "c-go":
            connection_string = self.query_one("#c-conn", Input).value.strip()
            try:
                baud_rate = int(self.query_one("#c-baud", Input).value.strip() or DEFAULT_BAUD_RATE)
            except ValueError:
                baud_rate = DEFAULT_BAUD_RATE
            if not connection_string and not self.app.demo:
                self._status("Please enter a connection string.")
                return

            self._start_connect(connection_string, baud_rate)

    def _start_connect(self, connection_string: str, baud_rate: int) -> None:
        """Disable the Connect button and kick off the connection attempt in the background."""
        self.query_one("#c-go", Button).disabled = True
        self._status("Connecting ...")
        self.run_worker(lambda: self._connect(connection_string, baud_rate), thread=True)

    def _status(self, text: str) -> None:
        """Show a short status line under the Connect button."""
        self.query_one("#c-status", Static).update(text)

    def _connect(self, connection_string: str, baud_rate: int) -> None:
        """Background worker: open the MAVLink connection and, on success, switch to the main wizard screen."""
        app = self.app
        try:
            vehicle = app.make_vehicle()
            vehicle.connect(
                connection_string,
                baud_rate,
                progress=lambda downloaded, total: app.call_from_thread(
                    self._status, f"Downloading parameters {downloaded}/{total} ..."
                ),
            )
            app.call_from_thread(app.connected, vehicle)
        # shown to the user, connection can be retried
        except Exception as exc:
            app.call_from_thread(self._failed, str(exc) or repr(exc))

    def _failed(self, message: str) -> None:
        """Show a connection error and re-enable the Connect button so the pilot can retry."""
        self._status("Connection failed: " + escape_markup(message))
        self.query_one("#c-go", Button).disabled = False
