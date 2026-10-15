"""Link robustness: automatic reconnect after the autopilot goes away, and no leaked connections on failed opens."""

import itertools
import time

import pytest

pytest.importorskip("pymavlink")

import rover_wizard.vehicle.link as vehicle_module  # noqa: E402
from rover_wizard.util.errors import VehicleError  # noqa: E402
from rover_wizard.vehicle.link import Vehicle  # noqa: E402

from util.fake_autopilot import FakeAutopilot  # noqa: E402

PORTS = itertools.count(14650)


def wait_until(condition, timeout=10.0):
    """Poll until condition() holds (real time: the link runs on the real clock)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return condition()


def test_reconnects_when_the_autopilot_comes_back(monkeypatch):
    monkeypatch.setattr(vehicle_module, "RECONNECT_AFTER_SECONDS", 1.0)
    monkeypatch.setattr(vehicle_module, "RECONNECT_RETRY_INTERVAL", 0.3)
    port = next(PORTS)
    autopilot = FakeAutopilot(port)
    vehicle = Vehicle()
    vehicle.connect(f"udpin:127.0.0.1:{port}", timeout=10)
    first_link = vehicle.master
    try:
        # the autopilot goes silent (e.g. rebooted, radio out of range)
        autopilot.close()
        assert wait_until(lambda: vehicle.link_status().startswith("❗"))
        assert wait_until(lambda: "reconnecting" in vehicle.link_error)
        # ... and comes back
        autopilot = FakeAutopilot(port)
        assert wait_until(lambda: "OK" in vehicle.link_status() and vehicle.link_error == "")
        assert vehicle.master is not first_link, "a fresh link was opened"
        assert vehicle.parameter_count > 0, "cached parameters are kept"
    finally:
        vehicle.disconnect()
        autopilot.close()


class ScriptedHeartbeat:
    """A HEARTBEAT as a link would deliver it."""

    def __init__(self, vehicle_type):
        """A heartbeat from an ArduPilot autopilot of the given MAV_TYPE."""
        self.autopilot = 3
        self.type = vehicle_type
        self.base_mode = 0
        self.custom_mode = 4

    def get_type(self):
        """Message type, as pymavlink reports it."""
        return "HEARTBEAT"

    def get_srcSystem(self):
        """Sender's system id."""
        return 1

    def get_srcComponent(self):
        """Sender's component id."""
        return 1


class ScriptedLink:
    """A link that is silent, or answers as a given vehicle type; remembers whether it was closed."""

    created: list = []

    def __init__(self, vehicle_type=None):
        """vehicle_type None = silent link, otherwise it answers with that vehicle's heartbeats."""
        self.vehicle_type = vehicle_type
        self.closed = False
        ScriptedLink.created.append(self)

    def recv_match(self, type=None, blocking=True, timeout=1):
        """Next message: nothing (silent) or a heartbeat."""
        time.sleep(0.01)
        return None if self.vehicle_type is None else ScriptedHeartbeat(self.vehicle_type)

    def close(self):
        """Remember that the wizard closed this link."""
        self.closed = True


@pytest.mark.parametrize("vehicle_type, error", [(None, "No ArduPilot heartbeat"), (1, "only supports Rover/Boat")])
def test_failed_open_closes_its_link(vehicle_type, error):
    ScriptedLink.created.clear()
    vehicle = Vehicle()
    vehicle._mavutil = type(
        "Mavutil", (), {"mavlink_connection": staticmethod(lambda *a, **k: ScriptedLink(vehicle_type))}
    )
    with pytest.raises(VehicleError, match=error):
        vehicle._open(0.3)
    assert ScriptedLink.created and all(link.closed for link in ScriptedLink.created)


def test_keeps_reconnecting_after_a_reboot_that_takes_too_long(monkeypatch):
    # The board isn't back within the reboot wait: the wizard reports it, but must not give up - once the board
    # answers, the link is back by itself (it used to stay at "NO HEARTBEAT" until the wizard was restarted).
    monkeypatch.setattr(vehicle_module, "REBOOT_RECONNECT_TIMEOUT", 1.0)
    monkeypatch.setattr(vehicle_module, "REBOOT_OPEN_ATTEMPT_SECONDS", 0.3)
    monkeypatch.setattr(vehicle_module, "RECONNECT_AFTER_SECONDS", 1.0)
    monkeypatch.setattr(vehicle_module, "RECONNECT_RETRY_INTERVAL", 0.3)
    port = next(PORTS)
    autopilot = FakeAutopilot(port)
    vehicle = Vehicle()
    vehicle.connect(f"udpin:127.0.0.1:{port}", timeout=10)
    try:
        # the board goes down for longer than the wizard waits
        autopilot.close()
        with pytest.raises(VehicleError, match="keeps trying in the background"):
            vehicle.reboot()
        autopilot = FakeAutopilot(port)
        assert wait_until(lambda: "OK" in vehicle.link_status() and vehicle.link_error == "", timeout=20)
        # the reconnected link works: telemetry arrives again
        assert wait_until(lambda: vehicle.latest("HEARTBEAT", max_age=2) is not None)
    finally:
        vehicle.disconnect()
        autopilot.close()


def test_announces_itself_while_waiting_for_the_vehicle():
    # a GCS heartbeat goes out while the wizard waits for the vehicle's first heartbeat
    sent = []

    class AnnouncingLink(ScriptedLink):
        """A silent link that records the heartbeats the wizard sends."""

        def __init__(self):
            super().__init__(None)
            self.mav = type("Mav", (), {"heartbeat_send": staticmethod(lambda *args: sent.append(args))})()

    vehicle = Vehicle()
    assert vehicle._wait_for_ardupilot_heartbeat(AnnouncingLink(), 0.3, announce=True) is None and sent
    sent.clear()
    vehicle._wait_for_ardupilot_heartbeat(AnnouncingLink(), 0.3, announce=False)
    assert not sent, "link_only (-p) mode stays silent"


def test_a_failed_connect_leaves_nothing_running(monkeypatch):
    # e.g. the parameter download fails after the link is open: no reader thread or open port may stay behind
    port = next(PORTS)
    autopilot = FakeAutopilot(port)
    vehicle = Vehicle()

    def download_fails(progress=None):
        """The parameter list never arrives."""
        raise VehicleError("Could not download the parameter list.")

    monkeypatch.setattr(vehicle, "fetch_params", download_fails)
    try:
        with pytest.raises(VehicleError, match="parameter list"):
            vehicle.connect(f"udpin:127.0.0.1:{port}", timeout=10)
        assert not vehicle._reader_thread.is_alive()
    finally:
        autopilot.close()
