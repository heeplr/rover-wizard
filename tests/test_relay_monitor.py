"""Relay buttons in the side panel: which relays are set up and on (RELAY_STATUS, or the parameters on older
firmware), and toggling one."""

import pytest

from rover_wizard.util.errors import VehicleError
from rover_wizard.steps.relays import toggle_relay
from rover_wizard.vehicle.link import Vehicle

from util.helpers import make, wait_until

# a relay set up in the simulator (function Relay, a pin) and one that isn't
RELAY = 2
UNUSED_RELAY = 3


@pytest.fixture
def wizard():
    """A simulated vehicle with relay 2 set up; the log goes into a list."""
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    vehicle.set_param(f"RELAY{RELAY}_FUNCTION", 1)
    vehicle.set_param(f"RELAY{RELAY}_PIN", 50)
    yield vehicle, ctx, log
    vehicle.disconnect()


def test_only_relays_set_up_are_reported(wizard):
    vehicle, ctx, log = wizard
    assert wait_until(lambda: vehicle.relays() == {RELAY: False})


def test_toggle_switches_to_the_opposite_state(wizard):
    vehicle, ctx, log = wizard
    wait_until(lambda: RELAY in vehicle.relays())
    toggle_relay(ctx, RELAY)
    assert wait_until(lambda: vehicle.relays() == {RELAY: True}) and log[-1] == f"Relay {RELAY} switched ON."
    toggle_relay(ctx, RELAY)
    assert wait_until(lambda: vehicle.relays() == {RELAY: False}) and log[-1] == f"Relay {RELAY} switched OFF."


def test_a_refused_relay_is_reported(wizard):
    vehicle, ctx, log = wizard
    with pytest.raises(VehicleError, match=f"Relay {UNUSED_RELAY} refused"):
        toggle_relay(ctx, UNUSED_RELAY)


def test_older_firmware_without_relay_status_uses_the_pin_parameters():
    # firmware before 4.5: no RELAY_STATUS and no RELAYn_FUNCTION, just RELAY_PIN, RELAY_PIN2, ... (-1 = unused)
    vehicle = Vehicle()
    vehicle._params.update({"RELAY_PIN": 54, "RELAY_PIN2": -1, "RELAY_PIN3": 55})
    vehicle._switched_relays[3] = True
    assert vehicle.relays() == {1: False, 3: True}
