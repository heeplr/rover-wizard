"""The "Test RC failsafe" action (Pre-drive checks) against the simulator's emulation of Rover's radio failsafe."""

import pytest

from rover_wizard.rover_defs import MODES
from rover_wizard.steps.predrive import action_test_rc_failsafe

from util.helpers import make


@pytest.fixture
def wizard():
    """An armed simulated vehicle; the log goes into a list; prompts switch the simulated transmitter."""
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    vehicle.arm()
    answers = {"switch_to_manual": True}

    def pilot(message, during=None):
        """Do what the prompt asks with the transmitter."""
        log.append(">> " + message)
        if "TRANSMITTER OFF" in message:
            vehicle.transmitter_on = False
        elif "back ON" in message:
            vehicle.transmitter_on = True

    def answer(message):
        """Record the question and give the prepared answer."""
        log.append("?? " + message)
        return answers["switch_to_manual"]

    ctx.pause = pilot
    ctx.confirm = answer
    yield vehicle, ctx, log, answers
    vehicle.disconnect()


def test_failsafe_and_its_end_are_reported(wizard):
    vehicle, ctx, log, answers = wizard
    vehicle.set_mode("MANUAL")
    action_test_rc_failsafe(ctx)
    assert any(line.startswith("OK   failsafe reported: Radio Failsafe - mode now HOLD") for line in log)
    assert any(line == "OK   failsafe ended: Radio Failsafe Cleared" for line in log)
    assert vehicle.custom_mode == MODES["HOLD"] and "stays in HOLD" in log[-1]


def test_hold_is_left_for_the_test_after_asking(wizard):
    # in Hold, ArduPilot's failsafe stays silent - the reason the test used to see nothing a second time
    vehicle, ctx, log, answers = wizard
    assert vehicle.custom_mode == MODES["HOLD"]
    action_test_rc_failsafe(ctx)
    assert any(line.startswith("?? The vehicle is in HOLD") for line in log)
    assert any(line.startswith("OK   failsafe reported") for line in log)
    assert any(line.startswith("OK   failsafe ended") for line in log)


def test_staying_in_hold_is_not_a_test(wizard):
    vehicle, ctx, log, answers = wizard
    answers["switch_to_manual"] = False
    action_test_rc_failsafe(ctx)
    assert log[-1] == "Not tested: in HOLD the failsafe would stay silent." and vehicle.transmitter_on


def test_failsafe_in_hold_option(wizard):
    # FS_OPTIONS bit 0: the failsafe also acts in Hold - no need to leave it
    vehicle, ctx, log, answers = wizard
    vehicle.set_param("FS_OPTIONS", 1)
    action_test_rc_failsafe(ctx)
    assert not any(line.startswith("??") for line in log) and vehicle.custom_mode == MODES["HOLD"]
    assert any(line.startswith("OK   failsafe ended") for line in log)


def test_receiver_that_keeps_sending_is_reported(wizard):
    # a receiver set to hold the last values: the flight controller never sees RC loss
    vehicle, ctx, log, answers = wizard
    vehicle.set_mode("MANUAL")
    ctx.pause = lambda message, during=None: log.append(">> " + message)
    action_test_rc_failsafe(ctx)
    assert any(line.startswith("FAIL no failsafe message") for line in log)
    assert not any("wasn't reported as cleared" in line for line in log)
