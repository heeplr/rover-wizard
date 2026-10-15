"""Probes of serial devices try other baud rates when the port's current rate finds nothing.

Rangefinder, proximity and gimbal drivers open their port at its configured rate only, so the probes cycle
SERIALx_BAUD themselves. ArduPilot's GPS driver cycles baud rates on its own; the GPS probe just waits long enough.
"""

import pytest

from rover_wizard.steps.distance_sensors import action_probe_proximity, action_probe_rangefinder
from rover_wizard.steps.gimbal import action_probe_gimbal
from rover_wizard.steps.gps import ARDUPILOT_GPS_BAUD_RATES, ARDUPILOT_GPS_BAUD_STEP_SECONDS, GPS_DETECTION_SECONDS

from util.helpers import apply, enter, make

# SERIAL5 / SERIAL6 are unused in the simulator
FREE_PORT = 5


@pytest.fixture
def wizard():
    """A simulated vehicle with rangefinder and proximity sensor ticked, logging into a list, always saying yes."""
    vehicle, ctx = make()
    ctx.store.set("has_rangefinder", True)
    ctx.store.set("has_proximity", True)
    ctx.confirm = lambda message: True
    messages = []
    ctx.set_output(log=messages.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    yield vehicle, ctx, messages
    vehicle.disconnect()


def tried(messages):
    """The 'Trying ...' lines, in order."""
    return [line[len("Trying ") : -len(" ...")] for line in messages if line.startswith("Trying ")]


def test_rangefinder_found_at_another_baud_rate(wizard):
    vehicle, ctx, messages = wizard
    vehicle.set_param(f"SERIAL{FREE_PORT}_BAUD", 115)
    # a LightWare serial rangefinder that talks 38400 baud
    vehicle.true_rangefinder = (8, FREE_PORT)
    vehicle.true_rangefinder_baud = 38
    enter(ctx, "distance_sensors")
    ctx.set_field_value("rng_port", FREE_PORT)
    action_probe_rangefinder(ctx)
    assert (vehicle.get("RNGFND1_TYPE"), vehicle.get(f"SERIAL{FREE_PORT}_BAUD")) == (8, 38)
    serial_tries = [name for name in tried(messages) if "baud" in name]
    assert serial_tries[0] == "LightWare serial at 115200 baud", "the port's current rate comes first"


def test_rangefinder_tries_the_current_rate_first(wizard):
    vehicle, ctx, messages = wizard
    vehicle.set_param(f"SERIAL{FREE_PORT}_BAUD", 19)
    vehicle.true_rangefinder = (20, FREE_PORT)
    vehicle.true_rangefinder_baud = 19
    enter(ctx, "distance_sensors")
    ctx.set_field_value("rng_port", FREE_PORT)
    action_probe_rangefinder(ctx)
    serial_tries = [name for name in tried(messages) if "baud" in name]
    assert serial_tries == ["LightWare serial at 19200 baud", "Benewake serial at 19200 baud"]
    assert vehicle.get("RNGFND1_TYPE") == 20


def test_rangefinder_probe_restores_the_baud_rate_when_nothing_answers(wizard):
    vehicle, ctx, messages = wizard
    vehicle.set_param(f"SERIAL{FREE_PORT}_BAUD", 57)
    enter(ctx, "distance_sensors")
    ctx.set_field_value("rng_port", FREE_PORT)
    action_probe_rangefinder(ctx)
    serial_tries = [name for name in tried(messages) if "baud" in name]
    # two serial types at each of the 8 common rates (the current one, 57600, is among them)
    assert len(serial_tries) == 2 * 8
    assert (vehicle.get(f"SERIAL{FREE_PORT}_BAUD"), vehicle.get("RNGFND1_TYPE")) == (57, 0)


def test_proximity_sensor_found_at_an_unusual_baud_rate(wizard):
    vehicle, ctx, messages = wizard
    # an RPLidar set to 57600 baud - none of its usual rates
    vehicle.true_proximity = (5, 6, 57)
    enter(ctx, "distance_sensors")
    ctx.set_field_value("prx_port", 6)
    vehicle.set_param("SERIAL6_BAUD", 115)
    action_probe_proximity(ctx)
    assert (vehicle.get("PRX1_TYPE"), vehicle.get("SERIAL6_BAUD"), vehicle.get("SERIAL6_PROTOCOL")) == (5, 57, 11)
    assert tried(messages)[:2] == ["RPLidar at 115200 baud", "LightWare SF40/C at 115200 baud"]


def test_gimbal_found_at_another_baud_rate(wizard):
    vehicle, ctx, messages = wizard
    vehicle.set_param("SERIAL6_BAUD", 115)
    # a Siyi gimbal on SERIAL6 at 57600 baud
    vehicle.true_gimbal_type = 8
    vehicle.true_gimbal_serial = (6, 57)
    ctx.confirm = lambda message: (vehicle.gimbal_pitch != 0) if "did the gimbal tilt" in message else True
    enter(ctx, "gimbal")
    ctx.set_field_value("gimbal_port", 6)
    action_probe_gimbal(ctx)
    assert (vehicle.get("MNT1_TYPE"), vehicle.get("SERIAL6_BAUD"), vehicle.get("SERIAL6_PROTOCOL")) == (8, 57, 8)
    # Apply keeps the baud rate the probe found (it used to write 115200, and the gimbal went silent)
    apply(ctx, "gimbal")
    assert vehicle.get("SERIAL6_BAUD") == 57
    # coming back to the step later: the port in use is shown, so Apply works without choosing it again
    enter(ctx, "gimbal")
    assert ctx.value("gimbal_port") == 6


def test_gps_probe_waits_for_ardupilots_own_baud_cycle():
    # ArduPilot tries the configured rate plus 8 others, 1.2 s each; the probe waits two full rounds
    one_round = (ARDUPILOT_GPS_BAUD_RATES + 1) * ARDUPILOT_GPS_BAUD_STEP_SECONDS
    assert GPS_DETECTION_SECONDS >= 2 * one_round
