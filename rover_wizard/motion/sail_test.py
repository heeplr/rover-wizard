"""rover_wizard.motion.sail_test - moving a sail actuator from the wizard.

ArduPilot moves the sail outputs (main sail winch, wing sail elevator, mast rotation) only from its sail control - or
with its motor test: instance MOTOR_TEST_MAINSAIL sets the sail output to the test's percentage (AP_MotorsUGV::
output_test_pct). Like every motor test it arms the vehicle for its duration.

A sail winch is a range output: 0 % is SERVOx_MIN (sheeted in) and 100 % SERVOx_MAX (let out). A wing sail elevator
and a mast rotation are angle outputs: -100 % is SERVOx_MIN, 0 % TRIM and +100 % MAX. SERVOx_REVERSED swaps the ends.
"""

from __future__ import annotations

from typing import Optional

from ..boat_defs import ANGLE_SAIL_FUNCTIONS, SAIL_TEST_PERCENT_MAX


def sail_range(function: int) -> tuple:
    """(lowest, highest) motor-test percentage of a sail output."""
    return (-SAIL_TEST_PERCENT_MAX if function in ANGLE_SAIL_FUNCTIONS else 0, SAIL_TEST_PERCENT_MAX)


def end_percent(function: int, end: str, reversed_output: bool) -> Optional[float]:
    """Motor-test percentage that drives a sail output to its "MIN", "MAX" or "TRIM" pulse width (None: a sail winch
    has no TRIM position)."""
    low, high = sail_range(function)
    if end == "TRIM":
        return 0.0 if function in ANGLE_SAIL_FUNCTIONS else None
    at_max = (end == "MAX") != reversed_output
    return float(high if at_max else low)
