"""rover_wizard.steps.boat_loiter - the "Holding position (Loiter)" step of a boat.

A boat can't stop like a car: wind and current carry it away. In Loiter mode - and, on a boat, wherever Auto, Guided,
RTL and SmartRTL reach their destination - Rover drifts while it is within LOIT_RADIUS of the target and drives back
once it is further away, at (distance beyond the radius) x LOIT_SPEED_GAIN, at most WP_SPEED (Rover/mode_loiter.cpp).
A sailing boat uses SAIL_LOIT_RADIUS instead while it sails (Sails step).
"""

from __future__ import annotations

from ..boat_defs import MISSION_DONE_LOITER
from ..model import Step, param_field

LOITER_TYPE_CHOICES = [
    ("Forward or backward, whichever turns less", 0),
    ("Always bow first", 1),
    ("Always stern first", 2),
]

MISSION_DONE_CHOICES = [
    ("Hold (stop and drift)", 0),
    ("Loiter (hold the position)", MISSION_DONE_LOITER),
    ("Switch to Acro", 2),
    ("Switch to Manual", 3),
]

STEP = Step(
    "boat_loiter",
    "Holding position (Loiter)",
    [
        param_field("LOIT_RADIUS", "Drift freely within", unit="m", min_value=0, max_value=20),
        param_field("LOIT_TYPE", "Way back to the target", kind="choice", choices=LOITER_TYPE_CHOICES),
        param_field(
            "LOIT_SPEED_GAIN", "Speed back per metre outside the radius", unit="m/s per m", min_value=0, max_value=5
        ),
        param_field("MIS_DONE_BEHAVE", "After the last mission item", kind="choice", choices=MISSION_DONE_CHOICES),
    ],
    [],
    applies=lambda ctx: ctx.is_boat(),
)
