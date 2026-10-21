"""rover_wizard.steps.visual_odometry - the "Visual odometry" step."""

from __future__ import annotations

from .util.rover_hardware import has
from ..model import Step, param_field

VISUAL_ODOMETRY_TYPE_CHOICES = [("None", 0), ("MAVLink", 1), ("Intel T265", 2), ("VOXL", 3)]


STEP = Step(
    "visual_odometry",
    "Visual odometry",
    [
        param_field(
            "VISO_TYPE", "Visual odometry type", kind="choice", choices=VISUAL_ODOMETRY_TYPE_CHOICES, reboot=True
        ),
        param_field("VISO_POS_X", "Camera position X (forward)", unit="m"),
        param_field("VISO_POS_Y", "Camera position Y (right)", unit="m"),
        param_field("VISO_POS_Z", "Camera position Z (down)", unit="m"),
    ],
    [],
    applies=has("has_visual_odometry"),
)
