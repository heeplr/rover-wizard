"""rover_wizard.steps.optical_flow - the "Optical flow" step."""

from __future__ import annotations

from .util.rover_hardware import has
from ..model import Step, param_field

OPTICAL_FLOW_TYPE_CHOICES = [
    ("None", 0),
    ("PX4Flow", 1),
    ("Pixart", 2),
    ("Bebop", 3),
    ("CXOF", 4),
    ("MAVLink", 5),
    ("DroneCAN", 6),
    ("MSP", 7),
    ("UPFLOW", 8),
]


STEP = Step(
    "optical_flow",
    "Optical flow",
    [
        param_field("FLOW_TYPE", "Sensor type", kind="choice", choices=OPTICAL_FLOW_TYPE_CHOICES, reboot=True),
        param_field("FLOW_FXSCALER", "X scale correction", kind="int", unit="ppt"),
        param_field("FLOW_FYSCALER", "Y scale correction", kind="int", unit="ppt"),
        param_field("FLOW_ORIENT_YAW", "Mounting yaw", kind="int", unit="cdeg"),
        param_field("FLOW_POS_X", "Position X (forward)", unit="m"),
        param_field("FLOW_POS_Y", "Position Y (right)", unit="m"),
        param_field("FLOW_POS_Z", "Position Z (down)", unit="m"),
        param_field(
            "FLOW_ADDR", "I2C address (PX4Flow: 0-7)", kind="int", when=lambda ctx: int(ctx.number("FLOW_TYPE", 0)) == 1
        ),
    ],
    [],
    applies=has("has_optical_flow"),
)
