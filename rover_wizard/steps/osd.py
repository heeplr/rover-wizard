"""rover_wizard.steps.osd - the "On-screen display (OSD)" step."""

from __future__ import annotations

from .util.rover_hardware import has
from ..model import Step, param_field

OSD_TYPE_CHOICES = [
    ("None", 0),
    ("MAX7456 (onboard analog OSD)", 1),
    ("MSP", 3),
    ("TX only (e.g. CRSF menus)", 4),
    ("MSP DisplayPort", 5),
]

OSD_UNITS_CHOICES = [("Metric", 0), ("Imperial", 1), ("SI", 2), ("Aviation", 3)]

# RC5..RC16
OSD_CHANNEL_CHOICES = [("No screen switching", 0)] + [(f"RC{channel}", channel) for channel in range(5, 17)]


STEP = Step(
    "osd",
    "On-screen display (OSD)",
    [
        param_field("OSD_TYPE", "OSD type", kind="choice", choices=OSD_TYPE_CHOICES, reboot=True),
        param_field("OSD_CHAN", "Screen switched by RC channel", kind="choice", choices=OSD_CHANNEL_CHOICES),
        param_field("OSD_SW_METHOD", "Screen switching method", kind="int"),
        param_field("OSD_UNITS", "Units", kind="choice", choices=OSD_UNITS_CHOICES),
        param_field("OSD_FONT", "Font number", kind="int"),
        param_field("OSD_W_RSSI", "Warn below RSSI", kind="int", unit="%"),
        param_field("OSD_W_NSAT", "Warn below satellites", kind="int"),
        param_field("OSD_MSG_TIME", "Show messages for", kind="int", unit="s"),
    ],
    [],
    applies=has("has_osd"),
)
