"""rover_wizard.steps.battery - the "Battery monitor" step."""

from __future__ import annotations

from typing import Optional

from ..util.errors import VehicleError
from ..mavlink_defs import SYS_STATUS_CENTIAMPS_PER_AMP, SYS_STATUS_MILLIVOLTS_PER_VOLT
from ..util.timebase import now
from ..util.values import format_value
from .util.rover_hardware import has
from ..model import Action, Step, param_field, value_field

BATTERY_MONITOR_CHOICES = [
    ("Disabled", 0),
    ("Analog voltage only", 3),
    ("Analog voltage and current", 4),
    ("SMBus generic", 7),
    ("DroneCAN BatteryInfo", 8),
    ("ESC telemetry", 9),
]

BATTERY_FAILSAFE_ACTION_CHOICES = [
    ("Warn only", 0),
    ("RTL", 1),
    ("Hold", 2),
    ("SmartRTL", 3),
    ("SmartRTL or Hold", 4),
    ("Terminate", 5),
]

# BATT_FS_VOLTSRC: the voltage the low/critical failsafes compare
BATTERY_VOLTAGE_SOURCE_CHOICES = [("Measured voltage", 0), ("Sag-compensated (resting) voltage", 1)]

# battery voltage/current calibration averages this long ...
BATTERY_AVERAGE_SECONDS = 4.0

# ... the plain "read battery values" display this long
BATTERY_READ_SECONDS = 3.0

# [s] between battery readings while averaging
BATTERY_SAMPLE_SPACING = 0.25


def _average_battery_readings(ctx, seconds: float = BATTERY_AVERAGE_SECONDS) -> tuple:
    """Mean battery (volts, amps) reported over a few seconds; either is None if never reported."""
    volts = []
    amps = []
    start = now()
    while now() - start < seconds:
        status = ctx.vehicle.latest("SYS_STATUS")
        if status is not None:
            volts.append(status.voltage_battery / SYS_STATUS_MILLIVOLTS_PER_VOLT)
            # -1 means "no current sensor"
            if status.current_battery >= 0:
                amps.append(status.current_battery / SYS_STATUS_CENTIAMPS_PER_AMP)
        ctx.sleep(BATTERY_SAMPLE_SPACING)
    return (sum(volts) / len(volts) if volts else None), (sum(amps) / len(amps) if amps else None)


def action_read_battery(ctx):
    """Log the battery voltage and current the autopilot reports (3 s average)."""
    volts, amps = _average_battery_readings(ctx, BATTERY_READ_SECONDS)
    ctx.log(f"Battery: {format_value(volts)} V, {format_value(amps)} A (as reported by the autopilot)")


def _propose_corrected_multiplier(
    ctx, measured_key: str, reported_index: int, parameter: str, unit: str, missing_message: str
) -> None:
    """Scale a battery sensor multiplier by measured / reported value and propose the result in the form."""
    measured = ctx.number(measured_key, 0)
    multiplier = ctx.param_value(parameter)
    # checked before the readings are averaged: no waiting just to be told the input is missing
    if measured <= 0 or multiplier is None:
        raise VehicleError(missing_message)

    reported = _average_battery_readings(ctx)[reported_index]
    if not reported:
        raise VehicleError(missing_message)

    # The reading is linear in the multiplier, so multiplying it by (true / reported) makes the reading true.
    corrected = multiplier * measured / reported
    ctx.log(
        f"Reported {reported:.2f} {unit}, measured {measured:.2f} {unit} -> {parameter} "
        f"{format_value(multiplier)} -> {corrected:.4f}"
    )
    # calibration factor to 4 decimal places
    ctx.set_values(**{parameter: round(corrected, 4)})


def action_calibrate_battery_voltage(ctx):
    """Correct BATT_VOLT_MULT from a multimeter reading of the pack voltage."""
    _propose_corrected_multiplier(
        ctx,
        "meas_v",
        0,
        "BATT_VOLT_MULT",
        "V",
        "Enter the multimeter voltage first (and BATT_MONITOR must be an analog type, applied).",
    )


def action_calibrate_battery_current(ctx):
    """Correct BATT_AMP_PERVLT from a clamp-meter reading of the current."""
    _propose_corrected_multiplier(
        ctx,
        "meas_a",
        1,
        "BATT_AMP_PERVLT",
        "A",
        "Enter the measured current first (needs a real load and a current sensor).",
    )


def check_battery_thresholds(ctx) -> Optional[str]:
    """Critical must be below low (voltage and remaining capacity) - ArduPilot refuses to arm otherwise."""
    for low_key, critical_key, unit in (
        ("BATT_LOW_VOLT", "BATT_CRT_VOLT", "V"),
        ("BATT_LOW_MAH", "BATT_CRT_MAH", "mAh"),
    ):
        low = ctx.number(low_key, ctx.param_value(low_key) or 0)
        critical = ctx.number(critical_key, ctx.param_value(critical_key) or 0)
        # 0 switches a threshold off
        if low > 0 and critical > 0 and critical >= low:
            return (
                f"{critical_key} ({format_value(critical)} {unit}) must be below {low_key} "
                f"({format_value(low)} {unit})."
            )
    return None


STEP = Step(
    "battery",
    "Battery monitor",
    [
        param_field("BATT_MONITOR", "Monitor type", kind="choice", choices=BATTERY_MONITOR_CHOICES, reboot=True),
        param_field("BATT_VOLT_PIN", "Voltage pin", kind="int"),
        param_field("BATT_CURR_PIN", "Current pin", kind="int"),
        param_field("BATT_VOLT_MULT", "Voltage multiplier"),
        param_field("BATT_AMP_PERVLT", "Amps per volt"),
        param_field("BATT_AMP_OFFSET", "Current offset", unit="V"),
        param_field("BATT_CAPACITY", "Battery capacity", kind="int", unit="mAh"),
        param_field("BATT_LOW_VOLT", "Low voltage", unit="V"),
        param_field("BATT_CRT_VOLT", "Critical voltage", unit="V"),
        param_field("BATT_LOW_TIMER", "Below the voltage for", kind="int", unit="s"),
        param_field("BATT_FS_VOLTSRC", "Voltage compared", kind="choice", choices=BATTERY_VOLTAGE_SOURCE_CHOICES),
        param_field("BATT_LOW_MAH", "Low remaining capacity", kind="int", unit="mAh"),
        param_field("BATT_CRT_MAH", "Critical remaining capacity", kind="int", unit="mAh"),
        param_field("BATT_ARM_VOLT", "Minimum voltage to arm", unit="V"),
        param_field("BATT_FS_LOW_ACT", "Low battery action", kind="choice", choices=BATTERY_FAILSAFE_ACTION_CHOICES),
        param_field(
            "BATT_FS_CRT_ACT", "Critical battery action", kind="choice", choices=BATTERY_FAILSAFE_ACTION_CHOICES
        ),
        value_field("meas_v", "Measured pack voltage (multimeter)", unit="V", default=0.0),
        value_field("meas_a", "Measured current (clamp meter)", unit="A", default=0.0),
    ],
    [
        Action("Read battery values", action_read_battery),
        Action("Calibrate voltage from multimeter", action_calibrate_battery_voltage),
        Action("Calibrate current from clamp meter", action_calibrate_battery_current),
    ],
    applies=has("has_battery_monitor"),
    check=check_battery_thresholds,
)
