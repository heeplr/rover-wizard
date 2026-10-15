"""rover_wizard.steps.temperature - I2C/analog temperature sensors (TEMPn_ parameters)."""

from __future__ import annotations

from .util.hardware import has
from ..model import Step, param_field, value_field

TEMPERATURE_TYPE_CHOICES = [
    ("Disabled", 0),
    ("TSYS01", 1),
    ("MCP9600", 2),
    ("MAX31865 (2 or 4 wire)", 3),
    ("TSYS03", 4),
    ("MLX90614", 7),
    ("SHT3x", 8),
    ("MAX31865 (3 wire)", 9),
    ("TMP119", 10),
]

TEMPERATURE_LOG_CHOICES = [
    ("No logging", 0),
    ("Log all temperature reports", 1),
    ("Log only sensors not replacing another report", 2),
]

TEMPERATURE_SENSOR_CHOICES = [(f"Sensor {number}", number) for number in range(1, 4)]


def selected_temperature_sensor(ctx) -> int:
    """The sensor (1..3) the step's fields currently describe."""
    return int(ctx.values.get("temp_sensor") or 1)


def temperature_parameter(suffix: str):
    """TEMPn_<suffix> of the selected sensor."""
    return lambda ctx: f"TEMP{selected_temperature_sensor(ctx)}_{suffix}"


STEP = Step(
    "temperature",
    "Temperature sensors",
    [
        value_field(
            "temp_sensor",
            "Sensor to configure",
            kind="choice",
            choices=TEMPERATURE_SENSOR_CHOICES,
            default=lambda ctx: ctx.values.get("temp_sensor") or 1,
            reload_on_change=True,
        ),
        param_field(
            temperature_parameter("TYPE"),
            "Sensor type",
            key="temp_type",
            kind="choice",
            choices=TEMPERATURE_TYPE_CHOICES,
            reboot=True,
        ),
        param_field(temperature_parameter("BUS"), "I2C bus", key="temp_bus", kind="int", min_value=0, max_value=3),
        param_field(
            temperature_parameter("ADDR"),
            "I2C address (decimal)",
            key="temp_addr",
            kind="int",
            min_value=0,
            max_value=127,
        ),
        param_field("TEMP_LOG", "Logging", kind="choice", choices=TEMPERATURE_LOG_CHOICES),
    ],
    [],
    applies=has("has_temperature"),
)
