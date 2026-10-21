"""rover_wizard.steps.rpm - RPM sensors (up to four): type, pin or ESC selection, scaling, live reading."""

from __future__ import annotations

from ..rover_defs import RPM_SENSORS, SERVO_FUNCTION_GPIO
from .util.probing import LIVE_DISPLAY_SECONDS
from .util.outputs import RELAY_PIN_OUTPUT_CHOICES, assign_output_function
from .util.rover_hardware import has
from ..model import Action, Step, param_field, value_field

RPM_TYPE_CHOICES = [
    ("None", 0),
    ("GPIO pin (hall-effect / optical sensor)", 2),
    ("Harmonic notch frequency", 4),
    ("ESC telemetry (average of motors)", 5),
]

RPM_TYPE_GPIO = 2

RPM_TYPE_ESC_TELEMETRY = 5

RPM_SENSOR_CHOICES = [(f"Sensor {number}", number) for number in RPM_SENSORS]


def selected_rpm_sensor(ctx) -> int:
    """The sensor (1..4) the step's fields currently describe."""
    return int(ctx.values.get("rpm_sensor") or 1)


def rpm_parameter(suffix: str):
    """RPMn_<suffix> of the selected sensor."""
    return lambda ctx: f"RPM{selected_rpm_sensor(ctx)}_{suffix}"


def rpm_type(ctx) -> int:
    """Type chosen in the form for the selected sensor (the vehicle's value until chosen)."""
    return ctx.form_int("rpm_type", f"RPM{selected_rpm_sensor(ctx)}_TYPE")


def is_rpm_sensor_gpio(ctx) -> bool:
    """The selected sensor reads a GPIO pin."""
    return rpm_type(ctx) == RPM_TYPE_GPIO


def is_rpm_sensor_esc_telemetry(ctx) -> bool:
    """The selected sensor averages ESC telemetry."""
    return rpm_type(ctx) == RPM_TYPE_ESC_TELEMETRY


def apply_rpm(ctx):
    """A GPIO sensor on a servo output needs that output switched to GPIO."""
    output = int(ctx.number("rpm_servo_output", 0))
    if is_rpm_sensor_gpio(ctx) and output:
        assign_output_function(ctx, output, SERVO_FUNCTION_GPIO)


def action_show_rpm(ctx):
    """Log the first two sensors' readings once per second for 10 seconds (RPM message)."""
    for _ in range(LIVE_DISPLAY_SECONDS):
        reading = ctx.vehicle.latest("RPM", max_age=2.0)
        ctx.log(
            f"RPM 1: {reading.rpm1:.0f}   RPM 2: {reading.rpm2:.0f}"
            if reading is not None
            else "No RPM telemetry (sensor type set? rebooted?)"
        )
        ctx.sleep(1)


STEP = Step(
    "rpm",
    "RPM sensors",
    [
        value_field(
            "rpm_sensor",
            "Sensor to configure",
            kind="choice",
            choices=RPM_SENSOR_CHOICES,
            default=lambda ctx: ctx.values.get("rpm_sensor") or 1,
            reload_on_change=True,
        ),
        param_field(
            rpm_parameter("TYPE"),
            "Sensor type",
            key="rpm_type",
            kind="choice",
            choices=RPM_TYPE_CHOICES,
            reboot=True,
            refresh=True,
        ),
        param_field(
            rpm_parameter("PIN"), "GPIO pin number", key="rpm_pin", kind="int", reboot=True, when=is_rpm_sensor_gpio
        ),
        value_field(
            "rpm_servo_output",
            "Servo output used as this pin",
            kind="choice",
            choices=RELAY_PIN_OUTPUT_CHOICES,
            default=0,
            when=is_rpm_sensor_gpio,
        ),
        param_field(rpm_parameter("SCALING"), "Scaling", key="rpm_scaling", when=is_rpm_sensor_gpio),
        param_field(
            rpm_parameter("ESC_MASK"),
            "ESCs to average (bitmask)",
            key="rpm_esc_mask",
            kind="int",
            when=is_rpm_sensor_esc_telemetry,
        ),
        param_field(rpm_parameter("MIN"), "Minimum valid RPM", key="rpm_min"),
        param_field(rpm_parameter("MAX"), "Maximum valid RPM", key="rpm_max"),
        param_field(rpm_parameter("MIN_QUAL"), "Minimum signal quality", key="rpm_min_qual"),
    ],
    [Action("Show RPM (10 s)", action_show_rpm)],
    on_apply=apply_rpm,
    applies=has("has_rpm"),
)
