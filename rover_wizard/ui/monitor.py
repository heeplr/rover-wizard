"""rover_wizard.ui.monitor - text for the side panel (RC channel, servo output, accelerometer and gyro bargraphs,
compass tape), no widgets."""

from __future__ import annotations

from typing import Optional

from ..ardupilot_defs import RC_PWM_NOMINAL_MAX, RC_PWM_NOMINAL_MIN
from ..mavlink_defs import UINT16_MAX

# U+2581..U+2588: eight bar heights
BAR_LEVELS = "▁▂▃▄▅▆▇█"
# characters per channel column (fits two-digit channel numbers and the +/- buttons)
COLUMN_WIDTH = 3
NO_RC_TEXT = "(no RC_CHANNELS data - check transmitter and receiver)"
NO_SERVO_TEXT = "(no SERVO_OUTPUT_RAW data)"


def bar_for_fraction(fraction: float) -> str:
    """One bar character for a position 0.0 (lowest bar) .. 1.0 (highest bar); values outside are clamped."""
    # scaled to the eight bar heights and clamped
    level = max(0, min(len(BAR_LEVELS) - 1, int(fraction * len(BAR_LEVELS))))
    return BAR_LEVELS[level]


def bar_for_pwm(pwm: int) -> str:
    """One bar character for a pulse width; blank if the channel/output isn't active."""
    if not pwm or pwm >= UINT16_MAX:
        return " "

    # position of the pulse inside the nominal range, 0.0 (1000 us) .. 1.0 (2000 us)
    return bar_for_fraction((pwm - RC_PWM_NOMINAL_MIN) / (RC_PWM_NOMINAL_MAX - RC_PWM_NOMINAL_MIN))


def bar_for_signed(value: float, full_scale: float) -> str:
    """One bar character for a value -full_scale (lowest) .. +full_scale (highest); zero is half height."""
    # -full_scale .. +full_scale -> 0.0 .. 1.0
    return bar_for_fraction((value + full_scale) / (2 * full_scale))


def label_row(labels) -> str:
    """Column labels (channel numbers, axis names), each centred in its column."""
    return "".join(f"{label:^{COLUMN_WIDTH}}" for label in labels)


def number_row(count: int) -> str:
    """Channel/output numbers 1..count, each centred in its column."""
    return label_row(range(1, count + 1))


def column_row(bars) -> str:
    """One centred bar character per column."""
    return "".join(f"{bar:^{COLUMN_WIDTH}}" for bar in bars)


def bar_row(pwm_values: list) -> str:
    """One centred bar per pulse width."""
    return column_row(bar_for_pwm(pwm) for pwm in pwm_values)


def rc_bar_text(raw_pwm: Optional[list]) -> str:
    """Numbers and bars for every channel the receiver delivers (its channel count, not a fixed number)."""
    if not raw_pwm:
        return NO_RC_TEXT

    return number_row(len(raw_pwm)) + "\n" + bar_row(raw_pwm)


def servo_bar_text(raw_pwm: Optional[list], count: int) -> str:
    """Bars for servo outputs 1..count (the numbers and +/- buttons are separate rows in the panel)."""
    if raw_pwm is None:
        return NO_SERVO_TEXT

    # Pad with zeros (inactive outputs show blank), then cut to exactly `count` outputs.
    return bar_row((list(raw_pwm) + [0] * count)[:count])


# ---- compass tape --------------------------------------------------------------------------------------------------
# characters in the tape; odd, so the heading sits exactly in the middle column
COMPASS_TAPE_WIDTH = 47
# degrees per character (must divide 360, so the tape wraps around north seamlessly)
DEGREES_PER_CELL = 3
# a "╷" every 10 degrees (on the nearest cell), a "." on the cells between
COMPASS_TICK_DEGREES = 10
COMPASS_TICK = "╷"
COMPASS_HALF_TICK = "."
COMPASS_INDICATOR = "▲"
FULL_CIRCLE_DEGREES = 360
COMPASS_POINTS = {0: "N", 45: "NE", 90: "E", 135: "SE", 180: "S", 225: "SW", 270: "W", 315: "NW"}
NO_HEADING_TEXT = "(no ATTITUDE data)"
CELLS_PER_CIRCLE = FULL_CIRCLE_DEGREES // DEGREES_PER_CELL


def _nearest_cell(degrees: float) -> int:
    """The cell (0..CELLS_PER_CIRCLE-1) an angle is drawn on: the nearest one, so marks needn't fall on a cell."""
    return round(degrees / DEGREES_PER_CELL) % CELLS_PER_CIRCLE


# the cells that show a tick, and the cells where a compass point starts
TICK_CELLS = {_nearest_cell(degrees) for degrees in range(0, FULL_CIRCLE_DEGREES, COMPASS_TICK_DEGREES)}
POINT_CELLS = {_nearest_cell(degrees): point for degrees, point in COMPASS_POINTS.items()}


def compass_tape_text(heading: Optional[float]) -> str:
    """A scrolling compass tape with the heading [deg] in the middle, and a "▲" under it on the second line."""
    if heading is None:
        return NO_HEADING_TEXT

    half = COMPASS_TAPE_WIDTH // 2
    # the cell nearest to the heading sits in the middle column
    centre = _nearest_cell(heading)
    cells = [
        COMPASS_TICK if (centre + column) % CELLS_PER_CIRCLE in TICK_CELLS else COMPASS_HALF_TICK
        for column in range(-half, half + 1)
    ]
    # Compass points start at their own cell; a two-letter point covers the cell after it.
    for column in range(-half, half + 1):
        point = POINT_CELLS.get((centre + column) % CELLS_PER_CIRCLE)
        for offset, letter in enumerate(point or ""):
            if column + offset <= half:
                cells[column + offset + half] = letter

    return "".join(cells) + "\n" + " " * half + COMPASS_INDICATOR


# ---- accelerometer and gyro bargraphs (same columns as the RC and servo bars) --------------------------------------
AXES = "XYZ"
# full bar height = this many g / degrees per second (half height = 0)
ACCEL_FULL_SCALE_G = 2.0
GYRO_FULL_SCALE_DEG_S = 180.0
NO_IMU_TEXT = "(no SCALED_IMU data)"


def signed_bar_text(values: Optional[tuple], full_scale: float) -> str:
    """Axis names and one bar per axis (X, Y, Z); half height is zero."""
    if values is None:
        return NO_IMU_TEXT

    return label_row(AXES) + "\n" + column_row(bar_for_signed(value, full_scale) for value in values)


def accel_bars_text(accel: Optional[tuple]) -> str:
    """Accelerometer X/Y/Z [g] as bars."""
    return signed_bar_text(accel, ACCEL_FULL_SCALE_G)


def gyro_bars_text(gyro: Optional[tuple]) -> str:
    """Gyro X/Y/Z [deg/s] as bars."""
    return signed_bar_text(gyro, GYRO_FULL_SCALE_DEG_S)
