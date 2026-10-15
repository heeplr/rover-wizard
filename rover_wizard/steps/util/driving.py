"""rover_wizard.steps.util.driving - the steps that drive the vehicle: who drives, the free space, safety margins,
settling."""

from __future__ import annotations

from ...model import Field, value_field

WHO_DRIVES_CHOICES = [("Wizard drives automatically", "auto"), ("I drive with my transmitter (Manual mode)", "manual")]


def who_drives_field(key: str, default: str = "auto") -> Field:
    """The 'Who drives?' choice (automatic vs manual) offered by every step that drives the vehicle."""
    return value_field(key, "Who drives?", kind="choice", default=default, choices=WHO_DRIVES_CHOICES)


# Safety margins: proposed limits are a bit below what the vehicle could just barely do.
# for top-speed-derived limits, turn rates and feed-forward
LIMIT_SAFETY_FACTOR = 0.9


# straight runs use at most 60 % of the runway (the rest is for stopping)
RUNWAY_USE_FRACTION = 0.6


# free straight distance
DEFAULT_RUNWAY_M = 30.0


# free circle for the turning runs (to the vehicle's right; see the Pre-drive step)
DEFAULT_CIRCLE_DIAMETER_M = 20.0


# proposed gains are rounded to 4 decimal places
GAIN_DECIMALS = 4


def _is_speed_settled(samples: list, window: float, max_spread: float, min_run_seconds: float) -> bool:
    """Has |speed| stayed within `max_spread` (fraction of the current speed) for the last `window` seconds?"""
    # Too early to judge: the vehicle is still accelerating anyway.
    if samples[-1].time - samples[0].time < min_run_seconds:
        return False
    tail = [abs(sample.forward_speed) for sample in samples if samples[-1].time - sample.time <= window]
    # Spread relative to the speed itself (at least 0.2 m/s so a crawling vehicle doesn't need absurd precision).
    return max(tail) - min(tail) <= max_spread * max(max(tail), 0.2)
