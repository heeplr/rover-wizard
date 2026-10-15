"""rover_wizard.util.timebase - the wizard's single source of time.

Every wait and timestamp in the wizard goes through this module so the self-test can run the simulator faster
than real time by changing one factor. Other modules must never read the factor themselves - they call now(),
sleep() or wait_for() - because `from timebase import ...` of a plain variable would copy its value once and
silently ignore later changes.
"""

from __future__ import annotations

import threading
import time

# 1.0 = real time; the self-test raises it to fast-forward the simulator
_time_scale = 1.0


def set_time_scale(factor: float) -> None:
    """Make wizard time run `factor` times faster than real time (tests/simulator only)."""
    global _time_scale
    _time_scale = float(factor)


def now() -> float:
    """Current time in seconds on the wizard's (possibly fast-forwarded) clock."""
    # Real seconds x scale: with scale 8, eight wizard-seconds pass per real second. Monotonic: a clock set by NTP
    # (e.g. a companion computer right after boot) must not make telemetry look stale or stall the waits.
    return time.monotonic() * _time_scale


def sleep(seconds: float) -> None:
    """Pause for `seconds` of wizard time."""
    # Wizard seconds / scale = real seconds to wait (negative durations are treated as 0).
    time.sleep(max(0.0, seconds) / _time_scale)


def wait_for(event: threading.Event, seconds: float) -> bool:
    """Wait up to `seconds` of wizard time for `event`; returns True if it was set."""
    # Same conversion as sleep(): wizard seconds / scale = real seconds.
    return event.wait(max(0.0, seconds) / _time_scale)
