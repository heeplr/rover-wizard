"""rover_wizard.motion.analysis - turns recorded drive-test samples into vehicle characteristics and tuning suggestions.

Pure functions only: no vehicle access, no UI, no clock. Inputs are lists of DriveSample (see vehicle/link.py); outputs
are small result dataclasses (closed-loop tuning from step responses is in step_tuning). Thresholds are named module constants, so their meaning is documented in one place.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    # only for type hints: the analyses don't need the vehicle link
    from ..vehicle.link import DriveSample


# --------------------------------------------------------------------------- thresholds
# fewer samples than this are not a usable run
MIN_SAMPLES_FOR_RUN_ANALYSIS = 20
# throttle input above this counts as "accelerating hard"
FULL_THROTTLE_THRESHOLD = 0.5
# throttle input below this counts as "coasting / braking"
COASTING_THROTTLE_THRESHOLD = 0.1
# [m/s] slower than this the vehicle is treated as stopped
MIN_MOVING_SPEED = 0.3
# length of the "is the top speed steady?" check window
STEADY_SPEED_TAIL_SECONDS = 2.0
# speed may vary by at most 8 % of top speed in that window
STEADY_SPEED_MAX_SPREAD = 0.08
# [m/s] floor for the speed the spread is compared with: keeps it meaningful for very slow vehicles
STEADY_SPEED_MIN_REFERENCE = 0.1
# |steering| above this counts as full steering lock
FULL_LOCK_THRESHOLD = 0.8
# [m/s] minimum speed for a meaningful radius = speed / yaw rate
TURN_MIN_SPEED = 0.15
# [rad/s] minimum turn rate for a meaningful radius
TURN_MIN_YAW_RATE = math.radians(8)
# discard the first 1/3 of radius samples (turn still tightening)
TURN_SETTLING_FRACTION = 3
# need at least this many radius samples for a median
MIN_RADIUS_SAMPLES = 5
# |steering| above this marks "the turn has started"
STEER_ONSET_THRESHOLD = 0.5
# the steady-speed window needs more samples than this
MIN_STEADY_TAIL_SAMPLES = 5
# [rad/s] slower turns are too noisy for the steering feed-forward
FEEDFORWARD_MIN_YAW_RATE = math.radians(10)
# smaller steering inputs are ignored for the feed-forward
FEEDFORWARD_MIN_STEERING = 0.1
# smaller throttle inputs are ignored for the speed feed-forward
FEEDFORWARD_MIN_THROTTLE = 0.2
# [s] window for measuring how fast the turn rate builds up
STEER_ACCEL_WINDOW = 0.4
# only look this long after the turn started
STEER_ACCEL_SEARCH_SECONDS = 2.0
# [s] window used to decide whether a signal is steady
STEADY_WINDOW = 1.0
# [rad/s^2] yaw rate changing slower than this counts as steady
STEADY_YAW_RATE_SLOPE = 0.6
# [m/s^2] speed changing slower than this counts as steady
STEADY_SPEED_SLOPE = 0.3


# --------------------------------------------------------------------------- result types
@dataclass(frozen=True)
class TopSpeedResult:
    """Outcome of a straight-line full-throttle run."""

    # [m/s] highest 1-second average speed
    top_speed: float
    # [m/s^2] steepest 1-second speed increase at full throttle
    max_acceleration: Optional[float]
    # [m/s^2] steepest 1-second speed decrease while coasting
    max_deceleration: Optional[float]
    # False: speed was still rising, top_speed is only a lower bound
    speed_was_steady: bool


@dataclass(frozen=True)
class TurnResult:
    """Outcome of a full-lock turning run."""

    # [m] median turning radius at full lock
    radius: Optional[float]
    # [rad/s] highest 0.5-second average turn rate
    max_yaw_rate: float
    # [rad/s^2] how fast the turn rate built up
    max_yaw_acceleration: Optional[float]


@dataclass(frozen=True)
class SteeringFeedforwardResult:
    """Steering input needed per unit of turn rate, from steady turning."""

    # [steering input per rad/s]
    feedforward: Optional[float]
    # how many steady samples it is based on
    sample_count: int


@dataclass(frozen=True)
class SpeedFeedforwardResult:
    """Throttle needed per unit of speed, from steady straight driving."""

    # [m/s] mean steady speed
    cruise_speed: Optional[float]
    # mean throttle input (0..1) at that speed
    cruise_throttle: Optional[float]
    # [throttle per m/s]
    throttle_per_speed: Optional[float]
    sample_count: int


# --------------------------------------------------------------------------- signal helpers
def moving_average(times: list, values: list, window: float) -> list:
    """Trailing moving average of `values` over the last `window` seconds (irregular sample times allowed)."""
    averages = []
    window_start = 0
    running_sum = 0.0
    for index, value in enumerate(values):
        # Add the newest sample to the running sum.
        running_sum += value
        # Drop samples from the front until the window spans at most `window` seconds.
        while times[index] - times[window_start] > window:
            running_sum -= values[window_start]
            window_start += 1
        # Average = sum / number of samples currently inside the window.
        averages.append(running_sum / (index - window_start + 1))

    return averages


def window_slopes(times: list, values: list, window: float) -> list:
    """Average rate of change over ~`window` seconds starting at every sample, as (start, end, slope) tuples."""
    slopes = []
    window_end = 0
    for window_start in range(len(times)):
        # The window end never moves backwards, so the whole scan is linear in the number of samples.
        window_end = max(window_end, window_start)
        # Extend the end until the window covers `window` seconds (or the data ends).
        while window_end < len(times) - 1 and times[window_end] - times[window_start] < window:
            window_end += 1
        duration = times[window_end] - times[window_start]
        # Only keep windows that cover at least 80 % of the requested length (the last ones near the end are short).
        if duration >= 0.8 * window:
            # Slope = change in value / elapsed time.
            slopes.append((window_start, window_end, (values[window_end] - values[window_start]) / duration))

    return slopes


def steady_mask(
    times: list, values: list, window: float = STEADY_WINDOW, max_slope: float = STEADY_SPEED_SLOPE
) -> list:
    """For every sample: does it lie inside at least one window where `values` changed slower than `max_slope`?"""
    is_steady = [False] * len(times)
    for window_start, window_end, slope in window_slopes(times, values, window):
        # A window with a small absolute slope is "steady"; mark every sample it contains.
        if abs(slope) <= max_slope:
            for index in range(window_start, window_end + 1):
                is_steady[index] = True

    return is_steady


def _series(samples: list[DriveSample], attribute: str) -> tuple:
    """(seconds since the first sample, values) for one DriveSample attribute; missing values count as 0."""
    start = samples[0].time
    times = [sample.time - start for sample in samples]
    values = [getattr(sample, attribute) if getattr(sample, attribute) is not None else 0.0 for sample in samples]
    return times, values


# --------------------------------------------------------------------------- run analyses
def analyze_top_speed_run(samples: list[DriveSample]) -> Optional[TopSpeedResult]:
    """Top speed, acceleration and deceleration from a straight full-throttle run followed by coasting."""
    if len(samples) < MIN_SAMPLES_FOR_RUN_ANALYSIS:
        return None

    times, speeds = _series(samples, "forward_speed")
    # direction doesn't matter for a top speed
    speeds = [abs(speed) for speed in speeds]
    _, throttle = _series(samples, "throttle_input")

    # Top speed = highest 1-second average (averaging removes single noisy samples).
    averaged = moving_average(times, speeds, 1.0)
    top_speed = max(averaged)
    peak_index = averaged.index(top_speed)

    # Acceleration = steepest 1-second speed increase while throttle was fully applied at both window ends.
    slopes = window_slopes(times, speeds, 1.0)
    accelerations = [
        slope
        for start, end, slope in slopes
        if throttle[start] > FULL_THROTTLE_THRESHOLD and throttle[end] > FULL_THROTTLE_THRESHOLD
    ]
    # Deceleration = steepest 1-second speed *decrease* after the peak, while coasting and still clearly moving.
    decelerations = [
        -slope
        for start, end, slope in slopes
        if start > peak_index and throttle[start] < COASTING_THROTTLE_THRESHOLD and speeds[start] > MIN_MOVING_SPEED
    ]

    # "Steady" check: during the last 2 s of full throttle, speed must stay within 8 % of top speed; otherwise the
    # vehicle was still accelerating when the run ended and the measured top speed is only a lower bound.
    full_throttle_indices = [index for index, value in enumerate(throttle) if value > FULL_THROTTLE_THRESHOLD]
    speed_was_steady = False
    if full_throttle_indices:
        full_throttle_end = times[full_throttle_indices[-1]]
        tail = [
            speeds[index]
            for index in range(len(speeds))
            if full_throttle_end - STEADY_SPEED_TAIL_SECONDS <= times[index] <= full_throttle_end
        ]
        spread = max(tail) - min(tail) if tail else math.inf
        speed_was_steady = len(tail) > MIN_STEADY_TAIL_SAMPLES and spread <= STEADY_SPEED_MAX_SPREAD * max(
            top_speed, STEADY_SPEED_MIN_REFERENCE
        )

    return TopSpeedResult(
        top_speed=top_speed,
        max_acceleration=max(accelerations) if accelerations else None,
        max_deceleration=max(decelerations) if decelerations else None,
        speed_was_steady=speed_was_steady,
    )


def analyze_turn_run(samples: list[DriveSample], full_lock: float = FULL_LOCK_THRESHOLD) -> Optional[TurnResult]:
    """Turning radius, maximum turn rate and turn-rate build-up from a full-lock turning run."""
    if len(samples) < MIN_SAMPLES_FOR_RUN_ANALYSIS:
        return None
    times, yaw_rates = _series(samples, "yaw_rate")
    _, speeds = _series(samples, "forward_speed")
    _, steering = _series(samples, "steering_input")
    # left and right turns count the same
    abs_yaw_rates = [abs(rate) for rate in yaw_rates]

    # Maximum turn rate = highest 0.5-second average of |yaw rate|.
    max_yaw_rate = max(moving_average(times, abs_yaw_rates, 0.5))

    # Radius of a circle driven at speed v with turn rate ω is r = v / ω. Only use samples at full lock that are
    # moving and turning clearly, so noise near zero can't produce absurd radii.
    radii = [
        abs(speeds[index]) / abs_yaw_rates[index]
        for index in range(len(samples))
        if abs(steering[index]) >= full_lock
        and abs(speeds[index]) > TURN_MIN_SPEED
        and abs_yaw_rates[index] > TURN_MIN_YAW_RATE
    ]
    # The first third is the vehicle still tightening into the circle - skip it.
    radii = radii[len(radii) // TURN_SETTLING_FRACTION :]
    # The median ignores outliers (bumps, wheel slip) better than a mean.
    radius = statistics.median(radii) if len(radii) >= MIN_RADIUS_SAMPLES else None

    # Turn-rate build-up (steering acceleration): steepest 0.4-second increase of |yaw rate| in the first 2 s
    # after the steering was applied (starting two samples early so the onset itself is included).
    max_yaw_acceleration = None
    turn_start = next((index for index in range(len(samples)) if abs(steering[index]) >= STEER_ONSET_THRESHOLD), None)
    if turn_start is not None:
        build_ups = [
            slope
            for start, end, slope in window_slopes(times, abs_yaw_rates, STEER_ACCEL_WINDOW)
            if start >= max(0, turn_start - 2) and times[start] - times[turn_start] < STEER_ACCEL_SEARCH_SECONDS
        ]
        max_yaw_acceleration = max(build_ups) if build_ups else None

    return TurnResult(radius=radius, max_yaw_rate=max_yaw_rate, max_yaw_acceleration=max_yaw_acceleration)


def analyze_steering_feedforward(
    samples: list[DriveSample],
    min_yaw_rate: float = FEEDFORWARD_MIN_YAW_RATE,
    min_steering: float = FEEDFORWARD_MIN_STEERING,
) -> SteeringFeedforwardResult:
    """Steering feed-forward = steering input needed per rad/s of turn rate, from steady turning samples."""
    times, yaw_rates = _series(samples, "yaw_rate")
    _, steering = _series(samples, "steering_input")
    # Only samples where the turn rate had settled describe the steady relationship steering -> turn rate.
    is_steady = steady_mask(times, yaw_rates, STEADY_WINDOW, STEADY_YAW_RATE_SLOPE)
    steering_sum = yaw_rate_sum = 0.0
    count = 0
    for index in range(len(samples)):
        # Ignore near-zero steering/turn rate: dividing tiny numbers amplifies noise.
        if is_steady[index] and abs(steering[index]) >= min_steering and abs(yaw_rates[index]) >= min_yaw_rate:
            steering_sum += abs(steering[index])
            yaw_rate_sum += abs(yaw_rates[index])
            count += 1

    # Ratio of sums (not mean of ratios): weights large, reliable turns more than small, noisy ones.
    feedforward = steering_sum / yaw_rate_sum if yaw_rate_sum > 0 else None
    return SteeringFeedforwardResult(feedforward=feedforward, sample_count=count)


def analyze_speed_feedforward(
    samples: list[DriveSample], min_speed: float = MIN_MOVING_SPEED, min_throttle: float = FEEDFORWARD_MIN_THROTTLE
) -> SpeedFeedforwardResult:
    """Cruise speed/throttle and throttle needed per m/s, from steady straight driving."""
    times, speeds = _series(samples, "forward_speed")
    _, throttle = _series(samples, "throttle_input")
    # Only samples where the speed had settled show the steady relationship throttle -> speed.
    is_steady = steady_mask(times, speeds, STEADY_WINDOW, STEADY_SPEED_SLOPE)
    speed_sum = throttle_sum = 0.0
    count = 0
    for index in range(len(samples)):
        if is_steady[index] and throttle[index] >= min_throttle and abs(speeds[index]) >= min_speed:
            speed_sum += abs(speeds[index])
            throttle_sum += throttle[index]
            count += 1

    if count == 0:
        return SpeedFeedforwardResult(None, None, None, 0)
    # Mean steady speed and throttle; their ratio is the throttle needed per m/s (ratio of sums, see above).

    return SpeedFeedforwardResult(
        cruise_speed=speed_sum / count,
        cruise_throttle=throttle_sum / count,
        throttle_per_speed=throttle_sum / speed_sum,
        sample_count=count,
    )
