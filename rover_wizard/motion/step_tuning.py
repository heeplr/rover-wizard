"""rover_wizard.motion.step_tuning - tuning a controller (P, I and the steering's FF) from step responses.

Each round the vehicle follows a step of its target (speed or turn rate); step_response_metrics() measures how it
followed, GainTuner.next_gains() decides the gains for the next round. Pure functions and state only: no vehicle
access, no UI, no clock.

What makes it robust against real data:
* The target is what ArduPilot actually aimed for, when its controller reports it (PID_TUNING): ArduPilot limits the
  requested target first (ATC_STR_ACC_MAX, ATC_STR_RAT_MAX, ATC_TURN_MAX_G, ATC_ACCEL_MAX ...), and no gain can make
  the vehicle follow more than that.
* Overshoot is read from the smoothed signal, and overshoot or a steady-state error within the measurement noise
  counts as none: a gyro on a bumping vehicle is noisy, and single spikes are not the controller's doing.
* While the output is at its limit (full steering lock, full throttle) the target can't be reached at all: the
  gains are left alone.
* A steady-state error is corrected through FF where it can be (the output the vehicle needed, per unit achieved):
  ArduPilot's rate controllers rely on FF for it, I only builds up slowly within a short step. That measurement
  depends on the vehicle only, not on the gains, so every round near the target measures the same value - and FF is
  set to their average: a single round's noise doesn't make it swing.
* When measurement noise makes the controller's output jitter, an error filter (FLTE) well above the loop's
  bandwidth is proposed (suggest_error_filter).
* A gain whose change reverses the previous round's direction changes only half as much (in proportion) as before,
  so noisy rounds don't make the gains swing up and down; tuning ends once the changes have become small.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

# fewer samples inside a step than this -> no metrics
STEP_MIN_SAMPLES = 8
# "t63": time to reach 63 % of the target (one time constant)
STEP_RISE_FRACTION = 0.63
# steady-state value = mean over the last ≤ 2.5 s of the step ...
STEP_TAIL_MAX_SECONDS = 2.5
# ... or the last half of the step, whichever is shorter (long enough to average a noisy gyro)
STEP_TAIL_FRACTION = 0.5
# [s] the signal is averaged over this window (centred) before its peak and rise time are read
SMOOTHING_SECONDS = 0.3
# a deviation smaller than this many standard deviations of its measurement counts as noise
NOISE_SIGMAS = 2.0
# |output| at or above this (ArduPilot's controller outputs are -1..1): the controller is at its limit ...
SATURATED_OUTPUT = 0.95
# ... for more than this share of the steady part of the step
SATURATED_SHARE = 0.5

# rules (first match wins): thresholds on the step response, and (P, I) multipliers to apply.
# overshoot above 20 %: far too aggressive
LARGE_OVERSHOOT = 0.20
# overshoot of 10..20 %: slightly too aggressive
SMALL_OVERSHOOT = 0.10
# steady-state error above 10 %: FF or integrator too weak
LARGE_STEADY_ERROR = 0.10
# steady-state error of 5..10 %
SMALL_STEADY_ERROR = 0.05
# lower P and I clearly
LARGE_OVERSHOOT_FACTORS = (0.75, 0.6)
# lower P and I slightly
SMALL_OVERSHOOT_FACTORS = (0.85, 0.8)
# more proportional action
SLOW_RESPONSE_FACTORS = (1.3, 1.0)
# more integral action (and a little P to help) - when there is no FF to correct
LARGE_ERROR_FACTORS = (1.1, 1.5)
# a little more integral action
SMALL_ERROR_FACTORS = (1.0, 1.25)
# an FF correction changes FF by at most this factor (either way) per round
FF_MAX_FACTOR = 2.0
# each reversal of a gain's direction shrinks its changes (in log space, i.e. the exponent of its factor) by this
REVERSAL_DAMPING = 0.5
# [s] the P and D parts' jitter is measured around their average over this window
JITTER_WINDOW_SECONDS = 0.5
# the output may jitter by this much (one standard deviation, of full output) before an error filter is proposed
JITTER_MAX = 0.05
# an error filter this many times above the control loop's bandwidth hardly delays the control
FILTER_BANDWIDTH_FACTOR = 5
# [Hz] never proposed lower
ERROR_FILTER_MIN_HZ = 1.0
# a proposal must be below this fraction of the current filter to count
FILTER_CLEARLY_LOWER = 0.9
# changes of every gain within ±3 %: tuning has converged
CONVERGED_CHANGE = 0.03


@dataclass(frozen=True)
class StepResponse:
    """How a controlled quantity followed a step change of its target (all positive for a positive step)."""

    # fraction (of the target) by which the smoothed peak exceeded the target or the higher final value, 0 within
    # the noise
    overshoot: float
    # fraction of the target still missing at the end of the step (negative: settled above it), 0 within the noise
    steady_state_error: float
    # [s] time until the (smoothed) value reached 63 % of the target (None = never)
    rise_time_63: Optional[float]
    # mean value over the end of the step
    final_value: float
    # what the controller aimed for at the end of the step: ArduPilot's own target if reported, else the requested
    target: Optional[float] = None
    # standard deviation of the measurement noise
    noise: float = 0.0
    # the steady-state error as measured, even within the noise
    measured_error: float = 0.0
    # the controller's output was at its limit over the end of the step
    saturated: bool = False
    # mean controller output (FF + P + I + D) over the end of the step, None if not reported
    output: Optional[float] = None
    # how much the controller's P and D parts jitter over the end of the step (standard deviation, in output units
    # -1..1): noise in the measurement passed on to the actuator; None without the controller's reports
    jitter: Optional[float] = None


@dataclass(frozen=True)
class GainChange:
    """The gains for the next round, why, and whether tuning has ended."""

    # gain name ("p", "i", "ff") -> value
    gains: dict
    reason: str
    # tracking is good, or the changes have become small
    converged: bool = False
    # nothing the gains could improve (no data, output at its limit): stop tuning
    blocked: bool = False


def centred_average(times: list, values: list, window: float) -> list:
    """For every sample: the mean of the samples within ±window/2 seconds around it."""
    averages = []
    start = end = 0
    running = 0.0
    for index, time in enumerate(times):
        # extend the window to the right, then drop samples on the left
        while end < len(times) and times[end] <= time + window / 2:
            running += values[end]
            end += 1
        while times[start] < time - window / 2:
            running -= values[start]
            start += 1
        averages.append(running / (end - start))
    return averages


def step_response_metrics(
    times: list,
    values: list,
    target: float,
    step_on: float,
    step_off: float,
    desired: Optional[list] = None,
    outputs: Optional[list] = None,
    feedback: Optional[list] = None,
) -> Optional[StepResponse]:
    """Overshoot, steady-state error and rise time of `values` following a step to `target` between the two times.

    `desired` (ArduPilot's own target per sample), `outputs` (its controller output per sample) and `feedback` (the
    output's P + D part per sample) are optional, entries may be None.
    """
    # Mirror negative steps so the maths below only has to handle a positive target.
    direction = 1 if target > 0 else -1
    count = len(times)
    desired = desired or [None] * count
    outputs = outputs or [None] * count
    segment = [
        (time, value * direction, None if aim is None else aim * direction, None if out is None else out * direction)
        for time, value, aim, out in zip(times, values, desired, outputs)
        if step_on <= time <= step_off
    ]
    if len(segment) < STEP_MIN_SAMPLES or target == 0:
        return None

    # The steady part = the last 1.5 s or last 30 % of the step, whichever is shorter.
    tail_start = step_off - min(STEP_TAIL_MAX_SECONDS, STEP_TAIL_FRACTION * (step_off - step_on))
    tail = [entry for entry in segment if entry[0] >= tail_start]
    if len(tail) < 2:
        # the run ended (e.g. at the distance limit) before the end of the step: no steady state to judge
        return None

    aims = [aim for _, _, aim, _ in tail if aim is not None]
    aimed = sum(aims) / len(aims) if aims else abs(target)
    if aimed <= 0:
        return None

    segment_times = [time for time, _, _, _ in segment]
    raw = [value for _, value, _, _ in segment]
    smoothed = centred_average(segment_times, raw, SMOOTHING_SECONDS)
    # noise = what smoothing removed (its standard deviation), from the steady part
    residuals = [value - average for (time, value, _, _), average in zip(segment, smoothed) if time >= tail_start]
    noise = math.sqrt(sum(residual**2 for residual in residuals) / len(residuals))
    per_window = max(1, len(segment) * SMOOTHING_SECONDS / max(step_off - step_on, SMOOTHING_SECONDS))
    # The highest of n independent noise values lies about sqrt(2 ln n) standard deviations above their mean: noise
    # alone makes the smoothed peak that much higher (n = independent windows in the step), plus NOISE_SIGMAS.
    windows = max(2.0, (step_off - step_on) / SMOOTHING_SECONDS)
    peak_band = (math.sqrt(2 * math.log(windows)) + NOISE_SIGMAS) * noise / math.sqrt(per_window)

    final_value = sum(value for _, value, _, _ in tail) / len(tail)
    # Overshoot = how far the smoothed peak went past the target - or past the final value if the vehicle settled
    # above the target (that is a steady-state error, not a controller ringing) - unless within the smoothed noise.
    excess = max(smoothed) - max(aimed, final_value)
    overshoot = excess / aimed if excess > peak_band else 0.0
    # Steady-state error = fraction of the target missing at the end, unless within the noise of the mean.
    missing = aimed - final_value
    measured_error = missing / aimed
    steady_state_error = measured_error if abs(missing) > NOISE_SIGMAS * noise / math.sqrt(len(tail)) else 0.0
    # Rise time = seconds until the smoothed value first reached 63 % of the target (one first-order time constant).
    rise_time = next(
        (time - step_on for time, value in zip(segment_times, smoothed) if value >= STEP_RISE_FRACTION * aimed), None
    )
    tail_outputs = [out for _, _, _, out in tail if out is not None]
    saturated = bool(tail_outputs) and (
        sum(abs(out) >= SATURATED_OUTPUT for out in tail_outputs) > SATURATED_SHARE * len(tail_outputs)
    )

    return StepResponse(
        overshoot=overshoot,
        steady_state_error=steady_state_error,
        rise_time_63=rise_time,
        final_value=final_value,
        target=aimed,
        noise=noise,
        measured_error=measured_error,
        saturated=saturated,
        output=sum(tail_outputs) / len(tail_outputs) if tail_outputs else None,
        jitter=_jitter(times, feedback, tail_start, step_off) if feedback else None,
    )


def _jitter(times: list, feedback: list, start: float, end: float) -> Optional[float]:
    """Standard deviation of the feedback parts around their own (centred) average between start and end."""
    pairs = []
    for time, part in zip(times, feedback):
        # each report once: samples come faster than the reports, and a repeated one would hide the jitter
        if part is not None and start <= time <= end and (not pairs or part != pairs[-1][1]):
            pairs.append((time, part))
    if len(pairs) < STEP_MIN_SAMPLES:
        return None
    pair_times = [time for time, _ in pairs]
    parts = [part for _, part in pairs]
    averages = centred_average(pair_times, parts, JITTER_WINDOW_SECONDS)
    return math.sqrt(sum((part - average) ** 2 for part, average in zip(parts, averages)) / len(parts))


def suggest_error_filter(response: Optional[StepResponse], p_gain: float, current_hz: float) -> tuple:
    """(error filter frequency [Hz] to propose or None, why) when noise makes the controller's output jitter.

    The jitter is the controller's own (from its reports), else estimated as P x measurement noise. Above
    JITTER_MAX, an error filter (FLTE) at FILTER_BANDWIDTH_FACTOR times the loop's bandwidth (from the measured rise
    time: 1 / (2 pi t63)) takes out the noise while delaying the control hardly at all - a lower filter would add
    lag the loop notices. Proposed only if it is clearly below the current filter.
    """
    if response is None or response.rise_time_63 is None:
        return None, ""
    jitter = response.jitter if response.jitter is not None else p_gain * response.noise
    if jitter <= JITTER_MAX:
        return None, ""
    bandwidth = 1 / (2 * math.pi * response.rise_time_63)
    proposal = max(ERROR_FILTER_MIN_HZ, round(FILTER_BANDWIDTH_FACTOR * bandwidth, 1))
    shown = f"the output jitters by ±{jitter:.0%} from measurement noise"
    if current_hz and proposal >= current_hz * FILTER_CLEARLY_LOWER:
        return None, f"{shown}, and the error filter is already at {current_hz:g} Hz: lower P, or reduce vibration"
    return proposal, f"{shown} -> error filter {proposal:g} Hz"


class GainTuner:
    """Decides a controller's gains round by round from its step responses (see the module docstring)."""

    def __init__(self, gains: dict, slow_rise_time: float) -> None:
        """Start from the current gains ("p", "i" and, if FF is to be corrected, "ff"); a response slower than
        slow_rise_time [s] to 63 % counts as too slow."""
        self.gains = dict(gains)
        self.slow_rise_time = slow_rise_time
        # gain -> direction of its last change (+1 up, -1 down) and how often its direction reversed
        self._last_direction: dict = {}
        self._reversals: dict = {}
        # what FF should be, as measured by each round near the target (the FF correction uses their average)
        self._ff_estimates: list = []

    def next_gains(self, response: Optional[StepResponse]) -> GainChange:
        """The gains for the next round (also kept as self.gains)."""
        if response is None:
            return GainChange(dict(self.gains), "no usable data", blocked=True)
        if response.saturated:
            return GainChange(
                dict(self.gains),
                "the output was at its limit (full steering lock / full throttle): the target can't be reached, "
                "whatever the gains - lower the step target",
                blocked=True,
            )

        self._note_ff_estimate(response)
        if response.overshoot > LARGE_OVERSHOOT:
            return self._scale(LARGE_OVERSHOOT_FACTORS, f"overshoot {response.overshoot:.0%} -> lower P and I")
        if response.overshoot > SMALL_OVERSHOOT:
            return self._scale(SMALL_OVERSHOOT_FACTORS, f"overshoot {response.overshoot:.0%} -> slightly lower P and I")
        error = response.steady_state_error
        # FF first: with too little FF the vehicle also gets to the target late, which more P wouldn't fix. The
        # correction measures what FF should be, so it may act on an error within the noise: a right FF changes
        # little (and noise-driven back-and-forth is damped), which ends tuning.
        has_ff = "ff" in self.gains and response.output is not None and response.final_value > 0
        if has_ff and abs(response.measured_error) > SMALL_STEADY_ERROR:
            return self._correct_ff(response)
        # Never reached 63 % of the target, or too late: not enough proportional action.
        if response.rise_time_63 is None or response.rise_time_63 > self.slow_rise_time:
            return self._scale(SLOW_RESPONSE_FACTORS, "response too slow -> raise P")

        if abs(error) > SMALL_STEADY_ERROR:
            factors = LARGE_ERROR_FACTORS if abs(error) > LARGE_STEADY_ERROR else SMALL_ERROR_FACTORS
            more = "raise I" if abs(error) > LARGE_STEADY_ERROR else "slightly raise I"
            return self._scale(factors, f"steady-state error {error:.0%} -> {more}")

        return GainChange(dict(self.gains), "tracking is good", converged=True)

    def _note_ff_estimate(self, response: StepResponse) -> None:
        """Keep the FF this round measured (the output per unit achieved) - if it got near the target: far from it,
        the vehicle turns (or drives) at another operating point, which needn't take the same FF."""
        near = abs(response.measured_error) <= LARGE_STEADY_ERROR
        if "ff" in self.gains and response.output is not None and response.final_value > 0 and near:
            self._ff_estimates.append(response.output / response.final_value)

    def _correct_ff(self, response: StepResponse) -> GainChange:
        """FF = the output the vehicle needed per unit it achieved, so FF alone delivers it next time - averaged
        over the rounds near the target once there are any (each round's measurement is noisy)."""
        old = self.gains["ff"]
        needed, measured = response.output / response.final_value, "the output the turn needed"
        if self._ff_estimates and abs(response.measured_error) <= LARGE_STEADY_ERROR:
            needed = sum(self._ff_estimates) / len(self._ff_estimates)
            if len(self._ff_estimates) > 1:
                measured += f", average of {len(self._ff_estimates)} rounds"
        new = max(old / FF_MAX_FACTOR, min(old * FF_MAX_FACTOR, needed)) if old > 0 else needed
        return self._apply(
            {"ff": new / old if old > 0 else 1.0},
            f"steady-state error {response.measured_error:.0%} -> FF {old:.3g} -> {new:.3g} ({measured})",
            absolute={"ff": new} if old <= 0 else None,
        )

    def _scale(self, factors: tuple, reason: str) -> GainChange:
        """Multiply P and I by the rule's factors (damped, see _apply)."""
        p_factor, i_factor = factors
        return self._apply({"p": p_factor, "i": i_factor}, reason)

    def _apply(self, factors: dict, reason: str, absolute: Optional[dict] = None) -> GainChange:
        """Apply multipliers to the gains; a gain whose direction reverses changes less from now on (in log space,
        halved with every reversal). Converged once every change is within CONVERGED_CHANGE."""
        largest_change = 0.0
        for name, factor in factors.items():
            if factor == 1.0:
                continue
            direction = 1 if factor > 1 else -1
            previous = self._last_direction.get(name)
            if previous is not None and previous != direction:
                self._reversals[name] = self._reversals.get(name, 0) + 1
            self._last_direction[name] = direction
            damped = factor ** (REVERSAL_DAMPING ** self._reversals.get(name, 0))
            self.gains[name] *= damped
            largest_change = max(largest_change, abs(damped - 1))
        for name, value in (absolute or {}).items():
            self.gains[name] = value
            largest_change = 1.0
        if largest_change < CONVERGED_CHANGE:
            return GainChange(dict(self.gains), reason + " - the changes have become small", converged=True)
        return GainChange(dict(self.gains), reason)
