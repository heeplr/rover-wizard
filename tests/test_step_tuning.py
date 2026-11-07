"""Tuning a controller from step responses: the measurements (noise, ArduPilot's own target, saturation) and the
round-by-round gain decisions."""

import math
import random
from dataclasses import replace

import pytest

from rover_wizard.motion.step_tuning import (
    CONVERGED_CHANGE,
    FF_MAX_FACTOR,
    GainTuner,
    StepResponse,
    centred_average,
    step_response_metrics,
    suggest_error_filter,
)

from util.helpers import check

# a step from 1 s to 6 s, sampled at 20 Hz
STEP_ON, STEP_OFF, HZ = 1.0, 6.0, 20
TIMES = [index / HZ for index in range(int(7 * HZ))]


def first_order(target, tau=0.3, final_fraction=1.0, overshoot=0.0):
    """Values following a step to target * final_fraction with time constant tau (and a decaying overshoot bump)."""
    values = []
    for time in TIMES:
        if time < STEP_ON or time > STEP_OFF:
            values.append(0.0)
            continue
        elapsed = time - STEP_ON
        rise = 1 - math.exp(-elapsed / tau)
        bump = overshoot * math.exp(-((elapsed - 3 * tau) ** 2) / (2 * tau**2))
        values.append(target * (final_fraction * rise + bump))
    return values


def metrics(values, target=1.0, **kwargs):
    """The step response of these values."""
    return step_response_metrics(TIMES, values, target, STEP_ON, STEP_OFF, **kwargs)


def response(overshoot=0.0, error=0.0, rise_time=0.5, final_value=1.0, saturated=False, output=None):
    """A step response with these properties."""
    return StepResponse(
        overshoot=overshoot,
        steady_state_error=error,
        rise_time_63=rise_time,
        final_value=final_value,
        target=1.0,
        measured_error=error,
        saturated=saturated,
        output=output,
    )


# ---------------------------------------------------------------------------------------------- measurements
def test_a_clean_step_has_no_overshoot_and_no_error():
    result = metrics(first_order(1.0))
    assert result.overshoot == 0 and result.steady_state_error == pytest.approx(0, abs=0.01)
    # 63 % after one time constant (0.3 s), give or take the smoothing and a sample
    assert result.rise_time_63 == pytest.approx(0.3, abs=0.1)


def test_a_real_overshoot_is_measured():
    result = metrics(first_order(1.0, overshoot=0.3))
    check("overshoot found", 0.15 < result.overshoot < 0.35, result.overshoot)


def test_noise_spikes_are_not_overshoot_and_a_small_error_within_the_noise_is_none():
    noise = random.Random(1)
    # a gyro on a bumpy vehicle: ±0.1 noise and two single spikes on a step to 1.0 that settles 1 % short
    values = [value + noise.gauss(0, 0.1) for value in first_order(1.0, final_fraction=0.99)]
    values[60] += 0.5
    values[90] += 0.6
    result = metrics(values)
    check("spikes aren't overshoot", result.overshoot == 0, result.overshoot)
    check("1 % short is within the noise", result.steady_state_error == 0, result.steady_state_error)
    check("noise measured", 0.05 < result.noise < 0.15, result.noise)


def test_the_target_is_what_ardupilot_aimed_for():
    # requested 1.0, but ArduPilot capped it at 0.6 (e.g. ATC_STR_RAT_MAX) and the vehicle got there
    values = first_order(0.6)
    result = metrics(values, desired=[0.6] * len(TIMES))
    assert result.target == pytest.approx(0.6)
    assert result.steady_state_error == pytest.approx(0, abs=0.01)
    # without ArduPilot's report: 40 % short of the request
    assert metrics(values).steady_state_error == pytest.approx(0.4, abs=0.01)


def test_negative_steps_are_mirrored():
    result = metrics([-value for value in first_order(1.0, final_fraction=0.8)], target=-1.0, outputs=[-0.5] * 140)
    assert result.steady_state_error == pytest.approx(0.2, abs=0.01)
    assert result.output == pytest.approx(0.5)


def test_an_output_at_its_limit_is_saturation():
    values = first_order(1.0, final_fraction=0.7)
    assert metrics(values, outputs=[1.0] * len(TIMES)).saturated
    assert not metrics(values, outputs=[0.8] * len(TIMES)).saturated


def test_step_cut_short_before_its_end_gives_no_metrics():
    # the run stopped (distance limit) before the last part of the step: no steady state to judge
    times = [t / 10 for t in range(30)]
    assert step_response_metrics(times, [1.0] * 30, target=1.0, step_on=0.0, step_off=10.0) is None


def test_centred_average():
    assert centred_average([0, 1, 2, 3, 4], [0, 0, 3, 0, 0], window=2) == [0, 1, 1, 1, 0]


# ---------------------------------------------------------------------------------------------- gain decisions
@pytest.mark.parametrize(
    "overshoot, rise_time, error, factors, converged",
    [
        (0.3, 0.5, 0.0, (0.75, 0.6), False),
        (0.15, 0.5, 0.0, (0.85, 0.8), False),
        # never reached 63 % of the target
        (0.0, None, 0.5, (1.3, 1.0), False),
        (0.0, 5.0, 0.0, (1.3, 1.0), False),
        (0.0, 0.5, 0.2, (1.1, 1.5), False),
        # settled above the target counts as well
        (0.0, 0.5, -0.07, (1.0, 1.25), False),
        (0.05, 0.5, 0.02, (1.0, 1.0), True),
    ],
    ids=["large overshoot", "small overshoot", "never rises", "slow", "large error", "small error", "good"],
)
def test_pi_rules_without_ff(overshoot, rise_time, error, factors, converged):
    change = GainTuner({"p": 0.2, "i": 0.1}, slow_rise_time=3.0).next_gains(response(overshoot, error, rise_time))
    assert (change.gains["p"], change.gains["i"]) == pytest.approx((0.2 * factors[0], 0.1 * factors[1]))
    assert change.converged is converged


def test_steady_state_error_is_corrected_through_ff_first():
    tuner = GainTuner({"ff": 0.5, "p": 0.2, "i": 0.2}, slow_rise_time=1.0)
    # 20 % short and slow: with an output of 0.6 for 0.8 achieved, FF has to be 0.75 - not more P
    change = tuner.next_gains(response(error=0.2, rise_time=2.0, final_value=0.8, output=0.6))
    assert change.gains == pytest.approx({"ff": 0.75, "p": 0.2, "i": 0.2})
    assert "FF" in change.reason


def test_ff_follows_the_weighted_average_of_the_rounds_near_the_target():
    """Each round measures the FF the vehicle needs - with noise. FF follows their average, weighted by how precise
    each measurement was, not the last round."""
    tuner = GainTuner({"ff": 0.5, "p": 0.2, "i": 0.2}, slow_rise_time=1.0)
    # far from the target (another operating point): corrects FF, but isn't averaged in
    tuner.next_gains(response(error=0.5, final_value=0.5, output=0.4))
    assert tuner.gains["ff"] == pytest.approx(0.8)
    # near the target (within FF_ESTIMATE_BAND): 1.6 per unit achieved (precise), then 1.3 (three times as
    # uncertain: a ninth of the weight)
    tuner.next_gains(replace(response(error=0.07, final_value=0.93, output=0.93 * 1.6), error_uncertainty=0.01))
    assert tuner.gains["ff"] == pytest.approx(1.6)
    measured = replace(response(error=-0.12, final_value=1.12, output=1.12 * 1.3), error_uncertainty=0.03)
    change = tuner.next_gains(measured)
    assert "weighted average of 2 rounds" in change.reason
    # towards (9 x 1.6 + 1.3) / 10 = 1.57, damped: FF's direction reversed - and the reason names the value applied
    applied = 1.6 * (1.57 / 1.6) ** 0.5
    assert change.gains["ff"] == pytest.approx(applied)
    assert f"FF 1.6 -> {applied:.3g} " in change.reason


def test_ff_changes_at_most_by_its_factor_per_round():
    tuner = GainTuner({"ff": 0.2, "p": 0.2, "i": 0.2}, slow_rise_time=1.0)
    change = tuner.next_gains(response(error=0.5, final_value=0.5, output=1.5))
    assert change.gains["ff"] == pytest.approx(0.2 * FF_MAX_FACTOR)


def test_without_controller_reports_the_error_raises_i():
    tuner = GainTuner({"ff": 0.5, "p": 0.2, "i": 0.2}, slow_rise_time=1.0)
    change = tuner.next_gains(response(error=0.2))
    assert change.gains == pytest.approx({"ff": 0.5, "p": 0.22, "i": 0.3})


@pytest.mark.parametrize("measured", [None, response(saturated=True)], ids=["no data", "saturated"])
def test_nothing_to_improve_stops_tuning_and_keeps_the_gains(measured):
    change = GainTuner({"p": 0.2, "i": 0.1}, slow_rise_time=1.0).next_gains(measured)
    assert change.blocked and change.gains == {"p": 0.2, "i": 0.1}


def test_reversing_changes_are_damped_until_tuning_converges():
    """Noisy rounds alternating between "too much" and "too slow" no longer make P swing by the full factors."""
    tuner = GainTuner({"p": 1.0, "i": 1.0}, slow_rise_time=1.0)
    p_values = []
    for round_number in range(12):
        too_much = round_number % 2 == 0
        change = tuner.next_gains(response(overshoot=0.15) if too_much else response(rise_time=2.0))
        p_values.append(change.gains["p"])
        if change.converged:
            break
    steps = [abs(math.log(later / earlier)) for earlier, later in zip([1.0] + p_values, p_values)]
    check("each swing smaller than the one before", all(b < a for a, b in zip(steps[1:], steps[2:])), steps)
    check("converged", change.converged and "small" in change.reason, change.reason)
    assert steps[-1] < math.log(1 + CONVERGED_CHANGE) * 1.01


# ---------------------------------------------------------------------------------------------- error filter
def test_jitter_of_the_feedback_parts_is_measured():
    noise = random.Random(4)
    feedback = [noise.gauss(0, 0.08) for _ in TIMES]
    result = metrics(first_order(1.0), feedback=feedback)
    check("jitter measured", 0.06 < result.jitter < 0.1, result.jitter)
    assert metrics(first_order(1.0)).jitter is None


def jittery(jitter=None, noise=0.0, rise_time=0.4):
    """A step response with this output jitter / measurement noise."""
    return StepResponse(0.0, 0.0, rise_time, 1.0, target=1.0, noise=noise, jitter=jitter)


def test_an_error_filter_well_above_the_loop_bandwidth_is_proposed_for_a_jittery_output():
    hz, reason = suggest_error_filter(jittery(jitter=0.08), p_gain=0.3, current_hz=10)
    # bandwidth 1 / (2 pi 0.4 s) = 0.4 Hz, five times that
    assert hz == pytest.approx(2.0) and "jitters" in reason
    check("small jitter: nothing", suggest_error_filter(jittery(jitter=0.03), 0.3, 10) == (None, ""))
    check("never below 1 Hz", suggest_error_filter(jittery(jitter=0.08, rise_time=3.0), 0.3, 0)[0] == 1.0)


def test_without_controller_reports_the_jitter_is_p_times_the_noise():
    assert suggest_error_filter(jittery(noise=0.4), p_gain=0.3, current_hz=10)[0] == pytest.approx(2.0)
    assert suggest_error_filter(jittery(noise=0.1), p_gain=0.3, current_hz=10) == (None, "")


def test_an_already_low_error_filter_gets_advice_instead():
    hz, reason = suggest_error_filter(jittery(jitter=0.08), p_gain=0.3, current_hz=2.0)
    assert hz is None and "lower P" in reason
    assert suggest_error_filter(None, 0.3, 10) == (None, "")
