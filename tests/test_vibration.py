"""The vibration check of the drive tests: levels judged like ArduPilot's guide, clipping during a run."""

from rover_wizard.motion.vibration import check_vibration
from rover_wizard.vehicle.link import DriveSample, Vibration


def run(levels, clipping=(0, 0)):
    """Samples with these vibration levels and the clipping counter going from clipping[0] to clipping[1]."""
    samples = [
        DriveSample(time=float(index), vibration=Vibration((level, level, level), (clipping[0], 0, 0)))
        for index, level in enumerate(levels)
    ]
    samples[-1].vibration = Vibration((levels[-1],) * 3, (clipping[1], 0, 0))
    return samples


def test_low_vibration_is_fine():
    check = check_vibration(run([5, 12, 8]))
    assert (check.highest, check.clipping, check.notes) == (12, 0, ())


def test_vibration_levels_are_judged_like_ardupilots_guide():
    (note,) = check_vibration(run([10, 45])).notes
    assert "may cause problems" in note
    (note,) = check_vibration(run([10, 70])).notes
    assert "nearly always" in note


def test_clipping_during_the_run_counts_not_before_it():
    check = check_vibration(run([10, 12], clipping=(40, 43)))
    assert check.clipping == 3 and "clipped 3 times" in check.notes[0]
    assert check_vibration(run([10, 12], clipping=(40, 40))).clipping == 0


def test_no_reports_no_check():
    assert check_vibration([DriveSample(time=0.0)]) is None
