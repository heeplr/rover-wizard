"""The step catalog is the one place that orders the steps and puts them into phases."""

import importlib
import pkgutil

import rover_wizard.steps as steps_package
from rover_wizard.ui.step_catalog import PHASES, STEPS


def step_modules():
    """Every module directly in rover_wizard.steps (the shared code is in the steps.util package)."""
    return {
        importlib.import_module(f"{steps_package.__name__}.{info.name}")
        for info in pkgutil.iter_modules(steps_package.__path__)
        if not info.ispkg
    }


def test_steps_holds_only_steps():
    assert all(hasattr(module, "STEP") for module in step_modules()), "shared code belongs in rover_wizard.steps.util"


def test_every_step_module_is_in_exactly_one_phase_once():
    listed = [module for phase in PHASES for module in phase.steps]
    assert len(listed) == len(set(listed)), "a step is listed twice"
    assert set(listed) == step_modules(), "a step module is missing from the catalog (or isn't a step)"


def test_steps_carry_their_phase_in_catalog_order():
    assert [step.phase for step in STEPS] == [phase for phase in PHASES for _ in phase.steps]
    assert [phase.number for phase in PHASES] == list(range(1, len(PHASES) + 1))
    assert PHASES[0].heading == "PHASE 1 - Bench setup (disarmed, indoors OK)"
    assert [phase.drives for phase in PHASES] == [False, True, False]
