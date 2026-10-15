"""Pilot interaction, saved progress and step navigation."""

import os
import tempfile
import threading

from rover_wizard.util.errors import Cancelled
from rover_wizard.flow import StepFlow
from rover_wizard.ui.step_catalog import STEPS
from rover_wizard.session.store import STEP_DONE
from rover_wizard.session.store import STEP_SKIPPED
from rover_wizard.session.store import Store

from util.helpers import check, make, wait_until


def test_confirm_cancel_and_stop():
    """Cancel answers a yes/no question with 'no'; STOP must abort the action instead of being swallowed."""
    veh, ctx = make()
    outcome = {}

    def ask(label):
        """Ask a question in a worker thread and remember the answer or exception."""
        try:
            outcome[label] = ctx.confirm("Proceed?")
        except Cancelled:
            outcome[label] = "cancelled"

    for label, press in (("cancel", ctx.request_cancel), ("stop", ctx.emergency)):
        ctx.reset_for_new_action()
        worker = threading.Thread(target=ask, args=(label,), daemon=True)
        worker.start()
        wait_until(lambda: ctx.waiting)
        press()
        worker.join(5)
    check("Cancel during a question means 'no'", outcome.get("cancel") is False, str(outcome.get("cancel")))
    check("STOP during a question aborts", outcome.get("stop") == "cancelled", str(outcome.get("stop")))
    veh.disconnect()


def test_store_and_flow():
    """Progress file robustness and the step navigation rules."""
    root = tempfile.mkdtemp()
    with open(os.path.join(root, "vehicle_7.json"), "w", encoding="utf-8") as handle:
        handle.write("{ this is not json")
    store = Store(7, root)
    kept_aside = any(name.startswith("vehicle_7.json.corrupt") for name in os.listdir(root))
    check(
        "corrupt progress file is kept aside, defaults loaded", kept_aside and store.step_status("vehicle_type") is None
    )
    store.mark_step("vehicle_type", STEP_DONE)
    store.mark_step("vehicle_type", STEP_SKIPPED)
    check("done and skipped are mutually exclusive", store.step_status("vehicle_type") == STEP_SKIPPED)

    veh, ctx = make(skid=True)
    ctx.store.set("steer", "skid")
    flow = StepFlow(STEPS, ctx)
    index = {step.id: position for position, step in enumerate(STEPS)}
    check("resume at the first step", flow.first_open_step() == 0)
    check("second step locked until the first is finished", not flow.is_reachable(1))
    ctx.store.mark_step("vehicle_type", STEP_DONE)
    check(
        "steering output (with the steering-type selector) follows the vehicle step for every vehicle",
        flow.does_apply(index["steering_output"])
        and flow.neighbour(index["vehicle_type"], +1) == index["steering_output"],
    )
    check("motor output locked until the steering step is done", not flow.is_reachable(index["motor_output"]))
    ctx.store.mark_step("steering_output", STEP_DONE)
    check("then motor output is reachable", flow.is_reachable(index["motor_output"]))
    check("board orientation comes before the accelerometer calibration", index["orientation"] < index["accel"])
    check("wheel-encoder step shown when WENC_TYPE says encoders are fitted", flow.does_apply(index["wheel_encoders"]))
    veh.set_param("WENC_TYPE", 0)
    veh.set_param("WENC2_TYPE", 0)
    check("wheel-encoder step hidden without encoders", not flow.does_apply(index["wheel_encoders"]))
    ctx.store.set("has_wenc", True)
    check("the pilot's answer overrides the detection", flow.does_apply(index["wheel_encoders"]))
    veh.disconnect()


def test_parameter_comparison_is_exact_for_whole_numbers():
    """A relative tolerance must not hide a one-bit change of a large bitmask; floats keep their tolerance."""
    from rover_wizard.util.values import are_nearly_equal

    assert not are_nearly_equal(2097151, 2097150), "LOG_BITMASK with bit 0 cleared is a change"
    assert are_nearly_equal(0.2, 0.2 + 1e-9) and are_nearly_equal(70, 70.0)


def test_a_file_of_the_wrong_shape_is_set_aside(tmp_path):
    # valid JSON, but not a progress file: the wizard starts fresh and keeps the file
    (tmp_path / "vehicle_3.json").write_text("[1, 2, 3]", encoding="utf-8")
    store = Store(3, root=str(tmp_path))
    assert store.setting("runway") is None
    assert [path.name.startswith("vehicle_3.json.corrupt-") for path in tmp_path.iterdir()] == [True]
