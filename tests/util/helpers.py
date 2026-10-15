"""Shared helpers for the wizard's tests: a simulated vehicle + wizard context, and step/action shortcuts."""

import tempfile
import time

from rover_wizard.util import timebase
from rover_wizard.context import Ctx
from rover_wizard.util.errors import Cancelled, VehicleError
from rover_wizard.vehicle.simulator import SimVehicle
from rover_wizard.ui.step_catalog import STEPS
from rover_wizard.session.store import Store

# simulated time runs 10x faster than real time
TEST_TIME_SCALE = 10.0
timebase.set_time_scale(TEST_TIME_SCALE)

BY_ID = {step.id: step for step in STEPS}


def check(name, cond, info=""):
    """Assert one named expectation; the name (and info) becomes pytest's failure message."""
    assert cond, name + (f"  [{info}]" if info else "")


def make(skid=False, wrong=False):
    """Create a started simulated vehicle plus a wizard context pointed at it, ready for a test to drive."""
    veh = SimVehicle(skid=skid, wired_wrong=wrong)
    veh.start()
    ctx = Ctx(veh, Store(1, tempfile.mkdtemp()))
    ctx.set_output(log=lambda text: print("     |", text), on_wait=lambda waiting: None, on_values_changed=lambda: None)
    return veh, ctx


def act(ctx, step_id, label_part):
    """Run one step's action button (found by a substring of its label, including per-field inline actions like Test
    buttons).
    """
    step = BY_ID[step_id]
    candidates = list(step.actions)
    for field in step.fields:
        candidates.extend(field.inline_actions or [])
    a = next(x for x in candidates if label_part in x.label)
    ctx.reset_for_new_action()
    ctx.enter_step(step, reload=False)
    try:
        a.fn(ctx)
    except (Cancelled, VehicleError) as exc:
        print("     | !!", type(exc).__name__, exc)
        return False
    finally:
        ctx.cleanup()
    return True


def enter(ctx, step_id):
    """Load a step's fields with their starting values, as if the pilot had just opened it."""
    ctx.enter_step(BY_ID[step_id], reload=True)


def apply(ctx, step_id):
    """Write a step's current field values to the (simulated) vehicle, as the Apply button would."""
    return ctx.apply_step(BY_ID[step_id])


def wait_until(cond, timeout=30):
    """Wait until cond() is true or the timeout elapses (real time); returns whether it became true."""
    t0 = time.time()
    while time.time() - t0 < timeout and not cond():
        time.sleep(0.01)
    return bool(cond())


def pose_following_pause(ctx, veh, cancel_at=None):
    """A stand-in pilot for ctx.pause: tilts the simulated vehicle into the pose each prompt asks for.

    With cancel_at set, it behaves like pressing Cancel when a prompt containing that text appears.
    """

    def pause(message, during=None):
        """Log the prompt and put the vehicle into the requested pose (or cancel)."""
        ctx.log(">> " + message)
        if cancel_at and cancel_at in message:
            raise Cancelled()
        if "LEVEL" in message:
            veh.held_pitch_deg = 0.0
        elif "NOSE" in message:
            veh.held_pitch_deg = 90.0

    return pause
