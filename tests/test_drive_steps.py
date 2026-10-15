"""Phase 2 (driving and tuning) against the simulator."""

import threading
import time

import pytest

from rover_wizard.util import timebase

from util.helpers import act, apply, check, enter, make, wait_until


@pytest.mark.parametrize("skid", [False, True], ids=["ackermann", "skid"])
def test_phase2(skid):
    """Run every armed (driving) wizard step against the simulator, including a deliberately-reversed vehicle, and check
    the results.
    """
    print(f"== phase 2 ({'skid' if skid else 'ackermann'})")
    veh, ctx = make(skid=skid, wrong=True)
    ctx.store.set("steer", "skid" if skid else "ackermann")
    ctx.pause = lambda msg, during=None: (ctx.log(">> " + msg), None)[1]
    ctx.confirm = lambda msg: (ctx.log("?? " + msg + " -> yes"), True)[1]
    ctx.values.update(runway=40, circle_d=24)
    check("refuses to drive while disarmed", not act(ctx, "direction_check", "direction check"))
    veh.arm()

    ctx.values["how_dir"] = "auto"
    act(ctx, "direction_check", "direction check")
    check(
        "direction fixed",
        (
            (veh.get("SERVO1_REVERSED") == 1 and veh.get("SERVO3_REVERSED") == 1)
            if skid
            else (veh.get("SERVO3_REVERSED") == 1 and veh.get("SERVO1_REVERSED") == 1)
        ),
        f"S1={veh.get('SERVO1_REVERSED')} S3={veh.get('SERVO3_REVERSED')} F1={veh.get('SERVO1_FUNCTION')}",
    )
    veh.arm()
    act(ctx, "direction_check", "direction check")

    enter(ctx, "top_speed")
    veh.arm()
    act(ctx, "top_speed", "top speed")
    check("SPEED_MAX ~3", abs(ctx.values["SPEED_MAX"] - 3.0) < 0.35, str(ctx.values.get("SPEED_MAX")))
    apply(ctx, "top_speed")

    enter(ctx, "turning")
    veh.arm()
    act(ctx, "turning", "turn radius")
    if not skid:
        check("TURN_RADIUS ~1.5", abs(ctx.values["TURN_RADIUS"] - 1.5) < 0.3, str(ctx.values.get("TURN_RADIUS")))
    veh.arm()
    act(ctx, "turning", "max turn rate")
    check("turn rate found", ctx.values.get("ACRO_TURN_RATE", 0) > 10, str(ctx.values.get("ACRO_TURN_RATE")))
    apply(ctx, "turning")

    enter(ctx, "gains")
    veh.arm()
    act(ctx, "gains", "steering feed-forward")
    check("steering FF proposed", ctx.values.get("ATC_STR_RAT_FF", 0.2) != 0.2, str(ctx.values.get("ATC_STR_RAT_FF")))
    veh.arm()
    act(ctx, "gains", "cruise")
    check("cruise speed proposed", ctx.values.get("CRUISE_SPEED", 2) != 2, str(ctx.values.get("CRUISE_SPEED")))
    apply(ctx, "gains")

    enter(ctx, "verify")
    ctx.values.update(rounds=3, v_step=1.2, r_step=40)
    veh.arm()
    act(ctx, "verify", "SPEED controller")
    veh.arm()
    act(ctx, "verify", "STEERING-RATE")
    check("wizard left override released", ctx.drive.rc_driver.active is False)
    veh.disconnect()


def test_manual_topspeed():
    """Check the 'I drive manually' path: a scripted virtual pilot drives, and the top-speed measurement should still
    work.
    """
    veh, ctx = make()
    veh.arm()
    ctx.values["how_speed"] = "manual"

    def pilot():
        """Background thread: play the part of a human pilot driving at full throttle, then releasing the stick."""
        wait_until(lambda: ctx.waiting)
        veh.pilot_sticks = (0.0, 1.0)
        t0 = timebase.now()
        while timebase.now() - t0 < 9:
            time.sleep(0.005)
        veh.pilot_sticks = (0.0, 0.0)
        while timebase.now() - t0 < 16:
            time.sleep(0.005)
        ctx.press_continue()

    threading.Thread(target=pilot, daemon=True).start()
    act(ctx, "top_speed", "top speed")
    check("manual SPEED_MAX ~3", abs(ctx.values.get("SPEED_MAX", 0) - 3.0) < 0.35, str(ctx.values.get("SPEED_MAX")))
    veh.disconnect()


def test_emergency():
    """Check that pressing STOP mid-run aborts the action and leaves the vehicle disarmed."""
    veh, ctx = make()
    veh.arm()
    ctx.values["how_speed"] = "auto"
    ctx.pause = lambda msg, during=None: None
    # STOP once the rover is actually driving - a fixed delay would depend on the simulator's speed-up
    threading.Thread(target=lambda: (wait_until(lambda: veh.speed > 0.2), ctx.emergency()), daemon=True).start()
    ok = act(ctx, "top_speed", "top speed")
    check("run aborted by STOP", not ok and not veh.armed)
    veh.disconnect()


def test_rc_override_channel_limit():
    """Automatic driving refuses (instead of silently doing nothing) when the steering channel is above 8."""
    veh, ctx = make()
    veh.set_param("RCMAP_ROLL", 9)
    veh.arm()
    ctx.pause = lambda message, during=None: None
    ctx.set_field_value("how_dir", "auto")
    ok = act(ctx, "direction_check", "Run direction check")
    check("automatic run refused for RCMAP_ROLL=9", not ok)
    veh.disconnect()


def test_speed_tests_stay_within_wp_speed():
    """Guided mode caps speed targets at WP_SPEED: the wizard must not ask for more (it would read as slow response)."""
    import pytest
    from rover_wizard.util.errors import VehicleError
    from rover_wizard.steps.verify import (
        action_verify_speed_controller,
        action_verify_steering_controller,
        default_speed_step,
    )

    veh, ctx = make()
    veh.set_param("CRUISE_SPEED", 5)
    veh.set_param("WP_SPEED", 2)
    # 60 % of a 5 m/s cruise speed would be 3 m/s - more than Guided mode drives
    assert default_speed_step(ctx) == 2.0
    enter(ctx, "verify")
    ctx.values.update(v_step=2.5)
    with pytest.raises(VehicleError, match="WP_SPEED = 2.0"):
        action_verify_speed_controller(ctx)
    veh.set_param("WP_SPEED", 0.3)
    # a front-wheel-steered car needs at least 0.5 m/s to turn
    with pytest.raises(VehicleError, match="WP_SPEED = 0.3"):
        action_verify_steering_controller(ctx)
    veh.disconnect()
