"""Phase 1 (bench setup) against the simulator."""

import threading

import pytest

from rover_wizard.util.errors import VehicleError
from rover_wizard.flow import StepFlow
from rover_wizard.rover_defs import SERVO_FUNCTION_GROUND_STEERING
from rover_wizard.rover_defs import SERVO_FUNCTION_THROTTLE
from rover_wizard.rover_defs import SERVO_FUNCTION_THROTTLE_LEFT
from rover_wizard.rover_defs import SERVO_FUNCTION_THROTTLE_RIGHT
from rover_wizard.ui.step_catalog import STEPS
from rover_wizard.steps.steering_output import apply_steering_output

from util.helpers import BY_ID, act, apply, check, enter, make, wait_until


@pytest.mark.parametrize("skid", [False, True], ids=["ackermann", "skid"])
def test_phase1(skid):
    """Run every disarmed (indoor) wizard step against the simulator and check the resulting parameters."""
    print(f"== phase 1 ({'skid' if skid else 'ackermann'})")
    veh, ctx = make(skid=skid, wrong=True)
    ctx.store.set("steer", "skid" if skid else "ackermann")

    enter(ctx, "vehicle_type")
    ctx.values.update(heavy_vehicle=False)
    apply(ctx, "vehicle_type")
    check("heavy_vehicle persisted to store", ctx.store.get("heavy_vehicle") is False)
    enter(ctx, "steering_output")
    ctx.values["steer"] = "skid" if skid else "ackermann"
    apply(ctx, "steering_output")
    check(
        "steering type saved by the Steering output step", ctx.store.get("steer") == ("skid" if skid else "ackermann")
    )
    enter(ctx, "motor_output")
    ctx.values.update(MOT_PWM_TYPE=0)
    apply(ctx, "motor_output")

    if skid:
        enter(ctx, "motor_output")
        ctx.values.update(ch_left=2, ch_right=4)
        apply(ctx, "motor_output")
        funcs = {veh.get("SERVO2_FUNCTION"), veh.get("SERVO4_FUNCTION")}
        check(
            "motor output functions moved to channels 2/4",
            funcs == {SERVO_FUNCTION_THROTTLE_LEFT, SERVO_FUNCTION_THROTTLE_RIGHT} and veh.get("SERVO1_FUNCTION") == 0,
        )
    else:
        enter(ctx, "steering_output")
        ctx.values["ch_steer"] = 2
        apply(ctx, "steering_output")
        check(
            "steering output function moved to channel 2",
            veh.get("SERVO2_FUNCTION") == SERVO_FUNCTION_GROUND_STEERING and veh.get("SERVO1_FUNCTION") == 0,
        )

        enter(ctx, "motor_output")
        ctx.values["ch_thr"] = 4
        apply(ctx, "motor_output")
        check(
            "throttle output function moved to channel 4",
            veh.get("SERVO4_FUNCTION") == SERVO_FUNCTION_THROTTLE and veh.get("SERVO3_FUNCTION") == 0,
        )
    check("reboot flagged", bool(ctx.changes.reboot_reasons))

    # per-field inline Test buttons (steering_output for ackermann, motor_output for skid)
    if skid:
        enter(ctx, "motor_output")
        for lab in ["Test ↑"]:
            check(f"motor_output inline action '{lab}'", act(ctx, "motor_output", lab))
    else:
        enter(ctx, "steering_output")
        for lab in ["Test L", "Test R"]:
            check(f"steering_output inline action '{lab}'", act(ctx, "steering_output", lab))
        enter(ctx, "motor_output")
        check("motor_output inline action 'Test ↑'", act(ctx, "motor_output", "Test ↑"))

    # dead-zone: the inline "Probe" next to MOT_THR_MIN
    enter(ctx, "motor_output")
    veh.set_param("MOT_THR_MIN", 3)
    veh.motor_tests.clear()
    # Continue twice: "wheels off the ground?" (yes), then when the wheel turns (at about 7 %)
    t = threading.Thread(
        target=lambda: (
            wait_until(lambda: ctx.waiting),
            ctx.press_continue(),
            wait_until(lambda: any(p >= 7 for _, p in veh.motor_tests)),
            ctx.press_continue(),
        ),
        daemon=True,
    )
    t.start()
    act(ctx, "motor_output", "Probe")
    check("dead-zone found (~7-9 %)", 6 <= ctx.values.get("MOT_THR_MIN", 0) <= 10, str(ctx.values.get("MOT_THR_MIN")))
    check(
        "vehicle keeps its original MOT_THR_MIN until Apply", veh.get("MOT_THR_MIN") == 3, str(veh.get("MOT_THR_MIN"))
    )
    apply(ctx, "motor_output")
    check("MOT_THR_MIN written", 6 <= veh.get("MOT_THR_MIN") <= 10)

    # Reverse output checkbox (bool field bound to SERVOx_REVERSED)
    step_for_reverse = "motor_output" if skid else "steering_output"
    enter(ctx, step_for_reverse)
    key = "rev_left" if skid else "rev_steering"
    ctx.values[key] = True
    apply(ctx, step_for_reverse)
    ch = 2
    check(
        "Reverse output checkbox wrote SERVOx_REVERSED=1",
        veh.get(f"SERVO{ch}_REVERSED") == 1,
        str(veh.get(f"SERVO{ch}_REVERSED")),
    )

    # accel cal (scripted user)
    ctx.pause = lambda msg, during=None: (ctx.log(">> " + msg), None)[1]
    check("6-pos accel calibration", act(ctx, "accel", "6-position"))
    check("compass calibration", act(ctx, "compass", "compass calibration"))

    # RC calibration with scripted stick movements
    def rc_pause(msg, during=None):
        """Stand in for the pilot during RC calibration: move the simulated sticks to match whatever the step is asking
        for.
        """
        ctx.log(">> " + msg)
        if "Centre" in msg:
            veh.raw_rc_override = [1490] * 16
        elif "Move EVERY" in msg:
            for lo, hi in ((1000, 2000),):
                veh.raw_rc_override = [lo] * 16
                during()
                veh.raw_rc_override = [hi] * 16
                during()
            veh.raw_rc_override = [1490] * 16
        elif "THROTTLE" in msg:
            raw = [1490] * 16
            # throttle forward gives LOW pwm -> reversed channel
            raw[2] = 1000
            veh.raw_rc_override = raw
        elif "STEERING" in msg:
            raw = [1490] * 16
            raw[0] = 2000
            veh.raw_rc_override = raw

    ctx.pause = rc_pause
    enter(ctx, "rc")
    check("RC calibration", act(ctx, "rc", "Calibrate radio"))
    veh.raw_rc_override = None
    check("RC3 detected reversed", veh.get("RC3_REVERSED") == 1 and veh.get("RC1_REVERSED") == 0)
    check("RC min/max stored", veh.get("RC1_MIN") == 1000 and veh.get("RC1_MAX") == 2000)

    # wheel encoders
    def wenc_pause(msg, during=None):
        """Stand in for the pilot during wheel-encoder calibration: roll the simulated vehicle when asked to."""
        ctx.log(">> " + msg)
        if "FORWARD" in msg:
            veh.roll_by_hand(5.0)

    ctx.pause = wenc_pause
    enter(ctx, "wheel_encoders")
    ctx.values["roll_m"] = 5.0
    check("wheel encoder calibration", act(ctx, "wheel_encoders", "Calibrate wheel"))
    apply(ctx, "wheel_encoders")
    check("WENC_RADIUS ~0.10", abs(veh.get("WENC_RADIUS") - 0.10) < 0.002, str(veh.get("WENC_RADIUS")))

    # battery
    enter(ctx, "battery")
    ctx.values["meas_v"] = 12.4
    act(ctx, "battery", "Calibrate voltage")
    apply(ctx, "battery")
    check("BATT_VOLT_MULT ~10.1", abs(veh.get("BATT_VOLT_MULT") - 10.1) < 0.1, str(veh.get("BATT_VOLT_MULT")))

    act(ctx, "save", "Show all changes")
    veh.disconnect()


def test_setup_guards():
    """Output collisions are refused; a missing encoder parameter or failed compass is reported, not crashed on."""
    veh, ctx = make()
    ctx.store.set("steer", "ackermann")
    enter(ctx, "steering_output")
    # SERVO3 carries the throttle in the simulator
    ctx.set_field_value("ch_steer", 3)
    try:
        apply_steering_output(ctx)
        refused = False
    except VehicleError as exc:
        refused = "Throttle" in str(exc)
    check("steering on the throttle output is refused", refused)
    check("throttle output untouched", veh.get("SERVO3_FUNCTION") == SERVO_FUNCTION_THROTTLE)

    # firmware/board without a second encoder radius
    veh._params.pop("WENC2_RADIUS")
    ctx.pause = lambda message, during=None: veh.roll_by_hand(5.0) if "Roll" in message else None
    enter(ctx, "wheel_encoders")
    ctx.set_field_value("roll_m", 5.0)
    check(
        "wheel-encoder calibration survives a missing WENC2_RADIUS",
        act(ctx, "wheel_encoders", "Calibrate wheel radius"),
    )

    # MAG_CAL_STATUS "failed"
    veh.compass_calibration_status = 5
    ctx.pause = lambda message, during=None: None
    ok = act(ctx, "compass", "rotate by hand")
    check("failed compass calibration is reported as failure", not ok)
    check(
        "no reboot requested after a failed compass calibration",
        "compass calibration" not in ctx.changes.reboot_reasons,
    )
    veh.disconnect()


def test_optional_hardware():
    """Unchecked hardware hides its steps/sub-steps and is switched off in ArduPilot; detection sets the defaults."""
    veh, ctx = make()
    flow = StepFlow(STEPS, ctx)
    index = {step.id: position for position, step in enumerate(STEPS)}
    check(
        "GPS, compass, RC and battery steps shown when detected",
        all(flow.does_apply(index[step]) for step in ("gps", "compass", "rc", "battery")),
    )
    check(
        "distance-sensor / gimbal steps hidden when not configured",
        not any(flow.does_apply(index[step]) for step in ("distance_sensors", "gimbal")),
    )
    enter(ctx, "vehicle_type")
    for key in ("has_gps", "has_compass", "has_rc", "has_battery_monitor"):
        ctx.set_field_value(key, False)
    ctx.set_field_value("has_rangefinder", True)
    apply(ctx, "vehicle_type")
    check("no GPS -> GPS1_TYPE = 0", veh.get("GPS1_TYPE") == 0)
    check("no compass -> COMPASS_ENABLE = 0", veh.get("COMPASS_ENABLE") == 0)
    check("no battery monitor -> BATT_MONITOR = 0", veh.get("BATT_MONITOR") == 0)
    check(
        "GPS, compass, RC, battery and navigation steps hidden",
        not any(flow.does_apply(index[step]) for step in ("gps", "compass", "rc", "battery", "navigation")),
    )
    check("distance-sensor step shown once checked", flow.does_apply(index["distance_sensors"]))
    failsafe = BY_ID["failsafe"]
    visible = {field.key for field in failsafe.fields if ctx.is_visible(field)}
    check("RC failsafe settings hidden without RC", "FS_THR_ENABLE" not in visible and "FS_ACTION" in visible)
    predrive_actions = [action.label for action in BY_ID["predrive"].actions if action.when is None or action.when(ctx)]
    check("RC failsafe test hidden without RC", not any("RC failsafe" in label for label in predrive_actions))
    veh.disconnect()


def test_rc_channel_count():
    """The RC monitor/calibration use the receiver's channel count (RC_CHANNELS.chancount), not a fixed number."""
    from types import SimpleNamespace
    from rover_wizard.vehicle.link import Vehicle

    vehicle = Vehicle()
    channels = {f"chan{channel}_raw": 1000 + channel for channel in range(1, 19)}
    vehicle._store_message("RC_CHANNELS", SimpleNamespace(chancount=12, **channels))
    check("12 channels reported -> 12 values", len(vehicle.rc_raw()) == 12)
    vehicle._store_message("RC_CHANNELS", SimpleNamespace(chancount=18, **channels))
    check("18 channels reported -> 18 values", len(vehicle.rc_raw()) == 18 and vehicle.rc_raw()[17] == 1018)
    veh, ctx = make()
    check("simulated 12-channel receiver seen as 12 channels", len(veh.rc_raw()) == 12)
    veh.disconnect()


def test_steering_type_is_chosen_on_the_steering_output_step():
    """The steering fields follow the selector before Apply; Apply switches the outputs over cleanly."""
    # front-wheel: SERVO1 steering, SERVO3 throttle
    veh, ctx = make()
    ctx.store.set("steer", "ackermann")
    step = BY_ID["steering_output"]
    enter(ctx, "steering_output")
    shown = {field.key for field in step.fields if ctx.is_visible(field)}
    check("front-wheel: selector plus steering servo fields", {"steer", "ch_steer", "trim_steering"} <= shown)
    ctx.set_field_value("steer", "skid")
    shown = {field.key for field in step.fields if ctx.is_visible(field)}
    check("choosing skid hides the steering servo fields at once (before Apply)", shown == {"steer"}, str(shown))
    apply(ctx, "steering_output")
    check(
        "skid saved and the steering servo function removed",
        ctx.store.get("steer") == "skid" and veh.get("SERVO1_FUNCTION") == 0,
    )
    enter(ctx, "motor_output")
    ctx.values.update(ch_left=1, ch_right=3)
    apply(ctx, "motor_output")
    check(
        "motor step replaces the single throttle with left/right motors",
        (veh.get("SERVO1_FUNCTION"), veh.get("SERVO3_FUNCTION"))
        == (SERVO_FUNCTION_THROTTLE_LEFT, SERVO_FUNCTION_THROTTLE_RIGHT),
    )
    veh.disconnect()


def test_first_three_steps_hold_the_moved_settings():
    """Vehicle type: info, frame class, heavy-vehicle box, then the hardware boxes; motor settings on Motor output."""
    keys = {
        step_id: [field.key for field in BY_ID[step_id].fields]
        for step_id in ("vehicle_type", "steering_output", "motor_output")
    }
    check("step titles", [BY_ID[s].title for s in keys] == ["Vehicle type", "Steering output", "Motor output"])
    check(
        "vehicle step order",
        keys["vehicle_type"][:4] == ["vehicle_info", "FRAME_CLASS", "heavy_vehicle", "has_sails"],
        str(keys["vehicle_type"]),
    )
    check("steering type on top of Steering output", keys["steering_output"][0] == "steer")
    check("motor driver settings on top of Motor output", keys["motor_output"][:2] == ["MOT_PWM_TYPE", "MOT_PWM_FREQ"])
    check("no Frame step any more", "frame" not in BY_ID)


def test_missing_parameters_are_named_readably():
    from rover_wizard.model import parameter_names_text

    assert parameter_names_text("WP_ACCEL") == "WP_ACCEL"
    assert parameter_names_text(("MAV_GCS_SYSID", "SYSID_MYGCS")) == "MAV_GCS_SYSID or SYSID_MYGCS"
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    ctx.set_param(("NO_SUCH_A", "NO_SUCH_B"), 1, required=False)
    assert log == ["i NO_SUCH_A or NO_SUCH_B: not available on this firmware, skipped"]
    vehicle.disconnect()


def test_every_action_button_has_its_own_help():
    from rover_wizard.ui.step_catalog import STEPS

    actions = [(step.id, action) for step in STEPS for action in step.actions]
    assert actions and all(action.help for _, action in actions), [f"{s}.{a.label}" for s, a in actions if not a.help]
    # the same function in two steps may share a text; different buttons within a step don't
    for step in STEPS:
        texts = [action.help for action in step.actions]
        assert len(set(texts)) == len(texts), step.id
