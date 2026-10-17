"""Actions and safeguards not covered elsewhere: reverting every change, navigation suggestions, the live readouts,
battery calibration, a wheel encoder counting backwards, the manual tracking report, the GPS probe's answers, and
how a drive run stops (disarmed, run limit, no telemetry)."""

import threading

import pytest

from rover_wizard.util import timebase
from rover_wizard.util.errors import Cancelled, VehicleError
from rover_wizard.motion.driving import _DistanceTracker
from rover_wizard.steps.gps import action_probe_gps_type
from rover_wizard.vehicle.link import DriveSample

from util.helpers import BY_ID, act, apply, enter, make, wait_until


@pytest.fixture
def wizard():
    """A front-wheel simulated vehicle; the log goes into a list."""
    vehicle, ctx = make()
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    yield vehicle, ctx, log
    vehicle.disconnect()


def logged(log, text):
    """Whether a log line contains `text`."""
    return any(text in line for line in log)


# ---- Save step: revert every change -----------------------------------------------------------------------------------


@pytest.mark.parametrize("confirmed", [True, False], ids=["yes", "no"])
def test_revert_all_wizard_changes(wizard, confirmed):
    vehicle, ctx, log = wizard
    originals = {name: vehicle.get(name) for name in ("CRUISE_SPEED", "WP_SPEED")}
    ctx.set_param("CRUISE_SPEED", originals["CRUISE_SPEED"] + 1)
    ctx.set_param("WP_SPEED", originals["WP_SPEED"] + 1)
    ctx.confirm = lambda message: confirmed
    assert act(ctx, "save", "Revert ALL")
    if confirmed:
        assert {name: vehicle.get(name) for name in originals} == pytest.approx(originals)
        assert ctx.changes.items() == []
        assert "reverted parameters" in ctx.changes.reboot_reasons
        assert logged(log, "All wizard changes reverted.")
    else:
        assert vehicle.get("CRUISE_SPEED") == pytest.approx(originals["CRUISE_SPEED"] + 1)
        assert len(ctx.changes.items()) == 2


# ---- Navigation step --------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "speeds, expected",
    [
        # cruise speed, capped at 70 % of the top speed; turn radius rounded to 0.5 m
        ({"SPEED_MAX": 3.0, "CRUISE_SPEED": 2.5, "TURN_RADIUS": 1.3}, {"WP_SPEED": 2.1, "WP_RADIUS": 1.5}),
        # no cruise speed measured: 60 % of the top speed; the radius is at least 1 m
        ({"SPEED_MAX": 3.0, "CRUISE_SPEED": 0, "TURN_RADIUS": 0.2}, {"WP_SPEED": 1.8, "WP_RADIUS": 1.0}),
        # no top speed either: no speed proposal
        ({"SPEED_MAX": 0, "CRUISE_SPEED": 0, "TURN_RADIUS": 2.8}, {"WP_RADIUS": 3.0}),
    ],
)
def test_navigation_suggestions_from_the_measurements(wizard, speeds, expected):
    vehicle, ctx, log = wizard
    for name, value in speeds.items():
        vehicle.set_param(name, value)
    enter(ctx, "navigation")
    assert act(ctx, "navigation", "suggestions")
    assert {key: ctx.values.get(key) for key in expected} == pytest.approx(expected)
    if "WP_SPEED" not in expected:
        assert ctx.values["WP_SPEED"] == vehicle.get("WP_SPEED"), "left as it is"


# ---- live readouts ----------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "step_id, label, expected",
    [
        ("gps", "Show GPS status", "GPS: 3D, 12 sats, HDOP 0.9"),
        # the log shows "[" escaped (it parses markup)
        ("wheel_encoders", "Live encoder distance", "Distance \\[m]: 0.000, 0.000"),
        ("distance_sensors", "Show distances", "No DISTANCE_SENSOR readings"),
    ],
)
def test_live_readouts_log_the_telemetry(wizard, step_id, label, expected):
    vehicle, ctx, log = wizard
    enter(ctx, step_id)
    assert act(ctx, step_id, label)
    assert logged(log, expected), log[-3:]


def test_live_distances_of_a_fitted_rangefinder(wizard):
    vehicle, ctx, log = wizard
    # an I2C LightWare (type 7), configured
    vehicle.true_rangefinder = (7, 0)
    vehicle.set_param("RNGFND1_TYPE", 7)
    enter(ctx, "distance_sensors")
    assert act(ctx, "distance_sensors", "Show distances")
    assert logged(log, "sensor 0 (orientation 0): "), log[-3:]


# ---- wheel encoders ---------------------------------------------------------------------------------------------------


def test_wheel_encoder_counting_backwards_is_reported(wizard):
    vehicle, ctx, log = wizard

    def pause(message, during=None):
        """The pilot rolls the vehicle - but the encoder's A/B wires are swapped."""
        if "FORWARD" in message:
            vehicle.roll_by_hand(-5.0)

    ctx.pause = pause
    enter(ctx, "wheel_encoders")
    radius = vehicle.get("WENC_RADIUS")
    assert act(ctx, "wheel_encoders", "Calibrate wheel")
    assert logged(log, "distance DECREASED - swap its PINA/PINB")
    assert ctx.values.get("WENC_RADIUS") == radius, "no radius proposed"


# ---- battery ----------------------------------------------------------------------------------------------------------


def test_battery_reading_and_current_calibration(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "battery")
    assert act(ctx, "battery", "Read battery")
    assert logged(log, "Battery: ") and logged(log, " A (as reported by the autopilot)")

    reported = vehicle._battery_amps()
    multiplier = vehicle.get("BATT_AMP_PERVLT")
    # the clamp meter shows 10 % more than the autopilot reports
    ctx.set_field_value("meas_a", str(reported * 1.1))
    assert act(ctx, "battery", "current")
    assert ctx.values["BATT_AMP_PERVLT"] == pytest.approx(multiplier * 1.1, rel=0.02)


def test_current_calibration_needs_a_measurement_and_a_current_sensor(wizard):
    vehicle, ctx, log = wizard
    vehicle.reports_current = False
    enter(ctx, "battery")
    ctx.set_field_value("meas_a", "")
    with pytest.raises(VehicleError, match="Enter the measured current first"):
        _action(ctx, "battery", "current")(ctx)
    # let the readings without current arrive
    timebase.sleep(1)
    ctx.set_field_value("meas_a", "5")
    with pytest.raises(VehicleError, match="current sensor"):
        _action(ctx, "battery", "current")(ctx)


def _action(ctx, step_id, part):
    """The function of the step's action whose label contains `part`."""
    return next(action.fn for action in BY_ID[step_id].actions if part in action.label)


# ---- closed-loop verification: the manual (Acro) tracking report ------------------------------------------------------


# samples the recording takes with each stick position: more than the report needs (TRACKING_MIN_PAIRS = 20)
SAMPLES_PER_STICK_POSITION = 40


def test_manual_tracking_report(wizard, monkeypatch):
    vehicle, ctx, log = wizard
    vehicle.arm()
    enter(ctx, "verify")
    ctx.values["how_verify"] = "manual"
    # counted, so the pilot holds each position for enough samples however slowly the machine runs
    samples = []
    sample = vehicle.sample
    monkeypatch.setattr(vehicle, "sample", lambda: samples.append(1) or sample())

    def pilot():
        """Drive with varied sticks for a while, then press Continue."""
        wait_until(lambda: ctx.waiting)
        for sticks in ((0.0, 0.6), (0.7, 0.6), (-0.7, 0.4), (0.0, 0.8)):
            vehicle.pilot_sticks = sticks
            target = len(samples) + SAMPLES_PER_STICK_POSITION
            wait_until(lambda: len(samples) >= target)
        vehicle.pilot_sticks = (0.0, 0.0)
        ctx.press_continue()

    threading.Thread(target=pilot, daemon=True).start()
    assert act(ctx, "verify", "SPEED controller")
    assert logged(log, "Steering rate: achieved/desired") and logged(log, "Speed: achieved/desired"), log[-4:]


def test_speed_step_needs_enough_runway(wizard):
    vehicle, ctx, log = wizard
    vehicle.arm()
    enter(ctx, "verify")
    ctx.values.update(how_verify="auto", runway=1, v_step=1.0)
    with pytest.raises(VehicleError, match="Runway too short"):
        _action(ctx, "verify", "SPEED controller")(ctx)


# ---- GPS type probe: the pilot's answers ------------------------------------------------------------------------------


def test_gps_probe_declined_before_it_starts(wizard):
    vehicle, ctx, log = wizard
    ctx.confirm = lambda message: False
    enter(ctx, "gps")
    action_probe_gps_type(ctx)
    assert logged(log, "Cancelled - no changes made.")
    assert ctx.changes.items() == []


def test_gps_probe_detection_not_kept_restores_the_type(wizard):
    vehicle, ctx, log = wizard
    original = vehicle.get("GPS1_TYPE")
    # yes, start - then "no" to keeping each detected type
    answers = iter([True, False, False])
    ctx.confirm = lambda message: next(answers)
    enter(ctx, "gps")
    action_probe_gps_type(ctx)
    assert logged(log, "but not kept - original GPS type restored")
    assert vehicle.get("GPS1_TYPE") == original


def test_gps_port_already_used_by_the_other_gps_is_refused(wizard):
    vehicle, ctx, log = wizard
    vehicle.set_param("GPS2_TYPE", 1)
    vehicle.set_param("SERIAL4_PROTOCOL", 5)
    enter(ctx, "gps")
    ctx.values.update(gps_port=4)
    with pytest.raises(VehicleError, match="already carries the other GPS"):
        apply(ctx, "gps")


# ---- how a drive run stops --------------------------------------------------------------------------------------------


def test_run_stops_when_the_vehicle_is_disarmed(wizard):
    vehicle, ctx, log = wizard
    vehicle.arm()
    threading.Thread(target=lambda: (wait_until(lambda: vehicle.speed > 0.2), vehicle.disarm()), daemon=True).start()
    with pytest.raises(VehicleError, match="disarmed during the run"):
        ctx.drive.run_open_loop(lambda t: (0.0, 0.5), 20)


def test_run_stops_at_the_distance_limit(wizard):
    vehicle, ctx, log = wizard
    vehicle.arm()
    samples = ctx.drive.run_open_loop(lambda t: (0.0, 1.0), 30, limit_m=2.0)
    assert ctx.drive.hit_limit and logged(log, "run limit reached")
    assert samples[-1].run_time < 30


def test_run_without_speed_telemetry_is_aborted(wizard, monkeypatch):
    vehicle, ctx, log = wizard
    vehicle.arm()
    monkeypatch.setattr(vehicle, "sample", lambda: DriveSample(time=timebase.now()))
    with pytest.raises(VehicleError, match="No speed/yaw telemetry"):
        ctx.drive.run_open_loop(lambda t: (0.0, 0.3), 30)


def test_distance_without_a_position_estimate_integrates_the_speed():
    tracker = _DistanceTracker()
    for time in (0.0, 1.0, 2.5):
        tracker.update(DriveSample(time=time, forward_speed=-2.0, yaw_rate=0.0))
    # |speed| x 2.5 s - reversing counts too
    assert tracker.distance == pytest.approx(5.0)


# ---- the wizard context -----------------------------------------------------------------------------------------------


def test_pause_waits_for_continue_and_keeps_calling_during(wizard):
    vehicle, ctx, log = wizard
    calls = []
    threading.Thread(target=lambda: (wait_until(lambda: len(calls) >= 3), ctx.press_continue()), daemon=True).start()
    ctx.pause("Hold it still", during=lambda: calls.append(1))
    assert len(calls) >= 3 and not ctx.waiting


def test_cancel_during_a_pause_aborts_the_action(wizard):
    vehicle, ctx, log = wizard
    threading.Thread(target=lambda: (wait_until(lambda: ctx.waiting), ctx.request_cancel()), daemon=True).start()
    with pytest.raises(Cancelled):
        ctx.pause("Waiting")


def test_shutdown_stops_the_running_action_and_releases_the_steering(wizard):
    vehicle, ctx, log = wizard
    ctx.servos.hold_steering(1.0, seconds=60)
    assert wait_until(lambda: vehicle.servo_raw()[0] == 1900)
    ctx.shutdown()
    with pytest.raises(Cancelled):
        ctx.check()
    assert wait_until(lambda: vehicle.servo_raw()[0] == 1500)


def test_out_of_range_value_is_skipped_not_written(wizard):
    vehicle, ctx, log = wizard
    enter(ctx, "compass")
    original = vehicle.get("COMPASS_CAL_FIT")
    ctx.set_field_value("COMPASS_CAL_FIT", "999")
    apply(ctx, "compass")
    assert vehicle.get("COMPASS_CAL_FIT") == original
    assert logged(log, "'Calibration fitness limit' must be within")


def test_a_required_parameter_missing_on_the_firmware_is_an_error(wizard):
    vehicle, ctx, log = wizard
    with pytest.raises(VehicleError, match="NO_SUCH_PARAM is not available"):
        ctx.set_param("NO_SUCH_PARAM", 1)
    assert ctx.set_param("NO_SUCH_PARAM", 1, required=False) is None
    assert logged(log, "NO_SUCH_PARAM: not available on this firmware, skipped")


def test_a_number_falls_back_to_the_saved_setting_then_the_default(wizard):
    vehicle, ctx, log = wizard
    ctx.store.update_settings(runway=35)
    assert ctx.number("runway", 20) == 35
    ctx.values["runway"] = "42,5"
    assert ctx.number("runway", 20) == 42.5
    assert ctx.number("not_a_field", 7) == 7
