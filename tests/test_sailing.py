"""Sailing vehicles - a sailboat and a land yacht: switching sailing on, the sail actuator and its tests, the RC switches,
the wind vane (calibration, headwind offset, live wind) and skipping the throttle tests without a motor."""

import pytest

from rover_wizard.boat_defs import (
    MOTOR_TEST_INSTANCE_MAINSAIL,
    RC_OPTION_MAINSAIL,
    RC_OPTION_SAILBOAT_MOTOR_3POS,
    RC_OPTION_SAILBOAT_TACK,
    SERIAL_PROTOCOL_WIND_VANE,
    SERVO_FUNCTION_MAINSAIL,
    SERVO_FUNCTION_MAST_ROTATION,
    WIND_VANE_TYPE_ANALOG,
    WIND_VANE_TYPE_NMEA,
)
from rover_wizard.motion.sail_test import end_percent
from rover_wizard.steps.wind_vane import action_calibrate_direction, action_set_headwind_offset
from rover_wizard.steps.util.actions import action_reboot_flight_controller
from rover_wizard.steps.util.rover_hardware import is_motorised
from rover_wizard.util.errors import VehicleError
from rover_wizard.vehicle.sail_simulator import SailSimulation, SimLandYacht, SimSailboat

from util.helpers import BY_ID, act, apply, enter, make, wait_until

SAIL_OUTPUT = 4
THROTTLE_STEPS = ("motor_output", "direction_check", "top_speed", "turning", "gains", "verify")


@pytest.fixture
def sailboat():
    """A simulated sailing boat with sailing switched on, and its wizard context; the log goes into a list."""
    vehicle, ctx = make(simulator=SimSailboat)
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    yield vehicle, ctx, log
    vehicle.disconnect()


def visible(ctx, step_id):
    """Keys of the fields currently shown on a step."""
    return {field.key for field in BY_ID[step_id].fields if ctx.is_visible(field)}


def answer_hardware(ctx, **answers):
    """Tick / untick hardware boxes on the Vehicle type step (as the UI does: shown boxes get their values) and apply."""
    enter(ctx, "vehicle_type")
    for key, value in answers.items():
        ctx.set_field_value(key, value)
        enter_without_reload(ctx)
    apply(ctx, "vehicle_type")


def enter_without_reload(ctx):
    """Re-render the step keeping the edits (a refresh field changed)."""
    ctx.enter_step(BY_ID["vehicle_type"], reload=False)


def assign_sail(ctx, actuator=SERVO_FUNCTION_MAINSAIL):
    """Choose the sail actuator and its output on the Sails step and apply."""
    enter(ctx, "sails")
    ctx.set_field_value("sail_actuator", actuator)
    ctx.set_field_value("sail_output", SAIL_OUTPUT)
    apply(ctx, "sails")
    enter(ctx, "sails")


# ---- switching sailing on ---------------------------------------------------------------------------------------------------


def test_sailing_is_switched_on_and_its_settings_appear_after_a_reboot():
    vehicle, ctx = make(simulator=SimLandYacht, sails_enabled=False)
    try:
        assert not BY_ID["sails"].does_apply_to(ctx)
        answer_hardware(ctx, has_sails=True)
        assert vehicle.get_int("SAIL_ENABLE") == 1 and "SAIL_ENABLE" in ctx.changes.reboot_reasons
        enter(ctx, "sails")
        assert visible(ctx, "sails") == {"sail_reboot"}
        assert ctx.values["sail_reboot"].startswith("SAIL_ENABLE is set")
        action_reboot_flight_controller(ctx)
        enter(ctx, "sails")
        assert {"sail_actuator", "sail_output", "SAIL_ANGLE_IDEAL", "tack_switch"} <= visible(ctx, "sails")
        # a land yacht: sails without being a boat
        assert not ctx.is_boat() and BY_ID["wind_vane"].does_apply_to(ctx)
        answer_hardware(ctx, has_sails=False)
        assert vehicle.get_int("SAIL_ENABLE") == 0
    finally:
        vehicle.disconnect()


def test_motor_box_only_with_sails(sailboat):
    _, ctx, _ = sailboat
    enter(ctx, "vehicle_type")
    assert "has_motor" in visible(ctx, "vehicle_type")
    ctx.set_field_value("has_sails", False)
    assert "has_motor" not in visible(ctx, "vehicle_type")


@pytest.mark.parametrize("motor", [True, False])
def test_throttle_tests_need_a_motor(sailboat, motor):
    _, ctx, _ = sailboat
    answer_hardware(ctx, has_sails=True, has_motor=motor)
    assert is_motorised(ctx) == motor
    assert {step for step in THROTTLE_STEPS if BY_ID[step].does_apply_to(ctx)} == (
        set(THROTTLE_STEPS) if motor else set()
    )
    assert BY_ID["predrive"].does_apply_to(ctx) and BY_ID["navigation"].does_apply_to(ctx)
    enter(ctx, "sails")
    assert ("SAIL_WNDSPD_MIN" in visible(ctx, "sails")) == motor
    assert ("motor_switch" in visible(ctx, "sails")) == motor


# ---- the sail actuator ------------------------------------------------------------------------------------------------------


def test_sail_output_and_its_test(sailboat):
    vehicle, ctx, _ = sailboat
    enter(ctx, "sails")
    with pytest.raises(VehicleError, match="servo output"):
        apply(ctx, "sails")
    assign_sail(ctx)
    assert vehicle.get_int(f"SERVO{SAIL_OUTPUT}_FUNCTION") == SERVO_FUNCTION_MAINSAIL
    assert {"rev_mainsail", "min_mainsail", "max_mainsail", "SAIL_ANGLE_MIN"} <= visible(ctx, "sails")
    assert act(ctx, "sails", "Test +")
    assert vehicle.motor_tests[-1] == (MOTOR_TEST_INSTANCE_MAINSAIL, 100)
    assert vehicle._servo_output_pwm(SAIL_OUTPUT) == vehicle.get(f"SERVO{SAIL_OUTPUT}_MAX"), "a winch let out"
    assert act(ctx, "sails", "Test -")
    assert vehicle._servo_output_pwm(SAIL_OUTPUT) == vehicle.get(f"SERVO{SAIL_OUTPUT}_MIN"), "a winch pulled in"


def test_changing_the_actuator_moves_the_function(sailboat):
    vehicle, ctx, _ = sailboat
    assign_sail(ctx)
    assign_sail(ctx, SERVO_FUNCTION_MAST_ROTATION)
    assert vehicle.get_int(f"SERVO{SAIL_OUTPUT}_FUNCTION") == SERVO_FUNCTION_MAST_ROTATION
    assert "SAIL_ANGLE_MIN" not in visible(ctx, "sails"), "the boom angles are for a winch"
    assert act(ctx, "sails", "Test -")
    assert vehicle.motor_tests[-1] == (MOTOR_TEST_INSTANCE_MAINSAIL, -100), "an angle output goes to both sides"


def test_sail_endpoint_buttons_move_the_sail(sailboat):
    vehicle, ctx, log = sailboat
    assign_sail(ctx)
    ctx.servos.nudge_endpoint(SERVO_FUNCTION_MAINSAIL, "MAX", "max_mainsail", +1)
    assert vehicle.get(f"SERVO{SAIL_OUTPUT}_MAX") == 1910
    assert vehicle.motor_tests[-1] == (MOTOR_TEST_INSTANCE_MAINSAIL, 100)
    assert log[-1] == f"SERVO{SAIL_OUTPUT}_MAX = 1910 (sail moved there: +100%)"
    ctx.servos.nudge_endpoint(SERVO_FUNCTION_MAINSAIL, "TRIM", "trim_mainsail", +1)
    assert log[-1] == f"SERVO{SAIL_OUTPUT}_TRIM = 1510", "a winch has no centre position to move to"


@pytest.mark.parametrize(
    "function, end, reversed_output, percent",
    [
        (SERVO_FUNCTION_MAINSAIL, "MIN", False, 0),
        (SERVO_FUNCTION_MAINSAIL, "MAX", False, 100),
        (SERVO_FUNCTION_MAINSAIL, "MIN", True, 100),
        (SERVO_FUNCTION_MAINSAIL, "TRIM", False, None),
        (SERVO_FUNCTION_MAST_ROTATION, "MIN", False, -100),
        (SERVO_FUNCTION_MAST_ROTATION, "TRIM", True, 0),
        (SERVO_FUNCTION_MAST_ROTATION, "MAX", True, -100),
    ],
)
def test_sail_end_percent(function, end, reversed_output, percent):
    assert end_percent(function, end, reversed_output) == percent


def test_side_panel_moves_a_sail(sailboat):
    vehicle, ctx, _ = sailboat
    assign_sail(ctx)
    line = ctx.servos.nudge(SAIL_OUTPUT, +1)
    assert "sail motor test +5%" in line
    assert vehicle.motor_tests[-1] == (MOTOR_TEST_INSTANCE_MAINSAIL, 5)


def test_sailing_rc_switches(sailboat):
    vehicle, ctx, _ = sailboat
    answer_hardware(ctx, has_sails=True, has_motor=True)
    enter(ctx, "sails")
    ctx.set_field_value("sail_output", SAIL_OUTPUT)
    ctx.set_field_value("tack_switch", 6)
    ctx.set_field_value("mainsail_channel", 7)
    ctx.set_field_value("motor_switch", 9)
    apply(ctx, "sails")
    assert vehicle.get_int("RC6_OPTION") == RC_OPTION_SAILBOAT_TACK
    assert vehicle.get_int("RC7_OPTION") == RC_OPTION_MAINSAIL
    assert vehicle.get_int("RC9_OPTION") == RC_OPTION_SAILBOAT_MOTOR_3POS


# ---- the wind vane ----------------------------------------------------------------------------------------------------------


def set_up_vane(ctx, kind=WIND_VANE_TYPE_ANALOG):
    """Choose the vane type, apply and reboot, so its settings appear."""
    enter(ctx, "wind_vane")
    ctx.set_field_value("WNDVN_TYPE", kind)
    if kind == WIND_VANE_TYPE_NMEA:
        ctx.set_field_value("wind_port", 5)
    apply(ctx, "wind_vane")
    action_reboot_flight_controller(ctx)
    enter(ctx, "wind_vane")


def test_wind_vane_settings_appear_after_a_reboot(sailboat):
    vehicle, ctx, log = sailboat
    enter(ctx, "wind_vane")
    assert visible(ctx, "wind_vane") == {"WNDVN_TYPE", "wind_reboot"}
    set_up_vane(ctx)
    assert {"WNDVN_DIR_PIN", "WNDVN_DIR_V_MIN", "WNDVN_DIR_OFS", "WNDVN_SPEED_TYPE"} <= visible(ctx, "wind_vane")
    assert "wind_port" not in visible(ctx, "wind_vane")
    assert act(ctx, "wind_vane", "Show wind")
    assert log[-1].startswith("Wind apparent: from") and "true: from" in log[-1]


def test_nmea_vane_uses_a_serial_port(sailboat):
    vehicle, ctx, _ = sailboat
    set_up_vane(ctx, WIND_VANE_TYPE_NMEA)
    assert vehicle.get_int("SERIAL5_PROTOCOL") == SERIAL_PROTOCOL_WIND_VANE and vehicle.get_int("SERIAL5_BAUD") == 4
    assert "WNDVN_DIR_PIN" not in visible(ctx, "wind_vane") and "WNDVN_DIR_OFS" in visible(ctx, "wind_vane")


def test_wind_vane_direction_calibration(sailboat, monkeypatch):
    vehicle, ctx, log = sailboat
    set_up_vane(ctx)
    monkeypatch.setattr(ctx, "pause", lambda message, during=None: log.append(">> " + message))
    assert act(ctx, "wind_vane", "Calibrate direction")
    low, high = SailSimulation.TRUE_VANE_VOLTS
    assert (vehicle.get("WNDVN_DIR_V_MIN"), vehicle.get("WNDVN_DIR_V_MAX")) == (low, high)
    assert (ctx.values["WNDVN_DIR_V_MIN"], ctx.values["WNDVN_DIR_V_MAX"]) == (low, high)
    assert vehicle.get_int("WNDVN_CAL") == 0
    assert "WNDVN_CAL" not in str(ctx.changes.items()), "a command, not a setting"


def test_wind_speed_calibration(sailboat, monkeypatch):
    vehicle, ctx, log = sailboat
    set_up_vane(ctx)
    ctx.set_field_value("WNDVN_SPEED_TYPE", 2)
    apply(ctx, "wind_vane")
    action_reboot_flight_controller(ctx)
    enter(ctx, "wind_vane")
    assert {"WNDVN_SPEED_PIN", "WNDVN_SPEED_OFS"} <= visible(ctx, "wind_vane")
    monkeypatch.setattr(ctx, "pause", lambda message, during=None: None)
    assert act(ctx, "wind_vane", "Calibrate zero wind speed")
    assert vehicle.get("WNDVN_SPEED_OFS") == SailSimulation.TRUE_SPEED_SENSOR_ZERO_VOLTS


def test_headwind_offset(sailboat, monkeypatch):
    vehicle, ctx, log = sailboat
    set_up_vane(ctx)
    vehicle.held_vane_deg = 0.0
    monkeypatch.setattr(ctx, "pause", lambda message, during=None: None)
    assert act(ctx, "wind_vane", "Set headwind offset")
    expected = (-SailSimulation.TRUE_VANE_ZERO_DEG) % 360
    assert ctx.values["WNDVN_DIR_OFS"] == round(expected)
    apply(ctx, "wind_vane")
    assert act(ctx, "wind_vane", "Set headwind offset")
    assert "reads +0 deg" in " ".join(log)


def test_calibration_refused_while_armed(sailboat, monkeypatch):
    vehicle, ctx, _ = sailboat
    set_up_vane(ctx)
    monkeypatch.setattr(ctx, "pause", lambda message, during=None: None)
    vehicle.armed = True
    assert not act(ctx, "wind_vane", "Calibrate direction")


# ---- edge cases ---------------------------------------------------------------------------------------------------------


def test_sails_before_the_reboot_write_nothing():
    vehicle, ctx = make(simulator=SimLandYacht, sails_enabled=False)
    try:
        vehicle.set_param("SAIL_ENABLE", 1)
        enter(ctx, "sails")
        apply(ctx, "sails")
        assert not ctx.changes.items()
        assert not act(ctx, "sails", "Test +"), "no sail output yet"
    finally:
        vehicle.disconnect()


def test_sail_switches_without_a_motor(sailboat):
    vehicle, ctx, _ = sailboat
    answer_hardware(ctx, has_sails=True, has_motor=False)
    enter(ctx, "sails")
    ctx.set_field_value("sail_output", SAIL_OUTPUT)
    ctx.set_field_value("mainsail_channel", 7)
    apply(ctx, "sails")
    assert vehicle.get_int("RC7_OPTION") == 0, "without a motor the throttle stick moves the sail"


def test_reversed_winch(sailboat):
    vehicle, ctx, _ = sailboat
    assign_sail(ctx)
    vehicle.set_param(f"SERVO{SAIL_OUTPUT}_REVERSED", 1)
    assert act(ctx, "sails", "Test -")
    assert vehicle._servo_output_pwm(SAIL_OUTPUT) == vehicle.get(f"SERVO{SAIL_OUTPUT}_MAX")


def test_wind_vane_notes_and_switches(sailboat):
    vehicle, ctx, log = sailboat
    vehicle.set_param("WNDVN_TYPE", WIND_VANE_TYPE_ANALOG)
    enter(ctx, "wind_vane")
    assert ctx.values["wind_reboot"].startswith("WNDVN_TYPE is set")
    assert act(ctx, "wind_vane", "Show wind")
    assert log[-1].startswith("No wind reported")
    # no sensor: the wind from the heading at arming, turned by an RC knob
    ctx.set_field_value("WNDVN_TYPE", 2)
    ctx.set_field_value("wind_offset_channel", 6)
    apply(ctx, "wind_vane")
    assert vehicle.get_int("RC6_OPTION") == 97


def test_nmea_vane_needs_a_port(sailboat):
    _, ctx, _ = sailboat
    enter(ctx, "wind_vane")
    ctx.set_field_value("WNDVN_TYPE", WIND_VANE_TYPE_NMEA)
    with pytest.raises(VehicleError, match="serial port"):
        apply(ctx, "wind_vane")


def test_nmea_baud_rate_kept(sailboat):
    vehicle, ctx, _ = sailboat
    vehicle.set_param("SERIAL5_PROTOCOL", SERIAL_PROTOCOL_WIND_VANE)
    vehicle.set_param("SERIAL5_BAUD", 38)
    vehicle.set_param("WNDVN_TYPE", WIND_VANE_TYPE_NMEA)
    action_reboot_flight_controller(ctx)
    enter(ctx, "wind_vane")
    assert ctx.values["wind_port"] == 5 and ctx.values["wind_baud"] == 38


def test_failed_direction_calibration(sailboat, monkeypatch):
    vehicle, ctx, _ = sailboat
    set_up_vane(ctx)
    monkeypatch.setattr(ctx, "pause", lambda message, during=None: None)
    # the vane was hardly turned
    vehicle.TRUE_VANE_VOLTS = (1.2, 1.8)
    assert not act(ctx, "wind_vane", "Calibrate direction")
    assert "Calibration failed" in vehicle.last_text()
    assert vehicle.get("WNDVN_DIR_V_MAX") == 3.3, "nothing stored"


def test_calibration_that_never_ends(sailboat, monkeypatch):
    vehicle, ctx, log = sailboat
    set_up_vane(ctx)
    monkeypatch.setattr(ctx, "pause", lambda message, during=None: None)
    monkeypatch.setattr(vehicle, "_advance_wind_calibration", lambda timestamp: None)
    with pytest.raises(VehicleError, match="didn't finish the calibration"):
        action_calibrate_direction(ctx)


def test_calibration_of_a_vane_without_one(sailboat):
    vehicle, _, _ = sailboat
    vehicle.set_param("WNDVN_TYPE", WIND_VANE_TYPE_NMEA)
    vehicle.reboot()
    vehicle.set_param("WNDVN_CAL", 2)
    assert wait_until(lambda: vehicle.get_int("WNDVN_CAL") == 0)
    assert vehicle.last_text().endswith("WindVane: No cal required")


def test_headwind_offset_without_wind(sailboat, monkeypatch):
    vehicle, ctx, log = sailboat
    set_up_vane(ctx)
    monkeypatch.setattr(ctx, "pause", lambda message, during=None: None)
    monkeypatch.setattr(vehicle, "_publish_telemetry", lambda timestamp: None)
    vehicle._messages.clear()
    with pytest.raises(VehicleError, match="No apparent wind reported"):
        action_set_headwind_offset(ctx)


def test_wind_from_the_wind_message_only(sailboat):
    vehicle, ctx, _ = sailboat
    from types import SimpleNamespace

    from rover_wizard.steps.wind_vane import describe_wind

    vehicle._store_message("WIND", SimpleNamespace(direction=270.0, speed=4.0, speed_z=0.0))
    assert describe_wind(vehicle) == "Wind true: from 270 deg at 4.0 m/s"


def test_other_motor_tests_on_a_sailboat(sailboat):
    vehicle, ctx, _ = sailboat
    # the throttle's motor test still drives the throttle output
    assert vehicle._motor_test_output(1) == vehicle._output_with_function(70)
