"""Boats (FRAME_CLASS Boat): texts worded for the water, hardware only boats (or only land vehicles) have, the Torqeedo
motor, the depth sounder, holding position, vectored thrust and the GPS motion model."""

import pytest

from rover_wizard.rover_defs import FRAME_CLASS_BOAT, FRAME_CLASS_ROVER, RANGEFINDER_ORIENT_DOWN
from rover_wizard.boat_defs import BATTERY_MONITOR_TORQEEDO, SERIAL_PROTOCOL_TORQEEDO, TORQEEDO_SERIAL_BAUD
from rover_wizard.model import Wording, worded
from rover_wizard.steps.util.actions import action_reboot_flight_controller
from rover_wizard.steps.util.rover_hardware import has
from rover_wizard.texts import all_texts
from rover_wizard.util.errors import VehicleError
from rover_wizard.vehicle.boat_simulator import SimBoat

from util.helpers import BY_ID, act, apply, enter, make

# the hidden echosounder: rangefinder 2, NMEA on SERIAL5 at 9600 baud (SERIALx_BAUD 9)
SOUNDER = ("2", 17, 5)
SOUNDER_BAUD = 9
# the hidden Torqeedo: tiller connection on SERIAL6
TORQEEDO = (1, 6)


@pytest.fixture
def boat():
    """A simulated boat and its wizard context; the log goes into a list."""
    vehicle, ctx = make(simulator=SimBoat)
    log = []
    ctx.set_output(log=log.append, on_wait=lambda waiting: None, on_values_changed=lambda: None)
    yield vehicle, ctx, log
    vehicle.disconnect()


@pytest.fixture
def rover():
    """A simulated rover and its wizard context."""
    vehicle, ctx = make()
    yield vehicle, ctx
    vehicle.disconnect()


def visible(ctx, step_id):
    """Keys of the fields currently shown on a step."""
    return {field.key for field in BY_ID[step_id].fields if ctx.is_visible(field)}


# ---- wording ------------------------------------------------------------------------------------------------------------


def test_a_wording_is_the_land_text_and_carries_the_water_one():
    text = Wording("Lift the wheels.", "Tie the boat up.")
    assert text == "Lift the wheels." and worded(text, boat=False) == "Lift the wheels."
    assert worded(text, boat=True) == "Tie the boat up."
    assert worded("Plain text", boat=True) == "Plain text"


def test_texts_follow_the_frame_class(boat, rover):
    vehicle, ctx, _ = boat
    _, rover_ctx = rover
    intro = BY_ID["motor_output"].intro
    assert "tied up" in ctx.words(intro) and "wheels" in rover_ctx.words(intro)
    assert "propeller" in ctx.prompt("motor_output.ramp_watch")
    assert "wheel" in rover_ctx.prompt("motor_output.ramp_watch")
    # a text without a water version reads the same on both
    assert ctx.words(BY_ID["barometer"].help) == rover_ctx.words(BY_ID["barometer"].help)

    vehicle.set_param("FRAME_CLASS", FRAME_CLASS_ROVER)
    assert "wheels" in ctx.words(intro)


def test_every_water_text_has_a_land_text():
    texts = all_texts()
    reworded = [
        key
        for section in ("fields", "actions", "probes", "prompts")
        for key, text in texts[section].items()
        if isinstance(text, Wording)
    ]
    assert "motor_output.rev_throttle" in reworded and "motor_output.ramp_start" in reworded


# ---- hardware offered ---------------------------------------------------------------------------------------------------


def test_boat_hardware_is_offered_on_boats_only(boat, rover):
    _, ctx, _ = boat
    _, rover_ctx = rover
    enter(ctx, "vehicle_type")
    enter(rover_ctx, "vehicle_type")
    assert {"has_torqeedo", "has_depth_sounder", "has_sails"} <= visible(ctx, "vehicle_type")
    assert "has_wenc" not in visible(ctx, "vehicle_type")
    assert "has_wenc" in visible(rover_ctx, "vehicle_type")
    assert not {"has_torqeedo", "has_depth_sounder"} & visible(rover_ctx, "vehicle_type")
    # the form follows the frame class chosen, before Apply
    rover_ctx.set_field_value("FRAME_CLASS", FRAME_CLASS_BOAT)
    assert "has_torqeedo" in visible(rover_ctx, "vehicle_type")


def test_wheel_encoders_are_no_boat_hardware(boat):
    vehicle, ctx, _ = boat
    # the simulator has encoders configured, but a boat has none
    assert vehicle.get_int("WENC_TYPE") and not has("has_wenc")(ctx)
    assert not BY_ID["wheel_encoders"].does_apply_to(ctx)


def test_unticking_boat_hardware_leaves_land_settings_alone(boat):
    vehicle, ctx, _ = boat
    enter(ctx, "vehicle_type")
    apply(ctx, "vehicle_type")
    # not offered on a boat, so not "unticked": the encoders stay configured
    assert vehicle.get_int("WENC_TYPE")


# ---- boat-only settings in existing steps --------------------------------------------------------------------------------


def test_vectored_thrust_and_gps_motion_model_on_boats(boat, rover):
    _, ctx, _ = boat
    _, rover_ctx = rover
    for context, shown in ((ctx, True), (rover_ctx, False)):
        enter(context, "steering_output")
        enter(context, "gps")
        assert ("MOT_VEC_ANGLEMAX" in visible(context, "steering_output")) == shown
        assert ("GPS_NAVFILTER" in visible(context, "gps")) == shown


def test_steering_type_is_named_for_boats(boat):
    vehicle, ctx, _ = boat
    labels = [ctx.words(label) for label, _ in BY_ID["steering_output"].fields[0].choices]
    assert labels == ["Rudder or steered motor (outboard)", "Two motors (differential thrust)"]


def test_loiter_step_on_boats_only(boat, rover):
    vehicle, ctx, _ = boat
    _, rover_ctx = rover
    assert BY_ID["boat_loiter"].does_apply_to(ctx) and not BY_ID["boat_loiter"].does_apply_to(rover_ctx)
    enter(ctx, "boat_loiter")
    assert ctx.values["MIS_DONE_BEHAVE"] == 1, "boats loiter after a mission by default"
    ctx.set_field_value("LOIT_TYPE", 1)
    ctx.set_field_value("LOIT_RADIUS", "3")
    apply(ctx, "boat_loiter")
    assert vehicle.get("LOIT_TYPE") == 1 and vehicle.get("LOIT_RADIUS") == 3


# ---- Torqeedo ---------------------------------------------------------------------------------------------------------------


def tick(ctx, key):
    """Tick a hardware box on the Vehicle type step and apply."""
    enter(ctx, "vehicle_type")
    ctx.set_field_value(key, True)
    apply(ctx, "vehicle_type")


def test_torqeedo_setup(boat):
    vehicle, ctx, log = boat
    vehicle.true_torqeedo = TORQEEDO
    tick(ctx, "has_torqeedo")
    assert BY_ID["torqeedo"].does_apply_to(ctx)
    enter(ctx, "torqeedo")
    assert ctx.values["trq_reboot"].startswith("Choose how"), "the settings appear only after type + reboot"
    ctx.set_field_value("trq_type", TORQEEDO[0])
    assert "trq_port" in visible(ctx, "torqeedo")
    with pytest.raises(VehicleError, match="serial port"):
        apply(ctx, "torqeedo")
    ctx.set_field_value("trq_port", TORQEEDO[1])
    ctx.set_field_value("trq_battery", True)
    apply(ctx, "torqeedo")
    assert vehicle.get_int("TRQD_TYPE") == TORQEEDO[0] and "TRQD_TYPE" in ctx.changes.reboot_reasons
    assert vehicle.get_int("SERIAL6_PROTOCOL") == SERIAL_PROTOCOL_TORQEEDO
    assert vehicle.get_int("SERIAL6_BAUD") == TORQEEDO_SERIAL_BAUD
    assert vehicle.get_int("BATT_MONITOR") == BATTERY_MONITOR_TORQEEDO

    action_reboot_flight_controller(ctx)
    enter(ctx, "torqeedo")
    shown = visible(ctx, "torqeedo")
    assert {"trq_onoff_pin", "trq_slew", "trq_option_1"} <= shown and "trq_reboot" not in shown
    assert "trq_power" not in shown, "motor power only applies to the motor connection"
    assert act(ctx, "torqeedo", "Show motor status")
    assert log[-1] == "Torqeedo: 850 rpm, 29.4 V, 12.5 A, 31 deg C"


def test_torqeedo_without_answer(boat):
    vehicle, ctx, log = boat
    vehicle.set_param("TRQD_TYPE", 2)
    action_reboot_flight_controller(ctx)
    tick(ctx, "has_torqeedo")
    enter(ctx, "torqeedo")
    assert "trq_power" in visible(ctx, "torqeedo")
    assert act(ctx, "torqeedo", "Show motor status")
    assert log[-1].startswith("No motor telemetry")


# ---- depth sounder ----------------------------------------------------------------------------------------------------------


def test_depth_sounder_setup(boat):
    vehicle, ctx, log = boat
    vehicle.true_rangefinder = SOUNDER
    vehicle.true_rangefinder_baud = SOUNDER_BAUD
    tick(ctx, "has_depth_sounder")
    enter(ctx, "depth_sounder")
    assert ctx.values["depth_rangefinder"] == 1, "the first free rangefinder"
    ctx.set_field_value("depth_rangefinder", 2)
    enter(ctx, "depth_sounder")
    ctx.set_field_value("depth_model", 1)
    assert act(ctx, "depth_sounder", "Fill in")
    assert ctx.values["depth_type"] == 17 and ctx.values["depth_baud"] == SOUNDER_BAUD
    assert ctx.values["depth_max"] == 70
    ctx.set_field_value("depth_port", SOUNDER[2])
    apply(ctx, "depth_sounder")
    assert vehicle.get_int("RNGFND2_TYPE") == 17 and vehicle.get_int("RNGFND2_ORIENT") == RANGEFINDER_ORIENT_DOWN
    assert vehicle.get("RNGFND2_MAX") == 70
    assert vehicle.get_int("SERIAL5_BAUD") == SOUNDER_BAUD

    action_reboot_flight_controller(ctx)
    enter(ctx, "depth_sounder")
    assert ctx.values["depth_rangefinder"] == 2, "found again: the rangefinder pointing down"
    assert act(ctx, "depth_sounder", "Show depth")
    assert log[-1] == "Depth 12.50 m, water 14.0 deg C"


def test_depth_needs_a_choice(boat):
    _, ctx, log = boat
    enter(ctx, "depth_sounder")
    assert not act(ctx, "depth_sounder", "Fill in")
    assert act(ctx, "depth_sounder", "Show depth")
    assert log[-1].startswith("No depth reported")


# ---- edge cases ---------------------------------------------------------------------------------------------------------


def test_torqeedo_before_its_reboot(boat):
    vehicle, ctx, _ = boat
    tick(ctx, "has_torqeedo")
    enter(ctx, "torqeedo")
    apply(ctx, "torqeedo")
    assert vehicle.get_int("TRQD_TYPE") == 0, "type 0: nothing to set up"
    ctx.set_field_value("trq_type", 2)
    ctx.set_field_value("trq_port", TORQEEDO[1])
    apply(ctx, "torqeedo")
    assert vehicle.get_int("BATT_MONITOR") != BATTERY_MONITOR_TORQEEDO, "battery box left unticked"
    enter(ctx, "torqeedo")
    assert ctx.values["trq_reboot"].startswith("The Torqeedo type is set")


def test_torqeedo_reports_on_its_output(boat):
    vehicle, ctx, log = boat
    vehicle.true_torqeedo = TORQEEDO
    vehicle.set_param("TRQD_TYPE", TORQEEDO[0])
    vehicle.set_param(f"SERIAL{TORQEEDO[1]}_PROTOCOL", SERIAL_PROTOCOL_TORQEEDO)
    vehicle.set_param(f"SERIAL{TORQEEDO[1]}_BAUD", TORQEEDO_SERIAL_BAUD)
    # no throttle output: ArduPilot reports on the first slot
    vehicle.set_param("SERVO3_FUNCTION", 0)
    action_reboot_flight_controller(ctx)
    tick(ctx, "has_torqeedo")
    assert act(ctx, "torqeedo", "Show motor status")
    assert log[-1].startswith("Torqeedo: 850 rpm")


def test_depth_sounder_needs_its_serial_port(boat):
    vehicle, ctx, _ = boat
    tick(ctx, "has_depth_sounder")
    enter(ctx, "depth_sounder")
    ctx.set_field_value("depth_type", 23)
    with pytest.raises(VehicleError, match="serial port"):
        apply(ctx, "depth_sounder")


def test_depth_sounder_found_among_other_rangefinders(boat):
    vehicle, ctx, _ = boat
    # rangefinder 1 looks forward, rangefinder 2 down: the depth sounder is the second
    for instance, orientation in (("1", 0), ("2", RANGEFINDER_ORIENT_DOWN)):
        vehicle.set_param(f"RNGFND{instance}_TYPE", 17)
        vehicle.set_param(f"RNGFND{instance}_ORIENT", orientation)
    vehicle.set_param("SERIAL5_PROTOCOL", 9)
    vehicle.set_param("SERIAL5_BAUD", SOUNDER_BAUD)
    assert has("has_depth_sounder")(ctx)
    enter(ctx, "depth_sounder")
    assert ctx.values["depth_rangefinder"] == 2
    # the baud rate follows the chosen port
    ctx.set_field_value("depth_port", 5)
    ctx.values.pop("depth_baud")
    ctx.enter_step(BY_ID["depth_sounder"], reload=False)
    assert ctx.values["depth_baud"] == SOUNDER_BAUD


def test_no_depth_from_a_sounder_not_pointing_down(boat):
    vehicle, ctx, log = boat
    vehicle.true_rangefinder = SOUNDER
    vehicle.true_rangefinder_baud = SOUNDER_BAUD
    vehicle.set_param("RNGFND2_TYPE", SOUNDER[1])
    vehicle.set_param("SERIAL5_PROTOCOL", 9)
    vehicle.set_param("SERIAL5_BAUD", SOUNDER_BAUD)
    enter(ctx, "depth_sounder")
    assert act(ctx, "depth_sounder", "Show depth")
    assert log[-1].startswith("No depth reported"), "forward-looking: a rangefinder, not a depth sounder"
    vehicle.set_param("RNGFND2_ORIENT", RANGEFINDER_ORIENT_DOWN)
    vehicle.set_param("FRAME_CLASS", FRAME_CLASS_ROVER)
    assert act(ctx, "depth_sounder", "Show depth")
    assert log[-1].startswith("No depth reported"), "Rover reports the depth for boats only"


def test_unhealthy_depth_sounder(boat):
    vehicle, ctx, _ = boat
    from types import SimpleNamespace

    from rover_wizard.steps.depth_sounder import describe_depth

    vehicle._store_message("WATER_DEPTH", SimpleNamespace(distance=0.0, temperature=0.0, healthy=0))
    assert describe_depth(ctx) == "Depth 0.00 m, water 0.0 deg C (NOT healthy)"


def test_demo_vehicles():
    from rover_wizard.vehicle.demo import DEMO_VEHICLES, demo_vehicle
    from rover_wizard.vehicle.rover_simulator import SimVehicle
    from rover_wizard.vehicle.sail_simulator import SimSailboat

    kinds = {kind: demo_vehicle(kind) for kind in DEMO_VEHICLES}
    assert type(kinds["boat"]) is SimBoat and type(kinds["sailboat"]) is SimSailboat
    assert type(kinds["rover"]) is SimVehicle and kinds["skid"].skid and not kinds["rover"].skid
