"""Automatic probes and calibrations that reboot or need the pilot's confirmation."""

from rover_wizard.util.rotation import ORIENTATION_CHOICES
from rover_wizard.vehicle.simulator import SimVehicle
from rover_wizard.steps.gps import action_probe_gps_type
from rover_wizard.steps.orientation import action_probe_orientation
from rover_wizard.steps.gps import GPS_TYPE_AUTO
from rover_wizard.steps.gps import GPS_TYPE_NMEA

from util.helpers import BY_ID, act, check, enter, make, pose_following_pause, wait_until


def test_orientation_probe():
    """End-to-end orientation probe for several true mountings - including a correctly mounted board (None)."""
    for true_name in ("None", "Yaw45Roll90", "Pitch180", "Yaw270", "Yaw135Roll270"):
        SimVehicle.TRUE_BOARD_ORIENTATION = true_name
        veh, ctx = make()
        # yes to "start" and to "apply"
        ctx.confirm = lambda message: True
        ctx.pause = pose_following_pause(ctx, veh)
        enter(ctx, "orientation")
        action_probe_orientation(ctx)
        expected = dict(ORIENTATION_CHOICES)[true_name]
        check(
            f"probe finds true mounting {true_name}",
            veh.get("AHRS_ORIENTATION") == expected,
            f"got {veh.get('AHRS_ORIENTATION')}, expected {expected}",
        )
        veh.disconnect()
    SimVehicle.TRUE_BOARD_ORIENTATION = "Yaw45Roll90"


def test_probe_restores_on_cancel():
    """A probe cancelled half-way must put the original setting back (never leave AHRS_ORIENTATION at None)."""
    veh, ctx = make()
    veh.set_param("AHRS_ORIENTATION", 5)
    ctx.confirm = lambda message: True
    ctx.pause = pose_following_pause(ctx, veh, cancel_at="NOSE")
    enter(ctx, "orientation")
    ok = act(ctx, "orientation", "Probe")
    check("cancelled probe reports cancellation", not ok)
    check("AHRS_ORIENTATION restored after cancel", veh.get("AHRS_ORIENTATION") == 5, str(veh.get("AHRS_ORIENTATION")))
    veh.disconnect()


def test_gps_probe():
    """Check the GPS-type auto-probe: a normal receiver is found under Auto immediately; an NMEA-only receiver
    is found only after Auto is tried and fails."""

    # Case 1: a normal receiver (Auto detects it fine) - should apply GPS_TYPE=Auto without ever trying NMEA.
    veh, ctx = make()
    ctx.confirm = lambda msg: True
    enter(ctx, "gps")
    action_probe_gps_type(ctx)
    check("normal receiver: probe settles on Auto", veh.get("GPS1_TYPE") == GPS_TYPE_AUTO, str(veh.get("GPS1_TYPE")))
    veh.disconnect()

    # Case 2: an NMEA-only receiver - Auto should fail to detect it, then NMEA should succeed.
    veh, ctx = make()
    veh.true_gps_type = GPS_TYPE_NMEA
    ctx.confirm = lambda msg: True
    enter(ctx, "gps")
    action_probe_gps_type(ctx)
    check(
        "NMEA-only receiver: probe settles on NMEA after Auto fails",
        veh.get("GPS1_TYPE") == GPS_TYPE_NMEA,
        str(veh.get("GPS1_TYPE")),
    )
    veh.disconnect()

    # Case 3: nothing responds at all - probe should restore the original type and raise a clear error.
    veh, ctx = make()
    # no configurable GPS_TYPE will ever match this
    veh.true_gps_type = 99
    original = veh.get("GPS1_TYPE")
    ctx.confirm = lambda msg: True
    enter(ctx, "gps")
    ok = act(ctx, "gps", "Probe")
    check("no receiver: probe fails cleanly", not ok)
    check("no receiver: original GPS_TYPE restored", veh.get("GPS1_TYPE") == original, str(veh.get("GPS1_TYPE")))
    veh.disconnect()


def test_large_vehicle_magcal():
    """Check the Large Vehicle MagCal path: GPS-fix requirement, heading requirement, and a successful run."""
    veh, ctx = make()
    step = BY_ID["compass"]

    # Not checked "heavy vehicle" -> only the normal rotate-by-hand action should be offered.
    enter(ctx, "compass")
    offered = [a.label for a in step.actions if a.when is None or a.when(ctx)]
    check("normal calibration offered when not heavy", any("rotate by hand" in lab for lab in offered))
    check("Large Vehicle MagCal NOT offered when not heavy", not any("Large Vehicle" in lab for lab in offered))

    ctx.store.set("heavy_vehicle", True)
    enter(ctx, "compass")
    offered = [a.label for a in step.actions if a.when is None or a.when(ctx)]
    check("Large Vehicle MagCal offered once heavy vehicle is checked", any("Large Vehicle" in lab for lab in offered))
    check("normal calibration NOT offered once heavy", not any("rotate by hand" in lab for lab in offered))

    # No GPS fix -> should refuse with a clear reason, no command sent.
    veh.gps_fix_type = 1
    # let the simulator publish the new state
    wait_until(lambda: veh.latest("GPS_RAW_INT").fix_type == 1)
    ctx.values["magcal_heading"] = 90.0
    ok = act(ctx, "compass", "Large Vehicle")
    check("refuses without GPS fix", not ok)

    # GPS fix but no heading entered -> should also refuse.
    veh.gps_fix_type = 3
    wait_until(lambda: veh.latest("GPS_RAW_INT").fix_type == 3)
    ctx.values["magcal_heading"] = ""
    ok = act(ctx, "compass", "Large Vehicle")
    check("refuses without a heading", not ok)

    # Everything present -> should succeed and report a result.
    ctx.values["magcal_heading"] = 45.0
    ok = act(ctx, "compass", "Large Vehicle")
    check("Large Vehicle MagCal succeeds with fix + heading", ok)
    check("reboot flagged after Large Vehicle MagCal", "compass calibration" in ctx.changes.reboot_reasons)
    veh.disconnect()


def test_peripheral_probes():
    """Rangefinder, proximity-sensor and gimbal probes find the hidden device, or restore everything."""
    from rover_wizard.steps.gimbal import action_probe_gimbal
    from rover_wizard.steps.distance_sensors import action_probe_proximity, action_probe_rangefinder

    cases = [("I2C rangefinder (VL53L0X)", (16, 0), 0), ("serial rangefinder (LightWare on SERIAL5)", (8, 5), 5)]
    for name, truth, port in cases:
        veh, ctx = make()
        veh.true_rangefinder = truth
        ctx.confirm = lambda message: True
        enter(ctx, "distance_sensors")
        ctx.set_field_value("rng_port", port)
        action_probe_rangefinder(ctx)
        check(f"{name} found", veh.get("RNGFND1_TYPE") == truth[0], str(veh.get("RNGFND1_TYPE")))
        veh.disconnect()
    veh, ctx = make()
    ctx.confirm = lambda message: True
    original_protocol = veh.get("SERIAL5_PROTOCOL")
    enter(ctx, "distance_sensors")
    ctx.set_field_value("rng_port", 5)
    action_probe_rangefinder(ctx)
    check(
        "no rangefinder: type and serial port restored",
        veh.get("RNGFND1_TYPE") == 0 and veh.get("SERIAL5_PROTOCOL") == original_protocol,
    )
    veh.disconnect()

    veh, ctx = make()
    # RPLidar S1 on SERIAL6 at 256000 baud
    veh.true_proximity = (5, 6, 256)
    ctx.confirm = lambda message: True
    enter(ctx, "distance_sensors")
    ctx.set_field_value("prx_port", 6)
    action_probe_proximity(ctx)
    check(
        "RPLidar S1 found with the right baud rate",
        veh.get("PRX1_TYPE") == 5 and veh.get("SERIAL6_BAUD") == 256 and veh.get("SERIAL6_PROTOCOL") == 11,
    )
    veh.disconnect()

    veh, ctx = make()
    # Siyi
    veh.true_gimbal_type = 8
    # The stand-in pilot says "yes" only when the gimbal really tilted.
    ctx.confirm = lambda message: (veh.gimbal_pitch != 0) if "did the gimbal tilt" in message else True
    enter(ctx, "gimbal")
    ctx.set_field_value("gimbal_port", 6)
    action_probe_gimbal(ctx)
    check(
        "Siyi gimbal found by the pilot's confirmation", veh.get("MNT1_TYPE") == 8 and veh.get("SERIAL6_PROTOCOL") == 8
    )
    check("gimbal levelled again after the test", veh.gimbal_pitch == 0)
    veh.disconnect()
