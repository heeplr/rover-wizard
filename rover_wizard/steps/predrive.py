"""rover_wizard.steps.predrive - the "Pre-drive checks" step."""

from __future__ import annotations

from ..rover_defs import FS_OPTIONS_ACTIVE_IN_HOLD, MODE_NAMES, MODES
from ..mavlink_defs import (
    EKF_ATTITUDE,
    EKF_POS_HORIZ_ABS,
    EKF_POS_HORIZ_REL,
    EKF_VELOCITY_HORIZ,
    GPS_FIX_TYPE_3D_FIX,
    GPS_RAW_INT_EPH_SCALE,
    SYS_STATUS_MILLIVOLTS_PER_VOLT,
)
from ..util.timebase import now
from .util.driving import DEFAULT_CIRCLE_DIAMETER_M, DEFAULT_RUNWAY_M
from .util.rover_hardware import has
from ..model import Action, Step, value_field

PREDRIVE_MIN_SATELLITES = 8

PREDRIVE_MAX_HDOP = 2.0

PREDRIVE_MIN_BATTERY_MILLIVOLTS = 1000

# The wizard's runs rely on the flight controller dropping stale commands: RC override (RC_OVERRIDE_TIME, default
# 3 s; 0 = overrides ignored, -1 = never expire) and Guided targets (GUID_TIMEOUT, default 3 s).
MAX_COMMAND_TIMEOUT_SECONDS = 5.0
# Guided targets are sent at 10 Hz (motion.driving): a shorter timeout would stop runs on a little jitter.
MIN_GUIDED_TIMEOUT_SECONDS = 0.5

# [s] the RC failsafe test waits for the flight controller's message beyond FS_TIMEOUT (RC loss detection, telemetry)
FAILSAFE_MESSAGE_MARGIN_SECONDS = 3.0
# how often it looks for the message
FAILSAFE_POLL_SECONDS = 0.2
# ArduPilot's failsafe messages: "Radio Failsafe", "Radio Failsafe Cleared"
FAILSAFE_WORD = "Failsafe"
CLEARED_WORD = "Cleared"


def apply_predrive(ctx):
    """Save runway length and circle diameter; every later drive run is limited by them."""
    ctx.store.update_settings(
        runway=ctx.number("runway", DEFAULT_RUNWAY_M), circle_d=ctx.number("circle_d", DEFAULT_CIRCLE_DIAMETER_M)
    )


def action_run_predrive_checks(ctx):
    """Check GPS, EKF, battery, RC, the RC-override identity and arming before any drive; log what to fix."""
    vehicle = ctx.vehicle
    all_ok = True

    def check(passed, good, bad):
        """Log one check as OK/FAIL and fold it into the overall result."""
        nonlocal all_ok
        ctx.log(("OK   " + good) if passed else ("FAIL " + bad))
        all_ok = all_ok and bool(passed)

    gps = vehicle.latest("GPS_RAW_INT")
    # HDOP = eph / 100 (eph is HDOP x 100); lower is better, < 2 is good enough for driving tests.
    if has("has_gps")(ctx):
        check(
            gps is not None
            and gps.fix_type >= GPS_FIX_TYPE_3D_FIX
            and gps.satellites_visible >= PREDRIVE_MIN_SATELLITES
            and gps.eph / GPS_RAW_INT_EPH_SCALE < PREDRIVE_MAX_HDOP,
            f"GPS fix good ({gps.satellites_visible if gps else 0} sats)",
            f"GPS: need 3D fix, >={PREDRIVE_MIN_SATELLITES} satellites, HDOP < {PREDRIVE_MAX_HDOP:g} (go outside!)",
        )
    ekf = vehicle.latest("EKF_STATUS_REPORT")
    check(
        ekf is not None
        and ekf.flags & EKF_ATTITUDE
        and ekf.flags & EKF_VELOCITY_HORIZ
        and ekf.flags & (EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS),
        "EKF has attitude/velocity/position",
        "EKF not healthy yet (wait for GPS fix, calibrate compass, or check EK3 sources)",
    )
    status = vehicle.latest("SYS_STATUS")
    # mV -> V
    volts = status.voltage_battery / SYS_STATUS_MILLIVOLTS_PER_VOLT if status else 0
    if has("has_battery_monitor")(ctx):
        check(
            status is not None and status.voltage_battery > PREDRIVE_MIN_BATTERY_MILLIVOLTS,
            f"battery {volts:.1f} V",
            "no battery voltage reported",
        )

    if has("has_rc")(ctx):
        check(bool(vehicle.rc_raw()), "RC receiver data present", "no RC_CHANNELS data (transmitter on?)")
    check(
        vehicle.gcs_system_id == vehicle.configured_gcs_system_id(),
        "ground-station system id matches the wizard (RC override accepted)",
        "the ground-station system id (MAV_GCS_SYSID / SYSID_MYGCS) differs from the wizard's",
    )

    override_time = vehicle.get("RC_OVERRIDE_TIME")
    if override_time is not None:
        check(
            0 < override_time <= MAX_COMMAND_TIMEOUT_SECONDS,
            f"RC override expires {override_time:g} s after the last command",
            "RC_OVERRIDE_TIME = "
            + (
                "-1: RC override never expires - after a lost link the vehicle keeps driving"
                if override_time < 0
                else (
                    "0: RC override is switched off - the wizard can't drive"
                    if override_time == 0
                    else f"{override_time:g} s: too long after a lost link"
                )
            )
            + " (set 3 s, Ctrl+P)",
        )

    guided_timeout = vehicle.get("GUID_TIMEOUT")
    if guided_timeout is not None:
        check(
            MIN_GUIDED_TIMEOUT_SECONDS <= guided_timeout <= MAX_COMMAND_TIMEOUT_SECONDS,
            f"Guided mode stops {guided_timeout:g} s after the last command",
            f"GUID_TIMEOUT = {guided_timeout:g} s: needs {MIN_GUIDED_TIMEOUT_SECONDS:g} .. "
            f"{MAX_COMMAND_TIMEOUT_SECONDS:g} s (set 3 s, Ctrl+P)",
        )
    check(vehicle.armed, "vehicle is ARMED", ctx.prompt("predrive.not_armed"))
    ctx.log(
        "Everything looks fine - continue with the next step." if all_ok else "Fix the FAIL items above before driving."
    )


def _wait_for_failsafe_text(ctx, marker: int, cleared: bool, seconds: float) -> list:
    """Flight-controller messages about a failsafe (cleared=True: its end) that arrive within `seconds`."""
    deadline = now() + seconds
    while True:
        texts = [
            text
            for text in ctx.vehicle.status_texts_since(marker)
            if FAILSAFE_WORD in text and (CLEARED_WORD in text) == cleared
        ]
        if texts or now() >= deadline:
            return texts
        ctx.sleep(FAILSAFE_POLL_SECONDS)


def _mode_name(ctx) -> str:
    """The vehicle's current flight mode by name."""
    return MODE_NAMES.get(ctx.vehicle.custom_mode, str(ctx.vehicle.custom_mode))


def action_test_rc_failsafe(ctx):
    """Pilot switches the transmitter off and on again; check that the flight controller reports the radio failsafe
    and its end.

    ArduPilot's failsafe neither acts nor reports in Hold (unless FS_OPTIONS bit 0) or RTL - and the failsafe action
    itself usually ends in Hold. So a vehicle in one of these modes is first switched to Manual (asked first).
    """
    ctx.require_armed()
    vehicle = ctx.vehicle
    acts_in_hold = ctx.param_int("FS_OPTIONS") & FS_OPTIONS_ACTIVE_IN_HOLD
    if vehicle.custom_mode == MODES["RTL"] or (vehicle.custom_mode == MODES["HOLD"] and not acts_in_hold):
        if not ctx.confirm(
            f"The vehicle is in {_mode_name(ctx)}, where ArduPilot's failsafe doesn't act or report "
            "anything. Switch to MANUAL for the test? (Keep the throttle stick centred.)"
        ):
            ctx.log(f"Not tested: in {_mode_name(ctx)} the failsafe would stay silent.")
            return
        vehicle.set_mode("MANUAL")
    wait = float(ctx.param_value("FS_TIMEOUT", 0) or 0) + FAILSAFE_MESSAGE_MARGIN_SECONDS
    marker = vehicle.status_text_marker()
    ctx.pause(ctx.prompt("predrive.rc_failsafe_start"))
    started = _wait_for_failsafe_text(ctx, marker, cleared=False, seconds=wait)
    ctx.log(
        ("OK   failsafe reported: " + "; ".join(started) + f" - mode now {_mode_name(ctx)}")
        if started
        else "FAIL no failsafe message - check FS_THR_ENABLE / FS_THR_VALUE (the receiver may keep sending a normal "
        "throttle when the transmitter is off) and FS_ACTION."
    )
    marker = vehicle.status_text_marker()
    ctx.pause("Switch the transmitter back ON and press Continue.")

    ended = _wait_for_failsafe_text(ctx, marker, cleared=True, seconds=wait)
    if ended:
        ctx.log("OK   failsafe ended: " + "; ".join(ended))
    elif started:
        ctx.log(
            "FAIL the failsafe wasn't reported as cleared: another failsafe is still active (e.g. GCS or battery "
            "- see the FC messages), or the receiver isn't back yet."
        )
    ctx.log(f"The vehicle stays in {_mode_name(ctx)}: switch the mode with your transmitter before driving.")


STEP = Step(
    "predrive",
    "Pre-drive checks",
    [
        value_field(
            "runway",
            "Free straight runway AHEAD of the vehicle",
            unit="m",
            default=lambda ctx: ctx.number("runway", DEFAULT_RUNWAY_M),
            min_value=3,
            max_value=1000,
        ),
        value_field(
            "circle_d",
            "Free circle DIAMETER on the vehicle's RIGHT",
            unit="m",
            default=lambda ctx: ctx.number("circle_d", DEFAULT_CIRCLE_DIAMETER_M),
            min_value=2,
            max_value=1000,
        ),
    ],
    [
        Action("Run pre-drive checks", action_run_predrive_checks),
        Action("Test RC failsafe", action_test_rc_failsafe, armed=True, when=has("has_rc")),
    ],
    on_apply=apply_predrive,
)
