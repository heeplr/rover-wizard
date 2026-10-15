"""rover_wizard.motion.servo_test - moving outputs by hand from the wizard (Test buttons, MIN/TRIM/MAX +/-, servo panel).

How each kind of output can be moved, according to ArduPilot Rover's behaviour:
* Steering servo: Rover drives the steering output from the steering stick even while disarmed, and an RC override
  counts as stick input. So the wizard holds the steering "stick" (RC override, Manual mode) - no arming, no motor
  test. The override is released automatically a few seconds after the last click.
* Motor outputs: only ArduPilot's motor test drives them. The motor test ARMS the vehicle for its duration and
  DISARMS it afterwards (Rover/motor_test.cpp), needs calibrated RC and passes the arming checks.
* Outputs without a function (or RC pass-through / RCINx): MAV_CMD_DO_SET_SERVO moves them directly; ArduPilot
  refuses it for every other function ("Channel x is already in use").
"""

from __future__ import annotations

import contextlib
import threading
from typing import Optional

from ..util import timebase
from .driving import RcOverrideDriver
from ..util.errors import VehicleError
from ..ardupilot_defs import (
    ARMING_CHECK_NONE,
    ARMING_SKIPCHK_ALL,
    AUX_FUNCTION_REFRESH_SECONDS,
    MODES,
    MODE_NAMES,
    MOTOR_TEST_INSTANCE_THROTTLE,
    MOTOR_TEST_INSTANCE_THROTTLE_LEFT,
    MOTOR_TEST_INSTANCE_THROTTLE_RIGHT,
    RC_PWM_NOMINAL_TRIM,
    SERVO_FUNCTIONS_DIRECTLY_SETTABLE,
    SERVO_FUNCTIONS_RESOLVED_AT_RUNTIME,
    SERVO_FUNCTION_DISABLED,
    SERVO_FUNCTION_GROUND_STEERING,
    SERVO_FUNCTION_THROTTLE,
    SERVO_FUNCTION_THROTTLE_LEFT,
    SERVO_FUNCTION_THROTTLE_RIGHT,
    SERVO_OUTPUTS,
)
from ..mavlink_defs import ACCEPTED_RESULTS

MOTOR_TEST_INSTANCE_BY_FUNCTION = {
    SERVO_FUNCTION_THROTTLE: MOTOR_TEST_INSTANCE_THROTTLE,
    SERVO_FUNCTION_THROTTLE_LEFT: MOTOR_TEST_INSTANCE_THROTTLE_LEFT,
    SERVO_FUNCTION_THROTTLE_RIGHT: MOTOR_TEST_INSTANCE_THROTTLE_RIGHT,
}
MOTOR_NAMES = {
    MOTOR_TEST_INSTANCE_THROTTLE: "throttle",
    MOTOR_TEST_INSTANCE_THROTTLE_LEFT: "LEFT motor",
    MOTOR_TEST_INSTANCE_THROTTLE_RIGHT: "RIGHT motor",
}
# [us] per click for outputs without a function
PANEL_PWM_STEP = 25
# steering stick change per click (10 % of full deflection)
PANEL_STEERING_STEP = 0.1
# [%] per click for motor outputs (small: the wheels really turn)
PANEL_MOTOR_STEP = 5
# how long each motor test holds the output
MOTOR_TEST_SECONDS = 2.0
# a 0 % test this short ends the running motor test almost at once
MOTOR_TEST_STOP_SECONDS = 0.1
# the Test L/R buttons hold the steering this long
STEERING_HOLD_SECONDS = 1.0
# the side panel's +/- buttons hold the steering this long after the last click, so consecutive clicks add up
PANEL_STEERING_HOLD_SECONDS = 3.0
# Function swap: an output whose function ArduPilot resolves at runtime (e.g. a gimbal servo) is switched to "none"
# while the panel moves it, and back PANEL_STEERING_HOLD_SECONDS after the last click. ArduPilot applies the switch
# within AUX_FUNCTION_REFRESH_SECONDS; this much later the function no longer overwrites the output.
FUNCTION_SWAP_SETTLE_SECONDS = AUX_FUNCTION_REFRESH_SECONDS + 0.3
# store key: output -> original function of swapped outputs, so a crash or lost link can't leave one switched off
SWAPPED_FUNCTIONS_KEY = "swapped_servo_functions"
# What can still make ArduPilot refuse a motor test (Rover/motor_test.cpp) when the arming checks are skipped
MOTOR_TEST_REQUIREMENTS = (
    "it needs the safety switch pressed and RCx_MIN/MAX within ArduPilot's limits "
    "(the wizard skips the arming checks for the test)."
)
# [us] per click on the MIN/TRIM/MAX +/- buttons
ENDPOINT_STEP = 10
# [us] sane range for any servo end point
PWM_LIMITS = (800, 2200)
# the servo panel always shows at least outputs 1-8
MIN_PANEL_OUTPUTS = 8
# Test buttons: % duty cycle steering deflection / motor power
DEFAULT_STEERING_TEST_PERCENT = 25
DEFAULT_MOTOR_TEST_PERCENT = 15
# how often the steering hold checks whether it is due for release
RELEASE_CHECK_SECONDS = 0.1
# wait this long after a stick hold / motor test before reading the output's pulse
OUTPUT_SETTLE_SECONDS = 0.5


def _clamp(value: float, low: float, high: float) -> float:
    """value limited to low..high."""
    return max(low, min(high, value))


@contextlib.contextmanager
def arming_checks_skipped(vehicle):
    """Skip ArduPilot's arming checks while a motor test starts, then restore them.

    The motor test arms the vehicle, and arming runs the arming checks - which a bench setup typically fails (no RC
    signal, compass not calibrated yet). Rover only checks when the test starts (Rover/motor_test.cpp), so the
    original value is restored right after the command - also when it fails. Written directly to the vehicle (not via
    the context), so this temporary change doesn't appear in the wizard's change log.
    """
    if vehicle.has_parameter("ARMING_SKIPCHK"):
        name, skip_all = "ARMING_SKIPCHK", ARMING_SKIPCHK_ALL
    elif vehicle.has_parameter("ARMING_CHECK"):
        name, skip_all = "ARMING_CHECK", ARMING_CHECK_NONE
    else:
        yield
        return

    original = vehicle.get(name)
    if original == skip_all:
        yield
        return

    vehicle.set_param(name, skip_all)
    try:
        yield
    finally:
        try:
            vehicle.set_param(name, original)
        except VehicleError as error:
            raise VehicleError(
                f"Could not restore {name} to {original:g} after the motor test ({error}) - "
                f"set it back before driving."
            ) from error


class ServoTester:
    """Moves outputs on request and remembers each output's test position so repeated clicks step from it."""

    def __init__(self, ctx) -> None:
        """Bind to the wizard context; nothing is sent until a method is called."""
        self._ctx = ctx
        # our own, separate from drive-test runs
        self._steering_driver = RcOverrideDriver(ctx.vehicle)
        # current held steering stick position, -1..1
        self._steering_position = 0.0
        # wizard-clock time the steering override ends, and how long the current hold lasts (for the log)
        self._steering_release_at = 0.0
        self._steering_hold_seconds = 0.0
        # the thread waiting to end the hold; None once it has decided to (under the lock), so a click after that
        # starts a new one instead of being released by the old one
        self._release_thread: Optional[threading.Thread] = None
        # while before this, an armed vehicle is armed by OUR motor test
        self._motor_test_ends_at = 0.0
        # output -> last pulse width sent with DO_SET_SERVO
        self._pwm: dict = {}
        # output -> last motor-test percentage
        self._percent: dict = {}
        # wizard-clock time the swapped functions are restored
        self._swap_release_at = 0.0
        self._swap_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        # output -> original SERVOx_FUNCTION while the panel moves it as a plain output (see FUNCTION_SWAP_...);
        # a swap left over from a crash or a lost link is restored by restore_leftovers()
        self._swapped: dict = {
            int(output): function for output, function in (ctx.store.get(SWAPPED_FUNCTIONS_KEY, {}) or {}).items()
        }
        self._has_leftovers = bool(self._swapped)

    # ------------------------------------------------------------------ steering (RC override, disarmed)
    def hold_steering(self, position: float, seconds: float = STEERING_HOLD_SECONDS) -> None:
        """Hold the steering stick at `position` (-1 left .. +1 right) for a few seconds; the vehicle stays disarmed."""
        if self._ctx.vehicle.armed and not self._steering_driver.active:
            raise VehicleError("Disarm first: the steering test takes over the steering and throttle sticks.")

        with self._lock:
            previous_mode = self._ctx.vehicle.custom_mode
            if not self._steering_driver.active:
                # Manual mode + override pump (throttle held at neutral)
                self._steering_driver.start()
            elif previous_mode != MODES["MANUAL"]:
                # e.g. a failsafe switched to Hold during the hold; the override keeps the radio failsafe cleared
                self._ctx.vehicle.set_mode("MANUAL")

            if previous_mode != MODES["MANUAL"]:
                # Rover centres the steering in Hold (ModeHold::update) and other modes steer by themselves
                self._ctx.log(
                    f"Switched to MANUAL mode for the steering test (was "
                    f"{MODE_NAMES.get(previous_mode, previous_mode)})."
                )

            self._steering_position = _clamp(position, -1.0, 1.0)
            self._steering_driver.set(self._steering_position, 0.0)
            self._steering_release_at = timebase.now() + seconds
            self._steering_hold_seconds = seconds
            if self._release_thread is None:
                self._release_thread = threading.Thread(target=self._release_when_due, daemon=True)
                self._release_thread.start()

    def _release_when_due(self) -> None:
        """Background: give the steering back to the transmitter once no click extended the hold."""
        while True:
            with self._lock:
                # decided and done under the lock: a click can't slip in between
                if timebase.now() >= self._steering_release_at:
                    was_held = self._steering_driver.active
                    self._release_thread = None
                    self._end_steering_hold()
                    break
            timebase.sleep(RELEASE_CHECK_SECONDS)

        if was_held:
            self._ctx.log(f"No button press for {self._steering_hold_seconds:g}s. Resetting servo output.")

    def release(self) -> None:
        """End any steering hold and give swapped outputs their function back (called by STOP and on exit)."""
        self._release_steering()
        self.restore_functions()

    def _release_steering(self) -> None:
        """End any steering hold now."""
        with self._lock:
            self._end_steering_hold()

    def _end_steering_hold(self) -> None:
        """Centre and release the steering override (call with the lock held)."""
        self._steering_release_at = 0.0
        self._steering_position = 0.0
        if self._steering_driver.active:
            self._steering_driver.abort()

    # ------------------------------------------------------------------ motor outputs (motor test)
    def run_motor_test(self, instance: int, percent: float, seconds: float = MOTOR_TEST_SECONDS) -> None:
        """Run ArduPilot's motor test on one motor output; refuses if the pilot armed the vehicle."""
        vehicle = self._ctx.vehicle
        if vehicle.armed and timebase.now() > self._motor_test_ends_at:
            raise VehicleError(
                "The vehicle is armed. ArduPilot's motor test disarms it when the test ends - " "disarm first."
            )

        with arming_checks_skipped(vehicle):
            result = vehicle.motor_test(instance, percent, seconds)

        if result not in ACCEPTED_RESULTS:
            raise VehicleError(f"Motor test refused (result {result}): {MOTOR_TEST_REQUIREMENTS} {vehicle.last_text()}")
        # While our test runs the vehicle is armed by it; a further click just updates the running test.
        self._motor_test_ends_at = timebase.now() + seconds + 1.0

    # ------------------------------------------------------------------ servo panel
    def output_count(self) -> int:
        """How many outputs the panel shows: up to the highest one that is active or has a function (at least 8)."""
        vehicle = self._ctx.vehicle
        highest = MIN_PANEL_OUTPUTS
        raw = vehicle.servo_raw() or []
        for output in SERVO_OUTPUTS:
            # active (reported with a pulse) or set up
            if (output <= len(raw) and raw[output - 1]) or vehicle.get_int(f"SERVO{output}_FUNCTION"):
                highest = max(highest, output)
        return highest

    def nudge(self, output: int, direction: int) -> str:
        """Move one output a step up (+1) or down (-1); returns a line for the log."""
        vehicle = self._ctx.vehicle
        function = vehicle.get_int(f"SERVO{output}_FUNCTION")
        if output in self._swapped or function in SERVO_FUNCTIONS_RESOLVED_AT_RUNTIME:
            return self._nudge_swapped(output, function, direction)

        if function in SERVO_FUNCTIONS_DIRECTLY_SETTABLE:
            # Start from our last command, else what the output shows now, else its trim.
            if not self._pwm.get(output):
                self._pwm[output] = self._current_pulse(output)
            return f"SERVO{output}: {self._step_pulse(output, direction)} us"

        if function == SERVO_FUNCTION_GROUND_STEERING:
            # Step from the held position if a hold is running, else from centre.
            start = self._steering_position if self._steering_driver.active else 0.0
            self.hold_steering(start + direction * PANEL_STEERING_STEP, PANEL_STEERING_HOLD_SECONDS)
            return (
                f"{self._reported_pulse(output)} (steering held at {self._steering_position:+.0%}, "
                "disarmed, no motor test)"
            )

        if function in MOTOR_TEST_INSTANCE_BY_FUNCTION:
            # one step
            percent = _clamp(self._percent.get(output, 0) + direction * PANEL_MOTOR_STEP, -100, 100)
            instance = MOTOR_TEST_INSTANCE_BY_FUNCTION[function]
            self.run_motor_test(instance, percent)
            self._percent[output] = percent
            return (
                f"{self._reported_pulse(output)} ({MOTOR_NAMES[instance]} motor test {percent:+.0f}% for "
                f"{MOTOR_TEST_SECONDS:g} s - arms the vehicle for the test, wheels off the ground!)"
            )

        raise VehicleError(
            f"SERVO{output} is used by function {function}: ArduPilot doesn't move it on command, and "
            "that function is only set up when the flight controller boots, so it can't be switched "
            "off for a moment either."
        )

    # ------------------------------------------------------------------ function swap (e.g. gimbal servos)
    def _nudge_swapped(self, output: int, function: int, direction: int) -> str:
        """Move an output whose function ArduPilot resolves at runtime: switch its function to "none" (once), step
        the pulse with DO_SET_SERVO, and give the function back PANEL_STEERING_HOLD_SECONDS after the last click."""
        vehicle = self._ctx.vehicle
        notes = []
        if output not in self._swapped:
            # start where the function left the output
            self._pwm[output] = self._current_pulse(output)
            vehicle.set_param(f"SERVO{output}_FUNCTION", SERVO_FUNCTION_DISABLED)
            with self._lock:
                self._swapped[output] = function
                self._persist_swaps()
            notes.append(f"{SERVO_FUNCTIONS_RESOLVED_AT_RUNTIME.get(function, f'function {function}')} paused")
            # until ArduPilot has picked up the change, the function would overwrite our pulse
            timebase.sleep(FUNCTION_SWAP_SETTLE_SECONDS)

        pwm = self._step_pulse(output, direction)
        with self._lock:
            self._swap_release_at = timebase.now() + PANEL_STEERING_HOLD_SECONDS
            if self._swap_thread is None:
                self._swap_thread = threading.Thread(target=self._restore_when_due, daemon=True)
                self._swap_thread.start()

        notes.append(f"its function returns {PANEL_STEERING_HOLD_SECONDS:g} s after the last click")
        return f"SERVO{output}: {pwm} us ({', '.join(notes)})"

    def _restore_when_due(self) -> None:
        """Background: give swapped outputs their function back once no click extended the hold."""
        while True:
            with self._lock:
                # decided under the lock; a later click starts a new thread
                if timebase.now() >= self._swap_release_at:
                    self._swap_thread = None
                    # 0 = restore_functions() already ran (STOP, quit)
                    already_restored = self._swap_release_at == 0.0
                    break
            timebase.sleep(RELEASE_CHECK_SECONDS)

        if not already_restored:
            self.restore_functions()

    def restore_leftovers(self) -> None:
        """Once, when the wizard's log is ready: restore functions a crash or lost link left switched off."""
        if not self._has_leftovers:
            return

        self._has_leftovers = False
        self._ctx.log(
            "Restoring servo output functions the wizard had switched off: "
            + ", ".join(f"SERVO{output}" for output in self._swapped)
        )
        self.restore_functions()

    def restore_functions(self) -> None:
        """Give every swapped output its original function back (a failed write is kept and retried later)."""
        with self._lock:
            self._swap_release_at = 0.0
            swapped = dict(self._swapped)

        for output, function in swapped.items():
            try:
                self._ctx.vehicle.set_param(f"SERVO{output}_FUNCTION", function)
            except VehicleError as error:
                self._ctx.log(
                    f"❗ Could not give SERVO{output} its function {function} back ({error}) - the wizard "
                    f"retries on its next start; or set SERVO{output}_FUNCTION = {function} yourself."
                )
                continue

            with self._lock:
                self._swapped.pop(output, None)
                self._pwm.pop(output, None)
                self._persist_swaps()
            name = SERVO_FUNCTIONS_RESOLVED_AT_RUNTIME.get(function, f"function {function}")
            self._ctx.log(f"SERVO{output}: back to {name}.")

    def _persist_swaps(self) -> None:
        """Save the swapped outputs (call with the lock held)."""
        self._ctx.store.set(
            SWAPPED_FUNCTIONS_KEY, {str(output): function for output, function in self._swapped.items()}
        )

    def _current_pulse(self, output: int) -> int:
        """What the output shows now, else its trim."""
        vehicle = self._ctx.vehicle
        return vehicle.servo_pulse(output) or vehicle.get(f"SERVO{output}_TRIM", RC_PWM_NOMINAL_TRIM)

    def _step_pulse(self, output: int, direction: int) -> int:
        """Move an output PANEL_PWM_STEP up (+1) or down (-1) from its last commanded pulse, kept inside its MIN..MAX,
        with DO_SET_SERVO; returns the new pulse."""
        vehicle = self._ctx.vehicle
        low = vehicle.get(f"SERVO{output}_MIN", PWM_LIMITS[0])
        high = vehicle.get(f"SERVO{output}_MAX", PWM_LIMITS[1])
        pwm = int(_clamp(self._pwm[output] + direction * PANEL_PWM_STEP, low, high))
        result = vehicle.set_servo_pwm(output, pwm)
        if result not in ACCEPTED_RESULTS:
            raise VehicleError(f"SERVO{output}: set-servo refused (result {result}). {vehicle.last_text()}")

        self._pwm[output] = pwm
        return pwm

    def _reported_pulse(self, output: int) -> str:
        """'SERVOn: <pulse> us' as the output reports it once the command took effect.

        For steering and motor outputs the wizard sends a stick position or a motor-test percentage, not a pulse
        width; ArduPilot turns that into the pulse (MIN/TRIM/MAX, reversal, ...), so the reported value is shown.
        """
        timebase.sleep(OUTPUT_SETTLE_SECONDS)
        pulse = self._ctx.vehicle.servo_pulse(output)
        return f"SERVO{output}: {pulse} us" if pulse else f"SERVO{output}: (output not reported)"

    # ------------------------------------------------------------------ MIN / TRIM / MAX buttons
    def nudge_endpoint(self, function: int, suffix: str, field_key: str, direction: int) -> None:
        """Change SERVOn_<suffix> by one step immediately and, for the steering servo, show the new position.

        The steering servo is held at the changed end point (MIN = full left, MAX = full right, TRIM = centre,
        SERVOn_REVERSED respected). Motor outputs are not driven: a disarmed Rover sends the ESC its TRIM (no pulse
        if MOT_SAFE_DISARM = 1), and driving to MIN/MAX would mean full power.
        """
        ctx = self._ctx
        output = ctx.output_with_function(function)
        if not output:
            raise VehicleError("Assign the output first (enter its number and press Apply).")

        names = {part: f"SERVO{output}_{part}" for part in ("MIN", "TRIM", "MAX")}
        values = {part: ctx.param_value(name, RC_PWM_NOMINAL_TRIM) for part, name in names.items()}
        current = ctx.number(field_key, values[suffix])
        # Keep MIN < TRIM < MAX: each end point may only move up to 1 us short of its neighbour.
        low = values["MIN"] + 1 if suffix == "TRIM" else values["TRIM"] + 1 if suffix == "MAX" else PWM_LIMITS[0]
        high = values["MAX"] - 1 if suffix == "TRIM" else values["TRIM"] - 1 if suffix == "MIN" else PWM_LIMITS[1]
        new = int(_clamp(current + direction * ENDPOINT_STEP, low, high))
        ctx.set_param(names[suffix], new)
        ctx.set_values(**{field_key: new})
        if function == SERVO_FUNCTION_GROUND_STEERING:
            reversed_output = ctx.param_int(f"SERVO{output}_REVERSED") == 1
            # Stick -1 drives the MIN end, +1 the MAX end; a reversed output swaps them.
            position = {"TRIM": 0.0, "MIN": -1.0, "MAX": 1.0}[suffix] * (-1 if reversed_output else 1)
            self.hold_steering(position)
        else:
            ctx.log(
                f"{names[suffix]} = {new} (motor outputs are not driven; while disarmed the ESC receives TRIM, "
                "or no pulse if MOT_SAFE_DISARM = 1)"
            )


def steering_test_action(direction: int):
    """A Test L / Test R button: hold the steering at the chosen deflection (disarmed, no motor test)."""

    def run_steering_test(ctx):
        """Deflect the steering left/right by the 'Steering test deflection' percentage for a few seconds."""
        percent = ctx.number("test_pct", DEFAULT_STEERING_TEST_PERCENT)
        # percent of full stick deflection
        ctx.servos.hold_steering(direction * percent / 100)
        ctx.log(
            f"Steering test: {'RIGHT' if direction > 0 else 'LEFT'} {percent:.0f}% for "
            f"{STEERING_HOLD_SECONDS:g} s (vehicle stays disarmed)"
        )

    return run_steering_test
