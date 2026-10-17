"""rover_wizard.motion.driving - making the vehicle drive for a measurement, and recording what it did.

Two ways to drive a test run:
* open loop: the wizard moves the virtual sticks (RC override in Manual mode) - `RcOverrideDriver`
* closed loop: the wizard sends speed + turn-rate targets in Guided mode
and one way to record a pilot-driven run. `DriveSession` implements all three with the same safety rules: vehicle
must be armed, every run is limited in time and distance, Cancel/STOP and an unexpected disarm abort it, and the
vehicle is always commanded to zero and switched to Hold at the end.

A DriveSession talks to the pilot only through a small "prompter" (the wizard context): check(), log(),
require_armed(), pause(), begin_wait(), end_wait(), was_continue_pressed().
"""

from __future__ import annotations

import math
import threading
from typing import Optional
from collections.abc import Callable

from ..util import timebase
from ..util.errors import VehicleError
from ..vehicle.rc_scaling import stick_to_pwm
from ..vehicle.link import RC_OVERRIDE_CHANNELS, DriveSample, Vehicle

# re-send the override at 10 Hz (ArduPilot drops it after RC_OVERRIDE_TIME)
OVERRIDE_REFRESH_SECONDS = 0.1
# stick commands / recorded samples per second in open-loop and manual runs
OPEN_LOOP_RATE_HZ = 20
# velocity/turn-rate targets per second in Guided runs
GUIDED_RATE_HZ = 10
# a pilot-driven recording stops after 4 minutes at the latest
MANUAL_RECORD_MAX_SECONDS = 240
# abort a run after this many consecutive samples without speed/turn rate
MAX_MISSING_TELEMETRY_SAMPLES = 30
# [m/s] during the post-run tail, below this the vehicle counts as stopped
STOPPED_SPEED = 0.1
# [s] centred sticks are sent this long before the override is released (the vehicle sees "stop" first)
CENTRED_STOP_SECONDS = 0.4
# [s] longest wait for the override thread to end (it re-sends every OVERRIDE_REFRESH_SECONDS)
PUMP_STOP_WAIT_SECONDS = 1.0
# drive tests bring the throttle up over this time (no jolt, no wheelspin)
THROTTLE_RAMP_SECONDS = 1.0


def ramp_up(target: float, t: float, seconds: float = THROTTLE_RAMP_SECONDS) -> float:
    """Command for a run_open_loop() control: rises linearly from 0 to `target` over `seconds`, then holds it."""
    return target * min(1.0, t / seconds)


class RcOverrideDriver:
    """Holds virtual steering/throttle sticks via RC override in Manual mode (expires by itself if the wizard dies)."""

    def __init__(self, vehicle: Vehicle) -> None:
        """Bind to a vehicle; nothing is sent until start()."""
        self._vehicle = vehicle
        self._steering = self._throttle = 0.0
        self.active = False
        self._thread: Optional[threading.Thread] = None
        # set by set(): new sticks go out at once, not with the next refresh
        self._changed = threading.Event()

    def start(self) -> None:
        """Switch to Manual mode and start sending the virtual sticks (centred)."""
        if self.active:
            return
        steering_channel, throttle_channel = self._vehicle.rc_mapping()
        if max(steering_channel, throttle_channel) > RC_OVERRIDE_CHANNELS:
            raise VehicleError(
                f"RC override only covers channels 1-{RC_OVERRIDE_CHANNELS}, but RCMAP_ROLL/RCMAP_THROTTLE "
                f"are {steering_channel}/{throttle_channel}. Remap them or drive manually."
            )

        self._vehicle.set_mode("MANUAL")
        self._steering = self._throttle = 0.0
        self.active = True
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()

    def set(self, steering: float, throttle: float) -> None:
        """New virtual stick positions, each -1..1 - sent right away if they changed."""
        changed = (steering, throttle) != (self._steering, self._throttle)
        self._steering = steering
        self._throttle = throttle
        if changed:
            self._changed.set()

    def stop(self) -> None:
        """Centre the sticks for a moment, then release the override."""
        if not self.active:
            return

        self.set(0.0, 0.0)
        # a few centred packets so the vehicle sees "stop" before control returns to the pilot
        timebase.sleep(CENTRED_STOP_SECONDS)
        self._end_pump()
        self._vehicle.rc_release()

    def abort(self) -> None:
        """Release the override immediately (emergency stop)."""
        self._steering = self._throttle = 0.0
        # the pump may be about to send the old sticks: let it finish before releasing, or it re-applies them
        self._end_pump()
        self._vehicle.rc_release()

    def _end_pump(self) -> None:
        """Stop re-sending the override and wait for the sending thread to finish."""
        self.active = False
        # wake the thread now instead of after its refresh period
        self._changed.set()
        if self._thread:
            self._thread.join(PUMP_STOP_WAIT_SECONDS)

    def _pump(self) -> None:
        """Background thread: keep re-sending the virtual sticks so the override doesn't expire."""
        steering_channel, throttle_channel = self._vehicle.rc_mapping()
        while self.active:
            # cleared before reading the sticks: a set() from now on is sent by the next round
            self._changed.clear()
            self._vehicle.rc_override(
                {
                    steering_channel: stick_to_pwm(self._steering, self._vehicle.rc_calibration(steering_channel)),
                    throttle_channel: stick_to_pwm(self._throttle, self._vehicle.rc_calibration(throttle_channel)),
                }
            )
            timebase.wait_for(self._changed, OVERRIDE_REFRESH_SECONDS)


class _DistanceTracker:
    """How far the vehicle has got from where a run started, to enforce the runway/circle limit."""

    def __init__(self) -> None:
        """Start at zero distance."""
        self._origin: Optional[tuple] = None
        self._last_time: Optional[float] = None
        self.distance = 0.0

    def update(self, sample: DriveSample) -> float:
        """Add one sample and return the current distance [m]."""
        if sample.north is not None:
            # With a position estimate: straight-line distance from the start point (Pythagoras on north/east).
            if self._origin is None:
                self._origin = (sample.north, sample.east)
            self.distance = math.hypot(sample.north - self._origin[0], sample.east - self._origin[1])
        elif self._last_time is not None:
            # Without one: integrate |speed| over time (distance travelled - an upper bound of the displacement).
            self.distance += abs(sample.forward_speed or 0.0) * max(0.0, sample.time - self._last_time)

        self._last_time = sample.time
        return self.distance


class DriveSession:
    """Runs and records drive tests safely (see module docstring)."""

    def __init__(self, vehicle: Vehicle, prompter) -> None:
        """Bind to a vehicle and to the wizard context used for Cancel checks, logging and prompts."""
        self._vehicle = vehicle
        self._prompter = prompter
        self.rc_driver = RcOverrideDriver(vehicle)
        # True if the last run was stopped by the distance limit
        self.hit_limit = False

    # ------------------------------------------------------------------ public runs
    def safety_brief(self, what: str) -> None:
        """Explain what the wizard is about to do on its own, and wait for the pilot's go-ahead."""
        self._prompter.pause(
            "AUTOMATIC DRIVING: " + what + " Keep your transmitter in hand (switch to Hold/Manual "
            "to take over) and the STOP button ready. Clear the area, then press Continue to start."
        )

    def run_open_loop(
        self,
        profile: Callable,
        duration: float,
        limit_m: Optional[float] = None,
        hz: float = OPEN_LOOP_RATE_HZ,
        stop_if: Optional[Callable] = None,
        tail: float = 0.0,
    ) -> list:
        """Drive with virtual sticks: profile(t) -> (steering, throttle) in -1..1. Returns the recorded samples.

        The samples' steering/throttle inputs are the commanded stick positions (what was actually sent), so the
        analyses don't depend on the flight controller echoing the override back in RC_CHANNELS.
        """
        # before anything changes on the vehicle (Manual mode, override)
        self._prompter.require_armed()
        self.rc_driver.start()
        try:
            return self._run(
                lambda command: self.rc_driver.set(*command),
                profile,
                duration,
                limit_m,
                hz,
                stop_if,
                tail,
                commands_are_sticks=True,
            )
        finally:
            self.rc_driver.stop()
            self._switch_to_hold()

    def run_guided(
        self, profile: Callable, duration: float, limit_m: Optional[float] = None, hz: float = GUIDED_RATE_HZ
    ) -> list:
        """Drive with Guided-mode targets: profile(t) -> (speed m/s, turn rate rad/s). Returns the samples."""
        self._prompter.require_armed()
        self._vehicle.set_mode("GUIDED")
        try:
            return self._run(
                lambda command: self._vehicle.guided_cmd(*command),
                profile,
                duration,
                limit_m,
                hz,
                None,
                0.0,
                commands_are_sticks=False,
            )
        finally:
            # several zero targets: one lost packet must not leave the vehicle moving
            for _ in range(3):
                self._vehicle.guided_cmd(0.0, 0.0)
            self._switch_to_hold()

    def record_manual(
        self,
        message: str,
        hz: float = OPEN_LOOP_RATE_HZ,
        max_seconds: float = MANUAL_RECORD_MAX_SECONDS,
        mode: str = "MANUAL",
    ) -> list:
        """The pilot drives with the transmitter; record samples until Continue is pressed."""
        self._prompter.require_armed()
        try:
            self._vehicle.set_mode(mode)
        except VehicleError:
            self._prompter.log(f"WARNING: could not force {mode} mode - make sure your transmitter is in {mode}.")
        self._prompter.begin_wait(message)
        samples = []
        start = timebase.now()
        try:
            while not self._prompter.was_continue_pressed() and timebase.now() - start < max_seconds:
                self._prompter.check()
                sample = self._vehicle.sample()
                sample.run_time = timebase.now() - start
                if sample.is_usable():
                    samples.append(sample)
                # sampling period = 1 / sampling rate
                timebase.sleep(1 / hz)
        finally:
            self._prompter.end_wait()
        return samples

    def stop(self) -> None:
        """End any open-loop control normally (called after every action)."""
        self.rc_driver.stop()

    def abort(self) -> None:
        """Release open-loop control immediately (STOP button)."""
        self.rc_driver.abort()

    # ------------------------------------------------------------------ internals
    def _run(
        self,
        send_command: Callable,
        profile: Callable,
        duration: float,
        limit_m: Optional[float],
        hz: float,
        stop_if: Optional[Callable],
        tail: float,
        commands_are_sticks: bool,
    ) -> list:
        """Shared run loop: command, sample, and stop on duration / stop_if / distance limit (+ optional tail).
        The callers check that the vehicle is armed before they change its mode."""
        samples: list = []
        self.hit_limit = False
        distance = _DistanceTracker()
        missing_telemetry = 0
        start = timebase.now()
        # run time at which the post-trigger tail ends
        stop_at: Optional[float] = None
        try:
            while True:
                self._prompter.check()
                if not self._vehicle.armed:
                    raise VehicleError("Vehicle disarmed during the run - aborted.")

                run_time = timebase.now() - start
                if stop_at is None and run_time >= duration:
                    break

                # During the tail (after a stop trigger) everything is commanded to zero while we keep recording.
                command = profile(run_time) if stop_at is None else (0.0, 0.0)
                send_command(command)
                sample = self._vehicle.sample()
                sample.run_time = run_time
                sample.command = command
                if commands_are_sticks:
                    sample.steering_input, sample.throttle_input = command

                if not sample.is_usable():
                    missing_telemetry += 1
                    if missing_telemetry > MAX_MISSING_TELEMETRY_SAMPLES:
                        raise VehicleError("No speed/yaw telemetry (need ATTITUDE + position or VFR_HUD) - aborted.")
                    timebase.sleep(1 / hz)
                    continue

                missing_telemetry = 0
                samples.append(sample)
                if stop_at is None:
                    triggered = False
                    if limit_m and distance.update(sample) > limit_m:
                        self.hit_limit = triggered = True
                        self._prompter.log("WARNING: run limit reached - stopping early.")
                    elif stop_if and stop_if(samples):
                        triggered = True

                    if triggered:
                        if not tail:
                            break
                        stop_at = run_time + tail
                elif run_time >= stop_at or (
                    sample.forward_speed is not None and abs(sample.forward_speed) < STOPPED_SPEED
                ):
                    break
                timebase.sleep(1 / hz)
        finally:
            send_command((0.0, 0.0))
        return samples

    def _switch_to_hold(self) -> None:
        """Best effort: park the vehicle in Hold after a run (never raises - we may be cleaning up after an error)."""
        try:
            if self._vehicle.armed:
                self._vehicle.set_mode("HOLD", timeout=1.5)
        except VehicleError:
            pass
