"""rover_wizard.vehicle.link - the MAVLink link to an ArduPilot Rover/Boat.

Owns the connection, a background reader thread (with automatic reconnect), the parameter cache, command sending
and telemetry. Other modules use its methods only - its caches (messages, parameters, inboxes) are private.
pymavlink is imported lazily in connect(), so the rest of the wizard (and the self-test) works without it.
"""

from __future__ import annotations

import collections
import contextlib
import math
import re
import struct
import threading
from dataclasses import dataclass
from typing import Any, Optional
from collections.abc import Callable

from ..util import timebase
from ..util.errors import VehicleError
from ..util.values import escape_markup
from ..rover_defs import (
    LEGACY_RELAY_PIN,
    MODES,
    RC_PWM_NOMINAL_MAX,
    RC_PWM_NOMINAL_MIN,
    RC_PWM_NOMINAL_TRIM,
    RELAYS,
    RELAY_FUNCTION_RELAY,
    SERVO_OUTPUTS,
)
from ..mavlink_defs import (
    ACCEPTED_RESULTS,
    ARM_DISARM_FORCE_MAGIC_NUMBER,
    GUIDED_VELOCITY_YAW_RATE_TYPEMASK,
    MAVLINK_MSG_ID_ATTITUDE,
    MAVLINK_MSG_ID_AUTOPILOT_VERSION,
    MAVLINK_MSG_ID_DISTANCE_SENSOR,
    MAVLINK_MSG_ID_EKF_STATUS_REPORT,
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    MAVLINK_MSG_ID_GPS2_RAW,
    MAVLINK_MSG_ID_GPS_RAW_INT,
    MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
    MAVLINK_MSG_ID_RAW_IMU,
    MAVLINK_MSG_ID_RC_CHANNELS,
    MAVLINK_MSG_ID_RELAY_STATUS,
    MAVLINK_MSG_ID_RPM,
    MAVLINK_MSG_ID_SCALED_IMU,
    MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
    MAVLINK_MSG_ID_SYS_STATUS,
    MAVLINK_MSG_ID_VFR_HUD,
    MAVLINK_MSG_ID_WHEEL_DISTANCE,
    MAV_AUTOPILOT_ARDUPILOTMEGA,
    MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
    MAV_AUTOPILOT_INVALID,
    MAV_CMD_ACCELCAL_VEHICLE_POS,
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
    MAV_CMD_DO_GRIPPER,
    MAV_CMD_DO_MOTOR_TEST,
    MAV_CMD_DO_MOUNT_CONTROL,
    MAV_CMD_DO_SET_MODE,
    MAV_CMD_DO_SET_RELAY,
    MAV_CMD_DO_SET_SERVO,
    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
    MAV_CMD_REQUEST_MESSAGE,
    MAV_CMD_SET_EKF_SOURCE_SET,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    MAV_DATA_STREAM_ALL,
    MAV_FRAME_BODY_NED,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MAV_MOUNT_MODE_MAVLINK_TARGETING,
    MAV_PARAM_TYPE_REAL32,
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_IN_PROGRESS,
    MAV_SEVERITY_INFO,
    MAV_STATE_ACTIVE,
    MAV_TYPE_GCS,
    MAV_TYPE_GROUND_ROVER,
    MAV_TYPE_SURFACE_BOAT,
    MILLIRAD_PER_RAD,
    MILLI_G_PER_G,
    MOTOR_TEST_ORDER_DEFAULT,
    MOTOR_TEST_THROTTLE_PERCENT,
    PARAM_INDEX_UNKNOWN,
    RC_CHANNELS_MAX,
    UINT16_MAX,
    SCALED_PRESSURE_CENTIDEGREES_PER_DEGREE,
)
from .rc_scaling import RcChannelCalibration, pwm_to_stick

# this long without a heartbeat -> the reader starts reconnecting
RECONNECT_AFTER_SECONDS = 5.0
# minimum time between reconnect attempts
RECONNECT_RETRY_INTERVAL = 2.0
# a heartbeat younger than this means "link OK"
HEARTBEAT_FRESH_SECONDS = 3.0
# [s] between the wizard's own heartbeats. ArduPilot counts the ground station as present (GCS failsafe,
# FS_GCS_ENABLE) only from its heartbeats - and from its RC overrides: one override without heartbeats after it
# leaves the GCS failsafe on, which also keeps every other failsafe from being reported as cleared.
GCS_HEARTBEAT_PERIOD_SECONDS = 1.0
# older attitude/position/speed data is not used for drive samples
TELEMETRY_FRESH_SECONDS = 1.0
# older RC_CHANNELS data counts as "no RC"
RC_TELEMETRY_FRESH_SECONDS = 1.5
# RC_CHANNELS_OVERRIDE carries channels 1..8
RC_OVERRIDE_CHANNELS = 8
# our MAVLink identity until the ground-station id parameter is known: system id ...
GCS_SOURCE_SYSTEM = 255
# ... and component id (MAV_COMP_ID_MISSIONPLANNER)
GCS_SOURCE_COMPONENT = 190
# The ground-station system id parameter: current name first, then its name before the rename.
GCS_SYSTEM_ID_PARAMETERS = ("MAV_GCS_SYSID", "SYSID_MYGCS")
# stick below this means "driving backwards" (for unsigned ground speed)
REVERSE_THROTTLE_THRESHOLD = -0.05
# [s] a rebooting board must answer again within this time
REBOOT_RECONNECT_TIMEOUT = 90.0
# each reconnect attempt after a reboot waits this long for a heartbeat
REBOOT_OPEN_ATTEMPT_SECONDS = 6.0
# each automatic reconnect attempt waits this long for a heartbeat
RECONNECT_HEARTBEAT_SECONDS = 3.0
# [s] the reader waits this long for a message before it checks the heartbeat and the stop request
READ_TIMEOUT_SECONDS = 0.5
# [s] disconnect() waits this long for the reader - longer than one reconnect attempt
READER_STOP_WAIT_SECONDS = RECONNECT_HEARTBEAT_SECONDS + 2.0
# "received" time of a message that never arrived (a time long ago)
NEVER = -1e9
# how often the parameter download checks its progress
PARAM_PROGRESS_POLL_SECONDS = 0.25
# re-request at most this many missing parameters per round ...
PARAM_RE_REQUEST_BATCH = 60
# ... spaced this far apart [s] ...
PARAM_RE_REQUEST_SPACING = 0.01
# ... for at most this many rounds
PARAM_MAX_RETRY_ROUNDS = 10
# [s] no new parameter for this long: the download has stalled, ask for the missing ones
PARAM_STALL_SECONDS = 2.5
# PARAM_SET is repeated up to 3 times ...
PARAM_SET_ATTEMPTS = 3
# ... waiting this long each time for the confirming PARAM_VALUE
PARAM_SET_CONFIRM_SECONDS = 1.5
# PARAM_REQUEST_READ by name is repeated up to 3 times, waiting this long each time: older firmware doesn't answer
# for a name it doesn't have (current firmware sends PARAM_ERROR)
PARAM_READ_ATTEMPTS = 3
PARAM_READ_CONFIRM_SECONDS = 1.0
# PARAM_REQUEST_READ: param_index -1 means "look the name up"
PARAM_READ_BY_NAME = -1
# polling interval while waiting for a reply
POLL_SECONDS = 0.02
# [s] between the SET_MESSAGE_INTERVAL requests
STREAM_REQUEST_SPACING = 0.02
# AUTOPILOT_VERSION is requested up to 3 times ...
VERSION_REQUEST_ATTEMPTS = 3
# ... waiting this long each time
VERSION_REPLY_SECONDS = 1.5
# [s] for the heartbeat to show a new mode
MODE_CHANGE_TIMEOUT = 3.0
# [s] for the heartbeat to show "armed"
ARM_CONFIRM_SECONDS = 3.0
# serial connections: the usual USB / telemetry-radio speed
DEFAULT_BAUD_RATE = 115200
# telemetry older than this isn't shown in the live panel
MONITOR_MAX_AGE_SECONDS = 2.0
# barometer n reports in SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 (GCS_Common.cpp send_scaled_pressure*);
# they come with the raw-sensors telemetry group, which setup_streams() requests at ALL_STREAMS_RATE_HZ
PRESSURE_MESSAGES = {1: "SCALED_PRESSURE", 2: "SCALED_PRESSURE2", 3: "SCALED_PRESSURE3"}
# RCMAP_ROLL, RCMAP_THROTTLE defaults: steering on RC1, throttle on RC3
ARDUPILOT_DEFAULT_RCMAP = (1, 3)

# [s] between RELAY_STATUS reports: relays rarely change, and a click asks for the new state at once
RELAY_STATUS_PERIOD_SECONDS = 3.0
# Every telemetry group (REQUEST_DATA_STREAM, all streams) at this rate - for what STREAMS doesn't name, e.g. the
# compass calibration's MAG_CAL_PROGRESS/REPORT. Kept low: on a telemetry radio (57600 baud, half-duplex) all groups at
# 10 Hz fill the link, and the wizard's commands and the parameter download get through late. STREAMS (sent after
# it) sets the messages the wizard reads at their own rates.
ALL_STREAMS_RATE_HZ = 2
# Telemetry the wizard needs: (message id, rate in Hz).
STREAMS = [
    (MAVLINK_MSG_ID_ATTITUDE, 20),
    (MAVLINK_MSG_ID_VFR_HUD, 10),
    (MAVLINK_MSG_ID_RC_CHANNELS, 10),
    (MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
    (MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5),
    (MAVLINK_MSG_ID_GPS_RAW_INT, 2),
    (MAVLINK_MSG_ID_SYS_STATUS, 2),
    (MAVLINK_MSG_ID_WHEEL_DISTANCE, 10),
    (MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
    (MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 5),
    (MAVLINK_MSG_ID_SCALED_IMU, 10),
    (MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10),
    (MAVLINK_MSG_ID_DISTANCE_SENSOR, 10),
    (MAVLINK_MSG_ID_RAW_IMU, 5),
    (MAVLINK_MSG_ID_GPS2_RAW, 2),
    (MAVLINK_MSG_ID_RPM, 2),
    (MAVLINK_MSG_ID_RELAY_STATUS, 1 / RELAY_STATUS_PERIOD_SECONDS),
]

# Names of the "inboxes": queues of messages that actions consume one by one (in arrival order).
# positions requested by the 6-position accel calibration
INBOX_ACCEL_CALIBRATION = "ACCELCAL"
# compass calibration progress reports
INBOX_MAG_CAL_PROGRESS = "MAG_CAL_PROGRESS"
# compass calibration final reports
INBOX_MAG_CAL_REPORT = "MAG_CAL_REPORT"
# messages kept per inbox (an action that stops consuming must not let it grow forever)
INBOX_SIZE = 300
# a float32 is always given back exactly by 9 significant digits; fewer are tried first, from 6 on
FLOAT32_MIN_DIGITS = 6
FLOAT32_EXACT_DIGITS = 9
# a parameter-file line: NAME,VALUE (this wizard, Mission Planner, MAVProxy) or NAME VALUE; QGroundControl writes
# vehicle id, component id, NAME, VALUE, type - tab separated
PARAM_FILE_NAME = re.compile(r"[A-Z][A-Z0-9_]{0,15}")
QGC_PARAM_FILE_COLUMNS = 5
# where NAME, VALUE stand in a QGroundControl line
QGC_NAME_VALUE_COLUMNS = slice(2, 4)
# STATUSTEXT.text holds 50 characters; ArduPilot sends a longer text in pieces with the same id (GCS::send_textv) and
# ends it with a shorter piece. A text whose last piece got lost is shown after this long, as far as it arrived.
STATUSTEXT_PIECE_LENGTH = 50
STATUSTEXT_PIECES_WAIT_SECONDS = 1.0


@dataclass
class DriveSample:
    """What the vehicle was doing at one moment of a drive test."""

    # wizard-clock timestamp [s]
    time: float
    # signed speed along the vehicle's heading [m/s] (negative = reversing)
    forward_speed: Optional[float] = None
    # turn rate [rad/s] (positive = turning right)
    yaw_rate: Optional[float] = None
    # steering command, -1 (full left) .. +1 (full right)
    steering_input: Optional[float] = None
    # throttle command, -1 (full reverse) .. +1 (full forward)
    throttle_input: Optional[float] = None
    # local position north of the EKF origin [m]
    north: Optional[float] = None
    # local position east of the EKF origin [m]
    east: Optional[float] = None
    # seconds since the start of the current drive run
    run_time: float = 0.0
    # what the wizard commanded at this moment (open-loop or guided)
    command: Optional[tuple] = None

    def is_usable(self) -> bool:
        """Whether this sample has the speed and turn rate every analysis needs."""
        return self.forward_speed is not None and self.yaw_rate is not None


class Vehicle:
    """A MAVLink connection to one ArduPilot Rover/Boat, with cached telemetry and parameters."""

    def __init__(self) -> None:
        """Create an unconnected Vehicle. Call connect() to open the link."""
        # the pymavlink connection (replaced on reconnect/reboot)
        self.master = None
        # e.g. "/dev/ttyACM0" or "udpin:0.0.0.0:14550"
        self.connection_string = ""
        # serial speed (ignored for UDP/TCP)
        self.baud_rate = 115200
        # the vehicle's MAVLink system id
        self.system_id = 1
        # the autopilot's MAVLink component id
        self.component_id = 1
        # HEARTBEAT.type (MAV_TYPE_GROUND_ROVER / _SURFACE_BOAT)
        self.vehicle_type: Optional[int] = None
        # firmware (major, minor, patch)
        self.version: Optional[tuple] = None
        # from the latest heartbeat
        self.armed = False
        # flight mode number from the latest heartbeat
        self.custom_mode: Optional[int] = None
        # wizard-clock time of the latest heartbeat (0 = never)
        self.last_heartbeat_time = 0.0
        # last link problem ("" = none, "reconnecting ..." while retrying)
        self.link_error = ""
        # the pymavlink.mavutil module (set by connect())
        self._mavutil = None
        # parameter name -> value
        self._params: dict = {}
        # parameter name -> MAV_PARAM_TYPE
        self._param_types: dict = {}
        # how many parameters the vehicle reports it has
        self._param_total = 0
        # indices received during a full download
        self._param_indices_seen: set = set()
        # parameter name -> Event set when its PARAM_VALUE arrives
        self._param_wait_events: dict = {}
        # name -> MAV_PARAM_ERROR code of the latest PARAM_ERROR (current firmware: unknown name, write refused)
        self._param_errors: dict = {}
        # relay -> on, as the wizard last switched it (relays() without RELAY_STATUS: older firmware)
        self._switched_relays: dict = {}
        # connected with link_only (see connect)
        self._link_only = False
        # message type -> latest message
        self._messages: dict = {}
        # message type -> wizard-clock time it arrived
        self._message_times: dict = {}
        # (time, severity, text) of recent STATUSTEXT messages
        self._texts = collections.deque(maxlen=400)
        # total STATUSTEXTs ever received (a marker for "what's new")
        self._text_count = 0
        # component id -> (text id, severity, pieces so far, time of the latest piece) of a text arriving in pieces
        self._text_pieces: dict = {}
        # inbox name -> queue of messages for an action to consume (see INBOX_*), at most INBOX_SIZE each
        self._inbox = collections.defaultdict(lambda: collections.deque(maxlen=INBOX_SIZE))
        # command id -> latest COMMAND_ACK result
        self._command_acks: dict = {}
        # DISTANCE_SENSOR id -> (time, message): one entry per sensor/sector
        self._distance_sensors: dict = {}
        # tells the reader thread to exit
        self._stop_event = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        # serialises sends and link replacement
        self._lock = threading.Lock()

    # ================================================================== connection
    def connect(
        self,
        connection_string: str,
        # default baudrate
        baud_rate: int = 115200,
        # default timeout
        timeout: float = 20.0,
        progress: Optional[Callable] = None,
        link_only: bool = False,
    ) -> None:
        """Open the link and load everything the wizard needs (telemetry streams, firmware version, parameters).

        link_only: just the link (for reading/writing single parameters) - no telemetry requests, which ArduPilot
        saves as SRx_ stream rates, and no parameter download.
        """
        # imported lazily so everything else works without pymavlink
        from pymavlink import mavutil

        self._mavutil = mavutil
        self._link_only = link_only
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self._open(timeout)
        try:
            self._after_connect(progress)
        except BaseException:
            # e.g. the parameter download failed: don't leave the reader and the port behind for a retry
            self.disconnect()
            raise

    def _after_connect(self, progress: Optional[Callable] = None) -> None:
        """Runs after every (re)connect: request telemetry, read the firmware version, download all parameters."""
        if self._link_only:
            return
        self.setup_streams()
        self.read_version()
        self.fetch_params(progress)
        self._set_source(self.configured_gcs_system_id())

    def _new_link(self):
        """A fresh pymavlink connection with the wizard's default ground-station identity."""
        return self._mavutil.mavlink_connection(
            self.connection_string,
            baud=self.baud_rate,
            source_system=GCS_SOURCE_SYSTEM,
            source_component=GCS_SOURCE_COMPONENT,
        )

    def _open(self, timeout: float) -> None:
        """Open the link, wait for an ArduPilot Rover/Boat heartbeat, and start the reader thread.

        On failure the half-open connection is closed again, so repeated attempts (e.g. while waiting for a
        reboot) never leave a serial port blocked by a leaked handle.
        """
        try:
            link = self._new_link()
        # pymavlink raises whatever the transport raises (no such serial port, address in use, ...)
        except Exception as exc:
            raise VehicleError(f"Could not open {self.connection_string}: {exc}") from exc
        try:
            heartbeat = self._wait_for_ardupilot_heartbeat(link, timeout, announce=not self._link_only)
            if heartbeat is None:
                raise VehicleError("No ArduPilot heartbeat received - check the connection string / baud rate.")

            if heartbeat.type not in (MAV_TYPE_GROUND_ROVER, MAV_TYPE_SURFACE_BOAT):
                raise VehicleError(
                    f"Connected to an ArduPilot vehicle of MAV_TYPE {heartbeat.type}; "
                    "this wizard only supports Rover/Boat firmware."
                )
        except BaseException:
            _close_quietly(link)
            raise
        self.system_id = heartbeat.get_srcSystem()
        self.component_id = heartbeat.get_srcComponent()
        self.vehicle_type = heartbeat.type
        link.target_system = self.system_id
        link.target_component = self.component_id
        with self._lock:
            self.master = link

        self.last_heartbeat_time = timebase.now()
        self._start_reader()

    def _start_reader(self) -> None:
        """Start the reader thread (it also reconnects by itself whenever the heartbeat is missing)."""
        # a fresh event per reader: an old reader still finishing a reconnect attempt stays stopped
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader, args=(self._stop_event,), daemon=True)
        self._reader_thread.start()

    @staticmethod
    def _wait_for_ardupilot_heartbeat(link, timeout: float, announce: bool = False):
        """The first HEARTBEAT from an ArduPilot autopilot within `timeout` seconds, or None.

        announce: send the wizard's own GCS heartbeat meanwhile - UDP bridges (mavlink-router, Wi-Fi modules) often
        send only to a client they have heard from, and a reopened link has a new local port."""
        deadline = timebase.now() + timeout
        while timebase.now() < deadline:
            if announce:
                # a link that can't send yet (e.g. udpin before the first packet) just drops it
                with contextlib.suppress(Exception):
                    link.mav.heartbeat_send(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE)

            message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if message is not None and message.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA:
                return message

        return None

    def configured_gcs_system_id(self) -> int:
        """The system id the vehicle accepts RC overrides from: MAV_GCS_SYSID (SYSID_MYGCS before it was renamed)."""
        name = self.resolve(GCS_SYSTEM_ID_PARAMETERS)
        value = self.get(name) if name else None
        return int(value or GCS_SOURCE_SYSTEM)

    def _set_source(self, system_id: int) -> None:
        """RC override is only accepted from the ground station's system id, so send as that system id."""
        with self._lock:
            self.master.source_system = system_id
            self.master.mav.srcSystem = system_id

    @property
    def gcs_system_id(self) -> Optional[int]:
        """The MAVLink system id the wizard currently sends as (None before connecting)."""
        return None if self.master is None else int(self.master.source_system)

    def disconnect(self) -> None:
        """Stop the reader thread and close the link."""
        self._stop_event.set()
        if self._reader_thread:
            # the reader may be inside a reconnect attempt, waiting for a heartbeat
            self._reader_thread.join(READER_STOP_WAIT_SECONDS)

        with self._lock:
            _close_quietly(self.master)

    def reboot(self, progress: Optional[Callable] = None) -> None:
        """Reboot the flight controller and reconnect once it is back (up to 90 s)."""
        # param1 = 1: reboot the autopilot
        self.command_long(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1, wait=False)
        self.disconnect()
        # give the board time to actually go down before we look for it again
        timebase.sleep(4)
        self.armed = False
        deadline = timebase.now() + REBOOT_RECONNECT_TIMEOUT
        while True:
            try:
                self._open(REBOOT_OPEN_ATTEMPT_SECONDS)
                break
            # whatever the port or pymavlink raise while the board is still away: not back yet
            except Exception as error:
                if timebase.now() > deadline:
                    # Don't leave the link dead: the reader reconnects by itself, as after any lost link.
                    self.link_error = "reconnecting ..."
                    self._start_reader()
                    raise VehicleError(
                        f"The flight controller didn't answer within {REBOOT_RECONNECT_TIMEOUT:.0f} s "
                        f"after the reboot (last problem: {error}). The wizard keeps trying in the "
                        'background - the status bar shows "link OK" once it is back.'
                    ) from None
                timebase.sleep(1)
        self._after_connect(progress)

    # ================================================================== reader thread
    def _reader(self, stop: threading.Event) -> None:
        """Background thread: dispatch incoming messages; reconnect automatically after a lost heartbeat."""
        last_reconnect_attempt = 0.0
        last_heartbeat_sent = 0.0
        while not stop.is_set():
            # like every ground station; not for a quick parameter read (link_only): a GCS that appears and
            # vanishes again would trigger the GCS failsafe of an armed vehicle
            if not self._link_only and timebase.now() - last_heartbeat_sent >= GCS_HEARTBEAT_PERIOD_SECONDS:
                last_heartbeat_sent = timebase.now()
                self._send(
                    lambda: self.master.mav.heartbeat_send(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE)
                )
            try:
                message = self.master.recv_match(blocking=True, timeout=READ_TIMEOUT_SECONDS)
            # pymavlink raises assorted errors when a port vanishes / a socket resets
            except Exception as exc:
                self.link_error = str(exc)
                message = None
            self._finish_stale_text_pieces()

            if message is not None:
                self.link_error = ""
                try:
                    self._handle(message)
                # one malformed message must not kill the reader thread
                except Exception as exc:
                    self.link_error = f"bad message ignored: {exc}"
                continue

            silence = timebase.now() - self.last_heartbeat_time if self.last_heartbeat_time else 0.0
            if silence > RECONNECT_AFTER_SECONDS and timebase.now() - last_reconnect_attempt > RECONNECT_RETRY_INTERVAL:
                last_reconnect_attempt = timebase.now()
                self._attempt_reconnect(stop)

    def _attempt_reconnect(self, stop: threading.Event) -> None:
        """One reconnect attempt: reopen the same link in place, keeping all cached parameters and settings."""
        self.link_error = "reconnecting ..."
        try:
            with self._lock:
                _close_quietly(self.master)
            link = self._new_link()
            heartbeat = self._wait_for_ardupilot_heartbeat(
                link, RECONNECT_HEARTBEAT_SECONDS, announce=not self._link_only
            )

            if heartbeat is None:
                _close_quietly(link)
                # still down; the next attempt follows automatically
                return
            link.target_system = self.system_id
            link.target_component = self.component_id
            with self._lock:
                if stop.is_set():
                    # disconnected meanwhile: this link must not replace (or outlive) the closed one
                    _close_quietly(link)
                    return
                self.master = link

            self._set_source(self.configured_gcs_system_id())
            self.last_heartbeat_time = timebase.now()
            self.link_error = ""
            # a fresh link needs its telemetry requested again (never in link_only mode: ArduPilot saves the rates)
            if not self._link_only:
                self.setup_streams()
        # e.g. serial port not back yet
        except Exception as exc:
            self.link_error = f"reconnecting ... ({exc})"

    def _handle(self, message) -> None:
        """Update the cached state from one incoming message of the vehicle's autopilot.

        Other components of the vehicle (a MAVLink gimbal, a companion computer, ...) have their own heartbeat,
        parameters and command answers: only their texts are shown, everything else is the autopilot's.
        """
        msg_type = message.get_type()
        if msg_type == "BAD_DATA" or message.get_srcSystem() != self.system_id:
            return
        elif msg_type == "STATUSTEXT":
            self._handle_status_text(message)
            return
        elif message.get_srcComponent() != self.component_id:
            return

        timestamp = timebase.now()
        self._store_message(msg_type, message, timestamp)
        if msg_type == "HEARTBEAT":
            self.armed = bool(message.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
            self.custom_mode = message.custom_mode
            self.last_heartbeat_time = timestamp
        elif msg_type == "PARAM_VALUE":
            self._store_param(
                _mavlink_text(message.param_id),
                message.param_value,
                message.param_type,
                message.param_index,
                message.param_count,
            )
        elif msg_type == "PARAM_ERROR":
            self._store_param_error(_mavlink_text(message.param_id), message.error)
        elif msg_type == "COMMAND_ACK":
            self._command_acks[message.command] = message.result
        elif msg_type == "COMMAND_LONG" and message.command == MAV_CMD_ACCELCAL_VEHICLE_POS:
            self._push_inbox(INBOX_ACCEL_CALIBRATION, int(message.param1))
        elif msg_type in (INBOX_MAG_CAL_PROGRESS, INBOX_MAG_CAL_REPORT):
            self._push_inbox(msg_type, message)
        elif msg_type == "DISTANCE_SENSOR":
            self._store_distance_sensor(message, timestamp)
        elif msg_type == "NAMED_VALUE_FLOAT":
            # one per name, so each value can be read on its own (latest(named_value_key(name)))
            self._store_message(named_value_key(_mavlink_text(message.name)), message, timestamp)

    def _handle_status_text(self, message) -> None:
        """Collect a STATUSTEXT; a text sent in pieces (STATUSTEXT.id, MAVLink 2) is joined first, like QGroundControl
        does (StatusTextHandler) - otherwise e.g. a long PreArm reason would show only its first 50 characters."""
        component = message.get_srcComponent()
        text = _mavlink_text(message.text)
        # MAVLink 1 has no id: every message is a whole text
        text_id = getattr(message, "id", 0)
        pending = self._text_pieces.get(component)
        if pending is not None and pending[0] != text_id:
            # a new text before the end of the previous one: show what arrived
            self._finish_text_pieces(component)

        if text_id == 0:
            self._add_status_text(text, message.severity)
            return
        _, severity, pieces, _ = self._text_pieces.setdefault(component, (text_id, message.severity, {}, 0.0))
        pieces[getattr(message, "chunk_seq", 0)] = text
        self._text_pieces[component] = (text_id, severity, pieces, timebase.now())

        if len(text) < STATUSTEXT_PIECE_LENGTH:
            self._finish_text_pieces(component)

    def _finish_text_pieces(self, component: int) -> None:
        """Store a text that arrived in pieces; a lost piece shows as " ... "."""
        _, severity, pieces, _ = self._text_pieces.pop(component)
        self._add_status_text("".join(pieces.get(index, " ... ") for index in range(max(pieces) + 1)), severity)

    def _finish_stale_text_pieces(self) -> None:
        """Texts whose last piece never came: show them as far as they arrived."""
        for component, (_, _, _, latest) in list(self._text_pieces.items()):
            if timebase.now() - latest > STATUSTEXT_PIECES_WAIT_SECONDS:
                self._finish_text_pieces(component)

    # Small internal writers, also used by the simulator (a subclass) to inject telemetry.
    def _store_message(self, msg_type: str, message: Any, timestamp: Optional[float] = None) -> None:
        """Remember the latest message of a type and when it arrived."""
        self._messages[msg_type] = message
        self._message_times[msg_type] = timebase.now() if timestamp is None else timestamp

    def _store_param(self, name: str, value: float, param_type: int, index: int, total: int) -> None:
        """Cache one PARAM_VALUE and wake anyone waiting for it."""
        self._params[name] = value
        self._param_types[name] = param_type
        if index != PARAM_INDEX_UNKNOWN:
            self._param_indices_seen.add(index)

        self._param_total = total
        event = self._param_wait_events.get(name)
        if event:
            event.set()

    def _store_param_error(self, name: str, error: int) -> None:
        """Remember a PARAM_ERROR (the request or write failed) and wake anyone waiting for that parameter."""
        self._param_errors[name] = error
        event = self._param_wait_events.get(name)
        if event:
            event.set()

    def _add_status_text(self, text: str, severity: int = MAV_SEVERITY_INFO) -> None:
        """Remember one STATUSTEXT from the flight controller."""
        self._texts.append((timebase.now(), severity, text))
        self._text_count += 1

    def _store_distance_sensor(self, message: Any, timestamp: Optional[float] = None) -> None:
        """Remember the latest reading of each distance sensor (rangefinders and proximity sectors share the message)."""
        self._distance_sensors[message.id] = (timebase.now() if timestamp is None else timestamp, message)

    def _push_inbox(self, inbox: str, item: Any) -> None:
        """Queue an item for an action waiting on this inbox."""
        self._inbox[inbox].append(item)

    def _send(self, send: Callable) -> None:
        """Send one MAVLink message; a dead link is recorded in link_error instead of crashing the caller."""
        with self._lock:
            try:
                send()
            # pymavlink/serial/socket errors vary by transport
            except Exception as exc:
                # while reconnecting, the status bar keeps saying so
                if not self.link_error.startswith("reconnecting"):
                    self.link_error = str(exc)

    # ================================================================== telemetry access
    def latest(self, msg_type: str, max_age: Optional[float] = None):
        """The latest message of a type, or None if none arrived (or it is older than `max_age` seconds)."""
        message = self._messages.get(msg_type)
        if message is None or (max_age is not None and self.age(msg_type) > max_age):
            return None
        return message

    def age(self, msg_type: str) -> float:
        """Seconds since the latest message of this type (a huge number if none ever arrived)."""
        return timebase.now() - self._message_times.get(msg_type, NEVER)

    def clear_inbox(self, inbox: str) -> None:
        """Forget queued items (call before starting an operation that fills this inbox)."""
        self._inbox[inbox].clear()

    def pop_inbox(self, inbox: str):
        """The oldest queued item of an inbox, or None if it is empty."""
        queue = self._inbox[inbox]
        return queue.popleft() if queue else None

    def status_text_marker(self) -> int:
        """A marker for "now"; pass it to status_texts_since() later to get only newer texts."""
        return self._text_count

    def status_texts_since(self, marker: int) -> list:
        """Texts received since `marker` (at most the last 400)."""
        return [text for _, text in self.status_messages_since(marker)]

    def status_messages_since(self, marker: int) -> list:
        """(severity, text) of the texts received since `marker` (at most the last 400)."""
        new = self._text_count - marker
        if new <= 0:
            return []
        return [(severity, text) for _, severity, text in list(self._texts)[-min(new, len(self._texts)) :]]

    def last_text(self, contains: str = "") -> str:
        """The newest flight-controller message containing `contains`, formatted for an error message."""
        for _, _, text in reversed(list(self._texts)):
            if contains in text:
                return f"FC says: {text}"
        return ""

    def link_status(self) -> str:
        """Short link health text for the status bar."""
        if timebase.now() - self.last_heartbeat_time < HEARTBEAT_FRESH_SECONDS:
            return "[italic]link OK[/]"

        if self.link_error.startswith("reconnecting"):
            return f"❗ [yellow]{escape_markup(self.link_error)}[/]"

        return "❗ [red]NO HEARTBEAT[/]"

    # ================================================================== parameters
    def fetch_params(self, progress: Optional[Callable] = None, timeout: float = 90.0) -> None:
        """Download all parameters, re-requesting missing ones individually when the stream stalls."""
        self._params.clear()
        self._param_indices_seen.clear()
        self._param_total = 0
        self._send(lambda: self.master.mav.param_request_list_send(self.system_id, self.component_id))
        start = last_progress = timebase.now()
        received_before = 0
        retry_rounds = 0
        while timebase.now() - start < timeout:
            timebase.sleep(PARAM_PROGRESS_POLL_SECONDS)
            received = len(self._param_indices_seen)
            if progress:
                progress(received, self._param_total)

            if self._param_total and received >= self._param_total:
                return

            if received != received_before:
                received_before = received
                last_progress = timebase.now()
            # stalled: ask for what is missing
            elif timebase.now() - last_progress > PARAM_STALL_SECONDS:
                if self._param_total:
                    missing = [i for i in range(self._param_total) if i not in self._param_indices_seen][
                        :PARAM_RE_REQUEST_BATCH
                    ]
                    for index in missing:
                        self._send(
                            lambda index=index: self.master.mav.param_request_read_send(
                                self.system_id, self.component_id, b"", index
                            )
                        )
                        timebase.sleep(PARAM_RE_REQUEST_SPACING)
                    retry_rounds += 1
                    if retry_rounds > PARAM_MAX_RETRY_ROUNDS:
                        break
                else:
                    self._send(lambda: self.master.mav.param_request_list_send(self.system_id, self.component_id))
                last_progress = timebase.now()

        if not self._params:
            raise VehicleError("Could not download the parameter list.")

    @property
    def parameter_count(self) -> int:
        """How many parameters are cached."""
        return len(self._params)

    def parameter_download_warning(self) -> str:
        """A warning if the last full download gave up with parameters missing ("" when complete)."""
        missing = self._param_total - len(self._param_indices_seen)
        if self._param_total and missing > 0:
            return (
                f"WARNING: {missing} of {self._param_total} parameters could not be downloaded - settings may show "
                "as missing or unavailable. Check the link, then reboot the flight controller to retry."
            )
        return ""

    def get(self, name: str, default: Any = None) -> Any:
        """Cached value of a parameter, or `default` if the vehicle doesn't have it."""
        return self._params.get(name, default)

    def get_int(self, name: str, default: int = 0) -> int:
        """A parameter holding a type, function, option or bitmask, as int (`default` if the vehicle doesn't have it)."""
        value = self._params.get(name)
        return default if value is None else int(value)

    def parameters(self) -> dict:
        """A copy of every parameter downloaded from the vehicle: name -> value."""
        return dict(self._params)

    def has_parameter(self, name: str) -> bool:
        """Whether this parameter exists on the connected firmware."""
        return name in self._params

    def resolve(self, names) -> Optional[str]:
        """First existing parameter of a name or tuple of alternative names (names differ between versions)."""
        if isinstance(names, str):
            names = (names,)

        for name in names or ():
            if name in self._params:
                return name

        return None

    def read_param(self, name: str) -> Optional[float]:
        """Read one parameter by name from the vehicle (PARAM_REQUEST_READ) and cache it; None if the vehicle
        doesn't have it - current firmware says so with PARAM_ERROR, older firmware doesn't answer at all."""
        answered = threading.Event()
        self._param_wait_events[name] = answered
        self._param_errors.pop(name, None)
        self._params.pop(name, None)
        try:
            for _ in range(PARAM_READ_ATTEMPTS):
                answered.clear()
                self._send(
                    lambda: self.master.mav.param_request_read_send(
                        self.system_id, self.component_id, name.encode(), PARAM_READ_BY_NAME
                    )
                )
                if timebase.wait_for(answered, PARAM_READ_CONFIRM_SECONDS):
                    return None if name in self._param_errors else self._params.get(name)
            return None
        finally:
            self._param_wait_events.pop(name, None)

    def set_param(self, name: str, value: float) -> float:
        """Write a parameter and wait for the vehicle to confirm it; returns the value the vehicle stored."""
        if name not in self._params:
            raise VehicleError(f"Parameter {name} does not exist on this firmware.")
        param_type = self._param_types.get(name, MAV_PARAM_TYPE_REAL32)
        confirmed = threading.Event()
        self._param_wait_events[name] = confirmed
        self._param_errors.pop(name, None)
        try:
            for _ in range(PARAM_SET_ATTEMPTS):
                confirmed.clear()
                self._send(
                    lambda: self.master.mav.param_set_send(
                        self.system_id, self.component_id, name.encode(), float(value), param_type
                    )
                )
                if timebase.wait_for(confirmed, PARAM_SET_CONFIRM_SECONDS):
                    if name in self._param_errors:
                        # current firmware refuses with PARAM_ERROR (e.g. read-only, value not a number)
                        raise VehicleError(
                            f"The vehicle refused {name} = {value:g} " f"(MAV_PARAM_ERROR {self._param_errors[name]})."
                        )
                    return self._params[name]
            raise VehicleError(f"No confirmation from the vehicle when setting {name}.")
        finally:
            self._param_wait_events.pop(name, None)

    def write_param_file(self, path: str) -> None:
        """Save every cached parameter to a .param file (name,value per line, sorted)."""
        with open(path, "w", encoding="utf-8") as handle:
            for name in sorted(self._params):
                handle.write(f"{name},{_param_file_value(self._params[name])}\n")

    # ================================================================== commands
    def command_long(self, command: int, *params: float, wait: bool = True, timeout: float = 3.0) -> Optional[int]:
        """Send a COMMAND_LONG (params padded to 7) and optionally wait for its final COMMAND_ACK result."""
        # COMMAND_LONG always carries exactly 7 parameters
        padded = list(params) + [0] * (7 - len(params))
        self._command_acks.pop(command, None)
        self._send(lambda: self.master.mav.command_long_send(self.system_id, self.component_id, command, 0, *padded))
        if not wait:
            return None

        deadline = timebase.now() + timeout
        while timebase.now() < deadline:
            result = self._command_acks.get(command)
            if result is not None and result != MAV_RESULT_IN_PROGRESS:
                return result
            timebase.sleep(POLL_SECONDS)

        return self._command_acks.get(command)

    def setup_streams(self) -> None:
        """Ask for all telemetry groups at ALL_STREAMS_RATE_HZ (for messages STREAMS doesn't name), then for
        each message in STREAMS at its own rate."""
        self._send(
            lambda: self.master.mav.request_data_stream_send(
                self.system_id, self.component_id, MAV_DATA_STREAM_ALL, ALL_STREAMS_RATE_HZ, 1
            )
        )
        for message_id, rate_hz in STREAMS:
            # SET_MESSAGE_INTERVAL wants microseconds between messages
            interval_us = int(1e6 / rate_hz)
            self.command_long(MAV_CMD_SET_MESSAGE_INTERVAL, message_id, interval_us, wait=False)
            timebase.sleep(STREAM_REQUEST_SPACING)

    def read_version(self) -> None:
        """Firmware version from AUTOPILOT_VERSION (fallback: the version banner in STATUSTEXT)."""
        self._messages.pop("AUTOPILOT_VERSION", None)
        for _ in range(VERSION_REQUEST_ATTEMPTS):
            self.command_long(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AUTOPILOT_VERSION, wait=False)
            deadline = timebase.now() + VERSION_REPLY_SECONDS
            while timebase.now() < deadline:
                message = self._messages.get("AUTOPILOT_VERSION")
                if message is not None:
                    packed = message.flight_sw_version
                    # flight_sw_version packs major.minor.patch into bytes 3, 2 and 1 (byte 0 = release type).
                    self.version = ((packed >> 24) & 0xFF, (packed >> 16) & 0xFF, (packed >> 8) & 0xFF)
                    return
                timebase.sleep(POLL_SECONDS)
        for _, _, text in self._texts:
            match = re.search(r"Rover\s+V?(\d+)\.(\d+)\.(\d+)", text)
            if match:
                self.version = tuple(int(part) for part in match.groups())

    def version_str(self) -> str:
        """Firmware version as 'X.Y.Z' ('unknown' if it couldn't be read)."""
        return ".".join(map(str, self.version)) if self.version else "unknown"

    def set_mode(self, name: str, timeout: float = MODE_CHANGE_TIMEOUT) -> None:
        """Switch to a flight mode (by MODES name) and wait until the heartbeat confirms it."""
        mode_number = MODES[name]
        for _ in range(2):
            self.command_long(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_number, wait=False)
            # two attempts share the timeout
            deadline = timebase.now() + timeout / 2
            while timebase.now() < deadline:
                if self.custom_mode == mode_number:
                    return
                timebase.sleep(POLL_SECONDS)
        raise VehicleError(f"Could not switch to {name} mode. {self.last_text()}")

    def arm(self) -> None:
        """Arm and wait for the heartbeat to confirm; raises with the pre-arm reason if the vehicle refuses."""
        # param1 = 1: arm
        self.command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, timeout=4)
        deadline = timebase.now() + ARM_CONFIRM_SECONDS
        while not self.armed and timebase.now() < deadline:
            timebase.sleep(POLL_SECONDS)

        if not self.armed:
            raise VehicleError(f"Arming refused. {self.last_text('PreArm') or self.last_text()}")

    def disarm(self, force: bool = False) -> None:
        """Disarm (force = even if the vehicle thinks it is moving)."""
        self.command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, ARM_DISARM_FORCE_MAGIC_NUMBER if force else 0, timeout=3)

    def motor_test(self, instance: int, percent: float, seconds: float = 2.0) -> Optional[int]:
        """Run one output via ArduPilot's motor test (instance: see MOTOR_TEST_INSTANCE_* in rover_defs)."""
        return self.command_long(
            MAV_CMD_DO_MOTOR_TEST,
            instance,
            MOTOR_TEST_THROTTLE_PERCENT,
            percent,
            seconds,
            # param5 = 1: test a single motor
            1,
            MOTOR_TEST_ORDER_DEFAULT,
            0,
        )

    # ================================================================== driving primitives
    def rc_override(self, pwm_by_channel: dict) -> None:
        """Fake RC input on channels 1..8 (pulse widths in us); a channel not given is released (0)."""
        values = [pwm_by_channel.get(channel, 0) for channel in range(1, RC_OVERRIDE_CHANNELS + 1)]
        self._send(lambda: self.master.mav.rc_channels_override_send(self.system_id, self.component_id, *values))

    def rc_release(self) -> None:
        """Release every RC override (sent twice because a single lost packet would keep the override)."""
        for _ in range(2):
            self.rc_override({})

    def guided_cmd(self, speed: float, yaw_rate: float) -> None:
        """Guided-mode target: forward speed [m/s] along the vehicle's heading plus turn rate [rad/s]."""
        self._send(
            lambda: self.master.mav.set_position_target_local_ned_send(
                0,
                self.system_id,
                self.component_id,
                MAV_FRAME_BODY_NED,
                GUIDED_VELOCITY_YAW_RATE_TYPEMASK,
                0,
                0,
                0,
                speed,
                0,
                0,
                0,
                0,
                0,
                0,
                yaw_rate,
            )
        )

    def emergency_stop(self) -> None:
        """STOP button: release overrides, zero the guided target, force Hold and force-disarm (sent twice)."""
        for _ in range(2):
            self.rc_release()
            self.guided_cmd(0.0, 0.0)
            self.command_long(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODES["HOLD"], wait=False)
            self.command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, ARM_DISARM_FORCE_MAGIC_NUMBER, wait=False)

    # ================================================================== RC input
    def rc_calibration(self, channel: int) -> RcChannelCalibration:
        """The RCn_MIN/TRIM/MAX/REVERSED parameters of one channel (ArduPilot defaults where missing)."""
        return RcChannelCalibration(
            minimum=self.get(f"RC{channel}_MIN", RC_PWM_NOMINAL_MIN),
            trim=self.get(f"RC{channel}_TRIM", RC_PWM_NOMINAL_TRIM),
            maximum=self.get(f"RC{channel}_MAX", RC_PWM_NOMINAL_MAX),
            reversed=self.get_int(f"RC{channel}_REVERSED") == 1,
        )

    def rc_mapping(self) -> tuple:
        """(steering channel, throttle channel) numbers from RCMAP_ROLL / RCMAP_THROTTLE."""
        default_steering, default_throttle = ARDUPILOT_DEFAULT_RCMAP
        return self.get_int("RCMAP_ROLL", default_steering), self.get_int("RCMAP_THROTTLE", default_throttle)

    def rc_raw(self) -> Optional[list]:
        """Raw pulse widths of the channels the receiver actually delivers, or None without recent RC telemetry."""
        message = self.latest("RC_CHANNELS", max_age=RC_TELEMETRY_FRESH_SECONDS)
        if message is None:
            return None
        # chancount = number of channels the receiver sends (the message always has room for 18).
        count = max(0, min(RC_CHANNELS_MAX, int(getattr(message, "chancount", RC_CHANNELS_MAX) or 0)))
        return [getattr(message, f"chan{channel}_raw", 0) for channel in range(1, count + 1)]

    def rc_norm(self, channel: int, pwm: float) -> Optional[float]:
        """Stick position -1..1 of one channel's pulse width (None if the channel is not present)."""
        if not pwm or pwm >= UINT16_MAX:
            return None
        return pwm_to_stick(pwm, self.rc_calibration(channel))

    def heading_degrees(self) -> Optional[float]:
        """Heading [deg, 0..360, 0 = north] from fresh ATTITUDE data, or None."""
        attitude = self.latest("ATTITUDE", max_age=MONITOR_MAX_AGE_SECONDS)
        # ATTITUDE.yaw is -pi..pi radians
        return None if attitude is None else math.degrees(attitude.yaw) % 360

    def pressure(self, instance: int = 1) -> Optional[tuple]:
        """(pressure [hPa], temperature [deg C]) of barometer `instance` (1..3) from fresh telemetry, or None."""
        message = self.latest(PRESSURE_MESSAGES[instance], max_age=MONITOR_MAX_AGE_SECONDS)
        if message is None:
            return None
        return message.press_abs, message.temperature / SCALED_PRESSURE_CENTIDEGREES_PER_DEGREE

    def barometers_healthy(self) -> Optional[bool]:
        """Whether every barometer is healthy, as SYS_STATUS reports it (None: not reported, or no barometer)."""
        status = self.latest("SYS_STATUS", max_age=TELEMETRY_FRESH_SECONDS)
        present = getattr(status, "onboard_control_sensors_present", None)
        if present is None or not present & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE:
            return None
        return bool(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)

    def imu(self) -> Optional[tuple]:
        """((accel x, y, z) [g], (rotation rate x, y, z) [deg/s]) from fresh SCALED_IMU data, or None."""
        imu = self.latest("SCALED_IMU", max_age=MONITOR_MAX_AGE_SECONDS)
        if imu is None:
            return None
        # milli-g -> g, milli-radians per second -> degrees per second
        accel = tuple(getattr(imu, f"{axis}acc") / MILLI_G_PER_G for axis in "xyz")
        gyro = tuple(math.degrees(getattr(imu, f"{axis}gyro", 0) / MILLIRAD_PER_RAD) for axis in "xyz")
        return accel, gyro

    def servo_raw(self) -> Optional[list]:
        """Current pulse width of servo outputs 1..16 (0 = output not active), or None without recent telemetry."""
        message = self.latest("SERVO_OUTPUT_RAW", max_age=RC_TELEMETRY_FRESH_SECONDS)
        if message is None:
            return None
        return [getattr(message, f"servo{output}_raw", 0) or 0 for output in SERVO_OUTPUTS]

    def servo_pulse(self, output: int) -> int:
        """Current pulse width of one servo output; 0 = not active, not reported or no recent telemetry."""
        raw = self.servo_raw() or []
        return raw[output - 1] if 1 <= output <= len(raw) else 0

    def distance_sensors(self, max_age: float = TELEMETRY_FRESH_SECONDS) -> list:
        """Fresh DISTANCE_SENSOR readings, one per sensor id (a proximity sensor reports one per sector)."""
        now = timebase.now()
        return [message for received, message in list(self._distance_sensors.values()) if now - received <= max_age]

    def set_servo_pwm(self, output: int, pwm: int) -> Optional[int]:
        """Move an output that has no function assigned (MAV_CMD_DO_SET_SERVO); returns the command result."""
        return self.command_long(MAV_CMD_DO_SET_SERVO, output, pwm)

    def set_relay(self, relay: int, on: bool) -> Optional[int]:
        """Switch relay 1..6 on or off (MAV_CMD_DO_SET_RELAY numbers relays from 0); returns the command result."""
        result = self.command_long(MAV_CMD_DO_SET_RELAY, relay - 1, 1 if on else 0)
        if result in ACCEPTED_RESULTS:
            self._switched_relays[relay] = on
            # the new state now, not with the next RELAY_STATUS of the stream
            self.command_long(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_RELAY_STATUS, wait=False)
        return result

    def relays(self) -> dict:
        """Relay number -> switched on, for every relay set up on the vehicle (1..RELAY_COUNT).

        From RELAY_STATUS (firmware 4.5+). Without it (older firmware): the relays that have a pin, in the state
        the wizard last switched them to (off if it never did)."""
        # a little longer than the reporting period, so one late report doesn't make the buttons flicker
        status = self.latest("RELAY_STATUS", max_age=RELAY_STATUS_PERIOD_SECONDS + MONITOR_MAX_AGE_SECONDS)
        if status is not None:
            return {relay: bool(status.on >> (relay - 1) & 1) for relay in RELAYS if status.present >> (relay - 1) & 1}
        return {relay: self._switched_relays.get(relay, False) for relay in RELAYS if self._is_relay_set_up(relay)}

    def _is_relay_set_up(self, relay: int) -> bool:
        """RELAYn_FUNCTION is Relay (firmware 4.5+), or - older firmware - its pin parameter is set."""
        function = self.get(f"RELAY{relay}_FUNCTION")
        if function is not None:
            return int(function) == RELAY_FUNCTION_RELAY
        pin = self.get(LEGACY_RELAY_PIN[relay])
        return pin is not None and pin >= 0

    def point_gimbal(self, instance: int, pitch_deg: float, yaw_deg: float = 0.0) -> Optional[int]:
        """Point gimbal 1 or 2 (gimbal-manager command, firmware 4.3+); gimbal 1 falls back to DO_MOUNT_CONTROL."""
        # NaN rates = "use the angles"; flags 0; param7 = gimbal device id (1-based instance).
        result = self.command_long(
            MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, pitch_deg, yaw_deg, float("nan"), float("nan"), 0, 0, instance
        )
        if result != MAV_RESULT_ACCEPTED and instance == 1:
            # older firmware only knows the mount command
            result = self.mount_control(pitch_deg, yaw_deg)
        return result

    def set_ekf_source_set(self, source_set: int) -> Optional[int]:
        """Make the EKF use source set 1, 2 or 3 (EK3_SRCn_* parameters) from now on."""
        return self.command_long(MAV_CMD_SET_EKF_SOURCE_SET, source_set)

    def gripper(self, action: int) -> Optional[int]:
        """Grab (GRIPPER_ACTION_GRAB) or release (GRIPPER_ACTION_RELEASE) with gripper 1."""
        return self.command_long(MAV_CMD_DO_GRIPPER, 1, action)

    def mount_control(self, pitch_deg: float, yaw_deg: float = 0.0) -> Optional[int]:
        """Point the camera gimbal (MAV_CMD_DO_MOUNT_CONTROL in MAVLink-targeting mode); returns the command result."""
        return self.command_long(
            MAV_CMD_DO_MOUNT_CONTROL, pitch_deg, 0, yaw_deg, 0, 0, 0, MAV_MOUNT_MODE_MAVLINK_TARGETING
        )

    def rc_inputs(self) -> tuple:
        """Current (steering, throttle) stick positions -1..1, or (None, None) without RC."""
        raw = self.rc_raw()
        if raw is None:
            return None, None
        steering_channel, throttle_channel = self.rc_mapping()

        def position(channel):
            """Stick position of a channel, or None if the receiver doesn't deliver that many channels."""
            return self.rc_norm(channel, raw[channel - 1]) if channel <= len(raw) else None

        return position(steering_channel), position(throttle_channel)

    # ================================================================== drive samples
    def sample(self) -> DriveSample:
        """One snapshot of speed, turn rate, pilot inputs and position for the drive-test analyses."""
        steering, throttle = self.rc_inputs()
        sample = DriveSample(time=timebase.now(), steering_input=steering, throttle_input=throttle)
        attitude = self.latest("ATTITUDE", max_age=TELEMETRY_FRESH_SECONDS)
        position = self.latest("LOCAL_POSITION_NED", max_age=TELEMETRY_FRESH_SECONDS)
        hud = self.latest("VFR_HUD", max_age=TELEMETRY_FRESH_SECONDS)
        if attitude is not None:
            sample.yaw_rate = attitude.yawspeed
            if position is not None:
                # Project the north/east velocity onto the heading: forward speed = vN·cos(yaw) + vE·sin(yaw).
                # This keeps the sign (negative when reversing), unlike the ground speed.
                sample.forward_speed = position.vx * math.cos(attitude.yaw) + position.vy * math.sin(attitude.yaw)
                sample.north = position.x
                sample.east = position.y

        if sample.forward_speed is None and hud is not None:
            # Fallback without a position estimate: ground speed is unsigned, so borrow the sign from the throttle.
            reversing = throttle is not None and throttle < REVERSE_THROTTLE_THRESHOLD
            sample.forward_speed = -hud.groundspeed if reversing else hud.groundspeed

        return sample


def named_value_key(name: str) -> str:
    """The latest() key of the NAMED_VALUE_FLOAT with this name (e.g. "AppWndSpd")."""
    return f"NAMED_VALUE_FLOAT:{name}"


def read_param_file(path: str) -> dict:
    """The parameters in a parameter file (see PARAM_FILE_NAME above): name -> value. Empty lines and # comments are
    skipped; a line that isn't a parameter raises VehicleError (OSError if the file can't be read)."""
    values = {}
    with open(path, encoding="utf-8") as handle:
        for number, line in enumerate(handle, 1):
            words = line.split("#", 1)[0].replace(",", " ").split()
            if len(words) == QGC_PARAM_FILE_COLUMNS:
                words = words[QGC_NAME_VALUE_COLUMNS]
            if not words:
                continue
            try:
                if len(words) != 2 or not PARAM_FILE_NAME.fullmatch(words[0]):
                    raise ValueError
                value = float(words[1])
                if not math.isfinite(value):
                    raise ValueError
            except ValueError:
                raise VehicleError(f"Line {number} is not 'NAME,VALUE': {line.strip()}") from None
            values[words[0]] = value

    return values


def _as_float32(value: float) -> float:
    """value rounded to the float32 the vehicle stores."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


def _param_file_value(value: float) -> str:
    """A parameter value for a .param file: whole numbers in full, others with the fewest digits that still give the
    vehicle's float32 back exactly (8 significant digits can be one step off; 9 always fit, but show 0.0145 as
    0.0144999996)."""
    number = float(value)
    if number.is_integer():
        return str(int(number))

    stored = _as_float32(number)
    for digits in range(FLOAT32_MIN_DIGITS, FLOAT32_EXACT_DIGITS):
        text = f"{number:.{digits}g}"
        if _as_float32(float(text)) == stored:
            return text

    return f"{number:.{FLOAT32_EXACT_DIGITS}g}"


def _mavlink_text(field) -> str:
    """A MAVLink char[] field as text (pymavlink gives bytes or str, padded with NULs)."""
    text = field.decode(errors="ignore") if isinstance(field, bytes) else field
    return text.strip("\x00")


def _close_quietly(link) -> None:
    """Close a pymavlink connection, ignoring errors (it may already be dead)."""
    if link is None:
        return
    # closing a vanished serial port / reset socket raises transport-specific errors
    with contextlib.suppress(Exception):
        link.close()
