"""A minimal ArduPilot-Rover-like MAVLink peer (built on the real pymavlink) for testing the wizard's link layer.

It sends heartbeats and telemetry over UDP to the wizard (like a telemetry radio / MAVProxy output), answers
parameter downloads and writes, acknowledges commands (with ArduPilot's side effects for mode changes and arming),
and records every message the wizard sends so tests can check what actually went over the wire.
"""

import os
import socket
import threading
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

# ArduPilot talks MAVLink 2. pymavlink reads MAVLINK20 when it loads a dialect - at the first import of mavutil,
# which another test module may already have done: so load the dialect again with MAVLINK20 set.
os.environ["MAVLINK20"] = "1"
mavutil.set_dialect("ardupilotmega")

SYSTEM_ID = 1
COMPONENT_ID = 1
TELEMETRY_PERIOD = 0.1
FIRMWARE_VERSION = (4, 5, 7)
RC_CHANNEL_COUNT = 12
# bytes per LOG_DATA piece
LOG_PIECE_SIZE = 90


def default_parameters() -> dict:
    """name -> (value, MAV_PARAM_TYPE) - a small but realistic subset of a Rover's parameters."""
    params = {
        "SYSID_MYGCS": (255, mavlink.MAV_PARAM_TYPE_INT16),
        "RCMAP_ROLL": (1, mavlink.MAV_PARAM_TYPE_INT8),
        "RCMAP_THROTTLE": (3, mavlink.MAV_PARAM_TYPE_INT8),
        "GPS1_TYPE": (1, mavlink.MAV_PARAM_TYPE_INT8),
        "BATT_MONITOR": (4, mavlink.MAV_PARAM_TYPE_INT8),
        "CRUISE_SPEED": (2.0, mavlink.MAV_PARAM_TYPE_REAL32),
        "MOT_THR_MIN": (0.0, mavlink.MAV_PARAM_TYPE_REAL32),
    }
    for number in range(1, 17):
        params[f"SERVO{number}_FUNCTION"] = ({1: 26, 3: 70}.get(number, 0), mavlink.MAV_PARAM_TYPE_INT16)
        for suffix, value in (("MIN", 1100), ("TRIM", 1500), ("MAX", 1900)):
            params[f"SERVO{number}_{suffix}"] = (value, mavlink.MAV_PARAM_TYPE_INT16)
            params[f"RC{number}_{suffix}"] = (value, mavlink.MAV_PARAM_TYPE_INT16)
        params[f"SERVO{number}_REVERSED"] = (0, mavlink.MAV_PARAM_TYPE_INT8)
        params[f"RC{number}_REVERSED"] = (0, mavlink.MAV_PARAM_TYPE_INT8)
    return params


def _send_from_own_port(link) -> None:
    """Give pymavlink's udpout connection a socket on a free port of its own.

    On Windows pymavlink binds it to the destination port itself (0.0.0.0:<wizard's port>). The wizard's udpin socket
    (127.0.0.1:<same port>, SO_REUSEADDR) then shares that port, and Windows delivers the wizard's answers - sent to
    127.0.0.1:<that port> - to the more specific socket: the wizard's own. This fake would never hear a request.
    """
    link.port.close()
    link.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    link.port.bind(("127.0.0.1", 0))
    link.port.setblocking(False)
    link.fd = link.port.fileno()


class FakeAutopilot:
    """Runs in background threads; `received` lists every message from the wizard (newest last)."""

    def __init__(self, port: int, renamed: dict = None, param_errors: bool = False) -> None:
        """Start sending to the wizard at udp 127.0.0.1:port and serving its requests."""
        self.link = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{port}", source_system=SYSTEM_ID, source_component=COMPONENT_ID
        )
        _send_from_own_port(self.link)
        self.params = default_parameters()
        # renamed: {old name: (new name, value)} - e.g. a newer firmware's parameter names
        for old, (new, value) in (renamed or {}).items():
            self.params[new] = (value, self.params.pop(old)[1])
        self.names = list(self.params)
        # current firmware answers an unknown name with PARAM_ERROR; older firmware doesn't answer at all
        self.param_errors = param_errors
        # parameters it never sends (a lossy link), so a download stays incomplete
        self.withheld = set()
        # parameters lost from the full list only: sent when asked for one by one
        self.lost_from_list = set()
        # command id -> STATUSTEXT sent before refusing it (MAV_RESULT_FAILED, no side effect)
        self.refused_commands: dict = {}
        self.armed = False
        # HOLD
        self.custom_mode = 4
        self.received: list = []
        self.servo_pwm = [1500 if number in (1, 3) else 0 for number in range(1, 17)]
        # onboard logs: (UTC time [s], content), the oldest first
        self.logs: list = []
        # every this-many-th LOG_DATA piece of a whole-log request is lost on the way (0 = none)
        self.lose_every = 0
        # a transfer still running (sending, or left behind by an interrupted download): list / data requests are
        # ignored ("Log download in progress") until it has sent everything or LOG_REQUEST_END ends it
        self.log_transfer_running = False
        # offset -> seconds: the whole-log transfer pauses there (like a slow SD card in a large log)
        self.log_stalls: dict = {}
        # offsets where the next file read returns only half a piece - ArduPilot then sends that and ends the transfer
        self.log_short_reads: set = set()
        # until this time.time() the link is broken: nothing gets through in either direction
        self.silent_until = 0.0
        # counts transfers, so an ended one stops sending
        self._log_transfer_number = 0
        self._running = True
        self._lock = threading.Lock()
        for target in (self._telemetry_loop, self._receive_loop):
            threading.Thread(target=target, daemon=True).start()

    # ------------------------------------------------------------------ queries for tests
    def messages(self, msg_type: str) -> list:
        """Everything of one type the wizard has sent so far."""
        with self._lock:
            return [message for message in self.received if message.get_type() == msg_type]

    def commands(self, command: int) -> list:
        """COMMAND_LONGs with this command id."""
        return [message for message in self.messages("COMMAND_LONG") if message.command == command]

    def last_command(self, command: int):
        """The most recent COMMAND_LONG with this id, or None."""
        sent = self.commands(command)
        return sent[-1] if sent else None

    def wait_for(self, condition, timeout: float = 5.0) -> bool:
        """Poll until condition() holds."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if condition():
                return True
            time.sleep(0.02)
        return condition()

    def close(self) -> None:
        """Stop the threads and the socket."""
        self._running = False
        time.sleep(0.2)
        self.link.close()

    # ------------------------------------------------------------------ behaviour
    def _send_param(self, name: str) -> None:
        """One PARAM_VALUE, as ArduPilot answers requests and writes."""
        if name in self.withheld:
            return
        value, param_type = self.params[name]
        self.link.mav.param_value_send(name.encode(), float(value), param_type, len(self.names), self.names.index(name))

    @staticmethod
    def _param_name(message) -> str:
        """The parameter name of a PARAM_REQUEST_READ / PARAM_SET."""
        name = message.param_id if isinstance(message.param_id, str) else message.param_id.decode()
        return name.strip("\x00")

    def _answer_param(self, name: str) -> None:
        """The value of a parameter asked for by name - or, for an unknown one, PARAM_ERROR or nothing."""
        if name in self.params:
            value, param_type = self.params[name]
            # by name: param_index -1 (65535 in the uint16 field), as ArduPilot answers
            self.link.mav.param_value_send(name.encode(), float(value), param_type, len(self.names), 65535)
        elif self.param_errors:
            self.link.mav.param_error_send(255, 0, name.encode(), -1, mavlink.MAV_PARAM_ERROR_DOES_NOT_EXIST)

    @property
    def silent(self) -> bool:
        """Whether the link is broken right now (silent_until)."""
        return time.time() < self.silent_until

    def _telemetry_loop(self) -> None:
        """Heartbeat plus the telemetry the wizard reads, ten times a second."""
        while self._running:
            if self.silent:
                time.sleep(TELEMETRY_PERIOD)
                continue
            base_mode = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (
                mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0
            )
            self.link.mav.heartbeat_send(
                mavlink.MAV_TYPE_GROUND_ROVER,
                mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode,
                self.custom_mode,
                mavlink.MAV_STATE_STANDBY,
            )
            channels = [1000 + 10 * number for number in range(1, 19)]
            self.link.mav.rc_channels_send(0, RC_CHANNEL_COUNT, *channels, 200)
            self.link.mav.servo_output_raw_send(0, 0, *self.servo_pwm)
            self.link.mav.distance_sensor_send(
                0, 20, 700, 123, mavlink.MAV_DISTANCE_SENSOR_LASER, 0, mavlink.MAV_SENSOR_ROTATION_NONE, 0
            )
            time.sleep(TELEMETRY_PERIOD)

    def _receive_loop(self) -> None:
        """Answer what the wizard sends."""
        while self._running:
            try:
                message = self.link.recv_match(blocking=True, timeout=0.2)
            except ConnectionResetError:
                # Windows: "port unreachable" for a message sent before the wizard listened - nothing lost here
                continue
            except OSError:
                return
            if message is None or message.get_type() == "BAD_DATA" or self.silent:
                continue
            with self._lock:
                self.received.append(message)
            msg_type = message.get_type()
            if msg_type == "PARAM_REQUEST_LIST":
                for name in self.names:
                    if name not in self.lost_from_list:
                        self._send_param(name)
            elif msg_type == "PARAM_REQUEST_READ":
                if message.param_index == -1:
                    # by name, like GCS_MAVLINK::param_io_timer
                    self._answer_param(self._param_name(message))
                elif 0 <= message.param_index < len(self.names):
                    self._send_param(self.names[message.param_index])
            elif msg_type == "PARAM_SET":
                name = self._param_name(message)
                if name in self.params:
                    self.params[name] = (message.param_value, self.params[name][1])
                    self._send_param(name)
                elif self.param_errors:
                    self.link.mav.param_error_send(
                        message.get_srcSystem(),
                        message.get_srcComponent(),
                        name.encode(),
                        -1,
                        mavlink.MAV_PARAM_ERROR_DOES_NOT_EXIST,
                    )
            elif msg_type == "COMMAND_LONG":
                self._handle_command(message)
            elif msg_type.startswith("LOG_"):
                self._handle_log_request(message)

    def _handle_log_request(self, message) -> None:
        """LOG_REQUEST_LIST / _DATA / _END and LOG_ERASE, like AP_Logger_MAVLinkLogTransfer (nothing while armed)."""
        msg_type = message.get_type()
        if self.armed:
            self.link.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, b"Disarm for log download")
            return
        if msg_type == "LOG_REQUEST_END":
            self.log_transfer_running = False
            return
        if self.log_transfer_running and msg_type == "LOG_REQUEST_LIST":
            self.link.mav.statustext_send(mavlink.MAV_SEVERITY_INFO, b"Log download in progress")
            return
        if self.log_transfer_running and msg_type == "LOG_REQUEST_DATA":
            # from the link that is being sent to: dropped without a word
            return
        count = len(self.logs)
        if msg_type == "LOG_REQUEST_LIST":
            if not count:
                self.link.mav.log_entry_send(0, 0, 0, 0, 0)
            for log_id in range(max(message.start, 1), min(message.end, count) + 1):
                time_utc, content = self.logs[log_id - 1]
                self.link.mav.log_entry_send(log_id, count, count, time_utc, len(content))
        elif msg_type == "LOG_REQUEST_DATA" and 1 <= message.id <= count:
            self.log_transfer_running = True
            self._log_transfer_number += 1
            arguments = (self._log_transfer_number, message.id, message.ofs, message.count)
            threading.Thread(target=self._send_log_data, args=arguments, daemon=True).start()
        elif msg_type == "LOG_ERASE":
            self.logs.clear()

    def _send_log_data(self, number: int, log_id: int, start: int, count: int) -> None:
        """One transfer: LOG_DATA pieces until `count` bytes or the end of the log (or LOG_REQUEST_END) - while the
        receive loop goes on handling (and mostly ignoring) requests, like ArduPilot."""
        content = self.logs[log_id - 1][1]
        end = min(len(content), start + count)
        whole = start == 0 and end == len(content)
        for index, offset in enumerate(range(start, end, LOG_PIECE_SIZE)):
            if number != self._log_transfer_number or not self.log_transfer_running or not self._running:
                return
            if whole and offset in self.log_stalls:
                time.sleep(self.log_stalls[offset])
            if whole and self.lose_every and index % self.lose_every == self.lose_every - 1 or self.silent:
                # lost on the way (ArduPilot goes on sending: it doesn't know)
                time.sleep(0.001)
                continue
            piece = content[offset : min(offset + LOG_PIECE_SIZE, end)]
            short = offset in self.log_short_reads
            if short:
                self.log_short_reads.discard(offset)
                piece = piece[: LOG_PIECE_SIZE // 2]
            self.link.mav.log_data_send(log_id, offset, len(piece), piece.ljust(LOG_PIECE_SIZE, b"\0"))
            if short:
                break
            # like a radio link: don't flood the UDP socket
            time.sleep(0.001)
        if number == self._log_transfer_number:
            self.log_transfer_running = False

    def _handle_command(self, message) -> None:
        """Acknowledge a command, with ArduPilot's side effects for the ones the wizard relies on."""
        if message.command in self.refused_commands:
            self.link.mav.statustext_send(
                mavlink.MAV_SEVERITY_CRITICAL, self.refused_commands[message.command].encode()
            )
            self.link.mav.command_ack_send(message.command, mavlink.MAV_RESULT_FAILED)
            return
        if (
            message.command == mavlink.MAV_CMD_REQUEST_MESSAGE
            and int(message.param1) == mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION
        ):
            major, minor, patch = FIRMWARE_VERSION
            self.link.mav.autopilot_version_send(
                0, (major << 24) | (minor << 16) | (patch << 8) | 255, 0, 0, 0, b"\0" * 8, b"\0" * 8, b"\0" * 8, 0, 0, 0
            )
        elif message.command == mavlink.MAV_CMD_DO_SET_MODE:
            self.custom_mode = int(message.param2)
        elif message.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            self.armed = int(message.param1) == 1
        self.link.mav.command_ack_send(message.command, mavlink.MAV_RESULT_ACCEPTED)
