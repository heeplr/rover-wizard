"""rover_wizard.vehicle.sim_logs - the simulated vehicle's onboard logs, answering the LOG_* requests like ArduPilot
(AP_Logger_MAVLinkLogTransfer.cpp; see vehicle.log_transfer)."""

from __future__ import annotations

from types import SimpleNamespace

from .link import LOG_LIST_ALL

# bytes per LOG_DATA piece (MAVLink's LOG_DATA.data)
PIECE_SIZE = 90
# the status text ArduPilot answers log requests with while armed
ARMED_TEXT = "Disarm for log download"
# ... and while another transfer is still running
BUSY_TEXT = "Log download in progress"
# [s] UTC times and [bytes] sizes of the logs a demo vehicle has (0 = written before the GPS knew the time)
DEMO_LOGS = ((1767225600, 48_000), (0, 9_000), (1767312000, 120_000))


class SimulatedLogs:
    """The logs of a simulated vehicle; answers by calling the vehicle's listeners (synchronously)."""

    def __init__(self, vehicle) -> None:
        """Bind to the simulated vehicle (its `armed`, `_listeners` and `_add_status_text`)."""
        self.vehicle = vehicle
        # (UTC time [s], content) per log, the oldest first (tests fill it)
        self.logs: list = []
        # pieces lost on the way (tests): every this-many-th piece of a whole-log request goes missing, 0 = none
        self.drop_every = 0
        # LOG_DATA bytes sent so far, and how many more it sends before it stops answering (None = no limit)
        self.bytes_sent = 0
        self.answer_limit = None
        # a transfer still running (e.g. left behind by an interrupted download): requests are ignored until
        # LOG_REQUEST_END
        self.transfer_running = False

    def add_demo_logs(self) -> None:
        """Fill the card with DEMO_LOGS (made-up content) for trying the log options with --demo."""
        for time_utc, size in DEMO_LOGS:
            # any byte pattern will do (a prime period, so it doesn't line up with the pieces)
            self.logs.append((time_utc, bytes(index % 251 for index in range(size))))

    def _refused(self, busy_matters: bool = True) -> bool:
        """ArduPilot ignores log requests while armed, and list / data requests during a transfer (with texts)."""
        if self.vehicle.armed:
            self.vehicle._add_status_text(ARMED_TEXT)
            return True
        if busy_matters and self.transfer_running:
            self.vehicle._add_status_text(BUSY_TEXT)
            return True
        return False

    def end_transfer(self) -> None:
        """LOG_REQUEST_END: a running transfer ends."""
        if not self.vehicle.armed:
            self.transfer_running = False

    def _send(self, msg_type: str, **fields) -> None:
        """Hand a message to the listener of its type (if any)."""
        listener = self.vehicle._listeners.get(msg_type)
        if listener is not None:
            listener(SimpleNamespace(**fields))

    def request_list(self, first: int, last: int) -> None:
        """LOG_REQUEST_LIST -> LOG_ENTRY per log (id 0, num_logs 0 if there are none)."""
        if self._refused():
            return
        count = len(self.logs)
        if not count:
            self._send("LOG_ENTRY", id=0, num_logs=0, last_log_num=0, time_utc=0, size=0)
            return
        last = count if last == LOG_LIST_ALL else min(last, count)
        for log_id in range(max(first, 1), last + 1):
            time_utc, content = self.logs[log_id - 1]
            self._send("LOG_ENTRY", id=log_id, num_logs=count, last_log_num=count, time_utc=time_utc, size=len(content))

    def request_data(self, log_id: int, offset: int, count: int) -> None:
        """LOG_REQUEST_DATA -> LOG_DATA pieces (a short one at the end of the log)."""
        if self._refused() or not 1 <= log_id <= len(self.logs):
            return
        content = self.logs[log_id - 1][1]
        end = min(len(content), offset + count)
        dropping = self.drop_every and offset == 0 and end == len(content)
        for index, position in enumerate(range(offset, end, PIECE_SIZE)):
            if dropping and index % self.drop_every == self.drop_every - 1:
                continue
            piece = content[position : min(position + PIECE_SIZE, end)]
            if self.answer_limit is not None:
                if self.answer_limit < len(piece):
                    return
                self.answer_limit -= len(piece)
            self.bytes_sent += len(piece)
            self._send("LOG_DATA", id=log_id, ofs=position, count=len(piece), data=piece.ljust(PIECE_SIZE, b"\0"))

    def erase(self) -> None:
        """LOG_ERASE: all logs are gone."""
        if not self._refused(busy_matters=False):
            self.logs.clear()
