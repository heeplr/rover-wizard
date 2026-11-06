"""rover_wizard.vehicle.log_transfer - listing, downloading and erasing the vehicle's onboard logs over MAVLink.

ArduPilot's log protocol (AP_Logger_MAVLinkLogTransfer.cpp):
* LOG_REQUEST_LIST -> one LOG_ENTRY per log: its id (its position in the list, 1 = the oldest, NOT the number in the
  file name on the SD card), the number of logs, its time (UTC, 0 if unknown) and size.
* LOG_REQUEST_DATA(id, offset, count) -> LOG_DATA pieces of up to 90 bytes, until `count` bytes or the end of the log
  have been sent. ArduPilot sends what its file read returns: a shorter piece (or one of 0 bytes after a read error)
  ends the transfer early, and pieces asked for later may start anywhere.
* One transfer at a time, and it never times out: until it has sent everything asked for, a LOG_REQUEST_DATA from the
  same link is silently dropped and a LOG_REQUEST_LIST answered only with "Log download in progress" - also when the
  program that asked is gone (an interrupted download of a large log keeps the vehicle busy for minutes).
  LOG_REQUEST_END ends it at once, so the wizard sends it before every request.
* LOG_ERASE erases all logs - MAVLink has no message to erase a single one.
* While the vehicle is armed, ArduPilot ignores all of it ("Disarm for log download").
"""

from __future__ import annotations

import re
import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional

from ..util import timebase
from ..util.errors import VehicleError
from .link import HEARTBEAT_FRESH_SECONDS

# [s] no new LOG_ENTRY / LOG_DATA for this long: the vehicle has sent what it will send
LIST_IDLE_SECONDS = 2.0
DATA_IDLE_SECONDS = 1.5
POLL_SECONDS = 0.05
# lost list entries are asked for this often; lost log pieces this many rounds in a row without any progress
RETRIES = 5
# bytes in a full LOG_DATA piece
PIECE_SIZE = 90
# [s] missing parts closer together than the link sends in this time are asked for in one request (sending the bytes
# between them again is quicker than another request's round trip)
MERGE_SECONDS = 0.2
# pieces of an earlier partial download the vehicle sends again to compare, before the rest is resumed
RESUME_CHECKS = 3
# [s] a lost link (no heartbeat) may take this long to come back before a download gives up
LINK_WAIT_SECONDS = 300.0
SECONDS_PER_MINUTE = 60
# [s] after LOG_ERASE, the list is read again this much later (erasing takes a moment)
ERASE_SETTLE_SECONDS = 3.0


@dataclass(frozen=True)
class Partial:
    """What a download got so far: the content (the log's size) and the (start, end) byte ranges that arrived."""

    content: bytes
    ranges: list

    @property
    def received(self) -> int:
        """[bytes] arrived."""
        return sum(end - start for start, end in self.ranges)


def _check_pieces(ranges: list) -> list:
    """(offset, length) of up to RESUME_CHECKS pieces spread over the received ranges: at the start of the first,
    in the middle one and at the end of the last."""
    first, middle, last = ranges[0], ranges[len(ranges) // 2], ranges[-1]
    pieces = {
        (first[0], min(PIECE_SIZE, first[1] - first[0])),
        ((middle[0] + middle[1]) // 2, min(PIECE_SIZE, middle[1] - (middle[0] + middle[1]) // 2)),
        (max(last[0], last[1] - PIECE_SIZE), min(PIECE_SIZE, last[1] - last[0])),
    }
    return sorted(pieces)[:RESUME_CHECKS]


@dataclass(frozen=True)
class LogEntry:
    """One log on the vehicle."""

    # position in the vehicle's list (1 = oldest)
    id: int
    # [s] UTC time of the log (0 = unknown: no GPS time when it was written)
    time_utc: int
    # [bytes]
    size: int


class LogTransfer:
    """Lists, downloads and erases a vehicle's logs (see the module docstring)."""

    def __init__(self, vehicle) -> None:
        """Bind to a connected vehicle (a link, or the simulator)."""
        self.vehicle = vehicle
        self._lock = threading.Lock()
        self._entries: dict = {}
        self._count: Optional[int] = None
        # the running download: log id, content, one byte per content byte (1 = received), bytes received
        self._download_id = 0
        self._content = bytearray()
        self._have = bytearray()
        self._received = 0
        # the running request's range, and how far its pieces got (its end once the vehicle ended the transfer)
        self._request_end = 0
        self._request_start = 0
        self._reached = 0
        self._last_activity = 0.0

    # ------------------------------------------------------------------ listing
    def list_logs(self) -> list:
        """Every log on the vehicle (raises VehicleError if the vehicle doesn't answer)."""
        self._entries, self._count = {}, None
        self.vehicle.listen("LOG_ENTRY", self._on_entry)
        try:
            for _ in range(RETRIES):
                # ends a transfer still running on the vehicle, which would make it ignore the request
                self.vehicle.end_log_transfer()
                self.vehicle.request_log_list()
                self._wait_idle(
                    LIST_IDLE_SECONDS, lambda: self._count is not None and len(self._entries) >= self._count
                )
                if self._count is not None:
                    break
            else:
                raise VehicleError("The vehicle sent no log list (is it armed? ArduPilot only sends logs disarmed).")
            for _ in range(RETRIES):
                missing = [log_id for log_id in range(1, self._count + 1) if log_id not in self._entries]
                if not missing:
                    break
                for log_id in missing:
                    self.vehicle.request_log_list(log_id, log_id)
                self._wait_idle(LIST_IDLE_SECONDS, lambda: len(self._entries) >= self._count)
        finally:
            self.vehicle.listen("LOG_ENTRY", None)
        missing = [log_id for log_id in range(1, self._count + 1) if log_id not in self._entries]
        if missing:
            raise VehicleError(f"The log list stayed incomplete (missing {len(missing)} of {self._count} entries).")
        return [self._entries[log_id] for log_id in range(1, self._count + 1)]

    def _on_entry(self, message) -> None:
        """A LOG_ENTRY arrived (id 0 with num_logs 0: there are no logs)."""
        with self._lock:
            self._last_activity = timebase.now()
            self._count = message.num_logs
            if message.id:
                self._entries[message.id] = LogEntry(message.id, message.time_utc, message.size)

    # ------------------------------------------------------------------ download
    def download(
        self,
        entry: LogEntry,
        progress: Optional[Callable] = None,
        note: Optional[Callable] = None,
        resume: Optional[Partial] = None,
    ) -> bytes:
        """The log's content; progress(received bytes) is called while it arrives, note(text) when lost parts are
        asked for again or the link is lost. With `resume` (an earlier partial download of this log) only the rest is
        downloaded - if a few of its pieces match the vehicle's (_resume). Raises VehicleError when RETRIES rounds
        in a row bring nothing new although the link was up, or the link stays lost for LINK_WAIT_SECONDS; what
        arrived until then is in snapshot()."""
        with self._lock:
            self._download_id = entry.id
            self._content = bytearray(entry.size)
            self._have = bytearray(entry.size)
            self._received = 0
        self.vehicle.listen("LOG_DATA", self._on_data)
        try:
            resumed = resume is not None and self._resume(entry, resume, note)
            # [bytes/s] what the link carries (measured on the way)
            rate = 0.0
            fruitless = round_number = 0
            while fruitless < RETRIES:
                parts = self._missing_parts(entry.size, merge=rate * MERGE_SECONDS)
                if not parts:
                    break
                if note is not None and (round_number or resumed):
                    missing = entry.size - self._received
                    note(f"{missing} bytes missing, asking again in {len(parts)} request{'s' * (len(parts) > 1)}")
                round_number += 1
                before, started = self._received, timebase.now()
                link_was_up = True
                for offset, length in parts:
                    asked = timebase.now()
                    self._request(entry.id, offset, length, progress)
                    if not self._reached:
                        # nothing came: a lost link doesn't count as a failed round
                        link_was_up = self._check_link(asked, note) and link_was_up
                rate = max(rate, (self._received - before) / max(timebase.now() - started, POLL_SECONDS))
                if self._received > before:
                    fruitless = 0
                elif link_was_up:
                    fruitless += 1
        finally:
            self.vehicle.listen("LOG_DATA", None)
            self.vehicle.end_log_transfer()
        missing = entry.size - self._received
        if missing:
            raise VehicleError(f"Log {entry.id} stayed incomplete: {missing} of {entry.size} bytes didn't arrive.")
        with self._lock:
            return bytes(self._content)

    def snapshot(self) -> Partial:
        """What the current (or last) download got so far."""
        with self._lock:
            ranges = [(run.start(), run.end()) for run in re.finditer(b"\x01+", self._have)]
            return Partial(bytes(self._content), ranges)

    def _resume(self, entry: LogEntry, partial: Partial, note: Optional[Callable]) -> bool:
        """Take over an earlier partial download - after the vehicle sent RESUME_CHECKS of its pieces again and they
        all matched (a different log with the same time and size, or a damaged file, is never mixed in)."""
        checked = 0
        for offset, length in _check_pieces(partial.ranges):
            asked = timebase.now()
            # no progress reports yet: the earlier bytes aren't counted before the check
            self._request(entry.id, offset, length, None)
            with self._lock:
                arrived = self._have[offset : offset + length].count(0) == 0
                same = self._content[offset : offset + length] == partial.content[offset : offset + length]
            if not arrived:
                self._check_link(asked, note)
                continue
            if not same:
                if note is not None:
                    note("the earlier partial download doesn't match the log on the vehicle - starting over")
                return False
            checked += 1
        if not checked:
            if note is not None:
                note("the earlier partial download couldn't be checked - starting over")
            return False
        with self._lock:
            for start, end in partial.ranges:
                self._content[start:end] = partial.content[start:end]
                self._have[start:end] = b"\x01" * (end - start)
            self._received = self._have.count(1)
        if note is not None:
            note(f"resuming: {partial.received} bytes from the earlier download checked and kept")
        return True

    def _check_link(self, since: float, note: Optional[Callable]) -> bool:
        """Whether a heartbeat came after `since`. If none does, waits for the link to come back (the link
        reconnects by itself) and returns False - or raises VehicleError after LINK_WAIT_SECONDS."""
        if self._heartbeat_after(since, HEARTBEAT_FRESH_SECONDS):
            return True
        if note is not None:
            note(f"link lost - waiting up to {LINK_WAIT_SECONDS / SECONDS_PER_MINUTE:.0f} min for it to come back")
        if not self._heartbeat_after(since, LINK_WAIT_SECONDS):
            raise VehicleError(f"The link stayed lost for {LINK_WAIT_SECONDS / SECONDS_PER_MINUTE:.0f} min.")
        if note is not None:
            note("link back - continuing")
        return False

    def _heartbeat_after(self, since: float, seconds: float) -> bool:
        """Wait up to `seconds` for a heartbeat newer than `since`."""
        deadline = timebase.now() + seconds
        while self.vehicle.last_heartbeat_time <= since:
            if timebase.now() >= deadline:
                return False
            timebase.sleep(POLL_SECONDS)
        return True

    def _request(self, log_id: int, offset: int, length: int, progress: Optional[Callable]) -> None:
        """Ask for part of the log; return once its last piece arrived (anything missing in it was lost) or nothing
        came for DATA_IDLE_SECONDS."""
        with self._lock:
            self._request_start, self._request_end, self._reached = offset, offset + length, 0
        # ends a transfer still running: ArduPilot would drop the request otherwise
        self.vehicle.end_log_transfer()
        self.vehicle.request_log_data(log_id, offset, length)
        self._wait_idle(DATA_IDLE_SECONDS, lambda: self._reached >= self._request_end, progress)

    def _on_data(self, message) -> None:
        """A LOG_DATA piece arrived."""
        if message.id != self._download_id:
            return
        with self._lock:
            self._last_activity = timebase.now()
            start, end = message.ofs, min(message.ofs + message.count, len(self._content))
            if start < end:
                self._received += self._have[start:end].count(0)
                self._have[start:end] = b"\x01" * (end - start)
                self._content[start:end] = bytes(message.data[: end - start])
            if self._request_start <= start < self._request_end:
                # a short piece (or an empty one: read error) is the last the vehicle sends for this request
                self._reached = self._request_end if message.count < PIECE_SIZE else max(self._reached, end)

    def _missing_parts(self, size: int, merge: float) -> list:
        """(offset, length) of the parts still missing; parts less than `merge` bytes apart are joined."""
        parts: list = []
        with self._lock:
            runs = [(run.start(), run.end()) for run in re.finditer(b"\x00+", self._have)]
        for start, end in runs:
            if parts and start - (parts[-1][0] + parts[-1][1]) <= merge:
                parts[-1] = (parts[-1][0], end - parts[-1][0])
            else:
                parts.append((start, end - start))
        return parts

    # ------------------------------------------------------------------ erase
    def erase_all(self) -> list:
        """Erase every log; returns the list read afterwards (empty when it worked)."""
        self.vehicle.erase_logs()
        timebase.sleep(ERASE_SETTLE_SECONDS)
        return self.list_logs()

    # ------------------------------------------------------------------ waiting
    def _wait_idle(self, idle_seconds: float, done: Optional[Callable] = None, progress=None) -> None:
        """Wait until done() or nothing arrived for idle_seconds (also at the start)."""
        with self._lock:
            self._last_activity = timebase.now()
        while True:
            if progress is not None:
                progress(self._received)
            if done is not None and done():
                return
            with self._lock:
                quiet = timebase.now() - self._last_activity
            if quiet >= idle_seconds:
                return
            timebase.sleep(POLL_SECONDS)
