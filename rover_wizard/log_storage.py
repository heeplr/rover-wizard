"""rover_wizard.log_storage - downloaded onboard logs on disk: the finished files, and partial downloads to resume.

A partial download is two files next to the finished one: <name>.part (the log's size, the received bytes in place)
and <name>.part.json (which log it is and which byte ranges of .part have arrived). Every file is written to a
temporary name first and then renamed, so a crash never leaves a half-written one. .part is written before
.part.json: if a crash falls in between, the older .part.json only lists ranges the newer .part has as well.

A partial download is only used for a log with the same UTC time and size (log numbers shift when logs are erased
or added); the download then compares a few pieces with the vehicle before trusting it (vehicle.log_transfer).
"""

from __future__ import annotations

import json
import os
import time
from typing import Optional

from .vehicle.log_transfer import LogEntry, Partial

PART_SUFFIX = ".part"
META_SUFFIX = ".part.json"
TEMPORARY_SUFFIX = ".tmp"
# the .part.json layout; another version isn't used
META_VERSION = 1


def file_name(entry: LogEntry) -> str:
    """The file a downloaded log is saved as: log_<number>_<date and time, UTC>.bin."""
    when = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime(entry.time_utc)) if entry.time_utc else "no-time"
    return f"log_{entry.id:03d}_{when}.bin"


def write_atomically(path: str, data: bytes) -> None:
    """Write the file under a temporary name, then rename it: it is either complete or not there (or the old one)."""
    temporary = path + TEMPORARY_SUFFIX
    with open(temporary, "wb") as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def _remove(path: str) -> None:
    """Remove a file if it is there."""
    try:
        os.remove(path)
    except FileNotFoundError:
        pass


class StoredLog:
    """One log's files in the download folder."""

    def __init__(self, folder: str, entry: LogEntry) -> None:
        """The log's place in the folder."""
        self.entry = entry
        self.path = os.path.abspath(os.path.join(folder, file_name(entry)))

    def is_complete(self) -> bool:
        """Whether the finished file is there already (with the log's size)."""
        return os.path.isfile(self.path) and os.path.getsize(self.path) == self.entry.size

    def save(self, content: bytes) -> None:
        """Save the finished log; a partial download of it is no longer needed."""
        write_atomically(self.path, content)
        self.discard_partial()

    def load_partial(self) -> Optional[Partial]:
        """The partial download of this log, if there is a usable one."""
        try:
            with open(self.path + META_SUFFIX, encoding="utf-8") as handle:
                meta = json.load(handle)
            which = (meta["version"], meta["time_utc"], meta["size"])
            if which != (META_VERSION, self.entry.time_utc, self.entry.size):
                return None
            ranges = [(int(start), int(end)) for start, end in meta["ranges"]]
            with open(self.path + PART_SUFFIX, "rb") as handle:
                content = handle.read()
        # missing, unreadable or not what this version writes: start from scratch
        except (OSError, ValueError, KeyError, TypeError):
            return None
        if len(content) != self.entry.size or not all(0 <= start < end <= self.entry.size for start, end in ranges):
            return None
        return Partial(content, ranges) if ranges else None

    def save_partial(self, content: bytes, ranges: list) -> None:
        """Keep what has arrived so far (nothing to keep: an earlier partial download stays as it is)."""
        if not ranges:
            return
        write_atomically(self.path + PART_SUFFIX, content)
        meta = {
            "version": META_VERSION,
            "id": self.entry.id,
            "time_utc": self.entry.time_utc,
            "size": self.entry.size,
            "ranges": [list(part) for part in ranges],
        }
        write_atomically(self.path + META_SUFFIX, json.dumps(meta).encode("utf-8"))

    def discard_partial(self) -> None:
        """Remove the partial download (its description first: a .part without it is never used)."""
        _remove(self.path + META_SUFFIX)
        _remove(self.path + PART_SUFFIX)
