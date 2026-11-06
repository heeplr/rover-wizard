"""rover_wizard.log_cli - the vehicle's onboard logs from the command line, without the UI:

    rover-wizard --conn /dev/ttyACM0 --logs                 # list them
    rover-wizard --conn /dev/ttyACM0 --get-log 3,7-9        # download logs 3, 7, 8 and 9
    rover-wizard --conn /dev/ttyACM0 --delete-all-logs      # erase all of them (asks first)

Logs are numbered as the vehicle lists them: 1 = the oldest (MAVLink knows no file names). Downloads are saved in
the "logs" folder next to the wizard's progress files, named log_<number>_<date>.bin; the full path of each is
printed. A log already there is skipped. What arrived of an unfinished download (failed, lost link, Ctrl-C - and
every CHECKPOINT_SECONDS on the way) is kept, and the same --get-log resumes it (see log_storage). MAVLink can only
erase all logs at once, not a single one. The vehicle must be disarmed: ArduPilot doesn't send or erase logs while
armed.

Exit code: 0 if everything succeeded, 1 if something failed, 2 for a command line error (nothing done), 130 after
Ctrl-C.
"""

from __future__ import annotations

import os
import sys
import time
from collections.abc import Callable
from typing import Optional, TextIO

from .cli_connection import connect_for_command_line
from .log_storage import StoredLog
from .session.store import DEFAULT_ROOT
from .util.errors import VehicleError
from .vehicle.log_transfer import LogTransfer

EXIT_OK = 0
EXIT_FAILED = 1
EXIT_USAGE = 2
# like a shell's exit code after Ctrl-C (128 + SIGINT)
EXIT_INTERRUPTED = 130
LOG_FOLDER = os.path.join(DEFAULT_ROOT, "logs")
# the download progress is printed in steps of this many percent
PROGRESS_STEP_PERCENT = 10
BYTES_PER_KIB = 1024
BYTES_PER_MIB = 1024 * 1024
SECONDS_PER_MINUTE = 60
PERCENT = 100
# [s] what arrived of a running download is saved this often (to resume after a crash or a killed process)
CHECKPOINT_SECONDS = 60.0


def parse_selection(text: str) -> list:
    """Log numbers from "3", "3,5" or "7-9" (combined: "3,7-9"), sorted; raises ValueError for anything else."""
    numbers = set()
    for part in text.split(","):
        part = part.strip()
        first, dash, last = part.partition("-")
        try:
            if dash:
                low, high = int(first), int(last)
                if low > high:
                    raise ValueError
                numbers.update(range(low, high + 1))
            else:
                numbers.add(int(part))
        except ValueError:
            raise ValueError(
                f"'{part}' is not a log number or range (use numbers like 3, lists like 3,5 and ranges like 7-9)"
            ) from None
    if min(numbers) < 1:
        raise ValueError("log numbers start at 1")
    return sorted(numbers)


def size_text(size: int) -> str:
    """A size in B, KiB or MiB."""
    if size >= BYTES_PER_MIB:
        return f"{size / BYTES_PER_MIB:.1f} MiB"
    if size >= BYTES_PER_KIB:
        return f"{size / BYTES_PER_KIB:.0f} KiB"
    return f"{size} B"


def run(
    vehicle,
    show_list: bool,
    selection: Optional[list],
    delete_all: bool,
    out: TextIO,
    err: TextIO,
    ask: Callable,
    folder: Optional[str] = None,
) -> int:
    """List, download (into folder, default LOG_FOLDER) and/or erase - in this order - on a connected vehicle;
    returns the exit code."""
    if vehicle.armed:
        print("Error: disarm the vehicle first - ArduPilot doesn't send or erase logs while armed.", file=err)
        return EXIT_FAILED
    transfer = LogTransfer(vehicle)
    try:
        entries = transfer.list_logs()
    except VehicleError as error:
        print(f"Error: {error}", file=err)
        return EXIT_FAILED

    if show_list:
        _print_list(entries, out)
    failed = False
    if selection:
        failed = not _download(transfer, entries, selection, folder or LOG_FOLDER, out, err)
    if delete_all:
        failed = not _erase(transfer, entries, out, err, ask) or failed
    return EXIT_FAILED if failed else EXIT_OK


def _print_list(entries: list, out: TextIO) -> None:
    """One line per log: number, time (UTC) and size."""
    if not entries:
        print("No logs on the vehicle.", file=out)
        return
    print(f"{'Log':>4}  {'Date/time (UTC)':<19}  {'Size':>10}", file=out)
    for entry in entries:
        when = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(entry.time_utc)) if entry.time_utc else "(no time)"
        print(f"{entry.id:>4}  {when:<19}  {size_text(entry.size):>10}", file=out)


def _download(transfer: LogTransfer, entries: list, selection: list, folder: str, out: TextIO, err: TextIO) -> bool:
    """Download the selected logs into the folder; whether all of them worked."""
    by_id = {entry.id: entry for entry in entries}
    unknown = [number for number in selection if number not in by_id]
    if unknown:
        has = f"it has 1-{len(entries)}" if entries else "it has none"
        print(f"Error: no log {', '.join(map(str, unknown))} on the vehicle ({has}).", file=err)
    os.makedirs(folder, exist_ok=True)
    all_fine = not unknown
    for number in (number for number in selection if number in by_id):
        stored = StoredLog(folder, by_id[number])
        if stored.is_complete():
            print(f"Log {number} was downloaded already: {stored.path}", file=out)
            continue
        all_fine = _download_one(transfer, stored, out, err) and all_fine
    return all_fine


def _download_one(transfer: LogTransfer, stored: StoredLog, out: TextIO, err: TextIO) -> bool:
    """Download (or resume) one log; whether that worked. What arrived is kept if not."""
    entry = stored.entry
    output = _DownloadOutput(transfer, stored, err)
    try:
        content = transfer.download(entry, output.progress, output.note, resume=stored.load_partial())
    except VehicleError as error:
        output.keep_partial()
        print(f"Error: {error}", file=err)
        return False
    except KeyboardInterrupt:
        output.keep_partial()
        raise
    stored.save(content)
    print(f"Log {entry.id} ({size_text(entry.size)}) saved to {stored.path}", file=out)
    return True


class _DownloadOutput:
    """A download's messages on stderr - progress every PROGRESS_STEP_PERCENT with the time still needed, and notes -
    and its partial file, saved every CHECKPOINT_SECONDS."""

    def __init__(self, transfer: LogTransfer, stored: StoredLog, err: TextIO) -> None:
        """For this log."""
        self.transfer, self.stored, self.err = transfer, stored, err
        self.entry = stored.entry
        self.started = self.last_checkpoint = time.monotonic()
        # bytes at the first report (a resumed download starts with some), and at the last checkpoint
        self.first_received: Optional[int] = None
        self.checkpoint_received = 0
        self.printed = 0

    def note(self, text: str) -> None:
        """Something worth knowing on the way."""
        print(f"  log {self.entry.id}: {text}", file=self.err)

    def progress(self, received: int) -> None:
        """Print the next step once reached; save a checkpoint when due."""
        now = time.monotonic()
        if self.first_received is None:
            self.first_received = self.checkpoint_received = received
        if now - self.last_checkpoint >= CHECKPOINT_SECONDS and received > self.checkpoint_received:
            self.last_checkpoint, self.checkpoint_received = now, received
            snapshot = self.transfer.snapshot()
            self.stored.save_partial(snapshot.content, snapshot.ranges)
        size = self.entry.size
        percent = received * PERCENT // max(size, 1)
        if percent < self.printed + PROGRESS_STEP_PERCENT or percent >= PERCENT:
            return
        self.printed = percent - percent % PROGRESS_STEP_PERCENT
        # [bytes/s] (the max(): no division by zero right at the start)
        rate = (received - self.first_received) / max(now - self.started, 1e-3)
        left = (size - received) / rate if rate > 0 else 0.0
        text = f"  log {self.entry.id}: {self.printed}% ({size_text(received)} of {size_text(size)}"
        if left >= SECONDS_PER_MINUTE:
            text += f", about {left / SECONDS_PER_MINUTE:.0f} min left"
        print(text + ")", file=self.err)

    def keep_partial(self) -> None:
        """Save what arrived, and say how to resume."""
        snapshot = self.transfer.snapshot()
        if not snapshot.ranges:
            return
        self.stored.save_partial(snapshot.content, snapshot.ranges)
        self.note(
            f"{size_text(snapshot.received)} of {size_text(self.entry.size)} kept in {self.stored.path}.part - "
            "the same --get-log resumes it"
        )


def _erase(transfer: LogTransfer, entries: list, out: TextIO, err: TextIO, ask: Callable) -> bool:
    """Erase all logs after the pilot typed "yes"; whether that worked (declining counts as fine)."""
    if not entries:
        print("No logs to delete.", file=out)
        return True
    try:
        answer = ask(f"Delete ALL {len(entries)} logs on the vehicle? This can't be undone. Type yes: ")
    except EOFError:
        answer = ""
    if answer.strip().lower() != "yes":
        print("Not deleted.", file=out)
        return True
    try:
        remaining = transfer.erase_all()
    except VehicleError as error:
        print(f"Error: {error}", file=err)
        return False
    if remaining:
        print(f"Error: the vehicle still lists {len(remaining)} logs after erasing.", file=err)
        return False
    print("All logs deleted.", file=out)
    return True


def run_from_command_line(
    connection: str,
    baud_rate: int,
    show_list: bool,
    selection_text: Optional[str],
    delete_all: bool,
    demo: str = "",
    out: Optional[TextIO] = None,
    err: Optional[TextIO] = None,
    ask: Callable = input,
) -> int:
    """Check the selection, connect (link only), handle the log options, disconnect; returns the exit code."""
    out, err = out or sys.stdout, err or sys.stderr
    selection = None
    if selection_text is not None:
        try:
            selection = parse_selection(selection_text)
        except ValueError as error:
            print(f"Error: {error}. Nothing done.", file=err)
            return EXIT_USAGE

    vehicle = connect_for_command_line(connection, baud_rate, demo, err)
    if vehicle is None:
        return EXIT_FAILED
    try:
        return run(vehicle, show_list, selection, delete_all, out, err, ask)
    except KeyboardInterrupt:
        # the download's own clean-up has already told the vehicle to stop sending
        print("\nInterrupted.", file=err)
        return EXIT_INTERRUPTED
    finally:
        vehicle.disconnect()
