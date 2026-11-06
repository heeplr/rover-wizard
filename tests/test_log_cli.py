"""Onboard logs from the command line: --logs, --get-log NUMBERS and --delete-all-logs, against the simulator and,
over real MAVLink, the fake autopilot."""

import io
import itertools
import json
import os
import threading
import time
from types import SimpleNamespace

import pytest

from rover_wizard import log_cli
from rover_wizard.__main__ import main
from rover_wizard.log_cli import parse_selection, run
from rover_wizard.log_storage import StoredLog, file_name
from rover_wizard.util import timebase
from rover_wizard.vehicle import log_transfer
from rover_wizard.vehicle.log_transfer import LogEntry
from rover_wizard.vehicle.sim_logs import ARMED_TEXT

from util.helpers import TEST_TIME_SCALE, check, make

PORTS = itertools.count(14760)
# 2026-01-01 00:00:00 UTC
NEW_YEAR = 1767225600


def content(size, seed=0):
    """Log content that shows misplaced pieces."""
    return bytes((index * 7 + seed) % 256 for index in range(size))


# ---- the selection ------------------------------------------------------------------------------------------------
@pytest.mark.parametrize(
    "text, numbers",
    [("3", [3]), ("3,5", [3, 5]), ("7-9", [7, 8, 9]), (" 9, 3,7-9 ,1", [1, 3, 7, 8, 9]), ("4-4", [4])],
)
def test_numbers_lists_and_ranges(text, numbers):
    assert parse_selection(text) == numbers


@pytest.mark.parametrize("text", ["", "a", "3,", "5-3", "0", "1-", "-2", "1-2-3", "*"])
def test_anything_else_is_refused(text):
    with pytest.raises(ValueError):
        parse_selection(text)


def test_file_names_carry_number_and_utc_time():
    assert file_name(LogEntry(7, NEW_YEAR + 3723, 10)) == "log_007_2026-01-01_01-02-03.bin"
    assert file_name(LogEntry(12, 0, 10)) == "log_012_no-time.bin"


# ---- against the simulator ----------------------------------------------------------------------------------------
@pytest.fixture
def vehicle():
    """A simulated vehicle with three logs (the middle one without a time)."""
    vehicle, ctx = make()
    vehicle.onboard_logs.logs = [(NEW_YEAR, content(5000)), (0, content(900, 1)), (NEW_YEAR + 86400, content(2345, 2))]
    yield vehicle
    vehicle.disconnect()


def run_logs(vehicle, folder, show_list=False, selection=None, delete_all=False, answer="yes"):
    """Run the log options; (exit code, stdout lines, stderr lines, the questions asked)."""
    out, err, questions = io.StringIO(), io.StringIO(), []

    def ask(question):
        """The pilot answers."""
        questions.append(question)
        return answer

    code = run(vehicle, show_list, selection, delete_all, out, err, ask, folder=str(folder))
    return code, out.getvalue().splitlines(), err.getvalue().splitlines(), questions


def test_the_list(vehicle, tmp_path):
    code, out, err, _ = run_logs(vehicle, tmp_path, show_list=True)
    assert code == 0 and not err
    assert out == [
        " Log  Date/time (UTC)            Size",
        "   1  2026-01-01 00:00:00       5 KiB",
        "   2  (no time)                 900 B",
        "   3  2026-01-02 00:00:00       2 KiB",
    ]
    vehicle.onboard_logs.logs = []
    assert run_logs(vehicle, tmp_path, show_list=True)[1] == ["No logs on the vehicle."]


def test_downloads_fill_lost_pieces_and_print_the_full_path(vehicle, tmp_path):
    vehicle.onboard_logs.drop_every = 4
    code, out, err, _ = run_logs(vehicle, tmp_path, selection=[1, 3])
    assert code == 0, err
    paths = [str(tmp_path / "log_001_2026-01-01_00-00-00.bin"), str(tmp_path / "log_003_2026-01-02_00-00-00.bin")]
    assert out == [f"Log 1 (5 KiB) saved to {paths[0]}", f"Log 3 (2 KiB) saved to {paths[1]}"]
    for path, expected in zip(paths, (content(5000), content(2345, 2))):
        with open(path, "rb") as handle:
            assert handle.read() == expected
    assert any("log 1: " in line and "%" in line for line in err), "progress shown"


def test_unknown_numbers_are_errors_but_the_others_are_still_downloaded(vehicle, tmp_path):
    code, out, err, _ = run_logs(vehicle, tmp_path, selection=[2, 4, 5])
    assert code == 1
    assert err == ["Error: no log 4, 5 on the vehicle (it has 1-3)."]
    assert out == [f"Log 2 (900 B) saved to {tmp_path / 'log_002_no-time.bin'}"]


def test_nothing_while_armed(vehicle, tmp_path):
    vehicle.arm()
    code, out, err, _ = run_logs(vehicle, tmp_path, show_list=True, delete_all=True)
    assert code == 1 and not out and "disarm" in err[0]
    assert len(vehicle.onboard_logs.logs) == 3
    # ArduPilot itself ignores the requests while armed - with its own message
    marker = vehicle.status_text_marker()
    with pytest.raises(log_cli.VehicleError, match="armed"):
        log_transfer.LogTransfer(vehicle).list_logs()
    assert ARMED_TEXT in vehicle.status_texts_since(marker)


def test_a_transfer_left_running_on_the_vehicle_is_ended_first(vehicle, tmp_path):
    # an interrupted download: ArduPilot keeps sending that log and ignores new requests until LOG_REQUEST_END
    vehicle.onboard_logs.transfer_running = True
    code, out, err, _ = run_logs(vehicle, tmp_path, show_list=True, selection=[3])
    assert code == 0, err
    assert len(out) == 5 and out[-1].startswith("Log 3 (2 KiB) saved to")


def test_ctrl_c_stops_the_vehicles_transfer(vehicle, monkeypatch, capsys):
    def interrupted(log_id, offset, count):
        """The pilot presses Ctrl-C while the log arrives."""
        vehicle.onboard_logs.transfer_running = True
        raise KeyboardInterrupt

    monkeypatch.setattr(vehicle, "request_log_data", interrupted)
    monkeypatch.setattr(log_cli, "connect_for_command_line", lambda *args: vehicle)
    assert log_cli.run_from_command_line("sim", 0, False, "1", False) == log_cli.EXIT_INTERRUPTED
    assert "Interrupted." in capsys.readouterr().err
    assert not vehicle.onboard_logs.transfer_running


@pytest.mark.parametrize("answer, deleted", [("yes", True), ("y", False), ("", False)])
def test_deleting_needs_a_typed_yes(vehicle, tmp_path, answer, deleted):
    code, out, err, questions = run_logs(vehicle, tmp_path, delete_all=True, answer=answer)
    assert code == 0 and not err
    assert questions == ["Delete ALL 3 logs on the vehicle? This can't be undone. Type yes: "]
    assert out == ["All logs deleted." if deleted else "Not deleted."]
    assert (vehicle.onboard_logs.logs == []) is deleted


def test_list_download_and_delete_together_in_this_order(vehicle, tmp_path):
    code, out, err, _ = run_logs(vehicle, tmp_path, show_list=True, selection=[2], delete_all=True)
    assert code == 0, err
    assert out[-2:] == [f"Log 2 (900 B) saved to {tmp_path / 'log_002_no-time.bin'}", "All logs deleted."]
    assert len(out) == 6 and vehicle.onboard_logs.logs == []


# ---- resuming -----------------------------------------------------------------------------------------------------
LOG_1 = "log_001_2026-01-01_00-00-00.bin"


def test_an_unfinished_download_is_kept_and_resumed(vehicle, tmp_path):
    logs = vehicle.onboard_logs
    # the vehicle stops answering after 1800 of the 5000 bytes
    logs.answer_limit = 1800
    code, out, err, _ = run_logs(vehicle, tmp_path, selection=[1])
    assert code == 1 and not out
    assert f"  log 1: 2 KiB of 5 KiB kept in {tmp_path / LOG_1}.part - the same --get-log resumes it" in err, err
    assert not (tmp_path / LOG_1).exists()
    assert json.loads((tmp_path / f"{LOG_1}.part.json").read_text())["ranges"] == [[0, 1800]]

    logs.answer_limit, logs.bytes_sent = None, 0
    code, out, err, _ = run_logs(vehicle, tmp_path, selection=[1])
    assert code == 0, err
    assert (tmp_path / LOG_1).read_bytes() == content(5000)
    assert "  log 1: resuming: 1800 bytes from the earlier download checked and kept" in err, err
    # the rest, and the pieces compared
    check_pieces = 3 * 90
    assert logs.bytes_sent == 3200 + check_pieces
    assert sorted(path.name for path in tmp_path.iterdir()) == [LOG_1], "partial files removed"


def test_a_partial_download_that_doesnt_match_the_vehicle_is_not_used(vehicle, tmp_path):
    logs = vehicle.onboard_logs
    logs.answer_limit = 1800
    run_logs(vehicle, tmp_path, selection=[1])
    # the same time and size, other content (a different log after erasing - or a damaged file)
    logs.answer_limit = None
    logs.logs[0] = (NEW_YEAR, content(5000, seed=9))
    code, out, err, _ = run_logs(vehicle, tmp_path, selection=[1])
    assert code == 0, err
    assert any("doesn't match the log on the vehicle - starting over" in line for line in err), err
    assert (tmp_path / LOG_1).read_bytes() == content(5000, seed=9)


def test_ctrl_c_keeps_what_arrived(vehicle, monkeypatch, tmp_path):
    logs = vehicle.onboard_logs
    logs.drop_every = 3
    requests = []

    def interrupted(log_id, offset, count):
        """The first request arrives (with lost pieces); Ctrl-C during the second."""
        requests.append(offset)
        if len(requests) > 1:
            raise KeyboardInterrupt
        logs.request_data(log_id, offset, count)

    monkeypatch.setattr(vehicle, "request_log_data", interrupted)
    with pytest.raises(KeyboardInterrupt):
        run_logs(vehicle, tmp_path, selection=[1])
    ranges = json.loads((tmp_path / f"{LOG_1}.part.json").read_text())["ranges"]
    # two of every three pieces arrived
    assert sum(end - start for start, end in ranges) == pytest.approx(5000 * 2 / 3, abs=90)


def test_a_log_downloaded_already_is_skipped(vehicle, tmp_path):
    (tmp_path / LOG_1).write_bytes(content(5000))
    code, out, err, _ = run_logs(vehicle, tmp_path, selection=[1])
    assert code == 0 and out == [f"Log 1 was downloaded already: {tmp_path / LOG_1}"]
    assert vehicle.onboard_logs.bytes_sent == 0


@pytest.mark.parametrize(
    "change",
    [{"time_utc": NEW_YEAR + 1}, {"size": 5001}, {"version": 2}, {"ranges": [[0, 6000]]}, {"ranges": "?"}],
    ids=["other time", "other size", "other version", "range outside", "garbled"],
)
def test_partial_files_of_another_log_or_damaged_are_not_used(tmp_path, change):
    entry = LogEntry(1, NEW_YEAR, 5000)
    stored = StoredLog(str(tmp_path), entry)
    stored.save_partial(content(5000), [(0, 2000)])
    assert stored.load_partial().ranges == [(0, 2000)]
    meta_path = tmp_path / f"{LOG_1}.part.json"
    meta = json.loads(meta_path.read_text())
    meta_path.write_text(json.dumps({**meta, **change}))
    assert stored.load_partial() is None


def test_a_missing_part_file_is_not_used(tmp_path):
    stored = StoredLog(str(tmp_path), LogEntry(1, NEW_YEAR, 5000))
    stored.save_partial(content(5000), [(0, 2000)])
    (tmp_path / f"{LOG_1}.part").unlink()
    assert stored.load_partial() is None


# ---- the command line ---------------------------------------------------------------------------------------------
def test_command_line_with_the_demo(capsys, monkeypatch, tmp_path):
    monkeypatch.setattr(log_cli, "LOG_FOLDER", str(tmp_path))
    assert main(["--demo", "--logs", "--get-log", "2"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[0].startswith(" Log") and len(out) == 5
    assert out[-1] == f"Log 2 (9 KiB) saved to {tmp_path / 'log_002_no-time.bin'}"
    assert os.path.getsize(tmp_path / "log_002_no-time.bin") == 9000


def test_a_bad_selection_stops_before_connecting(capsys):
    assert main(["--demo", "--get-log", "3-1"]) == 2
    assert "Nothing done." in capsys.readouterr().err


@pytest.mark.parametrize(
    "arguments", [["--logs"], ["--demo", "--logs", "-p", "WP_SPEED"]], ids=["no connection", "with --param"]
)
def test_refused_combinations(arguments):
    with pytest.raises(SystemExit):
        main(arguments)


# ---- over MAVLink -------------------------------------------------------------------------------------------------
@pytest.fixture
def autopilot(monkeypatch):
    """A fake autopilot with two logs, a link to it, real time (real UDP) and a short wait after erasing."""
    pytest.importorskip("pymavlink")
    from rover_wizard.vehicle.link import Vehicle

    from util.fake_autopilot import FakeAutopilot

    timebase.set_time_scale(1.0)
    monkeypatch.setattr(log_transfer, "ERASE_SETTLE_SECONDS", 0.3)
    port = next(PORTS)
    fake = FakeAutopilot(port)
    fake.logs = [(NEW_YEAR, content(3000)), (0, content(1000, 1))]
    link = Vehicle()
    link.connect(f"udpin:127.0.0.1:{port}", timeout=10, link_only=True)
    yield fake, link
    link.disconnect()
    fake.close()
    timebase.set_time_scale(TEST_TIME_SCALE)


def test_over_mavlink(autopilot, tmp_path):
    fake, link = autopilot
    fake.lose_every = 5
    code, out, err, _ = run_logs(link, tmp_path, show_list=True, selection=[1, 2], delete_all=True)
    assert code == 0, err
    assert out[1:3] == ["   1  2026-01-01 00:00:00       3 KiB", "   2  (no time)                1000 B"]
    with open(tmp_path / "log_001_2026-01-01_00-00-00.bin", "rb") as handle:
        assert handle.read() == content(3000)
    with open(tmp_path / "log_002_no-time.bin", "rb") as handle:
        assert handle.read() == content(1000, 1)
    assert out[-1] == "All logs deleted." and fake.logs == []
    # the lost pieces were asked for again, and every transfer was ended
    assert len(fake.messages("LOG_REQUEST_DATA")) > 2
    requests = [message.get_type() for message in fake.received if message.get_type().startswith("LOG_")]
    last_data = len(requests) - 1 - requests[::-1].index("LOG_REQUEST_DATA")
    assert requests[last_data + 1] == "LOG_REQUEST_END"


def test_over_mavlink_a_transfer_left_running_is_ended_first(autopilot, tmp_path):
    fake, link = autopilot
    # an earlier download was cut off and its LOG_REQUEST_END lost: the vehicle still thinks it is sending
    fake.log_transfer_running = True
    code, out, err, _ = run_logs(link, tmp_path, show_list=True, selection=[2])
    assert code == 0, err
    assert len(out) == 4 and out[-1].startswith("Log 2 (1000 B) saved to")


def test_over_mavlink_while_armed(autopilot, tmp_path):
    fake, link = autopilot
    fake.armed = True
    with pytest.raises(log_cli.VehicleError, match="armed"):
        log_transfer.LogTransfer(link).list_logs()


def test_over_mavlink_a_slow_sd_card_and_lost_pieces(autopilot, tmp_path):
    fake, link = autopilot
    # a large log on a slow SD card: the transfer pauses longer than the wizard waits, again and again. The wizard
    # asks for the missing parts while the vehicle would still be sending, and ArduPilot drops such requests -
    # ending that transfer first makes them count.
    fake.logs[0] = (NEW_YEAR, content(9000))
    fake.lose_every = 4
    fake.log_stalls = {90 * piece: 1.8 for piece in range(10, 80, 10)}
    # ... and some reads come back short: ArduPilot sends that much and ends the transfer
    fake.log_short_reads = {90 * 15, 90 * 47}
    code, out, err, _ = run_logs(link, tmp_path, selection=[1])
    assert code == 0, "\n".join(err)
    with open(tmp_path / "log_001_2026-01-01_00-00-00.bin", "rb") as handle:
        assert handle.read() == content(9000)
    assert any("missing, asking again" in line for line in err), err


def test_over_mavlink_the_download_waits_for_a_lost_link(autopilot, tmp_path):
    fake, link = autopilot
    # a larger log, sent slower than usual
    fake.logs[0] = (NEW_YEAR, content(30000))
    fake.log_stalls = {90 * 50: 0.2}
    # the link breaks for 9 s soon after the download started (long enough for the link to reconnect)
    outage = threading.Timer(0.1, lambda: setattr(fake, "silent_until", time.time() + 9))
    outage.start()
    code, out, err, _ = run_logs(link, tmp_path, selection=[1])
    assert code == 0, "\n".join(err)
    with open(tmp_path / "log_001_2026-01-01_00-00-00.bin", "rb") as handle:
        assert handle.read() == content(30000)
    assert any("link lost" in line for line in err) and any("link back" in line for line in err), err


def test_over_mavlink_checkpoints_are_saved_on_the_way_and_removed_at_the_end(autopilot, monkeypatch, tmp_path):
    fake, link = autopilot
    fake.logs[0] = (NEW_YEAR, content(30000))
    monkeypatch.setattr(log_cli, "CHECKPOINT_SECONDS", 0)
    saved = []
    original = StoredLog.save_partial

    def save_partial(stored, data, ranges):
        """Record each checkpoint."""
        saved.append(sum(end - start for start, end in ranges))
        original(stored, data, ranges)

    monkeypatch.setattr(StoredLog, "save_partial", save_partial)
    code, out, err, _ = run_logs(link, tmp_path, selection=[1])
    assert code == 0, err
    check("saved while arriving", saved and min(saved) < 30000, saved)
    assert sorted(path.name for path in tmp_path.iterdir()) == [LOG_1]


# ---- missing parts ------------------------------------------------------------------------------------------------
def test_missing_parts_close_together_are_asked_for_at_once():
    transfer = log_transfer.LogTransfer(vehicle=None)
    # 1790 bytes; missing: 270-359, 450-629 and the last 80
    transfer._have = bytearray(
        0 if 270 <= index < 360 or 450 <= index < 630 or index >= 1710 else 1 for index in range(1790)
    )
    assert transfer._missing_parts(1790, merge=0) == [(270, 90), (450, 180), (1710, 80)]
    # less than 100 bytes between them: one request for both
    assert transfer._missing_parts(1790, merge=100) == [(270, 360), (1710, 80)]


def test_a_short_piece_ends_the_request_and_unaligned_pieces_count():
    """ArduPilot sends what its file read returns: a short read ends the transfer, the next request starts there."""
    transfer = log_transfer.LogTransfer(vehicle=None)
    transfer._download_id, transfer._content, transfer._have = 1, bytearray(300), bytearray(300)
    transfer._request_start, transfer._request_end = 0, 300

    def piece(offset, data):
        """One LOG_DATA."""
        transfer._on_data(SimpleNamespace(id=1, ofs=offset, count=len(data), data=data.ljust(90, b"\0")))

    piece(0, b"a" * 90)
    assert transfer._reached == 90
    piece(90, b"b" * 40)
    assert transfer._reached == 300, "the vehicle ended the transfer"
    transfer._request_start = 130
    piece(130, b"c" * 90)
    piece(220, b"d" * 80)
    # overlapping pieces are counted once
    piece(200, b"c" * 20 + b"d" * 70)
    assert transfer._received == 300 and transfer._missing_parts(300, 0) == []
    assert bytes(transfer._content) == b"a" * 90 + b"b" * 40 + b"c" * 90 + b"d" * 80
