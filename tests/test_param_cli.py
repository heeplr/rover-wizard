"""Command-line parameter mode: -p NAME / NAME=VALUE / PATTERN, without the UI."""

import itertools
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("pymavlink")

from pymavlink.dialects.v20 import ardupilotmega as mavlink  # noqa: E402

from rover_wizard.util import timebase  # noqa: E402
from rover_wizard.__main__ import main  # noqa: E402
from rover_wizard.util.errors import VehicleError  # noqa: E402
from rover_wizard.param_cli import ParamRequest, format_parameter_value, parse_request, run  # noqa: E402
from rover_wizard.vehicle.link import Vehicle  # noqa: E402

from util.fake_autopilot import FakeAutopilot  # noqa: E402
from util.helpers import make  # noqa: E402

# util.helpers fast-forwards the simulated time this much
TEST_TIME_SCALE = 10.0

PORTS = itertools.count(14710)

# PARAM_ERROR is newer than pymavlink 2.4.49: without it the fake autopilot can't play current firmware.
HAS_PARAM_ERROR = hasattr(mavlink, "MAVLink_param_error_message")
needs_param_error = pytest.mark.skipif(not HAS_PARAM_ERROR, reason="the installed pymavlink has no PARAM_ERROR message")
PROJECT = Path(__file__).resolve().parents[1]


# ---- the MAVLink side: reading single parameters by name ------------------------------------------------------------


@pytest.fixture
def real_time():
    """Real UDP to the fake autopilot: run the link's reply timeouts in real time, not fast-forwarded."""
    timebase.set_time_scale(1.0)
    yield
    # the other tests' speed-up (util.helpers)
    timebase.set_time_scale(TEST_TIME_SCALE)


def link_only(param_errors: bool):
    """A Vehicle connected with link_only to a fake autopilot (current firmware: param_errors=True)."""
    port = next(PORTS)
    autopilot = FakeAutopilot(port, param_errors=param_errors)
    vehicle = Vehicle()
    vehicle.connect(f"udpin:127.0.0.1:{port}", timeout=10, link_only=True)
    return vehicle, autopilot


@pytest.mark.parametrize(
    "param_errors", [pytest.param(True, marks=needs_param_error), False], ids=["current firmware", "older firmware"]
)
def test_read_by_name_without_downloading_or_requesting_streams(param_errors, real_time):
    vehicle, autopilot = link_only(param_errors)
    try:
        assert vehicle.read_param("CRUISE_SPEED") == pytest.approx(2.0)
        # unknown: PARAM_ERROR (current firmware) or no answer at all (older firmware)
        assert vehicle.read_param("NO_SUCH_PARAM") is None
        received = {message.get_type() for message in autopilot.received}
        assert "PARAM_REQUEST_LIST" not in received, "no full download"
        # ArduPilot saves requested stream rates as SRx_ parameters: a parameter read mustn't change them
        assert not received & {"REQUEST_DATA_STREAM"} and not autopilot.commands(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL)
        # no ground-station heartbeat: a GCS that appears and vanishes again would trigger the GCS failsafe
        assert "HEARTBEAT" not in received
        # a read parameter can be written
        assert vehicle.set_param("CRUISE_SPEED", 3.0) == pytest.approx(3.0)
    finally:
        vehicle.disconnect()
        autopilot.close()


@needs_param_error
def test_write_refused_with_param_error_is_not_taken_as_confirmation(real_time):
    vehicle, autopilot = link_only(param_errors=True)
    try:
        vehicle.read_param("CRUISE_SPEED")
        # the fake autopilot loses the parameter: its answer to the write is PARAM_ERROR
        del autopilot.params["CRUISE_SPEED"]
        with pytest.raises(VehicleError, match="refused CRUISE_SPEED"):
            vehicle.set_param("CRUISE_SPEED", 3.0)
    finally:
        vehicle.disconnect()
        autopilot.close()


# ---- the requests ------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "text, expected",
    [
        ("wp_speed", ParamRequest("WP_SPEED")),
        ("CRUISE_SPEED=2,5", ParamRequest("CRUISE_SPEED", 2.5)),
        ("servo?_*", ParamRequest("SERVO?_*")),
    ],
)
def test_parse_request(text, expected):
    assert parse_request(text) == expected


@pytest.mark.parametrize(
    "text, problem",
    [("WP_SPEED=fast", "not a number"), ("SERVO*_MIN=1000", "only for reading"), ("=3", "no parameter name")],
)
def test_bad_requests(text, problem):
    with pytest.raises(ValueError, match=problem):
        parse_request(text)


def test_values_are_printed_exactly():
    # device ids are large whole numbers: no scientific notation; float32 values without noise digits
    assert (format_parameter_value(2753028.0), format_parameter_value(0.10000000149011612)) == ("2753028", "0.1")


# ---- running them -------------------------------------------------------------------------------------------------------


@pytest.fixture
def vehicle():
    """A simulated vehicle."""
    vehicle, ctx = make()
    yield vehicle
    vehicle.disconnect()


def run_requests(vehicle, *texts):
    """Run requests; (exit code, stdout lines, stderr lines)."""
    out, err = [], []

    class Lines:
        """Collects print() output."""

        def __init__(self, target):
            self.target = target

        def write(self, text):
            self.target.append(text)

    code = run(vehicle, [parse_request(text) for text in texts], Lines(out), Lines(err))
    return code, "".join(out).splitlines(), "".join(err).splitlines()


def test_read_write_and_order(vehicle):
    code, out, err = run_requests(vehicle, "WP_SPEED", "WP_SPEED=3.5", "WP_SPEED", "SERIAL1_PROTOCOL=1")
    assert code == 0 and not err
    assert out == [
        "WP_SPEED=2",
        "WP_SPEED=3.5 (was 2)",
        "WP_SPEED=3.5",
        "SERIAL1_PROTOCOL=1 (was 2) - takes effect after a reboot",
    ]
    assert vehicle.get("WP_SPEED") == 3.5


def test_pattern_prints_every_match_alphabetically(vehicle):
    code, out, err = run_requests(vehicle, "servo1_*")
    assert code == 0 and out == [
        "SERVO1_FUNCTION=26",
        "SERVO1_MAX=1900",
        "SERVO1_MIN=1100",
        "SERVO1_REVERSED=0",
        "SERVO1_TRIM=1500",
    ]


def test_failures_are_reported_and_the_rest_still_handled(vehicle):
    code, out, err = run_requests(
        vehicle, "NO_SUCH", "COMPASS_CAL_FIT=50", "COMPASS_DEV_ID=1", "NOTHING_*", "WP_SPEED=2", "CRUISE_SPEED=3"
    )
    assert code == 1
    # a pattern is among them: the full list is downloaded first
    assert err == [
        "Downloading the parameter list ...",
        "Error: NO_SUCH: no such parameter on this vehicle",
        "Error: COMPASS_CAL_FIT=50: outside ArduPilot's range 4 .. 32 - nothing written",
        "Error: COMPASS_DEV_ID=1: read-only: ArduPilot doesn't accept a new value - nothing written",
        "Error: NOTHING_*: no parameter matches",
    ]
    assert out == ["WP_SPEED=2 (unchanged)", "CRUISE_SPEED=3 (was 2)"]
    assert (vehicle.get("COMPASS_CAL_FIT"), vehicle.get("COMPASS_DEV_ID")) == (16, 97539), "nothing written"


def test_command_line(capsys):
    assert main(["--demo", "-p", "WP_SPEED", "--param", "CRUISE_SPEED=2.5"]) == 0
    assert capsys.readouterr().out.splitlines() == ["WP_SPEED=2", "CRUISE_SPEED=2.5 (was 2)"]


def test_a_bad_argument_stops_before_connecting(capsys):
    assert main(["--demo", "-p", "CRUISE_SPEED=2.5", "-p", "WP_SPEED=fast"]) == 2
    captured = capsys.readouterr()
    assert captured.out == "" and "Nothing done." in captured.err


def test_param_needs_a_connection():
    with pytest.raises(SystemExit):
        main(["-p", "WP_SPEED"])


def test_works_without_textual():
    # textual made unimportable: the parameter mode must not need it
    code = (
        "import sys; sys.modules['textual'] = None\n"
        "from rover_wizard.__main__ import main\n"
        "sys.exit(main(['--demo', '-p', 'WP_SPEED']))"
    )
    result = subprocess.run([sys.executable, "-c", code], cwd=PROJECT, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "WP_SPEED=2"


def test_a_port_that_cant_be_opened_is_an_error_not_a_traceback(capsys):
    assert main(["--conn", "/dev/no-such-port", "-p", "WP_SPEED"]) == 1
    assert "Error: Could not open /dev/no-such-port" in capsys.readouterr().err
