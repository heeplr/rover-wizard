"""rover_wizard.param_cli - get and set parameters from the command line, without the UI:

    rover-wizard --conn /dev/ttyACM0 -p WP_SPEED -p CRUISE_SPEED=2.5 -p 'SERVO1_*'

Each -p/--param is NAME (print it), NAME=VALUE (write it) or a pattern with * ? [...] (print every match), handled
in the given order. Output: one NAME=value line per parameter on stdout; problems on stderr.

* Single names are read one by one (PARAM_REQUEST_READ), which is fast even over slow telemetry; a pattern needs
  the full parameter list, which is then downloaded once.
* The whole command line is checked before connecting, so a typo can't leave earlier writes half-done.
* Writes are checked like in the parameter list (read-only, ArduPilot's range, whole numbers where named values or
  bits apply) and confirmed by the vehicle. ArduPilot silently ignores writes to unknown parameters, so an unknown
  name is reported as an error, not as success.
* Not recorded in the wizard's change log: the printed old value ("was ...") is the record.

Exit code: 0 if everything succeeded, 1 if any parameter failed (the others are still handled), 2 for a command
line error (nothing done).
"""

from __future__ import annotations

import fnmatch
import sys
from dataclasses import dataclass
from typing import Optional, TextIO

from .util.errors import VehicleError
from .ui.param_docs import is_reboot_required, validation_error
from .util.values import are_nearly_equal

EXIT_OK = 0
EXIT_FAILED = 1
EXIT_USAGE = 2
WILDCARD_CHARACTERS = "*?["
# float32 parameters have about 7 significant digits
SIGNIFICANT_DIGITS = 7


@dataclass(frozen=True)
class ParamRequest:
    """One -p argument: a name or pattern, and the value to write (None = just print)."""

    name: str
    value: Optional[float] = None

    @property
    def is_pattern(self) -> bool:
        """Whether the name contains wildcards."""
        return any(character in self.name for character in WILDCARD_CHARACTERS)


def parse_request(text: str) -> ParamRequest:
    """NAME, NAME=VALUE or PATTERN -> ParamRequest (names are upper case); ValueError explains a bad argument."""
    name, has_value, value_text = text.partition("=")
    name = name.strip().upper()
    if not name:
        raise ValueError(f"{text!r}: no parameter name")

    request = ParamRequest(name)
    if not has_value:
        return request

    if request.is_pattern:
        raise ValueError(f"{text!r}: wildcards are only for reading - write each parameter by its name")

    try:
        # a decimal comma is accepted too
        value = float(value_text.strip().replace(",", "."))
    except ValueError:
        raise ValueError(f"{text!r}: {value_text.strip()!r} is not a number") from None

    return ParamRequest(name, value)


def format_parameter_value(value: float) -> str:
    """A parameter value as typed: whole numbers without ".0" (device ids too), others to float32 precision."""
    if float(value).is_integer():
        return str(int(value))
    return f"{value:.{SIGNIFICANT_DIGITS}g}"


class _ParameterSource:
    """Reads parameters by name, or - once a pattern needs it - from the full list downloaded once."""

    def __init__(self, vehicle, needs_full_list: bool) -> None:
        """needs_full_list: a pattern is among the requests, so the whole list is downloaded once."""
        self.vehicle = vehicle
        self.uses_full_list = needs_full_list

    def download(self, err: TextIO) -> None:
        """Download the full list (for patterns)."""
        print("Downloading the parameter list ...", file=err)
        self.vehicle.fetch_params()
        warning = self.vehicle.parameter_download_warning()
        if warning:
            print(warning, file=err)

    def value(self, name: str) -> Optional[float]:
        """The parameter's current value, or None if the vehicle doesn't have it."""
        if self.uses_full_list:
            return self.vehicle.get(name)
        return self.vehicle.read_param(name)

    def matching(self, pattern: str) -> list:
        """(name, value) of every parameter matching the pattern, alphabetically."""
        return [
            (name, value)
            for name, value in sorted(self.vehicle.parameters().items())
            if fnmatch.fnmatchcase(name, pattern)
        ]


def run(vehicle, requests: list, out: Optional[TextIO] = None, err: Optional[TextIO] = None) -> int:
    """Handle the requests in order on a connected vehicle; returns the exit code (output: stdout/stderr)."""
    # looked up now, not at import time, so redirected streams are honoured
    out, err = out or sys.stdout, err or sys.stderr
    source = _ParameterSource(vehicle, any(request.is_pattern for request in requests))
    if source.uses_full_list:
        try:
            source.download(err)
        except VehicleError as error:
            print(f"Error: {error}", file=err)
            return EXIT_FAILED

    failed = False
    for request in requests:
        try:
            if request.is_pattern:
                _print_matching(source, request.name, out)
            elif request.value is None:
                _print_one(source, request.name, out)
            else:
                _write(source, request.name, request.value, out)
        except VehicleError as error:
            print(f"Error: {error}", file=err)
            failed = True

    return EXIT_FAILED if failed else EXIT_OK


def _print_matching(source: _ParameterSource, pattern: str, out: TextIO) -> None:
    """NAME=value for every match."""
    matches = source.matching(pattern)
    if not matches:
        raise VehicleError(f"{pattern}: no parameter matches")

    for name, value in matches:
        print(f"{name}={format_parameter_value(value)}", file=out)


def _print_one(source: _ParameterSource, name: str, out: TextIO) -> None:
    """NAME=value."""
    value = source.value(name)
    if value is None:
        raise VehicleError(f"{name}: no such parameter on this vehicle")
    print(f"{name}={format_parameter_value(value)}", file=out)


def _write(source: _ParameterSource, name: str, value: float, out: TextIO) -> None:
    """Check and write one value; prints the confirmed value and the old one."""
    old = source.value(name)
    if old is None:
        raise VehicleError(f"{name}: no such parameter on this vehicle - nothing written")

    problem = validation_error(name, value)
    if problem:
        raise VehicleError(f"{name}={format_parameter_value(value)}: {problem} - nothing written")

    if are_nearly_equal(old, value):
        print(f"{name}={format_parameter_value(old)} (unchanged)", file=out)
        return

    confirmed = source.vehicle.set_param(name, value)
    reboot = " - takes effect after a reboot" if is_reboot_required(name) else ""
    print(f"{name}={format_parameter_value(confirmed)} (was {format_parameter_value(old)}){reboot}", file=out)


def run_from_command_line(
    connection: str,
    baud_rate: int,
    arguments: list,
    demo: str = "",
    out: Optional[TextIO] = None,
    err: Optional[TextIO] = None,
) -> int:
    """Check every -p argument, connect (link only), handle them, disconnect; returns the exit code."""
    out, err = out or sys.stdout, err or sys.stderr
    requests, problems = [], []
    for text in arguments:
        try:
            requests.append(parse_request(text))
        except ValueError as error:
            problems.append(str(error))

    if problems:
        for problem in problems:
            print(f"Error: {problem}", file=err)
        print("Nothing done.", file=err)
        return EXIT_USAGE

    if demo:
        from .vehicle.demo import demo_vehicle

        vehicle = demo_vehicle(demo)
    else:
        from .vehicle.link import Vehicle

        vehicle = Vehicle()

    try:
        vehicle.connect(connection, baud_rate, link_only=True)
    except VehicleError as error:
        print(f"Error: {error}", file=err)
        return EXIT_FAILED

    try:
        return run(vehicle, requests, out, err)
    finally:
        vehicle.disconnect()
