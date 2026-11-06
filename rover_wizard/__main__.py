"""rover_wizard.__main__ - command line entry point: the wizard UI, -p NAME[=VALUE] to get/set parameters, or the
onboard log options (--logs, --get-log, --delete-all-logs)."""

from __future__ import annotations

import argparse
import importlib.util
import sys
from typing import TYPE_CHECKING

from . import __version__
from .vehicle.demo import DEMO_VEHICLES
from .vehicle.link import DEFAULT_BAUD_RATE

if TYPE_CHECKING:
    # only for the annotation: importing it at run time would load Textual, which the -p mode must not need
    from .ui.wizard_app import WizardApp


def _is_missing(module: str) -> bool:
    """Whether an optional dependency is not installed (checked without importing it)."""
    return importlib.util.find_spec(module) is None


def app(conn="", baud=DEFAULT_BAUD_RATE, demo="") -> WizardApp:
    """textual app factory; `demo` names a simulated vehicle (vehicle.demo.DEMO_VEHICLES)"""
    from .ui.wizard_app import WizardApp

    return WizardApp(conn, baud, demo)


def main(argv=None) -> int:
    """Parse the command line and run the parameter mode (-p), the log options or the wizard UI; returns the exit
    code."""
    parser = argparse.ArgumentParser(
        prog="rover_wizard", description="ArduPilot Rover setup & tuning wizard (Textual UI)"
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument("--conn", default="", help="MAVLink connection string, e.g. /dev/ttyACM0, udpin:0.0.0.0:14550")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD_RATE, help="serial baud rate (ignored for UDP/TCP)")
    for kind, description in DEMO_VEHICLES.items():
        option = "--demo" if kind == "rover" else f"--demo-{kind}"
        parser.add_argument(option, dest=f"demo_{kind}", action="store_true", help=f"use a simulated {description}")
    parser.add_argument(
        "-p",
        "--param",
        action="append",
        metavar="NAME[=VALUE]",
        help="print a parameter (NAME, or a pattern like 'SERVO1_*') or write it (NAME=VALUE), then "
        "exit without the UI; repeatable, handled in order",
    )
    logs = parser.add_argument_group("onboard logs (logs are numbered as the vehicle lists them, 1 = the oldest)")
    logs.add_argument("--logs", action="store_true", help="list the logs on the vehicle, then exit without the UI")
    logs.add_argument(
        "--get-log",
        metavar="NUMBERS",
        help="download logs, e.g. 3, 3,5 or 7-9 (combinable: 3,7-9), into the 'logs' folder next to the progress "
        "files (the full path of each file is printed), then exit without the UI",
    )
    logs.add_argument(
        "--delete-all-logs",
        action="store_true",
        help="erase ALL logs on the vehicle after typing 'yes' (MAVLink can't erase single logs), then exit",
    )
    args = parser.parse_args(argv)
    # the simulated vehicle chosen ("" = a real one)
    demo = next((kind for kind in DEMO_VEHICLES if getattr(args, f"demo_{kind}")), "")
    log_options = args.logs or args.get_log is not None or args.delete_all_logs
    if args.param and log_options:
        parser.error("--param can't be combined with the log options")
    if args.param or log_options:
        if not demo and not args.conn:
            parser.error(f"{'--param' if args.param else 'the log options'} need --conn (or --demo)")
        if not demo and _is_missing("pymavlink"):
            print("Missing dependency: pip install pymavlink", file=sys.stderr)
            return 1
    if args.param:
        from .param_cli import run_from_command_line

        return run_from_command_line(args.conn, args.baud, args.param, demo=demo)
    if log_options:
        from . import log_cli

        return log_cli.run_from_command_line(
            args.conn, args.baud, args.logs, args.get_log, args.delete_all_logs, demo=demo
        )
    if _is_missing("textual") or (not demo and _is_missing("pymavlink")):
        print("Missing dependency: pip install textual pymavlink", file=sys.stderr)
        return 1
    return app(args.conn, args.baud, demo=demo).run()


if __name__ == "__main__":
    sys.exit(main())
