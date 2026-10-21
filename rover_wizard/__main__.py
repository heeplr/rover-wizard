"""rover_wizard.__main__ - command line entry point: the wizard UI, or -p NAME[=VALUE] to get/set parameters."""

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
    """Parse the command line and run the parameter mode (-p) or the wizard UI; returns the process exit code."""
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
    args = parser.parse_args(argv)
    # the simulated vehicle chosen ("" = a real one)
    demo = next((kind for kind in DEMO_VEHICLES if getattr(args, f"demo_{kind}")), "")
    if args.param:
        if not demo and not args.conn:
            parser.error("--param needs --conn (or --demo)")
        if not demo and _is_missing("pymavlink"):
            print("Missing dependency: pip install pymavlink", file=sys.stderr)
            return 1
        from .param_cli import run_from_command_line

        return run_from_command_line(args.conn, args.baud, args.param, demo=demo)
    if _is_missing("textual") or (not demo and _is_missing("pymavlink")):
        print("Missing dependency: pip install textual pymavlink", file=sys.stderr)
        return 1
    return app(args.conn, args.baud, demo=demo).run()


if __name__ == "__main__":
    sys.exit(main())
