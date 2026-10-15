"""rover_wizard.__main__ - command line entry point: the wizard UI, or -p NAME[=VALUE] to get/set parameters."""

from __future__ import annotations

import argparse
import importlib.util
import sys
from typing import TYPE_CHECKING

from . import __version__
from .vehicle.link import DEFAULT_BAUD_RATE

if TYPE_CHECKING:
    from .ui.wizard_app import WizardApp

def _is_missing(module: str) -> bool:
    """Whether an optional dependency is not installed (checked without importing it)."""
    return importlib.util.find_spec(module) is None


def app(conn="", baud=DEFAULT_BAUD_RATE, demo=False, demo_skid=False) -> WizardApp:
    """Launch wizard UI; returns the textual app"""
    from .ui.wizard_app import WizardApp

    return WizardApp(conn, baud, demo, demo_skid)


def main(argv=None) -> int:
    """Parse the command line and run the parameter mode (-p) or launch TUI"""
    parser = argparse.ArgumentParser(
        prog="rover_wizard", description="ArduPilot Rover setup & tuning wizard (Textual UI)"
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    parser.add_argument("--conn", default="", help="MAVLink connection string, e.g. /dev/ttyACM0, udpin:0.0.0.0:14550")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD_RATE, help="serial baud rate (ignored for UDP/TCP)")
    parser.add_argument("--demo", action="store_true", help="use a simulated front-wheel-steering rover")
    parser.add_argument("--demo-skid", action="store_true", help="use a simulated skid-steering rover")
    parser.add_argument(
        "-p",
        "--param",
        action="append",
        metavar="NAME[=VALUE]",
        help="print a parameter (NAME, or a pattern like 'SERVO1_*') or write it (NAME=VALUE), then "
        "exit without the UI; repeatable, handled in order",
    )
    args = parser.parse_args(argv)
    demo = args.demo or args.demo_skid
    if args.param:
        if not demo and not args.conn:
            parser.error("--param needs --conn (or --demo)")
        if not demo and _is_missing("pymavlink"):
            print("Missing dependency: pip install pymavlink", file=sys.stderr)
            return 1
        from .param_cli import run_from_command_line

        return run_from_command_line(args.conn, args.baud, args.param, demo=demo, skid=args.demo_skid)
    if _is_missing("textual") or (not demo and _is_missing("pymavlink")):
        print("Missing dependency: pip install textual pymavlink", file=sys.stderr)
        return 1
    return app(args.conn, args.baud, demo=demo, demo_skid=args.demo_skid).run()


if __name__ == "__main__":
    sys.exit(main())
