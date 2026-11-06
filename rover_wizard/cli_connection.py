"""rover_wizard.cli_connection - the vehicle link of the command-line modes (-p, --logs, ...): link only, no UI."""

from __future__ import annotations

from typing import Optional, TextIO

from .util.errors import VehicleError


def connect_for_command_line(connection: str, baud_rate: int, demo: str, err: TextIO) -> Optional[object]:
    """A connected vehicle (or a simulated one for `demo`), or None after printing why not."""
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
        return None
    return vehicle
