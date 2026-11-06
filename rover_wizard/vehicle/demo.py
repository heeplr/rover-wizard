"""rover_wizard.vehicle.demo - the simulated vehicles behind --demo, --demo-skid, --demo-boat and --demo-sailboat."""

from __future__ import annotations

# --demo-* option name -> what it simulates ("use a simulated ...")
DEMO_VEHICLES = {
    "rover": "front-wheel-steering rover",
    "skid": "skid-steering rover",
    "boat": "boat with rudder and motor",
    "sailboat": "sailing boat with a motor",
}


def demo_vehicle(kind: str, wired_wrong: bool = False):
    """A started-up simulated vehicle of one of the DEMO_VEHICLES kinds (imported only in demo mode)."""
    if kind == "boat":
        from .boat_simulator import SimBoat

        vehicle = SimBoat(wired_wrong=wired_wrong)
    elif kind == "sailboat":
        from .sail_simulator import SimSailboat

        vehicle = SimSailboat(wired_wrong=wired_wrong)
    else:
        from .rover_simulator import SimVehicle

        vehicle = SimVehicle(skid=kind == "skid", wired_wrong=wired_wrong)
    # a few logs on the "SD card" for trying --logs / --get-log
    vehicle.onboard_logs.add_demo_logs()
    return vehicle
