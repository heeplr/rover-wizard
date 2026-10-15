"""rover_wizard.util.errors - the two exceptions that cross module boundaries."""

from __future__ import annotations


class Cancelled(Exception):
    """The pilot pressed Cancel or STOP; the running action must stop immediately."""


class VehicleError(Exception):
    """Something the pilot has to know about; the message is shown in the log as-is."""
