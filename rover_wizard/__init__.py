"""rover_wizard - a terminal wizard that sets up, calibrates and tunes ArduPilot Rover vehicles over MAVLink."""

from __future__ import annotations

from importlib import metadata
from pathlib import Path

# the distribution name in pyproject.toml ([project] name)
DISTRIBUTION = "rover-wizard"


def _read_version() -> str:
    """[project] version from pyproject.toml.

    In a source checkout (also an editable install) the file next to the package is read, so a version bump shows
    without reinstalling; an installed package (wheel, PyInstaller bundle) has no pyproject.toml and reports the
    version recorded in its metadata at build time.
    """
    pyproject = Path(__file__).resolve().parent.parent / "pyproject.toml"
    try:
        # Python 3.11+; on 3.10 a checkout falls back to the installed metadata
        import tomllib

        with pyproject.open("rb") as project_file:
            project = tomllib.load(project_file)["project"]
        if project["name"] == DISTRIBUTION:
            return project["version"]
    # not a checkout, no tomllib, or not our pyproject.toml (TOMLDecodeError is a ValueError)
    except (ImportError, OSError, KeyError, ValueError):
        pass
    try:
        return metadata.version(DISTRIBUTION)
    except metadata.PackageNotFoundError:
        return "unknown"


__version__ = _read_version()
