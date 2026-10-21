"""The PyInstaller bundle's entry script and spec: the launcher runs the wizard's main(), the spec builds from it."""

import re
import runpy
import sys
from pathlib import Path

import pytest

from rover_wizard import __version__

PACKAGING = Path(__file__).resolve().parent.parent / "packaging"


def test_launcher_runs_the_wizard_main(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["rover-wizard", "--version"])
    with pytest.raises(SystemExit) as exit_info:
        runpy.run_path(str(PACKAGING / "rover_wizard_launcher.py"), run_name="__main__")
    assert exit_info.value.code == 0 and capsys.readouterr().out.strip().endswith(__version__)


def test_spec_builds_from_the_launcher_and_bundles_the_version():
    spec = (PACKAGING / "rover-wizard.spec").read_text(encoding="utf-8")
    assert '"rover_wizard_launcher.py"' in spec and 'copy_metadata("rover-wizard")' in spec


# ---- the command line -------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "argv, missing, message",
    [
        (["--conn", "/dev/ttyACM0"], {"pymavlink"}, "pip install textual pymavlink"),
        (["--demo"], {"textual"}, "pip install textual pymavlink"),
        (["--conn", "/dev/ttyACM0", "-p", "WP_SPEED"], {"pymavlink"}, "pip install pymavlink"),
    ],
)
def test_a_missing_dependency_is_named(monkeypatch, capsys, argv, missing, message):
    import rover_wizard.__main__ as entry

    monkeypatch.setattr(entry, "_is_missing", lambda module: module in missing)
    assert entry.main(argv) == 1
    assert message in capsys.readouterr().err


def test_the_ui_starts_with_the_command_line_settings(monkeypatch):
    import rover_wizard.__main__ as entry

    started = []

    class FakeApp:
        def __init__(self, *args):
            started.append(args)

        def run(self):
            return 0

    pytest.importorskip("textual")
    monkeypatch.setattr("rover_wizard.ui.wizard_app.WizardApp", FakeApp)
    monkeypatch.setattr(entry, "_is_missing", lambda module: False)
    assert entry.main(["--conn", "udpin:0.0.0.0:14550", "--baud", "57600", "--demo-skid"]) == 0
    assert started == [("udpin:0.0.0.0:14550", 57600, "skid")]


def test_every_ui_data_file_is_installed():
    """pyproject.toml lists the data files an installed package (and the bundle) gets: none may be missing."""
    root = PACKAGING.parent
    listed = re.search(r'"rover_wizard\.ui" = \[(.*?)\]', (root / "pyproject.toml").read_text(encoding="utf-8")).group(
        1
    )
    data_files = {path.name for path in (root / "rover_wizard" / "ui").iterdir() if path.suffix in (".json", ".css")}
    assert data_files <= set(re.findall(r'"([^"]+)"', listed))
