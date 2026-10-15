"""The PyInstaller bundle's entry script and spec: the launcher runs the wizard's main(), the spec builds from it."""

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
