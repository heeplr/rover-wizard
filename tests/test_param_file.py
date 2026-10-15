"""Parameter files: reading the formats of the ground stations, and what Restore writes."""

import pytest

from rover_wizard.util.errors import VehicleError
from rover_wizard.vehicle.link import read_param_file


def write(tmp_path, text):
    """A parameter file with this text."""
    path = tmp_path / "params.param"
    path.write_text(text, encoding="utf-8")
    return str(path)


def test_reads_wizard_mission_planner_mavproxy_and_qgc_lines(tmp_path):
    path = write(
        tmp_path,
        "# saved by a ground station\n\n"
        # this wizard, Mission Planner
        "CRUISE_SPEED,2.5\n"
        # MAVProxy
        "ATC_STR_RAT_FF   0.21   # tuned\n"
        # QGroundControl: vehicle, component, name, value, type
        "1\t1\tWP_SPEED\t3\t9\n",
    )
    assert read_param_file(path) == {"CRUISE_SPEED": 2.5, "ATC_STR_RAT_FF": 0.21, "WP_SPEED": 3.0}


@pytest.mark.parametrize("line", ["CRUISE_SPEED", "CRUISE_SPEED,fast", "cruise speed,2", "CRUISE_SPEED,nan", "A,1,2"])
def test_a_line_that_is_no_parameter_is_refused(tmp_path, line):
    with pytest.raises(VehicleError, match="Line 2 is not 'NAME,VALUE'"):
        read_param_file(write(tmp_path, f"WP_SPEED,3\n{line}\n"))


def test_restore_writes_only_differing_writable_parameters():
    param_screen = pytest.importorskip("rover_wizard.ui.param_screen")
    current = {"CRUISE_SPEED": 2.0, "WP_SPEED": 3.0, "INS_ACC_ID": 1, "FORMAT_VERSION": 16}
    saved = {"CRUISE_SPEED": 2.5, "WP_SPEED": 3.0, "INS_ACC_ID": 2, "FORMAT_VERSION": 14, "BATT2_MONITOR": 4}
    writes, equal, kept, missing = param_screen.restore_plan(current, saved)
    assert writes == {"CRUISE_SPEED": 2.5}
    assert equal == ["WP_SPEED"]
    assert kept == ["FORMAT_VERSION", "INS_ACC_ID"]
    assert missing == ["BATT2_MONITOR"]
    summary = param_screen.restore_summary("/x/backup.param", (writes, equal, kept, missing))
    assert summary.splitlines()[0] == "backup.param: 1 value(s) differ from the vehicle, 1 are equal."
    assert "FORMAT_VERSION, INS_ACC_ID" in summary and "BATT2_MONITOR" in summary and "restore again" in summary


def test_read_write_column():
    param_screen = pytest.importorskip("rover_wizard.ui.param_screen")
    assert param_screen.COLUMNS.index("R/W") == param_screen.COLUMNS.index("Unit") + 1
    assert [param_screen.access(name) for name in ("INS_ACC_ID", "STAT_RUNTIME", "CRUISE_SPEED")] == ["R", "R", "RW"]
