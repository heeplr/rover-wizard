"""pytest setup, loaded before any test module.

* HOME and the per-user data folder variables point to a throw-away folder BEFORE rover_wizard is imported: the
  store computes its default folder at import time, and tests (demo mode included) must never touch the real one.
* Every temporary folder the tests create (tempfile.mkdtemp() for a test's progress store, ...) goes below that
  folder, and the whole folder is removed when the test run ends.
* The project root and this folder are importable (rover_wizard, and the test support code in util/).
"""

import os
import shutil
import sys
import tempfile

TEST_HOME = tempfile.mkdtemp(prefix="rover_wizard_test_home_")
# HOME: Linux and macOS; XDG_DATA_HOME: Linux (if set, it wins over HOME); LOCALAPPDATA / USERPROFILE: Windows
for variable in ("HOME", "USERPROFILE", "LOCALAPPDATA"):
    os.environ[variable] = TEST_HOME
os.environ["XDG_DATA_HOME"] = os.path.join(TEST_HOME, ".local", "share")
tempfile.tempdir = TEST_HOME
TESTS = os.path.dirname(os.path.abspath(__file__))
for path in (os.path.dirname(TESTS), TESTS):
    if path not in sys.path:
        sys.path.insert(0, path)


def pytest_sessionfinish(session, exitstatus):
    """Remove the throw-away folder with everything the tests wrote into it."""
    shutil.rmtree(TEST_HOME, ignore_errors=True)
