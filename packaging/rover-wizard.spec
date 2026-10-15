# PyInstaller build of the single-file rover-wizard executable.
# Build from the project folder:  python -m PyInstaller --noconfirm --clean packaging/rover-wizard.spec
# Check the result:               dist/rover-wizard --version   (starts and finds its own metadata)
import os

from PyInstaller.utils.hooks import collect_data_files, collect_submodules, copy_metadata

# SPECPATH: the folder of this file (set by PyInstaller)
PROJECT = os.path.dirname(SPECPATH)

hiddenimports = (
    # Textual imports its widgets on demand (textual.widgets.__getattr__)
    collect_submodules("textual")
    # Rich loads its character-width tables by Unicode version at run time
    + collect_submodules("rich._unicode_data")
    # pymavlink loads its message definitions by name at run time
    + ["pymavlink.dialects.v20.ardupilotmega", "pymavlink.dialects.v10.ardupilotmega"]
)

datas = (
    # the step texts and ArduPilot's parameter descriptions (rover_wizard/ui/*.json)
    collect_data_files("rover_wizard")
    + collect_data_files("textual")
    # rover-wizard (shown in the title bar), Textual and Rich look up their version via importlib.metadata
    + copy_metadata("rover-wizard")
    + copy_metadata("textual")
    + copy_metadata("rich")
)

a = Analysis(
    [os.path.join(SPECPATH, "rover_wizard_launcher.py")],
    pathex=[PROJECT],
    hiddenimports=hiddenimports,
    datas=datas,
    excludes=["tkinter", "pytest"],
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="rover-wizard",
    # a terminal (Textual) application
    console=True,
    upx=False,
)
