"""rover_wizard.session.store - the wizard's saved progress for one vehicle (so setup can continue in a later session).

One JSON file per vehicle system id in the per-user data folder (default_root()). Callers use named methods (step status, settings,
flags); the JSON layout is private to this module.
"""

from __future__ import annotations

import contextlib
import json
import os
import sys
import threading
import time
from typing import Any, Optional

APP_FOLDER_NAME = "rover-wizard"


def default_root() -> str:
    """The per-user data folder for progress files and parameter backups:
    Linux/BSD: $XDG_DATA_HOME/rover-wizard (default ~/.local/share/rover-wizard),
    macOS: ~/Library/Application Support/rover-wizard, Windows: %LOCALAPPDATA%\\rover-wizard."""
    home = os.path.expanduser("~")
    if sys.platform == "win32":
        base = os.environ.get("LOCALAPPDATA") or os.path.join(home, "AppData", "Local")
    elif sys.platform == "darwin":
        base = os.path.join(home, "Library", "Application Support")
    else:
        # the XDG spec says to ignore a relative path
        xdg_data_home = os.environ.get("XDG_DATA_HOME", "")
        base = xdg_data_home if os.path.isabs(xdg_data_home) else os.path.join(home, ".local", "share")

    return os.path.join(base, APP_FOLDER_NAME)


DEFAULT_ROOT = default_root()
STEP_DONE = "done"
STEP_SKIPPED = "skipped"


class Store:
    """Wizard progress + measurements for one vehicle, persisted after every change."""

    def __init__(self, system_id: int, root: Optional[str] = None) -> None:
        """Load the saved progress for this vehicle, or start fresh if there is none yet."""
        self.root = root or DEFAULT_ROOT
        os.makedirs(self.root, exist_ok=True)
        self.path = os.path.join(self.root, f"vehicle_{system_id}.json")
        self._data: dict = {STEP_DONE: [], STEP_SKIPPED: [], "steer": None, "settings": {}, "changes": {}}
        # the UI thread and worker threads save; one temp file name, so one save at a time
        self._save_lock = threading.Lock()
        self._load()

    def _load(self) -> None:
        """Merge the progress file into the defaults. A damaged file is kept aside instead of being overwritten."""
        try:
            with open(self.path, encoding="utf-8") as handle:
                saved = json.load(handle)
            # valid JSON of the wrong shape (edited by hand, another program's file) counts as corrupt too
            if not isinstance(saved, dict) or not isinstance(saved.get("settings", {}), dict):
                raise ValueError("not a wizard progress file")
            self._data.update(saved)
            self._migrate()
        except FileNotFoundError:
            # first session with this vehicle
            pass
        except (OSError, ValueError):
            # Unreadable or corrupt: move it aside so the next save can't silently destroy it.
            os.replace(self.path, self.path + time.strftime(".corrupt-%Y%m%d_%H%M%S"))

    def _migrate(self) -> None:
        """Older progress files saved the free area for circles as a radius ('arena'): now a diameter ('circle_d')."""
        settings = self._data["settings"]
        if "arena" in settings:
            radius = settings.pop("arena")
            # an unusable old value is dropped: the default applies
            if "circle_d" not in settings:
                with contextlib.suppress(TypeError, ValueError):
                    settings["circle_d"] = 2 * float(radius)

    def save(self) -> None:
        """Write the progress file atomically (temp file + rename, so a crash never leaves half a file)."""
        temporary = self.path + ".tmp"
        with self._save_lock:
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(self._data, handle, indent=1)
            os.replace(temporary, self.path)

    # ------------------------------------------------------------------ generic values
    def get(self, key: str, default: Any = None) -> Any:
        """Read one saved value (e.g. 'steer', 'has_wenc', 'heavy_vehicle', 'changes')."""
        return self._data.get(key, default)

    def set(self, key: str, value: Any) -> None:
        """Save one value and write the file immediately."""
        self._data[key] = value
        self.save()

    def setting(self, key: str, default: Any = None) -> Any:
        """A pilot-entered setting that later steps reuse (e.g. 'runway', 'circle_d')."""
        return self._data["settings"].get(key, default)

    def update_settings(self, **values: Any) -> None:
        """Save several settings at once."""
        self._data["settings"].update(values)
        self.save()

    # ------------------------------------------------------------------ step progress
    def mark_step(self, step_id: str, status: str) -> None:
        """Record a step as done or skipped (the two are mutually exclusive)."""
        other = STEP_SKIPPED if status == STEP_DONE else STEP_DONE
        if step_id not in self._data[status]:
            self._data[status].append(step_id)

        self._data[other] = [existing for existing in self._data[other] if existing != step_id]
        self.save()

    def clear_step(self, step_id: str) -> None:
        """Neither done nor skipped any more (it has to be done again)."""
        for status in (STEP_DONE, STEP_SKIPPED):
            self._data[status] = [existing for existing in self._data[status] if existing != step_id]

        self.save()

    def step_status(self, step_id: str) -> Optional[str]:
        """STEP_DONE, STEP_SKIPPED or None for a step."""
        if step_id in self._data[STEP_DONE]:
            return STEP_DONE

        if step_id in self._data[STEP_SKIPPED]:
            return STEP_SKIPPED

        return None

    def is_finished(self, step_id: str) -> bool:
        """Whether a step is done or skipped (either unlocks the following steps)."""
        return self.step_status(step_id) is not None

    def backup_path(self) -> str:
        """Full path for a new, timestamped parameter-backup file."""
        return os.path.join(self.root, time.strftime("params_backup_%Y%m%d_%H%M%S.param"))
