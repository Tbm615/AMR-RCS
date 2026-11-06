"""Where each robot's data lives on this PC, and how the app left it.

  robots/<id>/missions.json   its missions
  robots/<id>/places.json     its saved places
  robots/<id>/events.csv      its event log
  robots/<id>/state.json      the app as it was left with this robot

state.json holds: layout, open mission, the choice on Robot programs (saved map
or SLAM), last position per map, the mission run that was going on, and when it
was last connected. It only ever brings the screens back: nothing here starts
the robot moving.
"""
from __future__ import annotations

import json
import re
import shutil
from pathlib import Path

from .profile import CONFIG_DIR, ProfileStore, atomic_write, drop_setting, load_settings

ROBOTS_DIR = CONFIG_DIR / "robots"


def robot_dir(robot_id: str) -> Path:
    return ROBOTS_DIR / re.sub(r"[^A-Za-z0-9_-]", "_", robot_id)


class RobotState:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.data: dict = {}
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                self.data = data
        except (OSError, ValueError):
            pass

    def get(self, key: str, default=None):
        return self.data.get(key, default)

    def update(self, **kw) -> None:
        """Written only when something changed: this runs on a timer."""
        if all(k in self.data and self.data[k] == v for k, v in kw.items()):
            return
        self.data.update(kw)
        try:
            atomic_write(self.path, json.dumps(self.data, indent=2))
        except OSError:
            pass


def migrate_legacy(profiles: ProfileStore) -> None:
    """Up to v0.5 missions and places were one list per PC. There is no telling
    which robot each one was for, so every robot gets a copy of the list; the
    old file stays as <name>.migrated."""
    for name in ("missions.json", "places.json"):
        old = CONFIG_DIR / name
        if not old.exists():
            continue
        for p in profiles.profiles:
            new = robot_dir(p.id) / name
            if not new.exists():
                new.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(old, new)
        old.replace(old.with_name(name + ".migrated"))

    active = profiles.active()
    old = CONFIG_DIR / "events.csv"
    if old.exists() and active:
        new = robot_dir(active.id) / "events.csv"
        new.parent.mkdir(parents=True, exist_ok=True)
        if not new.exists():
            shutil.copy2(old, new)
        old.replace(old.with_name("events.csv.migrated"))

    poses = load_settings().get("last_pose")
    if isinstance(poses, dict):
        by_robot: dict = {}
        for key, pose in poses.items():
            rid, _, map_name = key.partition("/")
            by_robot.setdefault(rid, {})[map_name] = pose
        for rid, per_map in by_robot.items():
            st = RobotState(robot_dir(rid) / "state.json")
            st.update(last_pose={**per_map, **st.get("last_pose", {})})
        drop_setting("last_pose")
