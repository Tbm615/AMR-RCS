"""A report for every mission run: started by hand or by the schedule, how it
ended, how far it got, and the battery before and after.

  robots/<id>/runs.json

Kept for good, like the battery log: nothing here deletes a report. The run
going on is written at each step, so a crash of the app leaves it marked
"interrupted" the next time, not lost.
"""
from __future__ import annotations

import csv
import json
import time
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List, Optional

from PySide6.QtCore import QObject, Signal

from .profile import atomic_write

RESULTS = ("running", "complete", "stopped", "failed", "skipped", "interrupted")


@dataclass
class Run:
    id: str
    mission_id: str
    mission: str
    trigger: str = "manual"         # manual | schedule
    schedule_id: str = ""
    map: str = ""
    start: float = 0.0              # unix time
    end: Optional[float] = None
    result: str = "running"
    reason: str = ""
    waypoints: int = 0              # in the mission
    reached: int = 0                # arrivals, over all laps
    laps: int = 0                   # whole laps done
    retries: int = 0
    distance_m: float = 0.0
    soc_start: Optional[float] = None
    soc_end: Optional[float] = None
    events: List[str] = field(default_factory=list)

    def duration(self) -> float:
        return ((self.end or time.time()) - self.start) if self.start else 0.0

    @classmethod
    def from_dict(cls, d: dict) -> "Run":
        known = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
        return cls(**known)


class RunLog(QObject):
    changed = Signal()

    def __init__(self, path: Path, parent=None) -> None:
        super().__init__(parent)
        self.path = Path(path)
        self.runs: List[Run] = []
        self.active: Optional[Run] = None
        self._load()

    def use(self, path: Path) -> None:
        self.path = Path(path)
        self.runs, self.active = [], None
        self._load()
        self.changed.emit()

    def _load(self) -> None:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            self.runs = [Run.from_dict(d) for d in data.get("runs", []) if isinstance(d, dict)]
        except (OSError, ValueError, TypeError):
            self.runs = []
        dirty = False
        for r in self.runs:
            if r.result == "running":       # the app went away during it
                r.result, r.reason = "interrupted", "the app closed or crashed during the run"
                r.end = r.end or r.start
                dirty = True
        if dirty:
            self._save()

    def _save(self) -> None:
        try:
            atomic_write(self.path, json.dumps({"runs": [asdict(r) for r in self.runs]}, indent=1))
        except OSError:
            pass

    def _touch(self) -> None:
        self._save()
        self.changed.emit()

    # ---- a run ----

    def start(self, mission, trigger: str = "manual", schedule_id: str = "",
              soc: Optional[float] = None) -> Run:
        if self.active is not None:
            self.finish("stopped", "another run started")
        r = Run(id=uuid.uuid4().hex[:10], mission_id=mission.id, mission=mission.name, trigger=trigger,
                schedule_id=schedule_id, map=mission.map, start=time.time(),
                waypoints=len(mission.waypoints), soc_start=soc)
        self.runs.append(r)
        self.active = r
        self._touch()
        return r

    def event(self, text: str) -> None:
        if self.active is not None:
            self.active.events.append(f"{time.strftime('%H:%M:%S')} {text}")
            self._touch()

    def reached(self, name: str) -> None:
        if self.active is not None:
            self.active.reached += 1
            self.event(f"arrived at {name}")

    def retry(self, text: str) -> None:
        if self.active is not None:
            self.active.retries += 1
            self.event(text)

    def lap(self) -> None:
        if self.active is not None:
            self.active.laps += 1
            self._touch()

    def add_distance(self, d: float) -> None:
        """Called often while driving: written with the next step, not each time."""
        if self.active is not None:
            self.active.distance_m += d

    def finish(self, result: str, reason: str = "", soc: Optional[float] = None) -> Optional[Run]:
        r = self.active
        if r is None:
            return None
        r.result, r.reason, r.end, r.soc_end = result, reason, time.time(), soc
        r.distance_m = round(r.distance_m, 1)
        self.active = None
        self._touch()
        return r

    def skipped(self, mission_id: str, mission: str, reason: str, schedule_id: str = "",
                soc: Optional[float] = None) -> Run:
        """A scheduled run that could not start: reported too, so a missing
        run is never a mystery."""
        now = time.time()
        r = Run(id=uuid.uuid4().hex[:10], mission_id=mission_id, mission=mission, trigger="schedule",
                schedule_id=schedule_id, start=now, end=now, result="skipped", reason=reason,
                soc_start=soc, soc_end=soc)
        self.runs.append(r)
        self._touch()
        return r

    # ---- out ----

    def export_csv(self, path: Path) -> int:
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["start", "end", "mission", "trigger", "result", "reason", "duration_s", "waypoints",
                        "reached", "laps", "retries", "distance_m", "battery_start_%", "battery_end_%", "map",
                        "events"])
            for r in self.runs:
                w.writerow([time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r.start)),
                            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r.end)) if r.end else "",
                            r.mission, r.trigger, r.result, r.reason, round(r.duration()), r.waypoints,
                            r.reached, r.laps, r.retries, r.distance_m,
                            "" if r.soc_start is None else r.soc_start, "" if r.soc_end is None else r.soc_end,
                            r.map, " | ".join(r.events)])
        return len(self.runs)
