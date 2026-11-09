"""Missions that start by themselves at set times.

  robots/<id>/schedules.json

A schedule is a mission, a time of day and the days of the week. It runs only
while the app is open and connected to that robot: the missions are driven
from the app. When a time comes and the robot is not ready (not connected,
Nav2 not running, E-STOP, low battery, another mission running, wrong map)
the run is skipped and the report says why. A time that passed while the app
was closed is not made up for later: the robot may not be where it was, and
nobody may expect it to move.
"""
from __future__ import annotations

import json
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timedelta
from pathlib import Path
from typing import Callable, List, Optional, Tuple

from PySide6.QtCore import QObject, QTimer, Signal

from .profile import atomic_write

DAYS = ("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")
TICK_MS = 5000
LATE_S = 120.0          # a time missed by more than this (PC asleep) is skipped, not run late


def parse_hhmm(text: str) -> Tuple[int, int]:
    h, m = text.strip().split(":")
    h, m = int(h), int(m)
    if not (0 <= h < 24 and 0 <= m < 60):
        raise ValueError(f"not a time of day: {text}")
    return h, m


@dataclass
class Schedule:
    id: str
    mission_id: str
    at: str = "08:00"               # HH:MM, local time of this PC
    days: List[int] = field(default_factory=lambda: [0, 1, 2, 3, 4])    # 0 = Monday
    enabled: bool = True
    min_soc: int = 30               # % of charge needed to start; 0 = do not check

    @classmethod
    def new(cls, mission_id: str, at: str = "08:00") -> "Schedule":
        return cls(id=uuid.uuid4().hex[:8], mission_id=mission_id, at=at)

    @classmethod
    def from_dict(cls, d: dict) -> "Schedule":
        s = cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})
        s.days = sorted({int(x) for x in s.days if 0 <= int(x) <= 6})
        parse_hhmm(s.at)
        return s

    def days_text(self) -> str:
        if self.days == list(range(7)):
            return "every day"
        if self.days == list(range(5)):
            return "Mon–Fri"
        return ", ".join(DAYS[d] for d in self.days) or "no day"

    def times_between(self, t0: float, t1: float) -> List[float]:
        """Its start times in (t0, t1]."""
        h, m = parse_hhmm(self.at)
        out = []
        day = datetime.fromtimestamp(t0).replace(hour=0, minute=0, second=0, microsecond=0)
        end = datetime.fromtimestamp(t1)
        while day <= end:
            at = day.replace(hour=h, minute=m)
            ts = at.timestamp()
            if t0 < ts <= t1 and at.weekday() in self.days:
                out.append(ts)
            day += timedelta(days=1)
        return out

    def next_time(self, after: float) -> Optional[float]:
        if not self.days:
            return None
        times = self.times_between(after, after + 8 * 86400)
        return times[0] if times else None


class ScheduleStore:
    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.schedules: List[Schedule] = []
        self.load()

    def use(self, path: Path) -> None:
        self.path = Path(path)
        self.load()

    def load(self) -> None:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            out = []
            for d in data.get("schedules", []):
                try:
                    out.append(Schedule.from_dict(d))
                except (TypeError, ValueError):
                    pass            # one bad entry must not lose the others
            self.schedules = out
        except (OSError, ValueError, AttributeError):
            self.schedules = []

    def save(self) -> None:
        try:
            atomic_write(self.path, json.dumps({"schedules": [asdict(s) for s in self.schedules]}, indent=2))
        except OSError:
            pass

    def upsert(self, s: Schedule) -> None:
        for i, cur in enumerate(self.schedules):
            if cur.id == s.id:
                self.schedules[i] = s
                break
        else:
            self.schedules.append(s)
        self.save()

    def remove(self, sid: str) -> None:
        self.schedules = [s for s in self.schedules if s.id != sid]
        self.save()

    def get(self, sid: str) -> Optional[Schedule]:
        return next((s for s in self.schedules if s.id == sid), None)


class Scheduler(QObject):
    """Checks the clock; when a schedule's time comes, asks `start` to run it.
    `start(schedule)` returns "" when the run started, else why it did not."""

    fired = Signal(object, str)         # Schedule, "" or why it was skipped

    def __init__(self, store: ScheduleStore, start: Callable[[Schedule], str],
                 clock: Callable[[], float] = time.time, parent=None) -> None:
        super().__init__(parent)
        self.store, self.start, self.clock = store, start, clock
        self.last = clock()             # nothing before the app started is made up for
        self.timer = QTimer(self)
        self.timer.setInterval(TICK_MS)
        self.timer.timeout.connect(self.tick)
        self.timer.start()

    def reset(self) -> None:
        self.last = self.clock()

    def tick(self) -> None:
        now = self.clock()
        due = []
        for s in self.store.schedules:
            if s.enabled:
                due += [(t, s) for t in s.times_between(self.last, now)]
        self.last = now
        for t, s in sorted(due, key=lambda x: x[0]):
            if now - t > LATE_S:
                why = f"its time {time.strftime('%H:%M', time.localtime(t))} passed while this PC was asleep"
            else:
                why = self.start(s)
            self.fired.emit(s, why)
