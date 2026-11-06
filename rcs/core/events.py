"""Event log: what happened and when (connection, E-STOP, goals, missions,
robot programs, lost sensors), for looking back at a run.

Kept on disk next to the other settings so it survives a restart or a crash;
one line per event, newest last.
"""
from __future__ import annotations

import csv
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List

from PySide6.QtCore import QObject, Signal

from .profile import CONFIG_DIR

EVENTS_PATH = CONFIG_DIR / "events.csv"
KEEP = 2000             # events shown and kept in the file
LEVELS = ("info", "warn", "error")


@dataclass
class Event:
    t: float            # unix time
    level: str          # info | warn | error
    source: str         # Connection, E-STOP, Navigation, Mission, Robot, Sensors
    text: str

    def time_str(self) -> str:
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.t))


def write_csv(path: Path, events: Iterable[Event]) -> int:
    n = 0
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time", "level", "source", "event"])
        for e in events:
            w.writerow([e.time_str(), e.level, e.source, e.text])
            n += 1
    return n


class EventLog(QObject):
    added = Signal(object)      # Event
    cleared = Signal()

    def __init__(self, path: Path = EVENTS_PATH, parent=None):
        super().__init__(parent)
        self.path = path
        self.events: deque = deque(maxlen=KEEP)
        self._load()

    def use(self, path: Path) -> None:
        """Show and write another robot's log from now on."""
        self.path = path
        self.events.clear()
        self._load()
        self.cleared.emit()

    def _load(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.path, newline="", encoding="utf-8") as f:
                for row in csv.reader(f):
                    if len(row) == 4:
                        self.events.append(Event(float(row[0]), row[1], row[2], row[3]))
        except (OSError, ValueError, csv.Error):
            pass
        self._rewrite()         # drops what fell out of the window

    def _rewrite(self) -> None:
        try:
            with open(self.path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                for e in self.events:
                    w.writerow([e.t, e.level, e.source, e.text])
        except OSError:
            pass

    def add(self, level: str, source: str, text: str) -> Event:
        e = Event(time.time(), level, source, text)
        self.events.append(e)
        try:
            with open(self.path, "a", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow([e.t, e.level, e.source, e.text])
        except OSError:
            pass
        self.added.emit(e)
        return e

    def clear(self) -> None:
        self.events.clear()
        self._rewrite()
        self.cleared.emit()

    def sources(self) -> List[str]:
        return sorted({e.source for e in self.events})
