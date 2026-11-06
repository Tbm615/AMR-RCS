"""Mission model: ordered waypoints (checkpoints) with optional loop route."""
from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import List, Optional

from .profile import MISSIONS_PATH, atomic_write


@dataclass
class Waypoint:
    id: str
    name: str
    x: float
    y: float
    yaw: float = 0.0
    tolerance: float = 0.25     # m, how close counts as arrived
    dwell: float = 0.0          # s to wait after arrival before next point

    @classmethod
    def new(cls, name: str, x: float, y: float, yaw: float = 0.0) -> "Waypoint":
        return cls(id=str(uuid.uuid4())[:8], name=name, x=x, y=y, yaw=yaw)


@dataclass
class Mission:
    id: str
    name: str
    loop: bool = False
    laps: int = 0               # with loop: laps to run, 0 = until stopped
    waypoints: List[Waypoint] = field(default_factory=list)
    map: str = ""               # saved map its coordinates are on; "" = SLAM or unknown

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "loop": self.loop,
            "laps": self.laps,
            "map": self.map,
            "waypoints": [asdict(w) for w in self.waypoints],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Mission":
        return cls(
            id=d.get("id") or str(uuid.uuid4())[:8],
            name=d.get("name", "mission"),
            loop=bool(d.get("loop", False)),
            laps=int(d.get("laps", 0)),
            map=str(d.get("map", "")),
            # ignore keys this version doesn't know (saved by a newer RCS)
            waypoints=[Waypoint(**{k: v for k, v in w.items() if k in Waypoint.__dataclass_fields__})
                       for w in d.get("waypoints", [])],
        )

    @classmethod
    def new(cls, name: str = "New mission") -> "Mission":
        return cls(id=str(uuid.uuid4())[:8], name=name)


class MissionStore:
    def __init__(self, path: Path = MISSIONS_PATH) -> None:
        self.path = path
        self.missions: List[Mission] = []
        self.load()

    def use(self, path: Path) -> None:
        """Work on another robot's file from now on."""
        self.path = path
        self.missions = []
        self.load()

    def load(self) -> None:
        if not self.path.exists():
            self.save()
            return
        try:
            data = json.loads(self.path.read_text())
        except (OSError, json.JSONDecodeError):
            data = {}
        self.missions = [Mission.from_dict(d) for d in data.get("missions", [])]

    def save(self) -> None:
        atomic_write(self.path, json.dumps(
            {"missions": [m.to_dict() for m in self.missions]}, indent=2))

    def upsert(self, m: Mission) -> None:
        for i, cur in enumerate(self.missions):
            if cur.id == m.id:
                self.missions[i] = m
                self.save()
                return
        self.missions.append(m)
        self.save()

    def remove(self, mid: str) -> None:
        self.missions = [m for m in self.missions if m.id != mid]
        self.save()

    def get(self, mid: str) -> Optional[Mission]:
        for m in self.missions:
            if m.id == mid:
                return m
        return None
