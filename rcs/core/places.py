"""Named places on the map (Charger, Dock A…): go there with one click.

A place remembers which saved map it was stored on, because coordinates mean
nothing on another map.
"""
from __future__ import annotations

import json
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import List, Optional

from .profile import CONFIG_DIR, atomic_write

PLACES_PATH = CONFIG_DIR / "places.json"


@dataclass
class Place:
    id: str
    name: str
    x: float
    y: float
    yaw: float = 0.0
    map: str = ""           # saved map it belongs to; "" = built with SLAM or unknown


class PlaceStore:
    def __init__(self, path: Path = PLACES_PATH) -> None:
        self.use(path)

    def use(self, path: Path) -> None:
        """Work on another robot's file from now on."""
        self.path = path
        self.places: List[Place] = []
        try:
            data = json.loads(self.path.read_text())
            self.places = [Place(**{k: d[k] for k in Place.__dataclass_fields__ if k in d})
                           for d in data.get("places", [])]
        except (OSError, ValueError, TypeError, AttributeError):
            pass

    def save(self) -> None:
        atomic_write(self.path, json.dumps({"places": [asdict(p) for p in self.places]}, indent=2))

    def add(self, name: str, x: float, y: float, yaw: float, map_name: str = "") -> Place:
        p = Place(str(uuid.uuid4())[:8], name, x, y, yaw, map_name)
        self.places.append(p)
        self.save()
        return p

    def remove(self, pid: str) -> None:
        self.places = [p for p in self.places if p.id != pid]
        self.save()

    def get(self, pid: str) -> Optional[Place]:
        return next((p for p in self.places if p.id == pid), None)

    def on_map(self, map_name: str) -> List[Place]:
        """Places usable on this map; with no map known, all of them."""
        return [p for p in self.places if not map_name or not p.map or p.map == map_name]
