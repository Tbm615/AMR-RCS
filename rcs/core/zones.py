"""Keepout and speed zones of a map, as the app edits them. The robot keeps
them next to the map and turns them into Nav2's costmap filters
(robot/zone_masks.py has the same rules and checks them again). No Qt here.

A zone: {"id", "name", "kind": "keepout" | "slow", "speed": percent (slow
only), "points": [[x, y], ...]} in the map frame, metres.
"""
from __future__ import annotations

import math
from typing import Iterable, List, Optional, Tuple

KINDS = ("keepout", "slow")
MAX_ZONES = 200
MAX_POINTS = 100
SPEED_MIN, SPEED_MAX = 10, 90
CAP = "zones"           # in the launcher's state when the robot side has them


def area(points) -> float:
    pts = [tuple(p) for p in points]
    return 0.5 * abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1])))


def centroid(points) -> Tuple[float, float]:
    pts = [tuple(p) for p in points]
    a = cx = cy = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        c = x0 * y1 - x1 * y0
        a += c
        cx += (x0 + x1) * c
        cy += (y0 + y1) * c
    if abs(a) < 1e-12:
        return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
    return cx / (3 * a), cy / (3 * a)


def contains(points, x: float, y: float) -> bool:
    """Is (x, y) inside the polygon (even-odd rule, as the robot's masks)."""
    pts = [tuple(p) for p in points]
    inside = False
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def check(zones) -> list:
    """The zones in a standard form; raises ValueError saying why not."""
    if not isinstance(zones, list):
        raise ValueError("zones must be a list")
    if len(zones) > MAX_ZONES:
        raise ValueError(f"at most {MAX_ZONES} zones per map")
    out, ids = [], set()
    for i, z in enumerate(zones, 1):
        if not isinstance(z, dict):
            raise ValueError(f"zone {i} is not an object")
        name = str(z.get("name", ""))[:40].strip() or f"Zone {i}"
        zid = str(z.get("id", "") or f"z{i}")
        if zid in ids or not zid.replace("-", "").replace("_", "").isalnum() or len(zid) > 40:
            raise ValueError(f"zone {name!r}: bad or repeated id")
        ids.add(zid)
        if z.get("kind") not in KINDS:
            raise ValueError(f"zone {name!r}: kind must be keepout or slow")
        pts = z.get("points")
        if not isinstance(pts, list) or not 3 <= len(pts) <= MAX_POINTS:
            raise ValueError(f"zone {name!r}: 3 to {MAX_POINTS} corners")
        corners = []
        for p in pts:
            try:
                x, y = float(p[0]), float(p[1])
            except (TypeError, ValueError, IndexError):
                raise ValueError(f"zone {name!r}: a corner is not [x, y]") from None
            if not (math.isfinite(x) and math.isfinite(y)) or max(abs(x), abs(y)) > 1e5:
                raise ValueError(f"zone {name!r}: a corner is not a usable coordinate")
            corners.append([round(x, 3), round(y, 3)])
        if area(corners) < 0.01:
            raise ValueError(f"zone {name!r} has no area")
        zone = {"id": zid, "name": name, "kind": z["kind"], "points": corners}
        if z["kind"] == "slow":
            try:
                speed = int(z.get("speed", 50))
            except (TypeError, ValueError):
                raise ValueError(f"zone {name!r}: speed is not a number") from None
            if not SPEED_MIN <= speed <= SPEED_MAX:
                raise ValueError(f"zone {name!r}: speed {SPEED_MIN} to {SPEED_MAX} %")
            zone["speed"] = speed
        out.append(zone)
    return out


def new_id(zones: Iterable[dict], kind: str) -> str:
    have = {z.get("id") for z in zones}
    i = 1
    while f"{kind[0]}{i}" in have:
        i += 1
    return f"{kind[0]}{i}"


def new_name(zones: Iterable[dict], kind: str, base: str = "") -> str:
    base = base or ("Keepout" if kind == "keepout" else "Slow")
    have = {z.get("name") for z in zones}
    i = 1
    while f"{base} {i}" in have:
        i += 1
    return f"{base} {i}"


def at(zones: Iterable[dict], x: float, y: float) -> List[dict]:
    """The zones that contain (x, y), keepout ones first."""
    hit = [z for z in zones if contains(z["points"], x, y)]
    return sorted(hit, key=lambda z: (z["kind"] != "keepout", z.get("speed", 0)))


def keepout_at(zones: Iterable[dict], x: float, y: float) -> Optional[dict]:
    for z in at(zones, x, y):
        if z["kind"] == "keepout":
            return z
    return None


def speed_at(zones: Iterable[dict], x: float, y: float) -> Optional[int]:
    """The speed limit (percent) at (x, y): the lowest slow zone there, or None."""
    speeds = [z["speed"] for z in at(zones, x, y) if z["kind"] == "slow"]
    return min(speeds) if speeds else None


def describe(z: dict) -> str:
    return f"{z['name']} · keepout" if z["kind"] == "keepout" else f"{z['name']} · slow {z['speed']} %"


def distance(points, x: float, y: float) -> float:
    """Metres from (x, y) to the polygon (0 inside)."""
    if contains(points, x, y):
        return 0.0
    pts = [tuple(p) for p in points]
    best = float("inf")
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        dx, dy = x1 - x0, y1 - y0
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(x - x0 - t * dx, y - y0 - t * dy))
    return best


NEAR_M = 1.0        # a keepout zone this close is worth saying when driving by hand


def note_at(zones: Iterable[dict], x: float, y: float) -> Tuple[str, List[str]]:
    """(level, sentences) about the zones at a robot position, for manual
    driving: level "" (nothing to say), "warn" or "info"."""
    zones = list(zones)
    inside = keepout_at(zones, x, y)
    if inside:
        return "warn", [f"Inside the keepout zone {inside['name']}: Nav2 cannot drive here.",
                        "Drive the robot out by hand."]
    near = sorted((distance(z["points"], x, y), z["name"]) for z in zones if z["kind"] == "keepout")
    speed = speed_at(zones, x, y)
    parts = []
    level = ""
    if near and near[0][0] <= NEAR_M:
        parts.append(f"Keepout zone {near[0][1]} {near[0][0]:.1f} m away.")
        level = "warn"
    if speed is not None:
        name = next(z["name"] for z in at(zones, x, y) if z["kind"] == "slow" and z["speed"] == speed)
        parts.append(f"In the slow zone {name}: Nav2 drives at most {speed} % here.")
        level = level or "info"
    if parts:
        parts.append("Manual driving is not limited by zones.")
    return level, parts
