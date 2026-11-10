"""Keepout and speed zones of a map, for Nav2's costmap filters.

The app draws the zones; they are kept on the robot next to their map as
<map>.zones.json:

    {"version": 1, "map": "office", "zones": [
        {"id": "k1", "name": "Stairs", "kind": "keepout", "points": [[x, y], ...]},
        {"id": "s1", "name": "Hall", "kind": "slow", "speed": 40, "points": [...]}]}

points are polygon corners in the map frame (metres); speed is the most the
robot may drive in a slow zone, in percent of its usual top speed.

Nav2 reads them as two filter masks the size of the map (same origin and
resolution), loaded in "raw" mode so a pixel is the value Nav2 gets:
  keepout mask  100 inside a keepout zone (lethal), 0 elsewhere
  speed mask    the zone's percent inside a slow zone (the lowest where two
                overlap), 0 elsewhere = no limit
and the filters are added to a copy of the robot's Nav2 parameters, so its
own files are never changed. No ROS imports here.
"""
from __future__ import annotations

import json
import math
import os
import re
from pathlib import Path
from typing import List, Tuple

import numpy as np

KINDS = ("keepout", "slow")
MAX_ZONES = 200
MAX_POINTS = 100
SPEED_MIN, SPEED_MAX = 10, 90
SUFFIX = ".zones.json"

# names the filters run under (robot/costmap_filters.launch.py and the costmaps)
KEEPOUT_INFO, KEEPOUT_MASK = "/costmap_filter_info", "/keepout_filter_mask"
SPEED_INFO, SPEED_MASK = "/speed_filter_info", "/speed_filter_mask"
SPEED_LIMIT = "/speed_limit"


def _finite(v, what: str) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"{what} is not a number") from None
    if not math.isfinite(f) or abs(f) > 1e5:
        raise ValueError(f"{what} is not a usable coordinate")
    return f


def _area(pts: List[Tuple[float, float]]) -> float:
    return 0.5 * abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1])))


def check_zones(zones) -> list:
    """The zones, checked and in a standard form; raises ValueError saying why not."""
    if not isinstance(zones, list):
        raise ValueError("zones must be a list")
    if len(zones) > MAX_ZONES:
        raise ValueError(f"at most {MAX_ZONES} zones per map")
    out, ids = [], set()
    for i, z in enumerate(zones, 1):
        if not isinstance(z, dict):
            raise ValueError(f"zone {i} is not an object")
        zid = str(z.get("id", "") or f"z{i}")
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", zid) or zid in ids:
            raise ValueError(f"zone {i}: bad or repeated id {zid!r}")
        ids.add(zid)
        name = str(z.get("name", ""))[:40].strip() or f"Zone {i}"
        kind = z.get("kind")
        if kind not in KINDS:
            raise ValueError(f"zone {name!r}: kind must be keepout or slow")
        pts = z.get("points")
        if not isinstance(pts, list) or not 3 <= len(pts) <= MAX_POINTS:
            raise ValueError(f"zone {name!r}: 3 to {MAX_POINTS} corners")
        corners = []
        for p in pts:
            if not isinstance(p, (list, tuple)) or len(p) != 2:
                raise ValueError(f"zone {name!r}: a corner is not [x, y]")
            corners.append((round(_finite(p[0], f"zone {name!r} x"), 3), round(_finite(p[1], f"zone {name!r} y"), 3)))
        if _area(corners) < 0.01:
            raise ValueError(f"zone {name!r} has no area")
        zone = {"id": zid, "name": name, "kind": kind, "points": [list(c) for c in corners]}
        if kind == "slow":
            try:
                speed = int(z.get("speed", 50))
            except (TypeError, ValueError):
                raise ValueError(f"zone {name!r}: speed is not a number") from None
            if not SPEED_MIN <= speed <= SPEED_MAX:
                raise ValueError(f"zone {name!r}: speed {SPEED_MIN} to {SPEED_MAX} %")
            zone["speed"] = speed
        out.append(zone)
    return out


def zones_path(map_yaml) -> Path:
    p = Path(map_yaml)
    return p.with_name(p.stem + SUFFIX)


def read_zones(map_yaml) -> list:
    """The zones kept for a map ([] when none, or when the file is unreadable)."""
    try:
        d = json.loads(zones_path(map_yaml).read_text(encoding="utf-8"))
        return check_zones(d.get("zones", []))
    except (OSError, ValueError, AttributeError):
        return []


def write_zones(map_yaml, zones) -> list:
    zones = check_zones(zones)
    path = zones_path(map_yaml)
    if not zones:
        try:
            path.unlink()
        except FileNotFoundError:
            pass
        return zones
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps({"version": 1, "map": Path(map_yaml).stem, "zones": zones}, indent=1),
                   encoding="utf-8")
    os.replace(tmp, path)
    return zones


def map_geometry(map_yaml) -> Tuple[int, int, float, float, float]:
    """(width, height, resolution, origin x, origin y) of a map; raises if it
    is not a .pgm map (its size is needed for the masks)."""
    yp = Path(map_yaml)
    text = yp.read_text(encoding="utf-8")
    fields = {}
    for line in text.splitlines():
        if ":" in line and not line.lstrip().startswith("#"):
            k, v = line.split(":", 1)
            fields[k.strip()] = v.split("#", 1)[0].strip().strip("'\"")
    image = fields.get("image", "")
    img = Path(image) if Path(image).is_absolute() else yp.parent / image
    if img.suffix.lower() != ".pgm":
        raise ValueError(f"zones need a .pgm map, not {img.suffix or 'this one'}")
    with open(img, "rb") as f:
        head = f.read(4096)
    w, h = _pgm_header(head)
    res = float(fields.get("resolution", "0.05"))
    origin = [float(v) for v in re.findall(r"-?[\d.]+(?:e-?\d+)?", fields.get("origin", "[0, 0, 0]"))[:2]]
    if not (res > 0 and len(origin) == 2):
        raise ValueError("the map's resolution or origin cannot be read")
    return w, h, res, origin[0], origin[1]


def _pgm_header(head: bytes) -> Tuple[int, int]:
    """Width and height from the start of a PGM (pgm_size wants it whole)."""
    tokens, i = [], 0
    while len(tokens) < 3 and i < len(head):
        if head[i:i + 1].isspace():
            i += 1
            continue
        if head[i:i + 1] == b"#":
            i = head.find(b"\n", i) + 1 or len(head)
            continue
        j = i
        while j < len(head) and not head[j:j + 1].isspace():
            j += 1
        tokens.append(head[i:j])
        i = j
    if len(tokens) < 3 or tokens[0] != b"P5":
        raise ValueError("not a binary PGM")
    return int(tokens[1]), int(tokens[2])


def inside(points, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """Which of the points (xs, ys) are inside the polygon (even-odd rule)."""
    res = np.zeros(xs.shape, bool)
    n = len(points)
    for k in range(n):
        x0, y0 = points[k]
        x1, y1 = points[(k + 1) % n]
        if y0 == y1:
            continue
        crosses = (y0 > ys) != (y1 > ys)
        xc = x0 + (ys - y0) * (x1 - x0) / (y1 - y0)
        res ^= crosses & (xs < xc)
    return res


def rasterize(zones, width: int, height: int, res: float, ox: float, oy: float) -> Tuple[np.ndarray, np.ndarray]:
    """(keepout, speed) masks as PGM rows (row 0 = the top of the map)."""
    keep = np.zeros((height, width), np.uint8)
    speed = np.zeros((height, width), np.uint8)
    for z in check_zones(zones):
        pts = z["points"]
        xs_, ys_ = [p[0] for p in pts], [p[1] for p in pts]
        c0 = max(0, int(math.floor((min(xs_) - ox) / res)))
        c1 = min(width, int(math.ceil((max(xs_) - ox) / res)) + 1)
        r0 = max(0, int(math.floor((min(ys_) - oy) / res)))    # rows from the bottom
        r1 = min(height, int(math.ceil((max(ys_) - oy) / res)) + 1)
        if c0 >= c1 or r0 >= r1:
            continue                                             # off the map
        cols, rows = np.meshgrid(np.arange(c0, c1), np.arange(r0, r1))
        hit = inside(pts, ox + (cols + 0.5) * res, oy + (rows + 0.5) * res)
        top = height - 1 - rows[hit]                             # PGM rows run from the top
        if z["kind"] == "keepout":
            keep[top, cols[hit]] = 100
        else:
            cur = speed[top, cols[hit]]
            speed[top, cols[hit]] = np.where((cur == 0) | (cur > z["speed"]), z["speed"], cur)
    return keep, speed


def _write_mask(path: Path, img: np.ndarray, res: float, ox: float, oy: float) -> Path:
    h, w = img.shape
    pgm = path.with_suffix(".pgm")
    tmp = pgm.with_name(pgm.name + ".tmp")
    tmp.write_bytes(b"P5\n%d %d\n255\n" % (w, h) + img.tobytes())
    os.replace(tmp, pgm)
    yml = path.with_suffix(".yaml")
    yml.write_text(f"image: {pgm.name}\nmode: raw\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\n"
                   "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n", encoding="utf-8")
    return yml


def write_masks(map_yaml, zones, out_dir) -> dict:
    """Both masks of a map's zones into out_dir; {"keepout": yaml, "speed": yaml,
    "keepout_cells", "slow_cells"}. Written even with no zones (empty masks),
    so the filters run and a later change applies without restarting Nav2."""
    w, h, res, ox, oy = map_geometry(map_yaml)
    keep, speed = rasterize(zones, w, h, res, ox, oy)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    return {"keepout": str(_write_mask(out / "keepout_mask", keep, res, ox, oy)),
            "speed": str(_write_mask(out / "speed_mask", speed, res, ox, oy)),
            "keepout_cells": int((keep > 0).sum()), "slow_cells": int((speed > 0).sum())}


def params_with_filters(src_yaml, dst_yaml) -> str:
    """A copy of Nav2's parameters with the keepout filter in both costmaps and
    the speed filter in the local one: one /speed_limit publisher, and the
    local costmap updates several times a second (the global one about once),
    so the limit comes within a few centimetres of the zone's edge."""
    import yaml     # python3-yaml comes with ROS
    d = yaml.safe_load(Path(src_yaml).read_text(encoding="utf-8")) or {}
    for name, filters in (("global_costmap", ("keepout_filter",)),
                          ("local_costmap", ("keepout_filter", "speed_filter"))):
        node = ((d.get(name) or {}).get(name) or {}).get("ros__parameters")
        if not isinstance(node, dict):
            continue
        have = list(node.get("filters") or [])
        node["filters"] = have + [f for f in filters if f not in have]
        node["keepout_filter"] = {"plugin": "nav2_costmap_2d::KeepoutFilter", "enabled": True,
                                  "filter_info_topic": KEEPOUT_INFO}
        if "speed_filter" in filters:
            node["speed_filter"] = {"plugin": "nav2_costmap_2d::SpeedFilter", "enabled": True,
                                    "filter_info_topic": SPEED_INFO, "speed_limit_topic": SPEED_LIMIT}
    ctrl = (d.get("controller_server") or {}).get("ros__parameters")
    if isinstance(ctrl, dict):
        ctrl["speed_limit_topic"] = SPEED_LIMIT
    Path(dst_yaml).write_text(yaml.safe_dump(d, sort_keys=False), encoding="utf-8")
    return str(dst_yaml)


def setup(map_yaml, nav2_params, out_dir) -> Tuple[str, dict]:
    """What the launcher does before starting Nav2 on a map: the masks of its
    zones and the parameters with the filters. (params file, masks)."""
    masks = write_masks(map_yaml, read_zones(map_yaml), out_dir)
    return params_with_filters(nav2_params, Path(out_dir) / "nav2_params.yaml"), masks


if __name__ == "__main__":
    # python3 zone_masks.py <map.yaml> <nav2 params.yaml> <out dir>
    #   prints: params file, keepout mask, speed mask (one per line); for sim/run_sim.sh
    import sys
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    params, m = setup(*sys.argv[1:])
    print(params, m["keepout"], m["speed"], sep="\n")
