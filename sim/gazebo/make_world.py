#!/usr/bin/env python3
"""Occupancy map -> Gazebo world with the same walls, in the same coordinates.

The world frame is the map frame, so a pose on the map is the same pose in
Gazebo: AMCL finds real walls where the map says they are, and the ground truth
of the simulated robot can be compared directly with what the app shows.

    make_world.py <map.yaml> <out.world> [--height 2.0]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

MATERIALS = ("Gazebo/Grey", "Gazebo/Wood", "Gazebo/Bricks", "Gazebo/Gold", "Gazebo/Green", "Gazebo/Blue")
BAND = 4.0          # m: walls change color every BAND metres, so the 3D map has something to show


def read_map(path: Path):
    meta = yaml.safe_load(path.read_text())
    img = np.array(Image.open(path.parent / meta["image"]).convert("L"))
    occ = (255 - img.astype(np.float32)) / 255.0 > float(meta.get("occupied_thresh", 0.65))
    if meta.get("negate"):
        occ = img.astype(np.float32) / 255.0 > float(meta.get("occupied_thresh", 0.65))
    return occ[::-1, :], float(meta["resolution"]), meta["origin"]     # row 0 = lowest y


def rectangles(occ: np.ndarray) -> list:
    """Cover the occupied cells with few rectangles: (row, col, rows, cols)."""
    left = occ.copy()
    out = []
    for r, c in zip(*np.nonzero(occ)):
        if not left[r, c]:
            continue
        w = 1
        while c + w < occ.shape[1] and left[r, c + w]:
            w += 1
        h = 1
        while r + h < occ.shape[0] and left[r + h, c:c + w].all():
            h += 1
        left[r:r + h, c:c + w] = False
        out.append((int(r), int(c), h, w))
    return out


def wall(i: int, r: int, c: int, h: int, w: int, res: float, ox: float, oy: float, height: float) -> str:
    x = ox + (c + w / 2) * res
    y = oy + (r + h / 2) * res
    size = f"{w * res:.3f} {h * res:.3f} {height:.2f}"
    material = MATERIALS[int((x + y) // BAND) % len(MATERIALS)]
    geometry = f"<geometry><box><size>{size}</size></box></geometry>"
    return f"""      <collision name="c{i}"><pose>{x:.3f} {y:.3f} {height / 2:.2f} 0 0 0</pose>{geometry}</collision>
      <visual name="v{i}"><pose>{x:.3f} {y:.3f} {height / 2:.2f} 0 0 0</pose>{geometry}
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name></script></material>
      </visual>"""


def world(walls: str, cx: float, cy: float, size: float, extra: str = "") -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <world name="rcs_sim">
    <physics type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.75 0.8 0.85 1</background><shadows>false</shadows></scene>
    <light name="sun" type="directional">
      <pose>0 0 20 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -0.9</direction><cast_shadows>false</cast_shadows>
    </light>
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <pose>{cx:.2f} {cy:.2f} -0.05 0 0 0</pose>
          <geometry><box><size>{size:.1f} {size:.1f} 0.1</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <pose>{cx:.2f} {cy:.2f} -0.05 0 0 0</pose>
          <geometry><box><size>{size:.1f} {size:.1f} 0.1</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/White</name></script></material>
        </visual>
      </link>
    </model>
    <model name="walls">
      <static>true</static>
      <link name="link">
{walls}
      </link>
    </model>
{extra}
  </world>
</sdf>
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("map")
    ap.add_argument("out")
    ap.add_argument("--height", type=float, default=2.0)
    a = ap.parse_args()
    occ, res, origin = read_map(Path(a.map))
    ox, oy = float(origin[0]), float(origin[1])
    rects = rectangles(occ)
    walls = "\n".join(wall(i, *r, res, ox, oy, a.height) for i, r in enumerate(rects))
    h, w = occ.shape
    cx, cy = ox + w * res / 2, oy + h * res / 2
    Path(a.out).write_text(world(walls, cx, cy, max(w, h) * res + 10.0))
    print(f"{a.out}: {len(rects)} wall boxes from {int(occ.sum())} occupied cells, {w * res:.1f} x {h * res:.1f} m")


if __name__ == "__main__":
    main()
