#!/usr/bin/env python3
"""Make rcs/assets/robot_model.npz, the Hinson AMR the app comes with, from the
robot's URDF.

Run on a PC with ROS 2 and this workspace built, whenever the robot's URDF or
meshes change; the app itself only needs the .npz:
    source ../install/setup.bash && python3 tools/build_robot_model.py

The URDF is read the way the app reads an imported one (rcs.core.robot_model),
with one thing changed for display: the URDF draws the camera as a plain box,
so the vendor's camera model is used, turned to lie across the robot and made
lighter (it has more triangles than the rest of the robot together).
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from rcs.core import robot_model as rm  # noqa: E402

WS = HERE.parent.parent
URDF = WS / "src/bringup/urdf/hinson_robot.urdf.xacro"
OUT = HERE.parent / "rcs/assets/robot_model.npz"
CAMERA_MESH = WS / "src/OrbbecSDK_ROS2-main/orbbec_description/meshes/AstraPlus.stl"
CAMERA_COLOR = (0.12, 0.13, 0.16, 1.0)
CAMERA_GRID = 0.002     # m, detail kept on the camera


def camera() -> np.ndarray:
    raw = rm.read_stl(CAMERA_MESH)
    if np.abs(raw).max() > 5:           # the vendor's model is in millimetres
        raw = raw * 0.001
    raw = rm.simplify(raw, CAMERA_GRID)
    flat = raw.reshape(-1, 3)
    raw = raw - (flat.min(0) + flat.max(0)) / 2          # centred on camera_link
    turn = rm.rpy_matrix(0, 0, np.pi / 2)                # long side across the robot
    return (raw.reshape(-1, 3) @ turn.T).reshape(-1, 3, 3)


def main() -> None:
    xml = subprocess.run(["xacro", str(URDF), "use_mock_hardware:=true"], capture_output=True, text=True,
                         check=True).stdout
    model, warnings = rm.from_urdf(URDF, text=xml, search=[WS / "src"],
                                   replace={"camera_link": (camera(), CAMERA_COLOR)})
    for w in warnings:
        print("warning:", w)
    model.name, model.source = "Hinson AMR", URDF.name
    for part, name in rm.PART_NAMES.items():
        sel = model.parts == part
        if sel.any():
            pts = model.triangles[sel].reshape(-1, 3)
            lo, hi = pts.min(0), pts.max(0)
            print(f"{name:13s} {sel.sum():6d} tris  x {lo[0]:+.3f}..{hi[0]:+.3f}  "
                  f"y {lo[1]:+.3f}..{hi[1]:+.3f}  z {lo[2]:+.3f}..{hi[2]:+.3f}")
    rm.save(model, OUT)
    lo, hi = model.bounds()
    print(f"all: {len(model)} triangles, size {hi - lo}, lo {lo}, hi {hi}, {OUT.stat().st_size // 1024} KB")


if __name__ == "__main__":
    main()
