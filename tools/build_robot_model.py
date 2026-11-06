#!/usr/bin/env python3
"""Make rcs/assets/robot_model.npz, the robot on the 3D Map page, from the URDF.

Run on a PC with ROS 2 and this workspace built, whenever the robot's URDF or
meshes change; the app itself only needs the .npz:
    source ../install/setup.bash && python3 tools/build_robot_model.py

Every mesh of the URDF is placed with the joints at zero and put in one list of
triangles, in base_link's frame, in metres, then lifted so the wheels stand on
the floor. Three things in the sources are corrected here, not in the URDF, which
drives the real robot: the drive wheel meshes have their origin on one face, the
camera's mesh is turned 90 degrees, and it has far more triangles than the rest
of the robot together.
"""
from __future__ import annotations

import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parent
URDF = WS / "src/bringup/urdf/hinson_robot.urdf.xacro"
OUT = HERE.parent / "rcs/assets/robot_model.npz"
# the URDF draws the camera as a plain box: use the vendor's model
CAMERA_MESH = WS / "src/OrbbecSDK_ROS2-main/orbbec_description/meshes/AstraPlus.stl"
CAMERA_COLOR = (0.12, 0.13, 0.16, 1.0)
BODY_COLOR = (0.62, 0.66, 0.74, 1.0)
DRIVE_WHEELS = ("left_wheel_link", "right_wheel_link")
CAMERA_GRID = 0.002     # m, detail kept on the camera


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin(el) -> np.ndarray:
    m = np.eye(4)
    if el is not None:
        m[:3, 3] = [float(v) for v in el.get("xyz", "0 0 0").split()]
        m[:3, :3] = rpy_matrix(*[float(v) for v in el.get("rpy", "0 0 0").split()])
    return m


def read_stl(path: Path) -> np.ndarray:
    """(n, 3, 3) triangles of a binary or text STL."""
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + 50 * n:
        v = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
        if not v or len(v) % 3:
            raise ValueError(f"{path} is not an STL")
        return np.array(v, dtype=np.float64).reshape(-1, 3, 3)
    rec = np.frombuffer(data, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), n, 84)
    return rec["v"].astype(np.float64)


def simplify(tris: np.ndarray, grid: float) -> np.ndarray:
    """Merge the corners that fall in one grid cell; drop what collapses."""
    keys = np.round(tris.reshape(-1, 3) / grid).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    idx = inv.reshape(-1, 3)
    good = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])
    idx = np.unique(np.sort(idx[good], axis=1), axis=0, return_index=True)[1]
    kept = inv.reshape(-1, 3)[good][np.sort(idx)]
    return (uniq[kept] * grid).astype(np.float64)


def mesh_path(uri: str) -> Path:
    pkg, rel = uri[len("package://"):].split("/", 1)
    return WS / "src" / pkg / rel


def main() -> None:
    xml = subprocess.run(["xacro", str(URDF), "use_mock_hardware:=true"], capture_output=True, text=True, check=True).stdout
    root = ET.fromstring(xml)
    parent = {}
    for j in root.findall("joint"):
        parent[j.find("child").get("link")] = (j.find("parent").get("link"), origin(j.find("origin")))

    def pose(link: str) -> np.ndarray:
        m = np.eye(4)
        while link in parent:
            link, t = parent[link]
            m = t @ m
        return m

    tris, cols = [], []
    for link in root.findall("link"):
        name = link.get("name")
        vis = link.find("visual")
        if vis is None:
            continue
        mesh = vis.find("geometry/mesh")
        rgba = vis.find("material/color")
        color = tuple(float(v) for v in rgba.get("rgba").split()) if rgba is not None else BODY_COLOR
        if name == "camera_link":
            raw = read_stl(CAMERA_MESH)
            if np.abs(raw).max() > 5:           # the vendor's model is in millimetres
                raw = raw * 0.001
            raw = simplify(raw, CAMERA_GRID)
            flat = raw.reshape(-1, 3)
            raw = raw - (flat.min(0) + flat.max(0)) / 2         # centred on camera_link
            color, scale, local = CAMERA_COLOR, 1.0, np.eye(4)
            local[:3, :3] = rpy_matrix(0, 0, np.pi / 2)         # long side across the robot
        elif mesh is not None:
            raw = read_stl(mesh_path(mesh.get("filename")))
            scale = float(mesh.get("scale", "1 1 1").split()[0])
            local = origin(vis.find("origin"))
        else:
            continue
        m = pose(name) @ local
        v = (raw.reshape(-1, 3) * scale) @ m[:3, :3].T + m[:3, 3]
        if name in DRIVE_WHEELS:        # centre the wheel on its axle
            v[:, 1] += pose(name)[1, 3] - (v[:, 1].min() + v[:, 1].max()) / 2
        tris.append(v.reshape(-1, 3, 3))
        cols.append(np.tile(np.array(color, np.float32), (len(raw), 1)))
        lo, hi = v.min(0), v.max(0)
        print(f"{name:16s} {len(raw):6d} tris  x {lo[0]:+.3f}..{hi[0]:+.3f}  y {lo[1]:+.3f}..{hi[1]:+.3f}  z {lo[2]:+.3f}..{hi[2]:+.3f}")
    t = np.concatenate(tris)
    t[:, :, 2] -= t[:, :, 2].min()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT, triangles=t.astype(np.float32), colors=np.concatenate(cols))
    lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
    print(f"all: {len(t)} triangles, size {hi - lo}, lo {lo}, hi {hi}, {OUT.stat().st_size // 1024} KB")


if __name__ == "__main__":
    main()
