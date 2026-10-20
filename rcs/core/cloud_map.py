"""Colored 3D map built on the operator PC.

Each camera frame (an RGB-D image pair, or a PointCloud2) is moved into the
map frame with the full 3D TF and merged into a voxel grid: one colored point
per voxel, first observation wins. This is the live counterpart of the
cloud3d.ply that bringup's save_run exports from RTAB-Map, and it saves and
loads the same PLY format.

The decode/merge functions run on the RosClient decoder thread; the GUI only
reads snapshots. Everything here is numpy, no Qt.
"""
from __future__ import annotations

import base64
import io
import threading
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

_OFF = 1 << 20          # voxel index offset: ±2^20 voxels per axis
_MASK = (1 << 21) - 1


def _b64bytes(data) -> bytes:
    if isinstance(data, str):
        return base64.b64decode(data)
    if isinstance(data, list):           # rosbridge without base64 (old versions)
        return bytes(bytearray(data))
    return bytes(data or b"")


def transform(pts: np.ndarray, m: np.ndarray) -> np.ndarray:
    return (pts @ m[:3, :3].T.astype(np.float32)) + m[:3, 3].astype(np.float32)


# ---------------------------------------------------------------- decoding

_PC2_TYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def decode_pointcloud2(msg: dict) -> Optional[Tuple[np.ndarray, Optional[np.ndarray]]]:
    """sensor_msgs/PointCloud2 -> (xyz float32 Nx3, rgb uint8 Nx3 or None).
    Non-finite points are dropped."""
    try:
        n = int(msg["width"]) * int(msg["height"])
        step = int(msg["point_step"])
        fields = {f["name"]: f for f in msg["fields"]}
    except (KeyError, TypeError, ValueError):
        return None
    if n == 0:
        return np.empty((0, 3), np.float32), None
    if not all(k in fields for k in ("x", "y", "z")):
        return None
    data = _b64bytes(msg.get("data", b""))
    if len(data) < n * step:
        return None
    big = bool(msg.get("is_bigendian", False))
    arr = np.frombuffer(data, dtype=np.uint8, count=n * step).reshape(n, step)

    def col(name: str) -> np.ndarray:
        f = fields[name]
        t = np.dtype(_PC2_TYPES.get(int(f.get("datatype", 7)), "f4")).newbyteorder(">" if big else "<")
        off = int(f["offset"])
        return arr[:, off: off + t.itemsize].copy().view(t).reshape(-1)

    xyz = np.stack([col("x"), col("y"), col("z")], axis=1).astype(np.float32)
    rgb = None
    key = "rgb" if "rgb" in fields else ("rgba" if "rgba" in fields else None)
    if key:
        off = int(fields[key]["offset"])
        packed = arr[:, off: off + 4].copy().view("<u4" if not big else ">u4").reshape(-1)
        rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], axis=1).astype(np.uint8)
    ok = np.isfinite(xyz).all(axis=1)
    return xyz[ok], (rgb[ok] if rgb is not None else None)


def decode_compressed_depth(msg: dict) -> Optional[np.ndarray]:
    """compressed_depth_image_transport message -> depth in metres (float32 HxW).
    Format string looks like "16UC1; compressedDepth png" (or "32FC1; ...")."""
    from PIL import Image
    raw = _b64bytes(msg.get("data"))
    fmt = str(msg.get("format", ""))
    if len(raw) < 12:
        return None
    # 12-byte ConfigHeader (int32 format + float depthQuantA/B), then the PNG.
    png_at = raw.find(b"\x89PNG")
    if png_at < 0:
        return None
    hdr, png = raw[:png_at], raw[png_at:]
    try:
        img = Image.open(io.BytesIO(png))
        img.load()
    except Exception:  # noqa: BLE001
        return None
    a = np.asarray(img)
    if a.dtype != np.uint16:
        a = a.astype(np.uint16)
    if fmt.startswith("32FC1"):
        qa, qb = np.frombuffer(hdr[4:12], "<f4") if len(hdr) >= 12 else (0.0, 0.0)
        d = np.zeros(a.shape, np.float32)
        nz = a > 0
        d[nz] = qa / (a[nz].astype(np.float32) - qb)
        return d
    return a.astype(np.float32) * 0.001        # 16UC1 is millimetres


def decode_raw_depth(msg: dict) -> Optional[np.ndarray]:
    """sensor_msgs/Image 16UC1 (mm) or 32FC1 (m) -> metres."""
    try:
        h, w = int(msg["height"]), int(msg["width"])
        enc = msg.get("encoding", "16UC1")
    except (KeyError, TypeError, ValueError):
        return None
    raw = _b64bytes(msg.get("data"))
    if enc in ("16UC1", "mono16"):
        if len(raw) < h * w * 2:
            return None
        return np.frombuffer(raw, "<u2", count=h * w).reshape(h, w).astype(np.float32) * 0.001
    if enc == "32FC1":
        if len(raw) < h * w * 4:
            return None
        return np.frombuffer(raw, "<f4", count=h * w).reshape(h, w).copy()
    return None


def decode_jpeg_rgb(data: bytes) -> Optional[np.ndarray]:
    from PIL import Image
    try:
        img = Image.open(io.BytesIO(data))
        return np.asarray(img.convert("RGB"))
    except Exception:  # noqa: BLE001
        return None


def project_depth(depth: np.ndarray, K, color: Optional[np.ndarray] = None,
                  step: int = 4, max_range: float = 4.0, min_range: float = 0.2
                  ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Back-project a depth image (metres) with pinhole intrinsics K (row-major
    3x3 as a flat list of 9). Returns points in the camera optical frame
    (x right, y down, z forward) and, if `color` is given, their RGB."""
    fx, cx, fy, cy = float(K[0]), float(K[2]), float(K[4]), float(K[5])
    if fx <= 0 or fy <= 0:
        return np.empty((0, 3), np.float32), None
    h, w = depth.shape
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = depth[vs, us]
    ok = np.isfinite(z) & (z > min_range) & (z < max_range)
    z, us, vs = z[ok], us[ok], vs[ok]
    pts = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1).astype(np.float32)
    rgb = None
    if color is not None and color.ndim == 3:
        ch, cw = color.shape[:2]
        cu = np.clip((us * cw) // w, 0, cw - 1)
        cv = np.clip((vs * ch) // h, 0, ch - 1)
        rgb = color[cv, cu, :3].astype(np.uint8)
    return pts, rgb


# ---------------------------------------------------------------- PLY

_PLY_TYPES = {"float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
              "uchar": "u1", "uint8": "u1", "char": "i1", "int8": "i1",
              "ushort": "u2", "uint16": "u2", "short": "i2", "int16": "i2",
              "uint": "u4", "uint32": "u4", "int": "i4", "int32": "i4"}


def read_ply(path) -> Tuple[np.ndarray, np.ndarray]:
    """Vertices of a PLY (binary little-endian or ASCII) -> (xyz, rgb)."""
    with open(path, "rb") as f:
        if f.readline().strip() != b"ply":
            raise ValueError("not a PLY file")
        props, n, fmt, in_vertex = [], 0, "binary_little_endian", False
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PLY header has no end_header")
            parts = line.decode("ascii", "replace").split()
            if not parts:
                continue
            if parts[0] == "format":
                fmt = parts[1]
            elif parts[0] == "element":
                in_vertex = parts[1] == "vertex"
                if in_vertex:
                    n = int(parts[2])
            elif parts[0] == "property" and in_vertex:
                if parts[1] == "list":
                    raise ValueError("PLY vertex lists are not supported")
                props.append((parts[2], _PLY_TYPES[parts[1]]))
            elif parts[0] == "end_header":
                break
        if fmt == "ascii":
            v = np.loadtxt(f, max_rows=n, ndmin=2)
            names = [p[0] for p in props]
            col = {nm: v[:, i] for i, nm in enumerate(names)}
        else:
            order = "<" if fmt == "binary_little_endian" else ">"
            dt = np.dtype([(nm, order + t) for nm, t in props])
            v = np.frombuffer(f.read(dt.itemsize * n), dtype=dt, count=n)
            col = {nm: v[nm] for nm in dt.names}
    xyz = np.column_stack([col["x"], col["y"], col["z"]]).astype(np.float32)
    if "red" in col:
        rgb = np.column_stack([col["red"], col["green"], col["blue"]]).astype(np.uint8)
    else:
        rgb = np.full((len(xyz), 3), 200, np.uint8)
    ok = np.isfinite(xyz).all(axis=1)
    return xyz[ok], rgb[ok]


def write_ply(path, xyz: np.ndarray, rgb: np.ndarray) -> None:
    n = len(xyz)
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                   ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    v = np.empty(n, dt)
    v["x"], v["y"], v["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    v["red"], v["green"], v["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    hdr = ("ply\nformat binary_little_endian 1.0\ncomment RCS 3D map\n"
           f"element vertex {n}\nproperty float x\nproperty float y\nproperty float z\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    tmp = Path(str(path) + ".part")
    with open(tmp, "wb") as f:
        f.write(hdr.encode("ascii"))
        f.write(v.tobytes())
    tmp.replace(path)


# ---------------------------------------------------------------- the map

class CloudMap:
    """Voxel-hashed colored point map. Thread-safe."""

    def __init__(self, voxel: float = 0.04, max_points: int = 2_000_000):
        self._lock = threading.Lock()
        self.voxel = voxel
        self.max_points = max_points
        self.version = 0            # bumps on every change; the GUI redraws on change
        self.full = False
        self._clear()

    def _clear(self) -> None:
        self._keys = np.empty(0, np.int64)       # sorted
        self._xyz = np.empty((0, 3), np.float32)
        self._rgb = np.empty((0, 3), np.uint8)

    def __len__(self) -> int:
        return len(self._keys)

    def clear(self) -> None:
        with self._lock:
            self._clear()
            self.full = False
            self.version += 1

    def set_voxel(self, voxel: float) -> None:
        """Change the voxel size; existing points are re-binned."""
        with self._lock:
            xyz, rgb = self._xyz, self._rgb
            self.voxel = voxel
            self._clear()
            self.full = False
            self._insert(xyz, rgb)
            self.version += 1

    def _pack(self, xyz: np.ndarray) -> np.ndarray:
        idx = np.floor(xyz / self.voxel).astype(np.int64) + _OFF
        idx &= _MASK
        return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]

    def _insert(self, xyz: np.ndarray, rgb: np.ndarray) -> int:
        if len(xyz) == 0:
            return 0
        keys = self._pack(xyz)
        keys, first = np.unique(keys, return_index=True)     # one point per voxel in this batch
        if len(self._keys):
            pos = np.searchsorted(self._keys, keys)
            hit = np.zeros(len(keys), bool)
            inb = pos < len(self._keys)
            hit[inb] = self._keys[pos[inb]] == keys[inb]
            new = ~hit
        else:
            pos = np.zeros(len(keys), np.int64)
            new = np.ones(len(keys), bool)
        room = self.max_points - len(self._keys)
        if room <= 0:
            self.full = True
            return 0
        sel = np.flatnonzero(new)
        if len(sel) > room:
            sel = sel[:room]
            self.full = True
        if len(sel) == 0:
            return 0
        src = first[sel]
        self._keys = np.insert(self._keys, pos[sel], keys[sel])
        self._xyz = np.insert(self._xyz, pos[sel], xyz[src], axis=0)
        self._rgb = np.insert(self._rgb, pos[sel], rgb[src], axis=0)
        return len(sel)

    def integrate(self, xyz: np.ndarray, rgb: Optional[np.ndarray]) -> int:
        """Add points already in the map frame. Returns how many voxels were new."""
        if rgb is None:
            rgb = height_colors(xyz)
        with self._lock:
            added = self._insert(xyz.astype(np.float32, copy=False), rgb)
            if added:
                self.version += 1
            return added

    def replace(self, xyz: np.ndarray, rgb: Optional[np.ndarray]) -> None:
        """Swap the whole map (a cloud that already is a map, or a loaded PLY)."""
        if rgb is None:
            rgb = height_colors(xyz)
        with self._lock:
            self._clear()
            self.full = False
            self._insert(xyz.astype(np.float32, copy=False), rgb)
            self.version += 1

    def snapshot(self) -> Tuple[np.ndarray, np.ndarray, int]:
        with self._lock:
            return self._xyz, self._rgb, self.version

    def save_ply(self, path) -> int:
        xyz, rgb, _ = self.snapshot()
        write_ply(path, xyz, rgb)
        return len(xyz)

    def load_ply(self, path) -> int:
        xyz, rgb = read_ply(path)
        self.replace(xyz, rgb)
        return len(self)


def height_colors(xyz: np.ndarray) -> np.ndarray:
    """Colour points by height (blue low → yellow high) when the source has no RGB."""
    if len(xyz) == 0:
        return np.empty((0, 3), np.uint8)
    t = np.clip((xyz[:, 2] + 0.1) / 2.0, 0.0, 1.0)
    r = (40 + 215 * t).astype(np.uint8)
    g = (90 + 140 * t).astype(np.uint8)
    b = (230 - 190 * t).astype(np.uint8)
    return np.stack([r, g, b], axis=1)
