"""Minimal TF cache for the operator app.

Every edge is stored as a full 3D transform (translation + quaternion) so
camera point clouds, whose optical frames are rolled and pitched relative to
the robot, land in the map correctly. The 2D helpers (pose_xyyaw) project
the composed transform onto the ground plane for the map views.

ingest() runs on the network thread and the readers on the GUI and decoder
threads, so the edge tables are guarded by a lock.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np


Pose2 = Tuple[float, float, float]   # x, y, yaw (in parent frame)


def _quat_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2 * (qw * qz + qx * qy)
    cosy = 1 - 2 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def quat_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


# edge: (translation xyz, quaternion xyzw)
Edge = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]


def stamp_sec(header: dict) -> float:
    st = (header or {}).get("stamp") or {}
    return float(st.get("sec", st.get("secs", 0))) + 1e-9 * float(st.get("nanosec", st.get("nsecs", 0)))


class TFCache:
    HISTORY_S = 3.0      # how far back stamped lookups (camera frames) can reach

    def __init__(self) -> None:
        # (parent, child) -> (edge, last_timestamp)
        self._edges: Dict[Tuple[str, str], Tuple[Edge, float]] = {}
        # (parent, child) -> recent (stamp, edge), for lookups at a message's stamp
        self._hist: Dict[Tuple[str, str], Deque[Tuple[float, Edge]]] = {}
        self._parent_of: Dict[str, str] = {}   # child -> parent
        self._lock = threading.Lock()

    def ingest(self, tf_msg: dict) -> None:
        now = time.monotonic()
        with self._lock:
            for tr in tf_msg.get("transforms", []):
                try:
                    parent = tr["header"]["frame_id"].lstrip("/")
                    child = tr["child_frame_id"].lstrip("/")
                    t = tr["transform"]["translation"]
                    r = tr["transform"]["rotation"]
                    edge: Edge = (
                        (float(t.get("x", 0.0)), float(t.get("y", 0.0)), float(t.get("z", 0.0))),
                        (float(r.get("x", 0.0)), float(r.get("y", 0.0)),
                         float(r.get("z", 0.0)), float(r.get("w", 1.0))),
                    )
                    stamp = stamp_sec(tr.get("header"))
                except (KeyError, TypeError, ValueError, AttributeError):
                    continue
                key = (parent, child)
                self._edges[key] = (edge, now)
                self._parent_of[child] = parent
                if stamp > 0:
                    h = self._hist.get(key)
                    if h is None:
                        h = self._hist[key] = deque()
                    if h and stamp < h[-1][0]:       # clock jumped back (sim restart)
                        h.clear()
                    h.append((stamp, edge))
                    while h and stamp - h[0][0] > self.HISTORY_S:
                        h.popleft()

    def _edge_at(self, key: Tuple[str, str], stamp: Optional[float]) -> Edge:
        if stamp:
            h = self._hist.get(key)
            if h and len(h) > 1:
                # nearest sample; /tf runs at 20-100 Hz so that is within ~25 ms
                best = min(h, key=lambda se: abs(se[0] - stamp))
                return best[1]
        return self._edges[key][0]

    def _chain(self, target: str, source: str, stamp: Optional[float] = None) -> Optional[List[Edge]]:
        """Edges from target down to source, or None if source is not below target."""
        chain = [source]
        cur = source
        while cur in self._parent_of and cur != target:
            cur = self._parent_of[cur]
            chain.append(cur)
            if len(chain) > 64:          # a loop in a broken TF tree
                return None
        if chain[-1] != target:
            return None
        return [self._edge_at((chain[i], chain[i - 1]), stamp) for i in range(len(chain) - 1, 0, -1)]

    def matrix(self, target: str, source: str, stamp: Optional[float] = None) -> Optional[np.ndarray]:
        """4x4 homogeneous transform taking points in `source` into `target`,
        at time `stamp` (seconds, robot clock) if given, else the latest."""
        target = target.lstrip("/")
        source = source.lstrip("/")
        if target == source:
            return np.eye(4)
        with self._lock:
            edges = self._chain(target, source, stamp)
        if edges is None:
            return None
        m = np.eye(4)
        for (t, q) in edges:
            e = np.eye(4)
            e[:3, :3] = quat_matrix(*q)
            e[:3, 3] = t
            m = m @ e
        return m

    def pose_xyyaw(self, target: str, source: str) -> Optional[Pose2]:
        """Return the pose of `source` expressed in `target`'s frame, projected
        onto the ground plane."""
        target = target.lstrip("/")
        source = source.lstrip("/")
        if target == source:
            return (0.0, 0.0, 0.0)
        with self._lock:
            edges = self._chain(target, source)
        if edges is None:
            return None
        # Planar robots: compose in SE(2) (cheap, called many times a second).
        px, py, pt = 0.0, 0.0, 0.0
        for (t, q) in edges:
            c, s = math.cos(pt), math.sin(pt)
            px, py = px + t[0] * c - t[1] * s, py + t[0] * s + t[1] * c
            pt += _quat_yaw(*q)
        return (px, py, math.atan2(math.sin(pt), math.cos(pt)))
