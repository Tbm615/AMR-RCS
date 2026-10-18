"""Minimal TF cache for the operator app. Robots we drive are planar, so we
store transforms as 2D poses (x, y, yaw) in the parent frame and compose
with a 2 x 2 rotation + translation."""
from __future__ import annotations

import math
import time
from typing import Dict, Optional, Tuple


Pose2 = Tuple[float, float, float]   # x, y, yaw (in parent frame)


def _quat_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2 * (qw * qz + qx * qy)
    cosy = 1 - 2 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _compose(parent_in_root: Pose2, child_in_parent: Pose2) -> Pose2:
    """child_in_root = parent_in_root ⊗ child_in_parent, SE(2) composition."""
    px, py, pt = parent_in_root
    cx, cy, ct = child_in_parent
    cos_p, sin_p = math.cos(pt), math.sin(pt)
    t = pt + ct
    return (
        px + cx * cos_p - cy * sin_p,
        py + cx * sin_p + cy * cos_p,
        math.atan2(math.sin(t), math.cos(t)),     # keep yaw in [-pi, pi]
    )


class TFCache:
    def __init__(self) -> None:
        # edge key: (parent, child) -> (child_in_parent_pose, last_timestamp)
        self._edges: Dict[Tuple[str, str], Tuple[Pose2, float]] = {}
        self._parent_of: Dict[str, str] = {}   # child -> parent

    def ingest(self, tf_msg: dict) -> None:
        now = time.monotonic()
        for tr in tf_msg.get("transforms", []):
            parent = tr["header"]["frame_id"].lstrip("/")
            child = tr["child_frame_id"].lstrip("/")
            t = tr["transform"]["translation"]
            r = tr["transform"]["rotation"]
            pose: Pose2 = (
                float(t.get("x", 0.0)),
                float(t.get("y", 0.0)),
                _quat_yaw(
                    float(r.get("x", 0.0)),
                    float(r.get("y", 0.0)),
                    float(r.get("z", 0.0)),
                    float(r.get("w", 1.0)),
                ),
            )
            self._edges[(parent, child)] = (pose, now)
            self._parent_of[child] = parent

    def pose_xyyaw(self, target: str, source: str) -> Optional[Pose2]:
        """Return the pose of `source` expressed in `target`'s frame."""
        target = target.lstrip("/")
        source = source.lstrip("/")
        if target == source:
            return (0.0, 0.0, 0.0)

        # Walk source up its ancestor chain to find target.
        chain = [source]
        cur = source
        while cur in self._parent_of:
            cur = self._parent_of[cur]
            chain.append(cur)
            if cur == target:
                break
        if not chain or chain[-1] != target:
            return None

        # chain = [source, ..., target]. We need source expressed in target.
        # Compose from target down to source: start with identity in target,
        # then for each step parent->child in the chain reversed, compose.
        pose: Pose2 = (0.0, 0.0, 0.0)
        for i in range(len(chain) - 1, 0, -1):
            parent = chain[i]
            child = chain[i - 1]
            edge, _ = self._edges[(parent, child)]
            pose = _compose(pose, edge)
        return pose
