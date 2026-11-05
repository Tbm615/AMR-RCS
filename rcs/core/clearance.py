"""Free distance around the robot from one lidar scan, shown on the Teleop page.

Same reduction and thresholds as the web console's LiDAR brake
(amr_control_center/safety.py), but here it is only displayed: nothing in
the app limits the speed with it.
"""
from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

STOP_DISTANCE = 0.25        # m from the footprint edge: red
SLOW_DISTANCE = 0.80        # m: amber
SIDE_MARGIN = 0.05          # m added to the body width for the front/rear corridor


class Clearance(NamedTuple):
    front: float            # free distance ahead of the front edge, in the body corridor
    rear: float             # same behind the rear edge
    nearest: float          # closest point to the footprint, any direction


def compute_clearance(lx: np.ndarray, ly: np.ndarray, laser_pose, half_l: float, half_w: float) -> Clearance:
    """lx, ly: scan points in the laser frame. laser_pose: (x, y, yaw) of the
    laser in base_link. Distances are math.inf when nothing is in range."""
    if not len(lx):
        return Clearance(math.inf, math.inf, math.inf)
    tx, ty, yaw = laser_pose
    c, s = math.cos(yaw), math.sin(yaw)
    x = lx * c - ly * s + tx
    y = lx * s + ly * c + ty

    dx = np.abs(x) - half_l
    dy = np.abs(y) - half_w
    # distance to the rectangle: 0 inside, straight to an edge, or to a corner
    gap = np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0))

    corridor = np.abs(y) <= half_w + SIDE_MARGIN
    ahead = x[corridor & (x > half_l)]
    behind = x[corridor & (x < -half_l)]
    return Clearance(
        float(ahead.min() - half_l) if len(ahead) else math.inf,
        float(-half_l - behind.max()) if len(behind) else math.inf,
        float(gap.min()),
    )
