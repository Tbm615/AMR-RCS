"""A synthetic RGB-D camera for the mock rosbridge.

Renders what an Orbbec-like camera mounted on the robot would see inside a
box-shaped room: depth (compressedDepth PNG, millimetres, like
compressed_depth_image_transport) and color (JPEG), with the same frames the
real driver uses (camera_link -> camera_color_optical_frame, registered depth).
Each wall has its own color so a wrong yaw in the 3D map is easy to spot.
"""
from __future__ import annotations

import base64
import io
import math
import struct

import numpy as np
from PIL import Image

W, H = 320, 240
FX = FY = 280.0 * W / 320
CX, CY = W / 2 - 0.5, H / 2 - 0.5
CAM_XYZ = (0.35, 0.0, 0.30)            # camera_link in base_link
# camera_link -> optical: z forward, x right, y down
OPT_Q = (-0.5, 0.5, -0.5, 0.5)
OPT_FRAME = "camera_color_optical_frame"

# room: x in [-4, 4], y in [-3, 3], walls up to 2.5 m
ROOM = {"x0": -4.0, "x1": 4.0, "y0": -3.0, "y1": 3.0, "zt": 2.5}
WALL_RGB = {"x1": (220, 60, 60), "x0": (60, 200, 90), "y1": (70, 110, 230), "y0": (230, 200, 60),
            "floor": (120, 120, 130), "ceil": (90, 90, 100)}


def _q_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def static_tf_msg(stamp: dict) -> dict:
    def tr(parent, child, t, q):
        return {"header": {"stamp": stamp, "frame_id": parent}, "child_frame_id": child,
                "transform": {"translation": dict(zip("xyz", t)), "rotation": dict(zip("xyzw", q))}}
    return {"transforms": [tr("base_link", "camera_link", CAM_XYZ, (0, 0, 0, 1)),
                           tr("camera_link", OPT_FRAME, (0, 0, 0), OPT_Q)]}


def render(x: float, y: float, yaw: float):
    """Depth (m, HxW float32) and color (HxWx3 uint8) seen from robot pose (x, y, yaw)."""
    v, u = np.mgrid[0:H, 0:W].astype(np.float64)
    d_opt = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u)], axis=-1)   # z = 1
    r_base = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    R = r_base @ _q_mat(OPT_Q)
    d = d_opt @ R.T                                                     # ray dirs in map
    o = np.array([x, y, 0.0]) + r_base @ np.array(CAM_XYZ)
    t = np.full((H, W), np.inf)
    which = np.full((H, W), "", dtype=object)
    planes = [("x1", 0, ROOM["x1"]), ("x0", 0, ROOM["x0"]), ("y1", 1, ROOM["y1"]), ("y0", 1, ROOM["y0"]),
              ("floor", 2, 0.0), ("ceil", 2, ROOM["zt"])]
    with np.errstate(divide="ignore", invalid="ignore"):
        for name, ax, val in planes:
            tt = (val - o[ax]) / d[..., ax]
            ok = (tt > 0) & (tt < t)
            t = np.where(ok, tt, t)
            which = np.where(ok, name, which)
    depth = t.astype(np.float32)                    # ray z-component is 1 => t is the optical depth
    hit = o + d * t[..., None]
    color = np.zeros((H, W, 3), np.uint8)
    for name, rgb in WALL_RGB.items():
        m = which == name
        color[m] = rgb
    # checker pattern (0.5 m) so the color image has texture
    chk = ((np.floor(hit[..., 0] * 2) + np.floor(hit[..., 1] * 2) + np.floor(hit[..., 2] * 2)) % 2) == 0
    color[chk] = (color[chk] * 0.7).astype(np.uint8)
    depth[~np.isfinite(depth)] = 0
    depth[depth > 8.0] = 0
    return depth, color


def compressed_depth_msg(depth: np.ndarray, stamp: dict) -> dict:
    mm = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
    bio = io.BytesIO()
    Image.fromarray(mm).save(bio, format="PNG")
    payload = struct.pack("<iff", 0, 0.0, 0.0) + bio.getvalue()
    return {"header": {"stamp": stamp, "frame_id": OPT_FRAME},
            "format": "16UC1; compressedDepth png", "data": base64.b64encode(payload).decode()}


def jpeg_msg(color: np.ndarray, stamp: dict) -> dict:
    bio = io.BytesIO()
    Image.fromarray(color).save(bio, format="JPEG", quality=70)
    return {"header": {"stamp": stamp, "frame_id": OPT_FRAME}, "format": "jpeg",
            "data": base64.b64encode(bio.getvalue()).decode()}


def camera_info_msg(stamp: dict) -> dict:
    return {"header": {"stamp": stamp, "frame_id": OPT_FRAME}, "width": W, "height": H,
            "distortion_model": "plumb_bob", "d": [0.0] * 5,
            "k": [FX, 0, CX, 0, FY, CY, 0, 0, 1],
            "r": [1, 0, 0, 0, 1, 0, 0, 0, 1], "p": [FX, 0, CX, 0, 0, FY, CY, 0, 0, 0, 1, 0]}
