"""Render every panel of the app to a PNG so the UI can be reviewed
without a display. Also feeds each panel representative data so the
visuals are realistic (map with obstacle, scan sweep, mission waypoints,
camera frame, point cloud).
"""
from __future__ import annotations

import base64
import io
import math
import os
import struct
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PIL import Image
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from rcs.core.mission import Mission, Waypoint
from rcs.ui import theme
from rcs.ui.main_window import MainWindow

OUT = HERE.parent / "docs" / "screenshots"
OUT.mkdir(parents=True, exist_ok=True)


def _fake_map():
    w, h = 60, 60
    data = [-1] * (w * h)
    # free space in the middle
    for y in range(5, h - 5):
        for x in range(5, w - 5):
            data[y * w + x] = 0
    # a horizontal wall and a square obstacle
    for x in range(10, 50):
        data[20 * w + x] = 100
    for y in range(30, 40):
        for x in range(30, 40):
            data[y * w + x] = 100
    return {
        "info": {"width": w, "height": h, "resolution": 0.1,
                 "origin": {"position": {"x": -3.0, "y": -3.0, "z": 0.0},
                            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
        "data": data,
    }


def _fake_scan():
    n = 180
    ranges = [1.5 + 0.5 * math.sin(math.radians(i * 2)) for i in range(n)]
    return {
        "angle_min": -math.pi / 2, "angle_max": math.pi / 2,
        "angle_increment": math.pi / n,
        "range_min": 0.1, "range_max": 10.0,
        "ranges": ranges,
    }


def _fake_path():
    poses = []
    for i in range(40):
        poses.append({"pose": {"position": {"x": -1.0 + i * 0.1,
                                            "y": 0.5 * math.sin(i * 0.15)}}})
    return {"poses": poses}


def _fake_jpeg(w=640, h=480):
    arr = np.zeros((h, w, 3), dtype=np.uint8)
    xx, yy = np.meshgrid(np.arange(w), np.arange(h))
    arr[..., 0] = (xx * 255 // w).astype(np.uint8)
    arr[..., 1] = (yy * 255 // h).astype(np.uint8)
    arr[..., 2] = 128
    # draw a crosshair + simulated "person"
    arr[h // 2 - 2 : h // 2 + 2, :] = (255, 255, 255)
    arr[:, w // 2 - 2 : w // 2 + 2] = (255, 255, 255)
    arr[160:380, 260:380] = (240, 160, 90)
    img = Image.fromarray(arr, "RGB")
    bio = io.BytesIO(); img.save(bio, format="JPEG", quality=72)
    return bio.getvalue()


def _fake_cloud(n=4000):
    rng = np.random.default_rng(42)
    pts = np.zeros((n, 3), dtype=np.float32)
    pts[:, 0] = rng.uniform(-2, 2, n)
    pts[:, 1] = rng.uniform(-2, 2, n)
    pts[:, 2] = 0.3 + 0.05 * np.sin(pts[:, 0] * 3) + 0.1 * rng.normal(size=n)
    step = 12
    buf = pts.tobytes()
    return {"width": n, "height": 1, "point_step": step,
            "fields": [{"name": "x", "offset": 0}, {"name": "y", "offset": 4},
                       {"name": "z", "offset": 8}],
            "data": base64.b64encode(buf).decode()}


def take(window, key, size=(1400, 880)):
    """Render a panel to PNG using QWidget.render() (works under offscreen Qt)."""
    from PySide6.QtGui import QImage, QPainter
    from PySide6.QtCore import QRect, QPoint
    window.btns[key].click()
    QApplication.processEvents()
    time.sleep(0.08)
    QApplication.processEvents()
    panel = window._pages[key]
    w, h = panel.width(), panel.height()
    if w <= 0 or h <= 0:
        w, h = size
        panel.resize(w, h)
        QApplication.processEvents()
    img = QImage(w, h, QImage.Format_ARGB32)
    img.fill(0xff0f1115)
    painter = QPainter(img)
    panel.render(painter, QPoint(), QRect(0, 0, w, h))
    painter.end()
    path = OUT / f"{key}.png"
    img.save(str(path))
    print(f"  wrote {path.relative_to(HERE.parent)}  ({path.stat().st_size // 1024} KB)")


def main() -> int:
    print("STEP: create app"); sys.stdout.flush()
    app = QApplication.instance() or QApplication(sys.argv)
    theme.apply(app)
    print("STEP: create MainWindow"); sys.stdout.flush()
    w = MainWindow()
    w.resize(1400, 880)
    w.show()
    QApplication.processEvents()
    print("STEP: window shown"); sys.stdout.flush()

    print("STEP: feed data"); sys.stdout.flush()
    # feed data into every panel
    map_msg = _fake_map()
    scan_msg = _fake_scan()
    path_msg = _fake_path()
    for idx, v in enumerate((w.p_map.view, w.p_nav.view, w.p_mission.view)):
        print(f"  view#{idx}: occupancy"); sys.stdout.flush()
        v.set_occupancy(map_msg)
        print(f"  view#{idx}: pose"); sys.stdout.flush()
        v.set_robot_pose(0.5, 0.0, 0.3)
        print(f"  view#{idx}: scan"); sys.stdout.flush()
        v.set_scan(scan_msg, 0.9, 0.0, 0.3)
        print(f"  view#{idx}: path"); sys.stdout.flush()
        v.set_path(path_msg)
        print(f"  view#{idx}: wps"); sys.stdout.flush()
        v.set_waypoints([(1.0, 1.0), (2.0, 0.5), (2.0, -1.0), (0.0, -1.5)])
    print("STEP: after view loop"); sys.stdout.flush()
    try:
        w.sb_pose.setText("pose: x=+0.50 y=+0.00 yaw=+0.30")
        print("STEP: sb_pose"); sys.stdout.flush()
        w.sb_conn.setText("● connected"); w.sb_conn.setStyleSheet("color:#39d98a;")
        w.sb_robot.setText("Hinson robot (ws://192.168.10.10:9090)")
        print("STEP: sb set"); sys.stdout.flush()

        # mission content
        m = Mission.new("Loop route — depot")
        m.loop = True
        m.waypoints = [
            Waypoint.new("depot", 0.0, 0.0),
            Waypoint.new("pick-A", 2.0, 0.5),
            Waypoint.new("pick-B", 2.0, -1.0),
            Waypoint.new("drop", -1.0, -1.5),
        ]
        for wp in m.waypoints:
            wp.tolerance = 0.25; wp.dwell = 1.5
        w.missions.upsert(m)
        print("STEP: mission upserted"); sys.stdout.flush()
        w.p_mission._reload_missions()
        w.p_mission._set_combo_to(m.id)
        print("STEP: mission reloaded"); sys.stdout.flush()

        # camera + logs (skip 3d — segfaults on offscreen when grabbing)
        w.p_cam.set_frame(_fake_jpeg())
        w.p_cam.set_rate(28.3)
        print("STEP: camera fed"); sys.stdout.flush()
        for lvl, msg in [(20, "RCS connected to robot"),
                         (20, "Loaded map 60x60 @ 0.10 m/px"),
                         (30, "Local costmap stale by 0.8 s"),
                         (20, "Nav2 goal accepted (1.50, 0.80)"),
                         (40, "Lost TF for 1.3 s"),
                         (20, "Mission 'depot' loop started")]:
            w.p_logs.append_log({"level": lvl, "name": "rcs", "msg": msg})
        print("STEP: logs appended"); sys.stdout.flush()
        w.p_nav.set_status("active")
        w.p_nav.set_feedback({"distance_remaining": 2.73,
                              "estimated_time_remaining": {"sec": 14.5}})
        print("STEP: nav status set"); sys.stdout.flush()
    except Exception as e:
        import traceback
        traceback.print_exc(); sys.stdout.flush()
        raise

    QApplication.processEvents()
    print("STEP: pre-take loop"); sys.stdout.flush()
    for k in ("conn", "status", "teleop", "map", "nav", "mission", "cam", "logs"):
        print(f"-> about to take {k}"); sys.stdout.flush()
        try:
            take(w, k)
            print(f"<- done take {k}"); sys.stdout.flush()
        except Exception as e:
            import traceback; traceback.print_exc(); sys.stdout.flush()
    print("STEP: all takes done"); sys.stdout.flush()

    print(f"\nAll screenshots → {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
