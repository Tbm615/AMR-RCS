"""Feature-level unit tests that don't need the full event loop.

Covers the pieces the main E2E test can't easily exercise:
  - MapView.set_occupancy / set_scan / set_path render into data items
  - View3DPanel._decode_pointcloud2 round-trips a hand-built cloud
  - CameraPanel decodes a real JPEG into a QPixmap
  - Mission executor advances waypoints and loops
  - TFCache composes parent/child transforms correctly
  - Occupancy LUT boundaries behave as designed

Run:
    QT_QPA_PLATFORM=offscreen python tests/test_features.py
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

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication
_app = QApplication.instance() or QApplication(sys.argv)

from PIL import Image

from rcs.core.mission import Mission, MissionStore, Waypoint
from rcs.core.tf_cache import TFCache
from rcs.ui.panels.camera import CameraPanel
from rcs.ui.panels.mission import MissionPanel
from rcs.ui.panels.view3d import View3DPanel, _decode_pointcloud2
from rcs.ui.widgets.map_view import MapView, _LUT


# ---- helpers ----

PASS = 0
FAIL = 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  OK   {name}")
    else:
        FAIL += 1
        print(f"  FAIL {name}  {detail}")


# ---- 1. MapView rendering ----

def test_map_view() -> None:
    print("\n[MapView]")
    v = MapView()
    v.set_robot_footprint(0.45, 0.30)

    # a 10x10 grid with a horizontal wall at row 5
    data = [0] * 100
    for x in range(10):
        data[5 * 10 + x] = 100
    data[0] = -1   # unknown
    grid = {
        "info": {"width": 10, "height": 10, "resolution": 0.1,
                 "origin": {"position": {"x": -0.5, "y": -0.5, "z": 0.0},
                            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
        "data": data,
    }
    v.set_occupancy(grid)
    img = v.map_img.image
    check("map image shape 10x10", img is not None and img.shape == (10, 10))
    check("occupancy LUT unknown is gray", tuple(_LUT[255]) == (90, 95, 110, 255))
    check("occupancy LUT free is lightest", _LUT[0][0] > _LUT[100][0])
    # pyqtgraph.ImageItem stores the rect as a QTransform; use sceneBoundingRect
    # or mapRectToParent to recover the world placement.
    r = v.map_img.mapRectToParent(v.map_img.boundingRect())
    check("map rect correctly placed",
          abs(r.x() + 0.5) < 1e-4 and abs(r.y() + 0.5) < 1e-4
          and abs(r.width() - 1.0) < 1e-4 and abs(r.height() - 1.0) < 1e-4,
          detail=f"got x={r.x()} y={r.y()} w={r.width()} h={r.height()}")

    # scan: 4 beams pointing N/E/S/W at 1 m, laser at origin, yaw 0
    scan = {
        "angle_min": 0.0, "angle_max": 3 * math.pi / 2,
        "angle_increment": math.pi / 2,
        "range_min": 0.0, "range_max": 10.0,
        "ranges": [1.0, 1.0, 1.0, 1.0],
    }
    v.set_scan(scan, map_tx=0.0, map_ty=0.0, map_yaw=0.0)
    pts = v.scan_scatter.getData()
    check("scan: 4 points", len(pts[0]) == 4)
    xs = sorted(pts[0]); ys = sorted(pts[1])
    check("scan xs cover ±1 m", abs(xs[0] + 1) < 1e-5 and abs(xs[-1] - 1) < 1e-5)
    check("scan ys cover ±1 m", abs(ys[0] + 1) < 1e-5 and abs(ys[-1] - 1) < 1e-5)

    # robot pose updates body polygon
    v.set_robot_pose(2.0, 1.0, math.pi / 2)
    bx, by = v.robot_body.getData()
    check("robot polygon has 5 pts (closed)", len(bx) == 5)
    check("robot polygon centered near (2,1)",
          abs(np.mean(bx[:4]) - 2) < 1e-4 and abs(np.mean(by[:4]) - 1) < 1e-4)

    # path
    path = {"poses": [{"pose": {"position": {"x": i * 0.1, "y": i * 0.2}}} for i in range(5)]}
    v.set_path(path)
    pxs, pys = v.path_line.getData()
    check("path has 5 points", len(pxs) == 5)
    check("path monotonic", (np.diff(pxs) > 0).all() and (np.diff(pys) > 0).all())

    # waypoints
    v.set_waypoints([(0, 0), (1, 2), (3, -1)])
    wx, wy = v.wp_scatter.getData()
    check("waypoints have 3 points", len(wx) == 3)


# ---- 1b. goal arrow: press-drag-release like RViz ----

def test_goal_arrow() -> None:
    print("\n[Goal arrow]")
    from PySide6.QtCore import QPointF
    from PySide6.QtTest import QTest
    v = MapView()
    v.resize(600, 600)
    v.show()
    v.getPlotItem().vb.setRange(xRange=(-5, 5), yRange=(-5, 5), padding=0)
    for _ in range(3):
        QApplication.processEvents()
    vb = v.getPlotItem().vb

    def px(x, y):
        sp = vb.mapViewToScene(QPointF(x, y))
        return v.mapFromScene(sp)

    got = []
    v.goal_requested.connect(lambda x, y, yaw: got.append((x, y, yaw)))
    a, b = px(1.0, 1.0), px(1.0, 3.0)            # drag straight "up" (+y)
    QTest.mousePress(v.viewport(), Qt.LeftButton, Qt.NoModifier, a)
    for k in range(1, 6):
        QTest.mouseMove(v.viewport(), a + (b - a) * (k / 5))
    check("arrow shown while dragging", v.aim_arrow.isVisible())
    QTest.mouseRelease(v.viewport(), Qt.LeftButton, Qt.NoModifier, b)
    check("drag sends one goal", len(got) == 1, f"{got}")
    if got:
        x, y, yaw = got[0]
        check("goal at the press point", abs(x - 1.0) < 0.1 and abs(y - 1.0) < 0.1, f"{got[0]}")
        check("goal heading = drag direction (+90°)", abs(yaw - math.pi / 2) < 0.05, f"{yaw:.3f}")
    check("goal drawn as an arrow", v.goal_arrow.isVisible() and v.goal_arrow.pose is not None)
    check("aim arrow hidden after release", not v.aim_arrow.isVisible())

    # click, then click again in the direction (-x)
    got.clear()
    c, d = px(-1.0, 0.0), px(-3.0, 0.0)
    QTest.mouseClick(v.viewport(), Qt.LeftButton, Qt.NoModifier, c)
    check("plain click does not send yet", not got)
    QTest.mouseMove(v.viewport(), d)
    QTest.mouseClick(v.viewport(), Qt.LeftButton, Qt.NoModifier, d)
    check("second click sends", len(got) == 1, f"{got}")
    if got:
        check("click-click heading (180°)", abs(abs(got[0][2]) - math.pi) < 0.05, f"{got[0][2]:.3f}")

    # right-click cancels
    got.clear()
    QTest.mouseClick(v.viewport(), Qt.LeftButton, Qt.NoModifier, c)
    QTest.mouseClick(v.viewport(), Qt.RightButton, Qt.NoModifier, d)
    QTest.mouseClick(v.viewport(), Qt.LeftButton, Qt.NoModifier, d)   # starts a new aim, no goal
    check("right-click cancels", not got)
    v.cancel_aim()

    # initial pose and waypoint modes use the same arrow
    poses, wps = [], []
    v.pose_requested.connect(lambda *a: poses.append(a))
    v.waypoint_requested.connect(lambda *a: wps.append(a))
    v.set_click_mode("pose")
    QTest.mousePress(v.viewport(), Qt.LeftButton, Qt.NoModifier, a)
    QTest.mouseMove(v.viewport(), px(3.0, 1.0))
    QTest.mouseRelease(v.viewport(), Qt.LeftButton, Qt.NoModifier, px(3.0, 1.0))
    check("initial pose with heading", len(poses) == 1 and abs(poses[0][2]) < 0.05, f"{poses}")
    v.set_click_mode("waypoint")
    QApplication.processEvents()
    a = px(1.0, 1.0)                   # recompute: the aspect lock may have re-laid out the view
    QTest.mousePress(v.viewport(), Qt.LeftButton, Qt.NoModifier, a)
    QTest.mouseMove(v.viewport(), px(1.0, -1.0))
    QTest.mouseRelease(v.viewport(), Qt.LeftButton, Qt.NoModifier, px(1.0, -1.0))
    check("waypoint with heading", len(wps) == 1 and abs(wps[0][2] + math.pi / 2) < 0.05, f"{wps}")
    (x0, x1), _ = vb.viewRange()
    p0, p1 = px(0.0, 0.0), px(2.0, 0.0)
    QTest.mousePress(v.viewport(), Qt.RightButton, Qt.NoModifier, p1)
    for k in range(1, 6):
        QTest.mouseMove(v.viewport(), p1 + (p0 - p1) * (k / 5))
    QTest.mouseRelease(v.viewport(), Qt.RightButton, Qt.NoModifier, p0)
    (y0, y1), _ = vb.viewRange()
    check("right-drag pans the map (2 m to the left)", abs((y0 - x0) - 2.0) < 0.1, f"{x0:.2f} -> {y0:.2f}")
    check("right-drag sends nothing", len(wps) == 1 and len(poses) == 1)
    v.set_waypoints([(0, 0, "1  a", 0.0), (1, 1, "2  b", 1.57)])
    check("waypoints drawn with heading arrows", len(v.wp_arrows) == 2)
    v.close()


# ---- 2. PointCloud2 decoder ----

def test_pointcloud() -> None:
    print("\n[PointCloud2 decoder]")
    pts = np.array([[1, 2, 3], [4, 5, 6], [0.1, 0.2, 0.3]], dtype=np.float32)
    step = 12
    buf = b"".join(struct.pack("<fff", *p) for p in pts)
    msg = {
        "width": pts.shape[0], "height": 1, "point_step": step,
        "fields": [
            {"name": "x", "offset": 0, "datatype": 7, "count": 1},
            {"name": "y", "offset": 4, "datatype": 7, "count": 1},
            {"name": "z", "offset": 8, "datatype": 7, "count": 1},
        ],
        "data": base64.b64encode(buf).decode(),
    }
    out = _decode_pointcloud2(msg)
    check("decoder returns np array", isinstance(out, np.ndarray))
    check("decoder preserves 3 points", out is not None and out.shape == (3, 3))
    check("decoder preserves values", np.allclose(out, pts))

    # with NaNs mixed in
    pts2 = np.array([[1.0, np.nan, 2.0], [3.0, 4.0, 5.0]], dtype=np.float32)
    buf2 = b"".join(struct.pack("<fff", *p) for p in pts2)
    msg["data"] = base64.b64encode(buf2).decode()
    msg["width"] = 2
    out2 = _decode_pointcloud2(msg)
    check("decoder filters NaN rows", out2 is not None and out2.shape == (1, 3))

    # pass a real cloud straight to the 3D panel and verify the scatter populated
    msg_full = {
        "width": 3, "height": 1, "point_step": step,
        "fields": msg["fields"],
        "data": base64.b64encode(buf).decode(),
    }
    panel = View3DPanel()
    panel.update_pointcloud(msg_full)
    from rcs.ui.panels.view3d import HAVE_GL
    if HAVE_GL:
        posdata = panel.scatter.pos
        check("3D panel scatter received 3 points",
              posdata is not None and len(posdata) == 3)
        # the 3D map stores points voxel-sorted, so compare as sets
        srt = lambda a: a[np.lexsort(a.T[::-1])]   # noqa: E731
        check("3D scatter preserves point values",
              posdata is not None and np.allclose(srt(np.asarray(posdata)), srt(pts)))
    else:
        check("3D panel skipped (no GL)", True)


# ---- 3. Camera JPEG decoder ----

def test_camera() -> None:
    print("\n[Camera JPEG]")
    # Encode a 64x48 red image to JPEG in memory
    img = Image.new("RGB", (64, 48), (220, 40, 40))
    bio = io.BytesIO()
    img.save(bio, format="JPEG", quality=70)
    jpeg = bio.getvalue()
    check("JPEG header looks right", jpeg[:3] == b"\xff\xd8\xff")

    panel = CameraPanel()
    panel.set_frame(jpeg)
    pm = panel._last_pix
    check("CameraPanel decoded a QPixmap", pm is not None and not pm.isNull())
    check("CameraPanel pixmap size matches",
          pm is not None and pm.width() == 64 and pm.height() == 48)


# ---- 4. TF cache composition ----

def test_tf() -> None:
    print("\n[TFCache]")
    cache = TFCache()
    # map->odom: identity;  odom->base_link: (1,2) yaw 90°;  base_link->laser: (0.4, 0)
    tf_msg = {"transforms": [
        {"header": {"frame_id": "map"}, "child_frame_id": "odom",
         "transform": {"translation": {"x": 0, "y": 0, "z": 0},
                       "rotation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
        {"header": {"frame_id": "odom"}, "child_frame_id": "base_link",
         "transform": {"translation": {"x": 1.0, "y": 2.0, "z": 0.0},
                       "rotation": {"x": 0, "y": 0,
                                    "z": math.sin(math.pi / 4),
                                    "w": math.cos(math.pi / 4)}}},
        {"header": {"frame_id": "base_link"}, "child_frame_id": "laser_frame",
         "transform": {"translation": {"x": 0.4, "y": 0, "z": 0},
                       "rotation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
    ]}
    cache.ingest(tf_msg)
    pose = cache.pose_xyyaw("map", "base_link")
    check("map→base_link translation", pose is not None
          and abs(pose[0] - 1.0) < 1e-5 and abs(pose[1] - 2.0) < 1e-5)
    check("map→base_link yaw 90°", pose is not None and abs(pose[2] - math.pi / 2) < 1e-5)

    laser = cache.pose_xyyaw("map", "laser_frame")
    # base_link is at (1,2) yaw 90°, laser is forward 0.4 m → laser at (1, 2.4)
    check("map→laser x", laser is not None and abs(laser[0] - 1.0) < 1e-5)
    check("map→laser y", laser is not None and abs(laser[1] - 2.4) < 1e-5)


# ---- 5. Mission executor ----

def test_mission_executor(tmp_path: Path) -> None:
    print("\n[Mission executor]")
    store = MissionStore(tmp_path / "missions.json")
    m = Mission.new("looper")
    m.loop = True
    m.waypoints = [
        Waypoint.new("A", 0.0, 0.0),
        Waypoint.new("B", 2.0, 0.0),
        Waypoint.new("C", 2.0, 2.0),
    ]
    for w in m.waypoints:
        w.tolerance = 0.3
        w.dwell = 0.0
    store.upsert(m)

    panel = MissionPanel(store)
    panel.cur = m
    panel._set_combo_to(m.id)

    goals_sent = []
    panel.goal_requested.connect(lambda x, y, yaw: goals_sent.append((x, y)))
    panel._play()

    # advance 10 "arrivals" so we also wrap through the loop
    for i in range(10):
        (panel.update_robot_pose(*goals_sent[-1], 0.0), panel.on_nav_status("succeeded")) if goals_sent else None
        time.sleep(0.01)
        panel._exec_tick()
        _app.processEvents()

    check("executor dispatched ≥3 goals (initial loop)", len(goals_sent) >= 3)
    first3 = goals_sent[:3]
    check("goals in waypoint order",
          first3 == [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)])
    if len(goals_sent) >= 4:
        check("loop wraps back to first waypoint",
              goals_sent[3] == (0.0, 0.0))
    else:
        check("loop wraps back to first waypoint", False, f"only {len(goals_sent)} goals")

    # now a non-looping mission should stop
    m2 = Mission.new("oneshot")
    m2.waypoints = [Waypoint.new("A", 0.0, 0.0), Waypoint.new("B", 1.0, 0.0)]
    for w in m2.waypoints:
        w.tolerance = 0.3; w.dwell = 0.0
    store.upsert(m2)
    panel.cur = m2; panel._exec_idx = -1
    goals_sent.clear()
    panel.goal_requested.disconnect()
    panel.goal_requested.connect(lambda x, y, yaw: goals_sent.append((x, y)))
    panel._play()
    for _ in range(5):
        (panel.update_robot_pose(*goals_sent[-1], 0.0), panel.on_nav_status("succeeded")) if goals_sent else None
        panel._exec_tick(); _app.processEvents()
    check("non-looping mission stops after last waypoint",
          not panel._exec_running and panel._exec_idx == -1)

    # Nav2 reports "succeeded" for a preempted goal while the robot is still
    # elsewhere: the executor must resend the waypoint, not wait forever.
    goals_sent.clear()
    panel.cur = m2; panel._exec_idx = -1
    panel.update_robot_pose(5.0, 5.0, 0.0)
    panel._play()
    first = len(goals_sent)
    panel.on_nav_status("succeeded")
    end = time.monotonic() + 2.5
    while time.monotonic() < end:
        _app.processEvents(); time.sleep(0.02)
    check("false 'succeeded' far from waypoint triggers a resend",
          len(goals_sent) == first + 1 and goals_sent[-1] == goals_sent[0],
          f"{goals_sent}")
    for _ in range(3):
        panel.on_nav_status("aborted")
        end = time.monotonic() + 2.2
        while time.monotonic() < end:
            _app.processEvents(); time.sleep(0.02)
    check("gives up after 3 failed retries", not panel._exec_running,
          panel.status_lbl.text())


# ---- main ----

def main() -> int:
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        test_map_view()
        test_goal_arrow()
        test_pointcloud()
        test_camera()
        test_tf()
        test_mission_executor(Path(td))
    print(f"\n--- {PASS} passed, {FAIL} failed ---")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    rc = main()
    print(f"exit code: {rc}")
    sys.stdout.flush(); sys.stderr.flush()
    # Skip interpreter teardown: Qt/PySide object destruction order at exit
    # can segfault after all checks already ran.
    if sys.platform == "win32":
        # ExitProcess runs Qt's DLL detach handlers, which crash while
        # widgets are still alive; TerminateProcess skips them.
        import ctypes
        from ctypes import wintypes
        k32 = ctypes.windll.kernel32
        k32.GetCurrentProcess.restype = wintypes.HANDLE
        k32.TerminateProcess.argtypes = [wintypes.HANDLE, wintypes.UINT]
        k32.TerminateProcess(k32.GetCurrentProcess(), rc)
    os._exit(rc)
