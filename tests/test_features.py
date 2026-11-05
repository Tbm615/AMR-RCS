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
from PySide6.QtGui import QVector3D
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

    # coordinates under the mouse and the ruler
    vb.setRange(xRange=(-5, 5), yRange=(-5, 5), padding=0)
    for _ in range(3):
        QApplication.processEvents()
    QTest.mouseMove(v.viewport(), px(2.0, -1.0))
    QApplication.processEvents()
    t = v.coord.text()
    check("coordinates under the mouse", "+2.0" in t and "-1.0" in t and v.coord.isVisible(), t)
    v.set_click_mode("measure")
    wps.clear()
    a, b = px(0.0, 0.0), px(3.0, 4.0)
    QTest.mousePress(v.viewport(), Qt.LeftButton, Qt.NoModifier, a)
    QTest.mouseMove(v.viewport(), b)
    QTest.mouseRelease(v.viewport(), Qt.LeftButton, Qt.NoModifier, b)
    check("ruler measures 5 m", v.ruler_m is not None and abs(v.ruler_m - 5.0) < 0.1, f"{v.ruler_m}")
    check("ruler label", v.ruler_text.isVisible() and v.ruler_text.textItem.toPlainText().endswith(" m"))
    check("measuring sends nothing", not wps and len(got) == 0)
    QTest.mouseMove(v.viewport(), px(-3.0, 0.0))
    check("ruler stays after release", abs(v.ruler_m - 5.0) < 0.1)
    QTest.keyClick(v, Qt.Key_Escape)
    check("Esc clears the ruler", v.ruler_m is None and not v.ruler_text.isVisible())
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


# ---- 2b. Fusion view: 2D layers and goals in the 3D page ----

def test_fusion() -> None:
    print("\n[Fusion view]")
    from PySide6.QtCore import QEvent, QPointF
    from PySide6.QtGui import QMouseEvent
    from rcs.ui.panels.view3d import HAVE_GL
    if not HAVE_GL:
        check("fusion skipped (no GL)", True)
        return
    p = View3DPanel()
    p.view.resize(800, 600)     # not shown: the offscreen platform has no OpenGL

    # laser at (1, 2) facing +y: a beam straight ahead at 1 m lands on (1, 3)
    scan = {"angle_min": 0.0, "angle_max": 0.0, "angle_increment": 0.1, "range_min": 0.0,
            "range_max": 10.0, "ranges": [1.0]}
    p.set_scan(scan, 1.0, 2.0, math.pi / 2)
    pts = p.scan_pts.pos
    check("lidar in the map frame", len(pts) == 1 and abs(pts[0][0] - 1) < 1e-4 and abs(pts[0][1] - 3) < 1e-4,
          f"{pts}")
    p.ck_scan.setChecked(False)
    p.set_scan({**scan, "ranges": [2.0]}, 0.0, 0.0, 0.0)
    check("lidar layer off skips the update", abs(p.scan_pts.pos[0][1] - 3) < 1e-4)
    p.ck_scan.setChecked(True)

    p.set_path({"poses": [{"pose": {"position": {"x": i, "y": 0}}} for i in range(4)]})
    check("planned path drawn", len(p.path_line.pos) == 4)
    p.set_path({"poses": []})
    check("empty path clears it", len(p.path_line.pos) == 0)

    p.set_goal(1.0, 1.0, 0.0)
    check("goal arrow shown", p.goal_arrow.visible() and abs(p.goal_arrow.pos[1][0] - 2.0) < 1e-4)
    p.set_goal(None, None)
    check("goal cleared", not p.goal_arrow.visible())

    p.set_waypoints([(0, 0, "1  a", 0.0), (1, 1, "2  b", 1.57)])
    check("waypoints, headings and labels", len(p.wp_pts.pos) == 2 and len(p.wp_labels) == 2
          and p.wp_heads.visible())
    p.ck_wps.setChecked(False)
    check("waypoints layer hides labels too", not p.wp_pts.visible() and not p.wp_labels[0].visible())
    p.ck_wps.setChecked(True)
    p.set_waypoints([])
    check("waypoints cleared", len(p.wp_pts.pos) == 0 and not p.wp_labels and not p.wp_heads.visible())

    # costmap in odom, odom 2 m ahead of map: pixel (0,0) lands at frame + origin
    cost = {"info": {"width": 2, "height": 3, "resolution": 0.5,
                     "origin": {"position": {"x": 1.0, "y": 0.0}, "orientation": {"z": 0.0, "w": 1.0}}},
            "data": [100, 0, 0, 0, 0, 0]}
    check("costmap fed only when its layer is on", p.layer_on("lcost") and not p.layer_on("gcost"))
    p.set_local_costmap(cost, (2.0, 0.0, 0.0))
    m = p.lcost.transform().map(QVector3D(1, 0, 0))
    check("costmap placed through its frame", abs(m.x() - 3.5) < 1e-4 and abs(m.y()) < 1e-4, f"{m}")

    p.set_view_mode("3d")
    check("3D only hides the 2D layers", p.ck_cloud.isChecked() and not p.ck_scan.isChecked()
          and not p.ck_floor.isChecked())
    p.set_view_mode("2d")
    check("2D only hides the cloud", not p.ck_cloud.isChecked() and p.ck_scan.isChecked())
    p.set_view_mode("fusion")
    check("fusion shows both", p.ck_cloud.isChecked() and p.ck_scan.isChecked() and p.ck_lcost.isChecked())

    # floor picking round-trips through the camera
    v = p.view
    v.setCameraPosition(distance=12, elevation=50, azimuth=-120)
    sx, sy = v.screen_point(1.5, -2.0)
    g = v.ground_point(sx, sy)
    check("floor point under the mouse", g is not None and abs(g[0] - 1.5) < 1e-2 and abs(g[1] + 2.0) < 1e-2,
          f"{g}")
    check("above the horizon misses the floor", (lambda e: (v.setCameraPosition(elevation=2),
          v.ground_point(v.width() / 2, 0.0), v.setCameraPosition(elevation=50))[1])(0) is None)

    def mouse(kind, x, y, btn=Qt.LeftButton, buttons=Qt.LeftButton):
        pos = QPointF(x, y)
        return QMouseEvent(kind, pos, v.mapToGlobal(pos), btn, buttons, Qt.NoModifier)

    got = []
    p.goal_requested.connect(lambda *a: got.append(("goal",) + a))
    p.pose_requested.connect(lambda *a: got.append(("pose",) + a))
    p.waypoint_requested.connect(lambda *a: got.append(("wp",) + a))

    def drag(ax, ay, bx, by):
        s0, s1 = v.screen_point(ax, ay), v.screen_point(bx, by)
        v.mousePressEvent(mouse(QEvent.MouseButtonPress, *s0))
        v.mouseMoveEvent(mouse(QEvent.MouseMove, *s1, Qt.NoButton))
        v.mouseReleaseEvent(mouse(QEvent.MouseButtonRelease, *s1, Qt.LeftButton, Qt.NoButton))

    for i, (key, name) in enumerate((("goal", "goal"), ("pose", "pose"), ("waypoint", "wp"))):
        p.mouse_mode.setCurrentIndex(p.mouse_mode.findData(key))
        got.clear()
        drag(1.0, 1.0, 1.0, 3.0)
        ok = len(got) == 1 and got[0][0] == name and abs(got[0][1] - 1) < 0.02 and abs(got[0][2] - 1) < 0.02 \
            and abs(got[0][3] - math.pi / 2) < 0.02
        check(f"drag on the floor: {key} with heading", ok, f"{got}")
    check("aim arrow hidden after release", not p.aim_arrow.visible())

    got.clear()
    s0 = v.screen_point(0, 0)
    v.mousePressEvent(mouse(QEvent.MouseButtonPress, *s0))
    check("aim arrow shown while dragging", p.aim_arrow.visible())
    v.mousePressEvent(mouse(QEvent.MouseButtonPress, *s0, Qt.RightButton, Qt.LeftButton | Qt.RightButton))
    v.mouseReleaseEvent(mouse(QEvent.MouseButtonRelease, *s0, Qt.LeftButton, Qt.NoButton))
    check("right-click cancels", not got and not p.aim_arrow.visible())

    p.mouse_mode.setCurrentIndex(0)
    got.clear()
    drag(0.0, 0.0, 1.0, 1.0)
    check("Orbit mode sends nothing", not got)


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


# ---- 4b. Clearance around the footprint ----

def test_clearance() -> None:
    print("\n[Clearance]")
    from rcs.core.clearance import compute_clearance
    from rcs.ui.panels.teleop import STALE_S, TeleopPanel
    pts = lambda *p: (np.array([a for a, _ in p], np.float32), np.array([b for _, b in p], np.float32))  # noqa: E731
    hl, hw = 0.45, 0.30
    lid = (0.4, 0.0, 0.0)           # lidar 0.4 m ahead of base_link, as on the robot
    c = compute_clearance(*pts((1.05, 0.0)), lid, hl, hw)
    check("point ahead: gap from the front edge, not the lidar", abs(c.front - 1.0) < 1e-5, f"{c}")
    check("nothing behind", math.isinf(c.rear))
    c = compute_clearance(*pts((-1.85, 0.0)), lid, hl, hw)
    check("point behind", abs(c.rear - 1.0) < 1e-5 and math.isinf(c.front), f"{c}")
    c = compute_clearance(*pts((-0.4, 1.0)), lid, hl, hw)
    check("point to the side: nearest only", abs(c.nearest - 0.7) < 1e-5
          and math.isinf(c.front) and math.isinf(c.rear), f"{c}")
    c = compute_clearance(*pts((0.35, 0.70)), lid, hl, hw)      # 0.3, 0.4 past the corner
    check("past a corner: distance to the corner", abs(c.nearest - 0.5) < 1e-5, f"{c}")
    c = compute_clearance(*pts((-0.4, 0.1)), lid, hl, hw)
    check("inside the body is zero", c.nearest == 0.0, f"{c}")
    c = compute_clearance(*pts((1.0, 0.0)), (0.0, 0.0, math.pi), hl, hw)
    check("a lidar mounted backwards", abs(c.rear - 0.55) < 1e-5 and math.isinf(c.front), f"{c}")
    c = compute_clearance(np.empty(0, np.float32), np.empty(0, np.float32), lid, hl, hw)
    check("empty scan is all clear", math.isinf(c.nearest))

    t = TeleopPanel()
    t._tx_timer.stop()
    sent = []
    t.cmd_vel.connect(lambda a, b: sent.append((a, b)))
    t.lin.setValue(0.4); t.ang.setValue(0.8)
    t._key_lin, t._key_ang = 1.0, 1.0
    t._tick()
    check("Full speed uses the limits", sent[-1] == (0.4, 0.8), f"{sent[-1]}")
    t.set_speed(0)
    t._tick()
    check("Slow is a quarter of them", abs(sent[-1][0] - 0.1) < 1e-6 and abs(sent[-1][1] - 0.2) < 1e-6, f"{sent[-1]}")
    from PySide6.QtTest import QTest
    QTest.keyClick(t, Qt.Key_2)
    t._tick()
    check("key 2 = Medium", abs(sent[-1][0] - 0.2) < 1e-6 and t.speed_btns.checkedId() == 1, f"{sent[-1]}")
    t._key_lin = t._key_ang = 0.0
    t.set_clearance(compute_clearance(*pts((0.2, 0.0)), lid, hl, hw))     # 0.15 m
    check("red under 0.25 m", t.bar_front.color() == "#ff6a6a", t.bar_front.text())
    t.set_clearance(compute_clearance(*pts((0.55, 0.0)), lid, hl, hw))    # 0.50 m
    check("amber under 0.80 m", t.bar_front.color() == "#ffb547", t.bar_front.text())
    t.set_clearance(compute_clearance(*pts((1.55, 0.0)), lid, hl, hw))    # 1.50 m
    check("green when clear", t.bar_front.color() == "#39d98a" and t.bar_front.text() == "1.50 m",
          t.bar_front.text())
    check("nothing there reads as > 2 m", t.bar_rear.text() == "> 2.0 m", t.bar_rear.text())
    t._clear_t = time.monotonic() - STALE_S - 0.1
    t._check_clearance_age()
    check("old scan shows 'no lidar', not a stale distance", t.bar_front.value is None
          and t.bar_front.text() == "no lidar")


# ---- 4b2. Alert sounds ----

def test_sounds(tmp_path: Path) -> None:
    print("\n[Sounds]")
    import wave
    from rcs.core import sound
    for name in sound.TONES:
        with wave.open(sound._wav_path(name)) as w:
            check(f"{name} tone is a real WAV", w.getnframes() > 1000 and w.getframerate() == sound.RATE)
    s = sound.Sounds()
    played = []
    s._player, real = "fake", sound.subprocess.Popen
    sound.subprocess.Popen = lambda cmd, **k: played.append(cmd)
    real_platform, sound.sys.platform = sound.sys.platform, "linux"
    try:
        check("first alert plays", s.play("warn") and len(played) == 1)
        check("a burst makes one sound", not s.play("alarm") and len(played) == 1)
        s._last = 0.0
        s.enabled = False
        check("off means silent", not s.play("alarm") and len(played) == 1)
    finally:
        sound.subprocess.Popen, sound.sys.platform = real, real_platform


# ---- 4c. Map preview made on the robot ----

def test_map_preview(tmp_path: Path) -> None:
    print("\n[Map preview]")
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "map_preview", Path(__file__).resolve().parent.parent / "robot" / "map_preview.py")
    mp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mp)
    from PySide6.QtGui import QPixmap

    h, w = 2000, 1000
    img = np.full((h, w), 254, np.uint8)
    img[500, :] = 0                                   # a one-cell wall
    pgm = tmp_path / "office.pgm"
    pgm.write_bytes(b"P5\n# CREATOR: map_saver\n%d %d\n255\n" % (w, h) + img.tobytes())
    yml = tmp_path / "office.yaml"
    yml.write_text("image: office.pgm\nmode: trinary\nresolution: 0.05\norigin: [-5, -5, 0]\n")
    check("reads map_saver's PGM (with a comment)", np.array_equal(mp.read_pgm(pgm), img))
    small = mp.shrink(img)
    check("shrunk to fit", max(small.shape) <= mp.MAX_PX, f"{small.shape}")
    check("a one-cell wall survives", (small == 0).any())
    out = mp.map_preview(str(yml))
    check("size in metres", out["width_m"] == 50.0 and out["height_m"] == 100.0, f"{out['width_m']}")
    png = base64.b64decode(out["png"])
    pix = QPixmap()
    check("valid PNG", pix.loadFromData(png, "PNG") and max(pix.width(), pix.height()) <= mp.MAX_PX)
    check("small enough for one message", len(out["png"]) < 200_000, f"{len(out['png'])}")
    px = pix.toImage()
    check("pixels survive encoding", px.pixelColor(0, 0).red() == 254)
    yml.write_text("image: office.png\nresolution: 0.05\n")
    try:
        mp.map_preview(str(yml))
        check("other image types refused", False)
    except ValueError as e:
        check("other image types refused", "no preview" in str(e), str(e))


# ---- 5b. Waypoint table: multi-select, remove, move, bulk settings ----

def test_mission_multiselect(tmp_path: Path) -> None:
    print("\n[Mission waypoint multi-select]")
    from PySide6.QtWidgets import QMessageBox
    store = MissionStore(tmp_path / "missions_sel.json")
    m = Mission.new("sel")
    m.waypoints = [Waypoint.new(n, float(i), 0.0) for i, n in enumerate("ABCDEF")]
    store.upsert(m)
    panel = MissionPanel(store)
    panel._set_combo_to(m.id)
    names = lambda: "".join(w.name for w in panel.cur.waypoints)   # noqa: E731

    panel.select_rows([1, 3, 4])
    check("three rows selected", panel.selected_rows() == [1, 3, 4], f"{panel.selected_rows()}")
    check("selection ringed on the map", len(panel.view.wp_sel.getData()[0]) == 3)
    check("selection count shown", panel.sel_lbl.text() == "3 selected", panel.sel_lbl.text())

    panel._move(-1)
    check("move up keeps the block together", names() == "BADECF", names())
    check("moved rows stay selected", panel.selected_rows() == [0, 2, 3], f"{panel.selected_rows()}")
    panel._move(-1)
    check("move up stops at the top", names() == "BADECF", names())
    panel._move(1)
    check("move down restores the order", names() == "ABCDEF", names())

    panel.select_rows([0, 2])
    panel.tol_sb.setValue(0.6); panel.dwell_sb.setValue(4.0)
    panel._apply_to_selected()
    w = panel.cur.waypoints
    check("apply to selected sets tolerance/dwell",
          w[0].tolerance == 0.6 and w[2].dwell == 4.0 and w[1].tolerance != 0.6,
          f"{[(x.tolerance, x.dwell) for x in w]}")

    before = names()
    panel.select_rows([0, 2, 5])
    panel._remove()
    exp = "".join(c for i, c in enumerate(before) if i not in (0, 2, 5))
    check("remove deletes every selected waypoint", names() == exp, f"{before} -> {names()}")
    check("saved to the store", len(store.get(m.id).waypoints) == 3)

    # Delete key on the table
    panel.table.setFocus()
    panel.select_rows([0])
    from PySide6.QtTest import QTest
    panel.show(); _app.processEvents()
    panel.table.setFocus(); _app.processEvents()
    QTest.keyClick(panel.table, Qt.Key_Delete)
    check("Delete key removes the selection", len(panel.cur.waypoints) == 2, names())

    # no edits under a running mission
    panel._exec_idx = 0
    panel.select_rows([0])
    panel._remove()
    check("remove blocked while a mission runs", len(panel.cur.waypoints) == 2)
    panel._exec_idx = -1

    orig = QMessageBox.question
    QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.Yes)
    try:
        panel._clear_all()
    finally:
        QMessageBox.question = orig
    check("clear all empties the mission", panel.cur.waypoints == [] and panel.table.rowCount() == 0)
    panel.close()


# ---- main ----

def main() -> int:
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        test_map_view()
        test_goal_arrow()
        test_pointcloud()
        test_fusion()
        test_camera()
        test_tf()
        test_clearance()
        test_sounds(Path(td))
        test_map_preview(Path(td))
        test_mission_executor(Path(td))
        test_mission_multiselect(Path(td))
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
