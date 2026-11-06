"""3D map page: colored point map in the map frame, like the 3D view of the
report.html that bringup's save_run writes.

The map itself (rcs.core.cloud_map.CloudMap) is built by RosClient on its
decoder thread from the camera's depth + color images (or a PointCloud2),
each frame placed with the TF at its own timestamp. This page draws it, plus
the robot, its trajectory and the 2D map on the floor, and saves/loads the map
as PLY (the same format as the report's map/cloud3d.ply).

Fusion view: the live 2D layers (lidar, planned path, costmaps, goal, mission
waypoints) are drawn in the same scene, from the data the 2D pages already
receive, so it costs no extra Wi-Fi. Goals, the initial pose and waypoints can
be set here too: press on the floor and drag toward the heading.
"""
from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont, QGuiApplication, QVector3D, QVector4D
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QLabel, QMenu, QMessageBox,
    QPushButton, QSlider, QToolButton, QVBoxLayout, QWidget,
)

from ...core import cloud_map as cm
from ...core import robot_model
from ..widgets.flow import FlowLayout

try:
    import pyqtgraph.opengl as gl
    from pyqtgraph import Transform3D, Vector
    HAVE_GL = True
except Exception:  # noqa: BLE001
    HAVE_GL = False

BG = (17, 21, 28)                 # same background as the report viewer
DISPLAY_MAX = 1_500_000           # points drawn at most (the map may hold more)
# heights of the 2D layers in the scene (m): stacked so they do not z-fight
Z_FLOOR, Z_GCOST, Z_LCOST, Z_PATH, Z_MARK, Z_SCAN = -0.01, 0.0, 0.005, 0.04, 0.06, 0.2

VIEW_MODES = [
    ("fusion", "Fusion 2D + 3D"),
    ("3d", "3D map only"),
    ("2d", "2D layers only"),
]
MOUSE_MODES = [
    ("orbit", "Orbit view"),
    ("goal", "Set goal"),
    ("pose", "Set initial pose"),
    ("waypoint", "Add waypoint"),
]


def _arrow_segments(x: float, y: float, yaw: float, z: float, length: float = 1.0) -> np.ndarray:
    """Line pairs of an arrow from (x, y) along yaw, for GLLinePlotItem 'lines'."""
    tip = np.array([x + length * math.cos(yaw), y + length * math.sin(yaw), z])
    base = np.array([x, y, z])
    pts = [base, tip]
    for side in (2.6, -2.6):
        a = yaw + side
        pts += [tip, tip + 0.28 * length * np.array([math.cos(a), math.sin(a), 0.0])]
    return np.array(pts, np.float32)


def _decode_pointcloud2(msg: dict) -> Optional[np.ndarray]:
    """XYZ of a PointCloud2 (kept for callers that only need positions)."""
    out = cm.decode_pointcloud2(msg)
    return None if out is None else out[0]


if HAVE_GL:
    class _GLView(gl.GLViewWidget):
        """Report-style controls: left-drag orbits, right/middle-drag (or
        shift+left) pans, wheel zooms. With a mouse mode other than orbit,
        left press-drag-release on the floor aims an arrow instead."""

        # the offscreen platform (headless tests) has no OpenGL to draw with
        def initializeGL(self):
            if QGuiApplication.platformName() != "offscreen":
                super().initializeGL()

        def paintGL(self):
            if QGuiApplication.platformName() != "offscreen":
                super().paintGL()

        mode = "orbit"
        on_aim = None           # (x, y, yaw) while aiming, None when it stops
        on_aimed = None         # (mode, x, y, yaw) on release
        _anchor = None

        def ground_point(self, px: float, py: float):
            """Map-frame (x, y) of the floor under a widget position, or None
            if that ray misses the floor (above the horizon)."""
            w, h = self.width(), self.height()
            if w <= 0 or h <= 0:
                return None
            vp = self.getViewport()
            m, ok = (self.projectionMatrix(vp, vp) * self.viewMatrix()).inverted()
            if not ok:
                return None
            nx, ny = 2.0 * px / w - 1.0, 1.0 - 2.0 * py / h
            a, b = m.map(QVector4D(nx, ny, -1, 1)), m.map(QVector4D(nx, ny, 1, 1))
            if a.w() == 0 or b.w() == 0:
                return None
            ax, ay, az = a.x() / a.w(), a.y() / a.w(), a.z() / a.w()
            bx, by, bz = b.x() / b.w(), b.y() / b.w(), b.z() / b.w()
            if abs(bz - az) < 1e-9:
                return None
            t = -az / (bz - az)
            if t <= 0:
                return None
            return ax + t * (bx - ax), ay + t * (by - ay)

        def screen_point(self, x: float, y: float, z: float = 0.0):
            """Widget position of a map-frame point (the inverse of ground_point)."""
            vp = self.getViewport()
            p = (self.projectionMatrix(vp, vp) * self.viewMatrix()).map(QVector3D(x, y, z))
            return (p.x() + 1) / 2 * self.width(), (1 - p.y()) / 2 * self.height()

        def _yaw_to(self, pos) -> float:
            g = self.ground_point(pos.x(), pos.y())
            if g is None or math.hypot(g[0] - self._anchor[0], g[1] - self._anchor[1]) < 1e-3:
                return 0.0
            return math.atan2(g[1] - self._anchor[1], g[0] - self._anchor[0])

        def cancel_aim(self) -> None:
            self._anchor = None
            if self.on_aim:
                self.on_aim(None)

        def mousePressEvent(self, ev):
            pos = ev.position() if hasattr(ev, "position") else ev.localPos()
            if self._anchor is not None and ev.button() == Qt.RightButton:
                self.cancel_aim()
                return
            if self.mode != "orbit" and ev.button() == Qt.LeftButton \
                    and not ev.modifiers() & Qt.ShiftModifier:
                g = self.ground_point(pos.x(), pos.y())
                if g is not None:
                    self._anchor = g
                    if self.on_aim:
                        self.on_aim((g[0], g[1], 0.0))
                return
            super().mousePressEvent(ev)

        def mouseReleaseEvent(self, ev):
            if self._anchor is not None and ev.button() == Qt.LeftButton:
                pos = ev.position() if hasattr(ev, "position") else ev.localPos()
                x, y = self._anchor
                yaw = self._yaw_to(pos)
                self.cancel_aim()
                if self.on_aimed:
                    self.on_aimed(self.mode, x, y, yaw)
                return
            super().mouseReleaseEvent(ev)

        def mouseMoveEvent(self, ev):
            pos = ev.position() if hasattr(ev, "position") else ev.localPos()
            if self._anchor is not None:
                if self.on_aim:
                    self.on_aim((self._anchor[0], self._anchor[1], self._yaw_to(pos)))
                return
            if not hasattr(self, "mousePos") or self.mousePos is None:
                self.mousePos = pos
            diff = pos - self.mousePos
            self.mousePos = pos
            b = ev.buttons()
            if b & (Qt.RightButton | Qt.MiddleButton) or (
                    b & Qt.LeftButton and ev.modifiers() & Qt.ShiftModifier):
                self.pan(diff.x(), diff.y(), 0, relative="view")
                if getattr(self, "on_pan", None):
                    self.on_pan()
            elif b & Qt.LeftButton:
                self.orbit(-diff.x(), diff.y())
                if self.opts["elevation"] < 2:
                    self.opts["elevation"] = 2
            self.user_moved = True

        def wheelEvent(self, ev):
            self.user_moved = True
            super().wheelEvent(ev)


class View3DPanel(QWidget):
    recording_changed = Signal(bool)      # main window turns the streams on/off
    source_changed = Signal(str)          # "rgbd" | "cloud"
    goal_requested = Signal(float, float, float)
    pose_requested = Signal(float, float, float)
    waypoint_requested = Signal(float, float, float)

    def __init__(self, cloud_map: Optional[cm.CloudMap] = None, parent=None):
        super().__init__(parent)
        self.cloud_map = cloud_map if cloud_map is not None else cm.CloudMap(voxel=0.03)
        self._drawn_version = -1
        self._robot = None              # (x, y, yaw) in the map frame
        self._traj: list = []
        self._traj_dirty = False
        self._last_stats: dict = {}
        self._fps_t: list = []
        self._rec_since = time.monotonic()
        self.source_hint = ""           # set by the main window: topics being recorded
        self.camera_problem = ""        # set by the main window: why no frames arrive
        self.robot_half_l, self.robot_half_w, self.robot_h = 0.45, 0.30, 0.35

        v = QVBoxLayout(self)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(6)

        if not HAVE_GL:
            lbl = QLabel("3D view requires PyOpenGL + pyqtgraph.opengl.\n"
                         "Install: pip install PyOpenGL pyqtgraph")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("color:#8b93a7;")
            v.addWidget(lbl)
            self.scatter = None
            return

        self.view = _GLView()
        self.view.user_moved = False
        self.view.setBackgroundColor(BG)
        self.view.setCameraPosition(pos=Vector(0, 0, 0), distance=12, elevation=50, azimuth=-120)
        self.grid = gl.GLGridItem()
        self.grid.setSize(40, 40)
        self.grid.setSpacing(1, 1)
        self.grid.setColor((60, 66, 82, 120))
        self.view.addItem(self.grid)
        for vec, col in (([1, 0, 0], (1, .3, .3, 1)), ([0, 1, 0], (.3, 1, .3, 1)), ([0, 0, 1], (.3, .5, 1, 1))):
            self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], vec], float), color=col,
                                                width=2, antialias=True))
        self.floor = None               # GLImageItem with the 2D occupancy map
        self.scatter = gl.GLScatterPlotItem(pos=np.zeros((0, 3), np.float32), size=2.0, pxMode=True)
        self.scatter.setGLOptions("opaque")
        self.view.addItem(self.scatter)
        self.traj_line = gl.GLLinePlotItem(pos=np.zeros((0, 3), np.float32), color=(1.0, 0.71, 0.28, 1.0),
                                           width=2, antialias=True)
        self.view.addItem(self.traj_line)
        self.box = gl.GLBoxItem(color=(90, 157, 246, 255))
        self.box.setSize(2 * self.robot_half_l, 2 * self.robot_half_w, self.robot_h)
        self.box.hide()
        self.view.addItem(self.box)
        self.model = robot_model.load()
        self.mesh = None                # the robot's real body, when it is this robot
        if self.model is not None:
            md = gl.MeshData(vertexes=self.model[0], faceColors=self.model[1])
            self.mesh = gl.GLMeshItem(meshdata=md, smooth=False, shader="shaded", glOptions="opaque")
            self.mesh.hide()
            self.view.addItem(self.mesh)
        self.heading = gl.GLLinePlotItem(pos=np.zeros((2, 3), np.float32), color=(0.35, 0.62, 0.96, 1),
                                         width=3, antialias=True)
        self.view.addItem(self.heading)
        # fusion layers
        self.gcost = None               # GLImageItems, made with the first costmap
        self.lcost = None
        self.scan_pts = gl.GLScatterPlotItem(pos=np.zeros((0, 3), np.float32), size=3.0, pxMode=True,
                                             color=(1.0, 0.42, 0.42, 1.0))
        self.scan_pts.setGLOptions("opaque")
        self.view.addItem(self.scan_pts)
        self.path_line = gl.GLLinePlotItem(pos=np.zeros((0, 3), np.float32), color=(0.22, 0.85, 0.54, 1),
                                           width=3, antialias=True)
        self.view.addItem(self.path_line)
        self.goal_arrow = gl.GLLinePlotItem(pos=np.zeros((2, 3), np.float32), color=(0.22, 0.85, 0.54, 1),
                                            width=3, antialias=True, mode="lines")
        self.goal_arrow.hide()
        self.view.addItem(self.goal_arrow)
        self.aim_arrow = gl.GLLinePlotItem(pos=np.zeros((2, 3), np.float32), color=(1.0, 0.71, 0.28, 1),
                                           width=3, antialias=True, mode="lines")
        self.aim_arrow.hide()
        self.view.addItem(self.aim_arrow)
        self.wp_pts = gl.GLScatterPlotItem(pos=np.zeros((0, 3), np.float32), size=11.0, pxMode=True,
                                           color=(1.0, 0.71, 0.28, 1.0))
        self.wp_pts.setGLOptions("opaque")
        self.view.addItem(self.wp_pts)
        self.wp_heads = gl.GLLinePlotItem(pos=np.zeros((2, 3), np.float32), color=(1.0, 0.71, 0.28, 1),
                                          width=2, antialias=True, mode="lines")
        self.wp_heads.hide()
        self.view.addItem(self.wp_heads)
        self.wp_labels: list = []
        self._goal = None
        self.view.on_aim = self._show_aim
        self.view.on_aimed = self._aimed
        v.addWidget(self.view, 1)

        # ---- controls: wrap onto more lines when the page is narrow
        flow = FlowLayout()
        v.addLayout(flow)
        r1 = r2 = r3 = flow
        self.rec_btn = QPushButton("●  Record 3D map")
        self.rec_btn.setCheckable(True)
        self.rec_btn.setChecked(True)
        self.rec_btn.setProperty("kind", "success")
        self.rec_btn.setToolTip("Add what the camera sees to the 3D map while this page is open")
        self.rec_btn.toggled.connect(self._rec_toggled)
        r1.addWidget(self.rec_btn)
        r1.addWidget(QLabel("Source"))
        self.source = QComboBox()
        self.source.addItem("Camera depth + color (light)", "rgbd")
        self.source.addItem("PointCloud2 topic", "cloud")
        self.source.setToolTip("Depth + color images use ~1 MB/s over WiFi; a PointCloud2 is several times more.\n"
                               "A PointCloud2 in the map frame (e.g. an RTAB-Map cloud) replaces the map.")
        self.source.currentIndexChanged.connect(lambda _i: self.source_changed.emit(self.source.currentData()))
        r1.addWidget(self.source)
        r1.addWidget(QLabel("Voxel"))
        self.voxel = QComboBox()
        for cmv in (2, 3, 5, 8):
            self.voxel.addItem(f"{cmv} cm", cmv / 100.0)
        self.voxel.setCurrentIndex(1)
        self.voxel.currentIndexChanged.connect(lambda _i: self.cloud_map.set_voxel(self.voxel.currentData()))
        r1.addWidget(self.voxel)
        r1.addWidget(QLabel("Max range"))
        self.range = QDoubleSpinBox()
        self.range.setRange(1.0, 10.0)
        self.range.setSingleStep(0.5)
        self.range.setValue(4.0)
        self.range.setSuffix(" m")
        self.range.setToolTip("Depth farther than this is too noisy to keep")
        r1.addWidget(self.range)
        self.keep_bg = QCheckBox("Keep recording on other pages")
        self.keep_bg.setToolTip("Uses WiFi bandwidth all the time; off = only while this page is open")
        r1.addWidget(self.keep_bg)

        # ---- view
        top = QPushButton("Top view"); top.clicked.connect(self.top_view)
        persp = QPushButton("Perspective"); persp.clicked.connect(self.perspective_view)
        fit = QPushButton("Whole map"); fit.clicked.connect(self.fit_map)
        r2.addWidget(top); r2.addWidget(persp); r2.addWidget(fit)
        self.follow = QCheckBox("Follow robot")
        self.follow.setChecked(True)
        self.follow.setToolTip("Keep the robot in the middle of the view (panning turns it off)")
        self.view.on_pan = lambda: self.follow.setChecked(False)
        r2.addWidget(self.follow)
        r2.addWidget(QLabel("View"))
        self.view_mode = QComboBox()
        for key, label in VIEW_MODES:
            self.view_mode.addItem(label, key)
        self.view_mode.setToolTip("Fusion: 3D map with the live lidar, path, costmap, goal and waypoints")
        r2.addWidget(self.view_mode)
        r2.addWidget(QLabel("Mouse"))
        self.mouse_mode = QComboBox()
        for key, label in MOUSE_MODES:
            self.mouse_mode.addItem(label, key)
        self.mouse_mode.setToolTip("Other than Orbit: press on the floor and drag toward the heading.\n"
                                   "Right-click cancels. Right-drag still pans, the wheel zooms.")
        self.mouse_mode.currentIndexChanged.connect(self._mouse_mode_changed)
        r2.addWidget(self.mouse_mode)
        r2.addWidget(QLabel("Point size"))
        self.psize = QSlider(Qt.Horizontal)
        self.psize.setRange(1, 8)
        self.psize.setValue(2)
        self.psize.setFixedWidth(110)
        self.psize.valueChanged.connect(lambda _v: self._redraw_cloud(force=True))
        r2.addWidget(self.psize)
        layers = QToolButton()
        layers.setText("Layers ▾")
        layers.setPopupMode(QToolButton.InstantPopup)
        lm = QMenu(layers)
        layers.setMenu(lm)

        def layer(label: str, on: bool = True):
            a = lm.addAction(label)
            a.setCheckable(True)
            a.setChecked(on)
            return a

        self.ck_cloud = layer("3D map")
        self.ck_robot = layer("Robot")
        self.ck_traj = layer("Trajectory")
        self.ck_floor = layer("2D map on floor")
        self.ck_scan = layer("Lidar scan")
        self.ck_path = layer("Planned path")
        self.ck_gcost = layer("Global costmap", False)
        self.ck_lcost = layer("Local costmap")
        self.ck_goal = layer("Goal")
        self.ck_wps = layer("Waypoints")
        self.ck_grid = layer("Grid")
        self.ck_cloud.toggled.connect(self.scatter.setVisible)
        self.ck_robot.toggled.connect(lambda b: (self._show_body(b and self._robot is not None),
                                                 self.heading.setVisible(b)))
        self.ck_traj.toggled.connect(self.traj_line.setVisible)
        self.ck_floor.toggled.connect(lambda b: self.floor.setVisible(b) if self.floor else None)
        self.ck_grid.toggled.connect(self.grid.setVisible)
        self.ck_scan.toggled.connect(self.scan_pts.setVisible)
        self.ck_path.toggled.connect(self.path_line.setVisible)
        self.ck_gcost.toggled.connect(lambda b: self.gcost.setVisible(b) if self.gcost else None)
        self.ck_lcost.toggled.connect(lambda b: self.lcost.setVisible(b) if self.lcost else None)
        self.ck_goal.toggled.connect(lambda b: self.goal_arrow.setVisible(b and self._goal is not None))
        self.ck_wps.toggled.connect(self._show_waypoints)
        r2.addWidget(layers)
        self.view_mode.currentIndexChanged.connect(lambda _i: self.set_view_mode(self.view_mode.currentData()))

        # ---- file + status
        op = QPushButton("Open PLY…"); op.clicked.connect(self._open_ply)
        op.setToolTip("Load a saved 3D map, e.g. result/run_*/map/cloud3d.ply from save_run")
        sv = QPushButton("Save PLY…"); sv.clicked.connect(self._save_ply)
        clr = QPushButton("Clear"); clr.clicked.connect(self._clear)
        clr.setProperty("kind", "danger")
        r3.addWidget(op); r3.addWidget(sv); r3.addWidget(clr)
        self.status = QLabel("not recording")
        self.status.setProperty("muted", True)
        self.status.setWordWrap(True)
        v.addWidget(self.status)

        self._timer = QTimer(self)
        self._timer.setInterval(400)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    # ---- recording state ----

    def wants_stream(self, page_open: bool) -> bool:
        if not HAVE_GL:
            return False
        return self.rec_btn.isChecked() and (page_open or self.keep_bg.isChecked())

    def _rec_toggled(self, on: bool) -> None:
        self.rec_btn.setText("●  Record 3D map" if on else "○  Paused")
        self.rec_btn.setProperty("kind", "success" if on else "")
        self.rec_btn.style().unpolish(self.rec_btn)
        self.rec_btn.style().polish(self.rec_btn)
        self._rec_since = time.monotonic()
        self._fps_t = []
        self.recording_changed.emit(on)

    def on_cloud_stats(self, st: dict) -> None:
        self._last_stats = st
        if "error" not in st:
            now = time.monotonic()
            self._fps_t = [t for t in self._fps_t if now - t < 5.0] + [now]

    # ---- drawing ----

    def _tick(self) -> None:
        if not self.isVisible():
            return
        self._redraw_cloud()
        if self._traj_dirty and self.ck_traj.isChecked():
            self._traj_dirty = False
            self.traj_line.setData(pos=np.asarray(self._traj, np.float32))
        st = self._last_stats
        n = len(self.cloud_map)
        if self.camera_problem and self.rec_btn.isChecked():
            msg = f"⚠ {self.camera_problem}"
        elif "error" in st:
            msg = f"⚠ {st['error']}"
        elif self.rec_btn.isChecked():
            hz = (len(self._fps_t) - 1) / max(1e-3, self._fps_t[-1] - self._fps_t[0]) if len(self._fps_t) > 1 else 0
            if hz > 0:
                msg = f"recording · {hz:.1f} frames/s"
                if st.get("world") == "odom":
                    msg += " · no map yet: odometry only, starts again when Nav2 or SLAM runs"
            elif time.monotonic() - self._rec_since > 8:
                msg = (f"⚠ no camera frames from {self.source_hint or 'the robot'}. Is the camera "
                       "running (bringup) and, for depth images, image_transport_plugins installed "
                       "on the robot? Or switch Source to PointCloud2")
            else:
                msg = "recording · waiting for camera frames…"
        else:
            msg = "paused"
        full = " · map full (raise the voxel size)" if self.cloud_map.full else ""
        self.status.setText(f"{msg} · {n:,} points · voxel {self.cloud_map.voxel * 100:.0f} cm{full}")

    def _redraw_cloud(self, force: bool = False) -> None:
        if not HAVE_GL:
            return
        xyz, rgb, ver = self.cloud_map.snapshot()
        if ver == self._drawn_version and not force:
            return
        self._drawn_version = ver
        if len(xyz) > DISPLAY_MAX:
            idx = np.random.default_rng(0).choice(len(xyz), DISPLAY_MAX, replace=False)
            xyz, rgb = xyz[idx], rgb[idx]
        col = np.empty((len(xyz), 4), np.float32)
        col[:, :3] = rgb / 255.0
        col[:, 3] = 1.0
        self.scatter.setData(pos=np.ascontiguousarray(xyz), color=col, size=float(self.psize.value()))

    def uses_model(self) -> bool:
        return self.mesh is not None and robot_model.fits(self.model[0], self.robot_half_l, self.robot_half_w)

    def _show_body(self, on: bool) -> None:
        model = self.uses_model()
        self.box.setVisible(on and not model)
        if self.mesh is not None:
            self.mesh.setVisible(on and model)

    def set_robot_footprint(self, half_l: float, half_w: float) -> None:
        self.robot_half_l = half_l
        self.robot_half_w = half_w
        if HAVE_GL:
            self.box.setSize(2 * half_l, 2 * half_w, self.robot_h)
            self._show_body(self.ck_robot.isChecked() and self._robot is not None)

    def update_robot_pose(self, x: float, y: float, yaw: float) -> None:
        if not HAVE_GL:
            return
        self._robot = (x, y, yaw)
        if not self._traj or math.hypot(x - self._traj[-1][0], y - self._traj[-1][1]) > 0.05:
            self._traj.append((x, y, 0.02))
            if len(self._traj) > 20000:
                del self._traj[: len(self._traj) - 20000]
            self._traj_dirty = True
        if not self.isVisible():
            return
        if self.ck_robot.isChecked():
            self.box.resetTransform()
            self.box.translate(-self.robot_half_l, -self.robot_half_w, 0)
            self.box.rotate(math.degrees(yaw), 0, 0, 1)
            self.box.translate(x, y, 0)
            if self.mesh is not None:
                self.mesh.resetTransform()
                self.mesh.rotate(math.degrees(yaw), 0, 0, 1)
                self.mesh.translate(x, y, 0)
            self._show_body(True)
            reach = self.robot_half_l + 0.4
            self.heading.setData(pos=np.array([[x, y, self.robot_h], [x + reach * math.cos(yaw),
                                               y + reach * math.sin(yaw), self.robot_h]], np.float32))
        if self.follow.isChecked():
            self.view.opts["center"] = Vector(x, y, 0)
            self.view.update()

    def set_occupancy(self, msg: dict) -> None:
        """2D map under the cloud: occupied light, free dark, unknown clear."""
        if not HAVE_GL:
            return
        from ...core.ros_client import grid_from_msg
        g = msg["_grid"] if "_grid" in msg else grid_from_msg(msg)
        if g is None:
            return
        info = msg.get("info", {})
        res = float(info.get("resolution", 0.05))
        o = info.get("origin", {}).get("position", {})
        rgba = np.zeros(g.shape + (4,), np.uint8)
        free = (g >= 0) & (g < 50)
        occ = g >= 50
        rgba[free] = (44, 50, 64, 200)
        rgba[occ] = (200, 205, 215, 255)
        data = np.ascontiguousarray(rgba.transpose(1, 0, 2))      # [x, y]
        if self.floor is None:
            self.floor = gl.GLImageItem(data, smooth=False)
            self.floor.setGLOptions("translucent")
            self.floor.setDepthValue(-10)       # drawn first: blended lines stay on top
            self.floor.setVisible(self.ck_floor.isChecked())
            self.view.addItem(self.floor)
        else:
            self.floor.setData(data)
        self.floor.resetTransform()
        self.floor.translate(float(o.get("x", 0.0)), float(o.get("y", 0.0)), -0.01)
        self.floor.scale(res, res, 1)

    # ---- fusion layers (same data and calls as the 2D map views) ----

    def layer_on(self, kind: str) -> bool:
        """Whether the main window should feed this costmap ('gcost'/'lcost')."""
        if not HAVE_GL:
            return False
        return (self.ck_gcost if kind == "gcost" else self.ck_lcost).isChecked()

    def set_view_mode(self, mode: str) -> None:
        """Turn the layers on and off for a view; Layers fine-tunes it after."""
        if not HAVE_GL:
            return
        two_d = (self.ck_floor, self.ck_scan, self.ck_path, self.ck_lcost, self.ck_goal, self.ck_wps)
        for a in two_d:
            a.setChecked(mode != "3d")
        self.ck_cloud.setChecked(mode != "2d")
        if mode == "2d":
            self.top_view()

    def set_scan(self, msg: dict, map_tx: float, map_ty: float, map_yaw: float) -> None:
        if not HAVE_GL or not self.ck_scan.isChecked():
            return
        from ...core.ros_client import scan_points
        lx, ly = msg["_xy"] if "_xy" in msg else scan_points(msg)
        c, s = math.cos(map_yaw), math.sin(map_yaw)
        pts = np.empty((len(lx), 3), np.float32)
        pts[:, 0] = lx * c - ly * s + map_tx
        pts[:, 1] = lx * s + ly * c + map_ty
        pts[:, 2] = Z_SCAN
        self.scan_pts.setData(pos=pts)

    def set_path(self, msg: dict) -> None:
        if not HAVE_GL:
            return
        poses = msg.get("poses", [])
        pts = np.array([[p["pose"]["position"]["x"], p["pose"]["position"]["y"], Z_PATH] for p in poses],
                       np.float32).reshape(-1, 3)
        self.path_line.setData(pos=pts)

    def set_goal(self, x, y, yaw=0.0) -> None:
        if not HAVE_GL:
            return
        self._goal = None if x is None else (float(x), float(y), float(yaw or 0.0))
        if self._goal is not None:
            self.goal_arrow.setData(pos=_arrow_segments(*self._goal, Z_MARK))
        self.goal_arrow.setVisible(self._goal is not None and self.ck_goal.isChecked())

    def set_waypoints(self, pts: list) -> None:
        """pts: (x, y), (x, y, label) or (x, y, label, yaw), as for MapView."""
        if not HAVE_GL:
            return
        for t in self.wp_labels:
            self.view.removeItem(t)
        self.wp_labels = []
        xyz = np.array([[float(p[0]), float(p[1]), Z_MARK] for p in pts], np.float32).reshape(-1, 3)
        self.wp_pts.setData(pos=xyz)
        heads = [_arrow_segments(float(p[0]), float(p[1]), float(p[3]), Z_MARK, 0.6)
                 for p in pts if len(p) >= 4 and p[3] is not None]
        if heads:
            self.wp_heads.setData(pos=np.concatenate(heads))
        self.wp_heads.setVisible(bool(heads) and self.ck_wps.isChecked())
        for p in pts:
            if len(p) >= 3 and p[2]:
                t = gl.GLTextItem(pos=(float(p[0]), float(p[1]), 0.25), text=str(p[2]),
                                  color=(255, 181, 71, 255), font=QFont("Sans", 11))
                t.setVisible(self.ck_wps.isChecked())
                self.view.addItem(t)
                self.wp_labels.append(t)

    def _show_waypoints(self, on: bool) -> None:
        self.wp_pts.setVisible(on)
        self.wp_heads.setVisible(on and len(self.wp_pts.pos) > 0)
        for t in self.wp_labels:
            t.setVisible(on)

    def _cost_item(self, attr: str, on: bool):
        item = getattr(self, attr)
        if item is None:
            item = gl.GLImageItem(np.zeros((1, 1, 4), np.uint8), smooth=False)
            item.setGLOptions("translucent")
            item.setDepthValue(-9 if attr == "gcost" else -8)
            item.setVisible(on)
            self.view.addItem(item)
            setattr(self, attr, item)
        return item

    def _set_cost(self, attr: str, on: bool, z: float, msg: dict, frame_pose) -> None:
        from ...core.ros_client import cost_rgba, grid_from_msg
        info = msg.get("info", {})
        rgba = msg.get("_rgba")
        if rgba is None:
            g = grid_from_msg(msg)
            if g is None:
                return
            rgba = cost_rgba(g)
        item = self._cost_item(attr, on)
        item.setData(np.ascontiguousarray(rgba.transpose(1, 0, 2)))     # [x, y]
        res = float(info.get("resolution", 0.05))
        origin = info.get("origin", {})
        o, q = origin.get("position", {}), origin.get("orientation", {})
        oyaw = math.atan2(2 * q.get("w", 1.0) * q.get("z", 0.0), 1 - 2 * q.get("z", 0.0) ** 2)
        fx, fy, fyaw = frame_pose
        # pixel -> costmap frame (origin, resolution) -> map frame (frame_pose)
        tr = Transform3D()
        tr.translate(fx, fy, z)
        tr.rotate(math.degrees(fyaw), 0, 0, 1)
        tr.translate(float(o.get("x", 0.0)), float(o.get("y", 0.0)), 0)
        tr.rotate(math.degrees(oyaw), 0, 0, 1)
        tr.scale(res, res, 1)
        item.setTransform(tr)

    def set_global_costmap(self, msg: dict, frame_pose=(0.0, 0.0, 0.0)) -> None:
        if HAVE_GL:
            self._set_cost("gcost", self.ck_gcost.isChecked(), Z_GCOST, msg, frame_pose)

    def set_local_costmap(self, msg: dict, frame_pose=(0.0, 0.0, 0.0)) -> None:
        if HAVE_GL:
            self._set_cost("lcost", self.ck_lcost.isChecked(), Z_LCOST, msg, frame_pose)

    # ---- goals on the floor ----

    def _mouse_mode_changed(self, _i: int) -> None:
        self.view.cancel_aim()
        self.view.mode = self.mouse_mode.currentData()

    def _show_aim(self, pose) -> None:
        if pose is None:
            self.aim_arrow.hide()
            return
        self.aim_arrow.setData(pos=_arrow_segments(*pose, Z_MARK + 0.02))
        self.aim_arrow.show()

    def _aimed(self, mode: str, x: float, y: float, yaw: float) -> None:
        {"goal": self.goal_requested, "pose": self.pose_requested,
         "waypoint": self.waypoint_requested}[mode].emit(x, y, yaw)

    # ---- camera presets ----

    def top_view(self) -> None:
        if HAVE_GL:
            self.view.setCameraPosition(elevation=90, azimuth=-90)

    def perspective_view(self) -> None:
        if HAVE_GL:
            self.view.setCameraPosition(elevation=40, azimuth=-120)

    def fit_map(self) -> None:
        if not HAVE_GL:
            return
        xyz, _, _ = self.cloud_map.snapshot()
        if len(xyz) == 0:
            return
        lo, hi = np.percentile(xyz, 1, axis=0), np.percentile(xyz, 99, axis=0)
        c = (lo + hi) / 2
        self.follow.setChecked(False)
        self.view.setCameraPosition(pos=Vector(float(c[0]), float(c[1]), 0.0),
                                    distance=float(max(hi[0] - lo[0], hi[1] - lo[1], 4.0)) * 1.1)

    # ---- files ----

    def _open_ply(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open 3D map", "", "PLY point cloud (*.ply)")
        if path:
            self.load_ply(path)

    def load_ply(self, path: str) -> int:
        try:
            n = self.cloud_map.load_ply(path)
        except Exception as e:  # noqa: BLE001
            QMessageBox.warning(self, "Open 3D map", f"Cannot read {path}:\n{e}")
            return 0
        # a loaded map is not live: stop adding to it unless the user resumes
        self.rec_btn.setChecked(False)
        self._redraw_cloud(force=True)
        self.fit_map()
        return n

    def _save_ply(self) -> None:
        if len(self.cloud_map) == 0:
            QMessageBox.information(self, "Save 3D map", "The 3D map is empty.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save 3D map", "map3d.ply", "PLY point cloud (*.ply)")
        if not path:
            return
        if not path.lower().endswith(".ply"):
            path += ".ply"
        try:
            n = self.cloud_map.save_ply(path)
        except OSError as e:
            QMessageBox.warning(self, "Save 3D map", f"Cannot write {path}:\n{e}")
            return
        self.status.setText(f"saved {n:,} points to {path}")

    def _clear(self) -> None:
        self.cloud_map.clear()
        self._traj = []
        self._traj_dirty = True
        self._redraw_cloud(force=True)

    # kept for older callers/tests: feed one PointCloud2 straight into the map
    def update_pointcloud(self, msg: dict) -> None:
        out = cm.decode_pointcloud2(msg)
        if out is None or len(out[0]) == 0:
            return
        self.cloud_map.integrate(*out)
        self._redraw_cloud(force=True)
