"""3D map page: colored point map in the map frame, like the 3D view of the
report.html that bringup's save_run writes.

The map itself (rcs.core.cloud_map.CloudMap) is built by RosClient on its
decoder thread from the camera's depth + color images (or a PointCloud2),
each frame placed with the TF at its own timestamp. This page only draws it,
plus the robot, its trajectory and the 2D map on the floor, and saves/loads
the map as PLY (the same format as the report's map/cloud3d.ply).
"""
from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QHBoxLayout, QLabel,
    QMessageBox, QPushButton, QSlider, QVBoxLayout, QWidget,
)

from ...core import cloud_map as cm

try:
    import pyqtgraph.opengl as gl
    from pyqtgraph import Vector
    HAVE_GL = True
except Exception:  # noqa: BLE001
    HAVE_GL = False

BG = (17, 21, 28)                 # same background as the report viewer
DISPLAY_MAX = 1_500_000           # points drawn at most (the map may hold more)


def _decode_pointcloud2(msg: dict) -> Optional[np.ndarray]:
    """XYZ of a PointCloud2 (kept for callers that only need positions)."""
    out = cm.decode_pointcloud2(msg)
    return None if out is None else out[0]


if HAVE_GL:
    class _GLView(gl.GLViewWidget):
        """Report-style controls: left-drag orbits, right/middle-drag (or
        shift+left) pans, wheel zooms."""

        def mouseMoveEvent(self, ev):
            pos = ev.position() if hasattr(ev, "position") else ev.localPos()
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
        self.heading = gl.GLLinePlotItem(pos=np.zeros((2, 3), np.float32), color=(0.35, 0.62, 0.96, 1),
                                         width=3, antialias=True)
        self.view.addItem(self.heading)
        v.addWidget(self.view, 1)

        # ---- row 1: recording
        r1 = QHBoxLayout()
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
        r1.addStretch(1)
        v.addLayout(r1)

        # ---- row 2: view
        r2 = QHBoxLayout()
        top = QPushButton("Top view"); top.clicked.connect(self.top_view)
        persp = QPushButton("Perspective"); persp.clicked.connect(self.perspective_view)
        fit = QPushButton("Whole map"); fit.clicked.connect(self.fit_map)
        r2.addWidget(top); r2.addWidget(persp); r2.addWidget(fit)
        self.follow = QCheckBox("Follow robot")
        self.follow.setChecked(True)
        self.follow.setToolTip("Keep the robot in the middle of the view (panning turns it off)")
        self.view.on_pan = lambda: self.follow.setChecked(False)
        r2.addWidget(self.follow)
        r2.addSpacing(12)
        r2.addWidget(QLabel("Point size"))
        self.psize = QSlider(Qt.Horizontal)
        self.psize.setRange(1, 8)
        self.psize.setValue(2)
        self.psize.setFixedWidth(110)
        self.psize.valueChanged.connect(lambda _v: self._redraw_cloud(force=True))
        r2.addWidget(self.psize)
        r2.addSpacing(12)
        self.ck_cloud = QCheckBox("3D map"); self.ck_cloud.setChecked(True)
        self.ck_robot = QCheckBox("Robot"); self.ck_robot.setChecked(True)
        self.ck_traj = QCheckBox("Trajectory"); self.ck_traj.setChecked(True)
        self.ck_floor = QCheckBox("2D map on floor"); self.ck_floor.setChecked(True)
        self.ck_grid = QCheckBox("Grid"); self.ck_grid.setChecked(True)
        self.ck_cloud.toggled.connect(self.scatter.setVisible)
        self.ck_robot.toggled.connect(lambda b: (self.box.setVisible(b and self._robot is not None),
                                                 self.heading.setVisible(b)))
        self.ck_traj.toggled.connect(self.traj_line.setVisible)
        self.ck_floor.toggled.connect(lambda b: self.floor.setVisible(b) if self.floor else None)
        self.ck_grid.toggled.connect(self.grid.setVisible)
        for w in (self.ck_cloud, self.ck_robot, self.ck_traj, self.ck_floor, self.ck_grid):
            r2.addWidget(w)
        r2.addStretch(1)
        v.addLayout(r2)

        # ---- row 3: file + status
        r3 = QHBoxLayout()
        op = QPushButton("Open PLY…"); op.clicked.connect(self._open_ply)
        op.setToolTip("Load a saved 3D map, e.g. result/run_*/map/cloud3d.ply from save_run")
        sv = QPushButton("Save PLY…"); sv.clicked.connect(self._save_ply)
        clr = QPushButton("Clear"); clr.clicked.connect(self._clear)
        clr.setProperty("kind", "danger")
        r3.addWidget(op); r3.addWidget(sv); r3.addWidget(clr)
        self.status = QLabel("not recording")
        self.status.setProperty("muted", True)
        r3.addWidget(self.status, 1)
        v.addLayout(r3)

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
        if "error" in st:
            msg = f"⚠ {st['error']}"
        elif self.rec_btn.isChecked():
            hz = (len(self._fps_t) - 1) / max(1e-3, self._fps_t[-1] - self._fps_t[0]) if len(self._fps_t) > 1 else 0
            if hz > 0:
                msg = f"recording · {hz:.1f} frames/s"
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

    def set_robot_footprint(self, half_l: float, half_w: float) -> None:
        self.robot_half_l = half_l
        self.robot_half_w = half_w
        if HAVE_GL:
            self.box.setSize(2 * half_l, 2 * half_w, self.robot_h)

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
            self.box.show()
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
            self.floor.setVisible(self.ck_floor.isChecked())
            self.view.addItem(self.floor)
        else:
            self.floor.setData(data)
        self.floor.resetTransform()
        self.floor.translate(float(o.get("x", 0.0)), float(o.get("y", 0.0)), -0.01)
        self.floor.scale(res, res, 1)

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
