"""3D view: pyqtgraph.opengl GLViewWidget rendering sensor_msgs/PointCloud2 and robot box."""
from __future__ import annotations

import math
import struct

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget

try:
    import pyqtgraph.opengl as gl
    HAVE_GL = True
except Exception:  # noqa: BLE001
    HAVE_GL = False


class View3DPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(8, 8, 8, 8)

        if not HAVE_GL:
            lbl = QLabel("3D view requires PyOpenGL + pyqtgraph.opengl.\n"
                         "Install: pip install PyOpenGL pyqtgraph")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("color:#8b93a7;")
            v.addWidget(lbl)
            return

        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=8, elevation=22, azimuth=-70)
        self.view.setBackgroundColor((15, 17, 21))
        grid = gl.GLGridItem(); grid.setSize(20, 20); grid.setSpacing(1, 1); grid.setColor((60, 65, 80, 160))
        self.view.addItem(grid)
        # frame axes at origin
        self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [1, 0, 0]]),
                                            color=(1, 0.3, 0.3, 1), width=2, antialias=True))
        self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 1, 0]]),
                                            color=(0.3, 1, 0.3, 1), width=2, antialias=True))
        self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 1]]),
                                            color=(0.3, 0.5, 1, 1), width=2, antialias=True))
        v.addWidget(self.view, 1)

        row = QHBoxLayout()
        self.ck_cloud = QCheckBox("Point cloud"); self.ck_cloud.setChecked(True)
        self.ck_robot = QCheckBox("Robot box"); self.ck_robot.setChecked(True)
        self.ck_cloud.toggled.connect(lambda b: self.scatter.setVisible(b) if self.scatter else None)
        row.addWidget(self.ck_cloud); row.addWidget(self.ck_robot); row.addStretch(1)
        v.addLayout(row)

        self.scatter = gl.GLScatterPlotItem(size=2, color=(0.8, 0.9, 1.0, 0.8), pxMode=True)
        self.view.addItem(self.scatter)

        # robot box (will be positioned on update_robot_pose)
        self.box: gl.GLBoxItem | None = None
        self.robot_half_l = 0.45
        self.robot_half_w = 0.30
        self.robot_h = 0.30

    def update_pointcloud(self, msg: dict) -> None:
        if not HAVE_GL:
            return
        pts = _decode_pointcloud2(msg)
        if pts is None or pts.size == 0:
            return
        # downsample to keep it snappy
        if pts.shape[0] > 60000:
            idx = np.random.choice(pts.shape[0], 60000, replace=False)
            pts = pts[idx]
        self.scatter.setData(pos=pts, color=(0.75, 0.9, 1.0, 0.85), size=2)

    def set_robot_footprint(self, half_l: float, half_w: float) -> None:
        self.robot_half_l = half_l
        self.robot_half_w = half_w

    def update_robot_pose(self, x: float, y: float, yaw: float) -> None:
        if not HAVE_GL or not self.ck_robot.isChecked():
            if self.box is not None:
                self.box.hide()
            return
        if self.box is None:
            self.box = gl.GLBoxItem()
            self.box.setColor((90, 157, 246, 255))
            self.view.addItem(self.box)
            self.box.setSize(2 * self.robot_half_l, 2 * self.robot_half_w, self.robot_h)
        self.box.resetTransform()
        self.box.translate(-self.robot_half_l, -self.robot_half_w, 0)
        self.box.rotate(math.degrees(yaw), 0, 0, 1)
        self.box.translate(x, y, 0)
        self.box.show()


# -------- PointCloud2 decoder (XYZ only, little-endian, per REP 118) --------

def _decode_pointcloud2(msg: dict) -> np.ndarray | None:
    try:
        n = int(msg["width"]) * int(msg["height"])
        step = int(msg["point_step"])
        fields = msg["fields"]
    except (KeyError, TypeError, ValueError):
        return None
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)

    data = msg.get("data", b"")
    if isinstance(data, str):
        import base64
        data = base64.b64decode(data)
    data = bytes(data)
    if len(data) < n * step:
        return None

    offx = offy = offz = None
    for f in fields:
        if f["name"] == "x":
            offx = int(f["offset"])
        elif f["name"] == "y":
            offy = int(f["offset"])
        elif f["name"] == "z":
            offz = int(f["offset"])
    if offx is None or offy is None or offz is None:
        return None

    arr = np.frombuffer(data[: n * step], dtype=np.uint8).reshape(n, step)
    def _col(off):
        return arr[:, off: off + 4].view(np.float32).reshape(-1)
    xs = _col(offx)
    ys = _col(offy)
    zs = _col(offz)
    pts = np.stack([xs, ys, zs], axis=1)
    mask = np.isfinite(pts).all(axis=1)
    return pts[mask]
