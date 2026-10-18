"""2D map view: OccupancyGrid + scan overlay + robot pose + click emits map coords.

Pyqtgraph-based so pan/zoom are built in. Overlays can be toggled.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QTransform

import pyqtgraph as pg


# occupancy -> rgba. Build once for the whole module.
def _build_lut() -> np.ndarray:
    lut = np.zeros((256, 4), dtype=np.uint8)
    # Unknown (-1 → 255 after int8 wrap) -> gray
    lut[255] = (90, 95, 110, 255)
    for v in range(0, 101):
        shade = int(240 - v * 2.2)
        lut[v] = (shade, shade, shade, 255)
    return lut


_LUT = _build_lut()


class MapView(pg.PlotWidget):
    goal_requested = Signal(float, float, float)   # x, y, yaw (map frame)
    pose_requested = Signal(float, float, float)   # x, y, yaw — set initial pose
    pose_clicked = Signal(float, float)             # x, y — for mission waypoints

    def __init__(self, parent=None):
        super().__init__(parent, background="#0f1115")
        self.setAspectLocked(True)
        self.getPlotItem().hideButtons()
        self.getPlotItem().setMenuEnabled(False)
        self.getAxis("bottom").setPen(pg.mkPen(0.3))
        self.getAxis("left").setPen(pg.mkPen(0.3))
        self.getAxis("bottom").setTextPen(pg.mkPen(0.6))
        self.getAxis("left").setTextPen(pg.mkPen(0.6))

        self.map_img = pg.ImageItem()
        self.map_img.setLookupTable(_LUT)
        self.map_img.setLevels([0, 255])
        self.addItem(self.map_img)

        self.cost_img = pg.ImageItem()
        self.cost_img.setZValue(5)
        self.cost_img.setOpacity(0.45)
        self.cost_img.setVisible(False)
        self.addItem(self.cost_img)

        self.local_cost_img = pg.ImageItem()
        self.local_cost_img.setZValue(6)
        self.local_cost_img.setOpacity(0.55)
        self.local_cost_img.setVisible(False)
        self.addItem(self.local_cost_img)

        self.scan_scatter = pg.ScatterPlotItem(
            size=3, pen=pg.mkPen(None), brush=pg.mkBrush("#ff6a6a"))
        self.scan_scatter.setZValue(10)
        self.addItem(self.scan_scatter)

        self.path_line = pg.PlotDataItem(pen=pg.mkPen("#39d98a", width=2))
        self.path_line.setZValue(8)
        self.addItem(self.path_line)

        self.wp_scatter = pg.ScatterPlotItem(
            size=12, pen=pg.mkPen("#ffb547", width=2), brush=pg.mkBrush("#ffb547"))
        self.wp_scatter.setZValue(11)
        self.addItem(self.wp_scatter)

        self.robot_body = pg.PlotDataItem(
            pen=pg.mkPen("#5a9df6", width=2), fillLevel=None)
        self.robot_body.setZValue(20)
        self.addItem(self.robot_body)
        self.robot_arrow = pg.PlotDataItem(pen=pg.mkPen("#5a9df6", width=2))
        self.robot_arrow.setZValue(21)
        self.addItem(self.robot_arrow)

        self.goal_marker = pg.ScatterPlotItem(
            size=18, pen=pg.mkPen("#39d98a", width=2), brush=pg.mkBrush(0, 0, 0, 0))
        self.goal_marker.setZValue(15)
        self.addItem(self.goal_marker)

        self._map_origin = (0.0, 0.0)
        self._map_res = 0.05
        self._robot_half_l = 0.45
        self._robot_half_w = 0.30
        self._click_mode = "goal"   # 'goal' | 'waypoint' | 'pose'
        self._pending: Optional[tuple] = None   # for drag-to-set-yaw

        self.scene().sigMouseClicked.connect(self._on_click)

        self._robot_xy = None
        self._fitted = False
        # floating view buttons (top-left corner of the map)
        from PySide6.QtWidgets import QPushButton
        css = ("QPushButton { background: rgba(21,25,35,215); border: 1px solid #2a3142;"
               " border-radius: 6px; padding: 5px 10px; font-size: 12px; }"
               " QPushButton:hover { background: rgba(40,48,68,235); }")
        self.btn_robot = QPushButton("⌖  Center on robot", self)
        self.btn_map = QPushButton("⤢  Whole map", self)
        for i, b in enumerate((self.btn_robot, self.btn_map)):
            b.setStyleSheet(css)
            b.setCursor(Qt.PointingHandCursor)
            b.adjustSize()
        self.btn_robot.clicked.connect(lambda: self.center_on_robot())
        self.btn_map.clicked.connect(self.fit_map)
        self._place_buttons()

    # ---- public setters ----

    def set_robot_footprint(self, half_length: float, half_width: float) -> None:
        self._robot_half_l = half_length
        self._robot_half_w = half_width

    def set_click_mode(self, mode: str) -> None:
        self._click_mode = mode

    def show_global_costmap(self, on: bool) -> None:
        self.cost_img.setVisible(on)

    def show_local_costmap(self, on: bool) -> None:
        self.local_cost_img.setVisible(on)

    # ---- data slots ----

    def set_occupancy(self, msg: dict) -> None:
        """Apply nav_msgs/OccupancyGrid to the base map image."""
        self._apply_grid(msg, self.map_img, update_origin=True)

    def set_global_costmap(self, msg: dict, frame_pose=(0.0, 0.0, 0.0)) -> None:
        self._apply_cost_grid(msg, self.cost_img, frame_pose)

    def set_local_costmap(self, msg: dict, frame_pose=(0.0, 0.0, 0.0)) -> None:
        """frame_pose: pose of the costmap's frame (usually odom) in the map frame."""
        self._apply_cost_grid(msg, self.local_cost_img, frame_pose)

    def set_scan(self, msg: dict, map_tx: float, map_ty: float, map_yaw: float) -> None:
        """(map_tx, map_ty, map_yaw) is the pose of the LASER frame in the map frame.
        The main window composes map→base_link ∘ base_link→laser so we only do a
        rigid 2D transform of the beams here."""
        ranges = msg.get("ranges", [])
        if not ranges:
            self.scan_scatter.setData([], [])
            return
        a0 = float(msg.get("angle_min", 0.0))
        da = float(msg.get("angle_increment", 0.0))
        rmin = float(msg.get("range_min", 0.0))
        rmax = float(msg.get("range_max", 100.0))
        # rosbridge encodes inf/NaN (no return) as JSON null -> None
        arr = np.array([np.inf if r is None else r for r in ranges], dtype=np.float32)
        angles = a0 + np.arange(arr.size, dtype=np.float32) * da
        mask = np.isfinite(arr) & (arr >= rmin) & (arr <= rmax)
        arr = arr[mask]; angles = angles[mask]
        xs = arr * np.cos(angles + map_yaw) + map_tx
        ys = arr * np.sin(angles + map_yaw) + map_ty
        self.scan_scatter.setData(xs, ys)

    def resizeEvent(self, ev) -> None:
        super().resizeEvent(ev)
        self._place_buttons()

    def _place_buttons(self) -> None:
        if hasattr(self, "btn_robot"):
            self.btn_robot.move(60, 10)
            self.btn_map.move(60 + self.btn_robot.width() + 8, 10)

    def center_on_robot(self, span: float = 10.0) -> None:
        if self._robot_xy is None:
            return
        x, y = self._robot_xy
        self.getPlotItem().vb.setRange(xRange=(x - span / 2, x + span / 2),
                                       yRange=(y - span / 2, y + span / 2), padding=0)

    def fit_map(self) -> None:
        if self.map_img.image is None:
            return
        self.getPlotItem().vb.setRange(rect=self.map_img.mapRectToParent(self.map_img.boundingRect()),
                                       padding=0.02)

    def set_robot_pose(self, x: float, y: float, yaw: float) -> None:
        self._robot_xy = (x, y)
        hl, hw = self._robot_half_l, self._robot_half_w
        corners = np.array([
            [hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw], [hl, hw]
        ], dtype=np.float32)
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s], [s, c]], dtype=np.float32)
        pts = corners @ R.T + np.array([x, y])
        # pyqtgraph needs contiguous arrays: column views of pts (strided)
        # corrupted the heap inside its QPainterPath fast path.
        self.robot_body.setData(np.ascontiguousarray(pts[:, 0]), np.ascontiguousarray(pts[:, 1]))
        # heading arrow
        reach = self._robot_half_l + 0.35      # sticks out past the front
        self.robot_arrow.setData(
            np.array([x, x + reach * c]), np.array([y, y + reach * s]))

    def set_path(self, msg: dict) -> None:
        poses = msg.get("poses", [])
        if not poses:
            self.path_line.setData([], [])
            return
        xs = np.array([p["pose"]["position"]["x"] for p in poses], dtype=np.float32)
        ys = np.array([p["pose"]["position"]["y"] for p in poses], dtype=np.float32)
        self.path_line.setData(xs, ys)

    def set_waypoints(self, pts: list) -> None:
        """pts: (x, y) or (x, y, label) tuples, drawn in order with labels."""
        for t in getattr(self, "_wp_labels", []):
            self.removeItem(t)
        self._wp_labels = []
        if not pts:
            self.wp_scatter.setData([], [])
            return
        xs = [float(p[0]) for p in pts]
        ys = [float(p[1]) for p in pts]
        self.wp_scatter.setData(xs, ys)
        for p in pts:
            if len(p) < 3 or not p[2]:
                continue
            t = pg.TextItem(str(p[2]), color="#ffb547", anchor=(-0.15, 0.5),
                            fill=pg.mkBrush(15, 17, 21, 190))
            t.setPos(float(p[0]), float(p[1]))
            t.setZValue(12)
            self.addItem(t)
            self._wp_labels.append(t)

    def set_goal(self, x: float | None, y: float | None) -> None:
        if x is None:
            self.goal_marker.setData([], [])
        else:
            self.goal_marker.setData([x], [y])

    # ---- internals ----

    def _apply_grid(self, msg: dict, item: pg.ImageItem, update_origin: bool = False) -> None:
        info = msg.get("info", {})
        w = int(info.get("width", 0))
        h = int(info.get("height", 0))
        if w <= 0 or h <= 0:
            return
        res = float(info.get("resolution", 0.05))
        origin = info.get("origin", {}).get("position", {})
        ox = float(origin.get("x", 0.0))
        oy = float(origin.get("y", 0.0))
        data = np.asarray(msg.get("data", []), dtype=np.int8)
        if data.size != w * h:
            return
        grid = data.reshape((h, w)).astype(np.uint8)   # -1 wraps to 255 (unknown)
        item.setImage(grid.T, autoLevels=False, levels=[0, 255])
        item.setRect(ox, oy, w * res, h * res)
        if update_origin:
            self._map_origin = (ox, oy)
            self._map_res = res
            # fit the first map only; later updates (SLAM) keep the user's view
            if not self._fitted:
                self._fitted = True
                self.getPlotItem().vb.disableAutoRange()
                self.fit_map()

    def _apply_cost_grid(self, msg: dict, item: pg.ImageItem, frame_pose=(0.0, 0.0, 0.0)) -> None:
        info = msg.get("info", {})
        w = int(info.get("width", 0))
        h = int(info.get("height", 0))
        if w <= 0 or h <= 0:
            return
        res = float(info.get("resolution", 0.05))
        origin = info.get("origin", {})
        opos = origin.get("position", {})
        oq = origin.get("orientation", {})
        oyaw = math.atan2(2 * (oq.get("w", 1.0) * oq.get("z", 0.0)), 1 - 2 * oq.get("z", 0.0) ** 2)
        data = np.asarray(msg.get("data", []), dtype=np.int8)
        if data.size != w * h:
            return
        c = data.reshape((h, w)).astype(np.int16)
        # Nav2 costs: 100 lethal, 99 inscribed, 1..98 inflation, 0 free, -1 unknown.
        # Inflation fades in with cost so low-cost space stays readable;
        # obstacles (inscribed/lethal) are solid magenta.
        rgba = np.zeros((h, w, 4), dtype=np.uint8)
        infl = (c > 0) & (c < 99)
        rgba[infl, 0], rgba[infl, 1], rgba[infl, 2] = 70, 130, 255
        rgba[infl, 3] = np.clip(c[infl] * 1.6, 0, 150).astype(np.uint8)
        leth = c >= 99
        rgba[leth] = (230, 40, 140, 210)
        item.setImage(rgba.transpose(1, 0, 2), autoLevels=False)
        # pixel -> costmap frame (origin, resolution) -> map frame (frame_pose)
        fx, fy, fyaw = frame_pose
        tr = QTransform()
        tr.translate(fx, fy)
        tr.rotate(math.degrees(fyaw))
        tr.translate(float(opos.get("x", 0.0)), float(opos.get("y", 0.0)))
        tr.rotate(math.degrees(oyaw))
        tr.scale(res, res)
        item.setTransform(tr)

    def _on_click(self, ev) -> None:
        if ev.button() != Qt.LeftButton:
            return
        pos = ev.scenePos()
        mp = self.getPlotItem().vb.mapSceneToView(pos)
        x, y = float(mp.x()), float(mp.y())
        if self._click_mode in ("goal", "pose"):
            if self._pending is None:
                self._pending = (x, y)
                self.set_goal(x, y)
            else:
                x0, y0 = self._pending
                yaw = math.atan2(y - y0, x - x0)
                self._pending = None
                self.set_goal(x0, y0)
                if self._click_mode == "goal":
                    self.goal_requested.emit(x0, y0, yaw)
                else:
                    self.pose_requested.emit(x0, y0, yaw)
                    self.set_goal(None, None)
        elif self._click_mode == "waypoint":
            self.pose_clicked.emit(x, y)
