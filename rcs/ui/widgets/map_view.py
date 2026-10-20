"""2D map view: OccupancyGrid + scan overlay + robot pose + goal arrows.

Pyqtgraph-based so pan/zoom are built in. Overlays can be toggled.

Goals, initial poses and waypoints are set like in RViz: press where the
robot should be and drag toward the heading; an arrow follows the mouse and
the pose is sent on release. A plain click (no drag) pins the position and
the arrow then follows the mouse until a second click. Right-click or Esc
cancels. Right- or middle-drag pans, the wheel zooms.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np
from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QPen, QPolygonF, QTransform
from PySide6.QtWidgets import QGraphicsPathItem, QGraphicsPolygonItem

import pyqtgraph as pg

from ...core.ros_client import cost_rgba, grid_from_msg, scan_points


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


def _arrow_polygon(x: float, y: float, yaw: float, length: float) -> QPolygonF:
    """Filled arrow from (x, y) pointing along yaw, `length` long (map units)."""
    shaft_w = length * 0.11
    head_l, head_w = length * 0.38, length * 0.30
    body = length - head_l
    pts = [(0.0, shaft_w / 2), (body, shaft_w / 2), (body, head_w / 2), (length, 0.0),
           (body, -head_w / 2), (body, -shaft_w / 2), (0.0, -shaft_w / 2)]
    c, s = math.cos(yaw), math.sin(yaw)
    return QPolygonF([QPointF(x + px * c - py * s, y + px * s + py * c) for px, py in pts])


class _Arrow(QGraphicsPolygonItem):
    """Arrow drawn in map coordinates; its size follows the zoom so it stays
    readable (about `px` screen pixels long)."""

    def __init__(self, color: str, z: float, px: float = 64.0):
        super().__init__()
        col = QColor(color)
        self.setPen(QPen(col.darker(160), 0))
        fill = QColor(col)
        fill.setAlpha(225)
        self.setBrush(QBrush(fill))
        self.setZValue(z)
        self.px = px
        self.pose: Optional[tuple] = None
        self.setVisible(False)

    def set_pose(self, pose: Optional[tuple], units_per_px: float) -> None:
        self.pose = pose
        if pose is None:
            self.setVisible(False)
            return
        self.setPolygon(_arrow_polygon(pose[0], pose[1], pose[2], self.px * units_per_px))
        self.setVisible(True)

    def rescale(self, units_per_px: float) -> None:
        if self.pose is not None:
            self.set_pose(self.pose, units_per_px)


class _Dots(QGraphicsPathItem):
    """Many same-size dots as one path of zero-length round-capped segments.
    ScatterPlotItem keeps per-point Python objects and took ~45 ms per
    5000-point scan on the GUI thread; this builds the path vectorised."""

    def __init__(self, color: str, size: float, z: float):
        super().__init__()
        pen = QPen(QColor(color), size)
        pen.setCosmetic(True)
        pen.setCapStyle(Qt.RoundCap)
        self.setPen(pen)
        self.setZValue(z)
        self._xy = (np.empty(0, np.float32), np.empty(0, np.float32))

    def setData(self, xs, ys) -> None:
        xs = np.asarray(xs, np.float64)
        ys = np.asarray(ys, np.float64)
        self._xy = (xs, ys)
        if xs.size == 0:
            self.setPath(pg.QtGui.QPainterPath())
            return
        self.setPath(pg.arrayToQPath(np.repeat(xs, 2), np.repeat(ys, 2), connect="pairs"))

    def getData(self):
        return self._xy

    def boundingRect(self):
        # include the pen so dots on the edge are not clipped
        return super().boundingRect().adjusted(-1, -1, 1, 1)


class MapView(pg.PlotWidget):
    goal_requested = Signal(float, float, float)   # x, y, yaw (map frame)
    pose_requested = Signal(float, float, float)   # x, y, yaw — set initial pose
    waypoint_requested = Signal(float, float, float)   # x, y, yaw — waypoint with heading

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

        self.scan_scatter = _Dots("#ff6a6a", 3, 10)
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

        # current Nav2 goal (green) and the arrow being dragged (amber / violet)
        self.goal_arrow = _Arrow("#39d98a", 15)
        self.addItem(self.goal_arrow)
        self.aim_arrow = _Arrow("#ffb547", 30)
        self.addItem(self.aim_arrow)
        self.wp_arrows: list = []
        self.getPlotItem().vb.sigRangeChanged.connect(self._rescale_arrows)

        self._map_origin = (0.0, 0.0)
        self._map_res = 0.05
        self._robot_half_l = 0.45
        self._robot_half_w = 0.30
        self._click_mode = "goal"   # 'goal' | 'waypoint' | 'pose'
        self._anchor: Optional[tuple] = None    # (x, y) pinned position while aiming
        self._press_px: Optional[QPointF] = None
        self._aiming = False                    # click-then-click: arrow follows the mouse
        self._pan_last = None                   # right/middle-drag pan in progress
        self.setMouseTracking(True)
        self.viewport().setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

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
        self.cancel_aim()

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
        lx, ly = msg["_xy"] if "_xy" in msg else scan_points(msg)   # laser frame
        if not len(lx):
            self.scan_scatter.setData([], [])
            return
        c, s = math.cos(map_yaw), math.sin(map_yaw)
        self.scan_scatter.setData(lx * c - ly * s + map_tx, lx * s + ly * c + map_ty)

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
        """pts: (x, y), (x, y, label) or (x, y, label, yaw) tuples, drawn in
        order with labels and, when yaw is given, a heading arrow."""
        for t in getattr(self, "_wp_labels", []):
            self.removeItem(t)
        self._wp_labels = []
        for a in self.wp_arrows:
            self.removeItem(a)
        self.wp_arrows = []
        if not pts:
            self.wp_scatter.setData([], [])
            return
        xs = [float(p[0]) for p in pts]
        ys = [float(p[1]) for p in pts]
        self.wp_scatter.setData(xs, ys)
        upp = self._units_per_px()
        for p in pts:
            if len(p) >= 4 and p[3] is not None:
                a = _Arrow("#ffb547", 11, px=34)
                a.setVisible(self.wp_scatter.isVisible())
                self.addItem(a)
                a.set_pose((float(p[0]), float(p[1]), float(p[3])), upp)
                self.wp_arrows.append(a)
            if len(p) < 3 or not p[2]:
                continue
            t = pg.TextItem(str(p[2]), color="#ffb547", anchor=(-0.15, 0.5),
                            fill=pg.mkBrush(15, 17, 21, 190))
            t.setPos(float(p[0]), float(p[1]))
            t.setZValue(12)
            self.addItem(t)
            self._wp_labels.append(t)

    def show_waypoints(self, on: bool) -> None:
        self.wp_scatter.setVisible(on)
        for a in self.wp_arrows:
            a.setVisible(on)
        for t in getattr(self, "_wp_labels", []):
            t.setVisible(on)

    def set_goal(self, x: float | None, y: float | None, yaw: float | None = 0.0) -> None:
        """Show the current goal as an arrow (None clears it)."""
        if x is None:
            self.goal_arrow.set_pose(None, 0)
        else:
            self.goal_arrow.set_pose((float(x), float(y), float(yaw or 0.0)), self._units_per_px())

    # ---- arrows / aiming ----

    def _units_per_px(self) -> float:
        try:
            pw, ph = self.getPlotItem().vb.viewPixelSize()
            return float(max(pw, ph)) or 0.02
        except Exception:  # noqa: BLE001  (view not laid out yet)
            return 0.02

    def _rescale_arrows(self, *_) -> None:
        upp = self._units_per_px()
        for a in [self.goal_arrow, self.aim_arrow] + self.wp_arrows:
            a.rescale(upp)

    _AIM_COLORS = {"goal": "#ffb547", "pose": "#b18cff", "waypoint": "#ffd27a"}

    def _view_xy(self, ev) -> tuple:
        pos = ev.position() if hasattr(ev, "position") else ev.localPos()
        mp = self.getPlotItem().vb.mapSceneToView(self.mapToScene(pos.toPoint()))
        return float(mp.x()), float(mp.y())

    def _in_plot(self, ev) -> bool:
        pos = ev.position() if hasattr(ev, "position") else ev.localPos()
        vb = self.getPlotItem().vb
        return vb.sceneBoundingRect().contains(self.mapToScene(pos.toPoint()))

    def _update_aim(self, x: float, y: float) -> None:
        ax, ay = self._anchor
        yaw = math.atan2(y - ay, x - ax) if math.hypot(x - ax, y - ay) > 1e-6 else (
            self.aim_arrow.pose[2] if self.aim_arrow.pose else 0.0)
        self.aim_arrow.set_pose((ax, ay, yaw), self._units_per_px())

    def cancel_aim(self) -> None:
        self._anchor = None
        self._press_px = None
        self._aiming = False
        self.aim_arrow.set_pose(None, 0)

    def _finish_aim(self) -> None:
        x, y, yaw = self.aim_arrow.pose
        mode = self._click_mode
        self.cancel_aim()
        if mode == "goal":
            self.set_goal(x, y, yaw)
            self.goal_requested.emit(x, y, yaw)
        elif mode == "pose":
            self.pose_requested.emit(x, y, yaw)
        elif mode == "waypoint":
            self.waypoint_requested.emit(x, y, yaw)

    def mousePressEvent(self, ev) -> None:
        if ev.button() == Qt.RightButton and self._anchor is not None:
            self.cancel_aim()
            ev.accept()
            return
        if ev.button() in (Qt.RightButton, Qt.MiddleButton) and self._in_plot(ev):
            self._pan_last = self._view_xy(ev)
            ev.accept()
            return
        if ev.button() != Qt.LeftButton or not self._in_plot(ev):
            super().mousePressEvent(ev)
            return
        x, y = self._view_xy(ev)
        if self._aiming:                      # second click of click-then-click
            self._update_aim(x, y)
            self._finish_aim()
            ev.accept()
            return
        self._anchor = (x, y)
        self._press_px = ev.position() if hasattr(ev, "position") else ev.localPos()
        self.aim_arrow.setBrush(QBrush(QColor(self._AIM_COLORS.get(self._click_mode, "#ffb547"))))
        self.aim_arrow.set_pose((x, y, 0.0), self._units_per_px())
        self.aim_arrow.setVisible(False)      # shown once the drag has a direction
        ev.accept()

    def mouseMoveEvent(self, ev) -> None:
        if self._pan_last is not None:
            x, y = self._view_xy(ev)
            self.getPlotItem().vb.translateBy(x=self._pan_last[0] - x, y=self._pan_last[1] - y)
            self._pan_last = self._view_xy(ev)     # same screen point, panned view
            ev.accept()
            return
        if self._anchor is None:
            super().mouseMoveEvent(ev)
            return
        x, y = self._view_xy(ev)
        self._update_aim(x, y)
        ev.accept()

    def mouseReleaseEvent(self, ev) -> None:
        if self._pan_last is not None and ev.button() in (Qt.RightButton, Qt.MiddleButton):
            self._pan_last = None
            ev.accept()
            return
        if self._anchor is None or self._aiming or ev.button() != Qt.LeftButton:
            super().mouseReleaseEvent(ev)
            return
        pos = ev.position() if hasattr(ev, "position") else ev.localPos()
        moved = (pos - self._press_px).manhattanLength() if self._press_px is not None else 0
        if moved >= 8:                        # press-drag-release: done
            x, y = self._view_xy(ev)
            self._update_aim(x, y)
            self._finish_aim()
        else:                                 # plain click: aim with the mouse, click again
            self._aiming = True
            self.aim_arrow.setVisible(True)
        ev.accept()

    def keyPressEvent(self, ev) -> None:
        if ev.key() == Qt.Key_Escape and self._anchor is not None:
            self.cancel_aim()
            ev.accept()
            return
        super().keyPressEvent(ev)

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
        g = msg["_grid"] if "_grid" in msg else grid_from_msg(msg)
        if g is None or g.shape != (h, w):
            return
        grid = g.astype(np.uint8)   # -1 wraps to 255 (unknown)
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
        rgba = msg.get("_rgba")
        if rgba is None:
            g = grid_from_msg(msg)
            if g is None:
                return
            rgba = cost_rgba(g)
        if rgba.shape[:2] != (h, w):
            return
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
