"""The robot seen from above: on the 2D maps, the Fleet cards and the Connection
page. Its body, drive wheels, casters and sensors come from its model
(rcs.core.robot_model); a robot without one is a box of its footprint.

Shapes are QPainterPaths in base_link metres (x forward, y left), made once
per model from its top view and placed with a transform, so a pose update is
cheap.
"""
from __future__ import annotations

import math
from typing import Callable, Dict, Optional

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen, QPolygonF, QTransform
from PySide6.QtWidgets import QGraphicsPathItem, QSizePolicy, QWidget

from ...core import robot_model as rm

BLUE = "#5a9df6"
# fill, outline per part
STYLE = {
    rm.BODY: ((90, 157, 246, 70), BLUE),
    rm.WHEEL: ((28, 32, 42, 235), "#c6cad6"),
    rm.CASTER: ((255, 181, 71, 200), "#7a5a22"),
    rm.SENSOR: ((79, 209, 255, 170), "#1f6f8b"),
}
LEVEL_COLORS = {"ok": "#39d98a", "warn": "#ffb547", "error": "#ff6a6a", "off": "#8b93a7"}


def mask_path(mask: np.ndarray, x0: float, y0: float, cell: float) -> QPainterPath:
    """Outline of the True cells of mask[row, col], in metres. Merged in whole
    cells, where the arithmetic is exact, then scaled: merging in metres left
    seams between the cells."""
    path = QPainterPath()
    padded = np.zeros((mask.shape[0], mask.shape[1] + 2), np.int8)
    padded[:, 1:-1] = mask
    d = np.diff(padded, axis=1)
    for r in range(mask.shape[0]):
        starts = np.nonzero(d[r] == 1)[0]
        ends = np.nonzero(d[r] == -1)[0]
        for s, e in zip(starts, ends):
            path.addRect(QRectF(float(s), float(r), float(e - s), 1.0))
    path = path.simplified()
    return QTransform(cell, 0, 0, cell, x0, y0).map(path)


def box_path(half_l: float, half_w: float) -> QPainterPath:
    p = QPainterPath()
    p.addRect(QRectF(-half_l, -half_w, 2 * half_l, 2 * half_w))
    return p


def paths_for(model: Optional[rm.Model], half_l: float, half_w: float) -> Dict[int, QPainterPath]:
    """Part -> outline. Made once per model."""
    if model is None:
        return {rm.BODY: box_path(half_l, half_w)}
    cached = getattr(model, "_qt_paths", None)
    if cached is None:
        tv = model.top_view()
        cached = {part: mask_path(m, tv.x0, tv.y0, tv.cell) for part, m in tv.masks.items()}
        model._qt_paths = cached
    return cached


def _pen(color, width: float = 2.0) -> QPen:
    pen = QPen(QColor(color))
    pen.setWidthF(width)
    pen.setCosmetic(True)       # pixels on screen, whatever the zoom
    pen.setJoinStyle(Qt.RoundJoin)
    return pen


class RobotShape:
    """The robot on a pyqtgraph map: one QGraphicsPathItem per part."""

    def __init__(self, add_item: Callable, remove_item: Callable, z: float = 20.0) -> None:
        self._add, self._remove, self._z = add_item, remove_item, z
        self.items: Dict[int, QGraphicsPathItem] = {}
        self.model: Optional[rm.Model] = None
        self._half = (0.45, 0.30)
        self._pose: Optional[tuple] = None
        self._visible = True
        self._build()

    def _build(self) -> None:
        for it in self.items.values():
            self._remove(it)
        self.items = {}
        for part, path in sorted(paths_for(self.model, *self._half).items()):
            fill, line = STYLE[part]
            it = QGraphicsPathItem(path)
            it.setBrush(QBrush(QColor(*fill)))
            it.setPen(_pen(line, 2.0 if part == rm.BODY else 1.0))
            it.setZValue(self._z + 0.1 * part)
            it.setVisible(self._visible and self._pose is not None)
            self._add(it)
            self.items[part] = it
        if self._pose is not None:
            self.set_pose(*self._pose)

    def set_model(self, model: Optional[rm.Model], half_l: float, half_w: float) -> None:
        if model is self.model and (model is not None or self._half == (half_l, half_w)):
            return
        self.model, self._half = model, (half_l, half_w)
        self._build()

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self._pose = (x, y, yaw)
        t = QTransform()
        t.translate(x, y)
        t.rotateRadians(yaw)
        for it in self.items.values():
            it.setTransform(t)
            it.setVisible(self._visible)

    def set_visible(self, on: bool) -> None:
        self._visible = on
        for it in self.items.values():
            it.setVisible(on and self._pose is not None)

    def parts(self) -> list:
        return sorted(self.items)


class RobotIcon(QWidget):
    """A small top view of a robot: heading up the widget for yaw = pi/2 (map
    north), the body coloured by a level, an arrow when it moves."""

    def __init__(self, size: int = 72, parent=None) -> None:
        super().__init__(parent)
        self.setFixedSize(size, size)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.model: Optional[rm.Model] = None
        self._half = (0.45, 0.30)
        self.yaw = math.pi / 2          # facing up
        self.level = "off"
        self.motion = 0.0               # m/s, sign: forward or back
        self.turn = 0.0                 # rad/s
        self.setToolTip("")

    def set_model(self, model: Optional[rm.Model], half_l: float = 0.45, half_w: float = 0.30) -> None:
        self.model, self._half = model, (half_l, half_w)
        self.update()

    def set_state(self, yaw: Optional[float], level: str, v: float = 0.0, w: float = 0.0) -> None:
        if yaw is not None and math.isfinite(yaw):
            self.yaw = yaw
        self.level, self.motion, self.turn = level, v, w
        self.update()

    def paintEvent(self, _ev) -> None:
        paths = paths_for(self.model, *self._half)
        if self.model is not None:
            lo, hi = self.model.bounds()
            reach = float(max(np.abs(lo[:2]).max(), np.abs(hi[:2]).max()))
        else:
            reach = math.hypot(*self._half)
        reach = max(reach, 0.05) * 1.25          # room for the heading arrow
        side = min(self.width(), self.height())
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.translate(self.width() / 2, self.height() / 2)
        s = side / (2 * reach)
        p.scale(s, -s)                           # y up, like the map
        p.rotate(math.degrees(self.yaw))
        level = QColor(LEVEL_COLORS.get(self.level, LEVEL_COLORS["off"]))
        for part, path in sorted(paths.items()):
            fill, line = STYLE[part]
            if part == rm.BODY:
                c = QColor(level)
                c.setAlpha(90)
                p.setBrush(QBrush(c))
                p.setPen(_pen(level, 1.6))
            else:
                p.setBrush(QBrush(QColor(*fill)))
                p.setPen(_pen(line, 0.8))
            p.drawPath(path)
        hl = self.model.footprint()[0] if self.model is not None else self._half[0]
        moving = abs(self.motion) > 0.02 or abs(self.turn) > 0.05
        if moving and abs(self.motion) > 0.02:
            d = 1.0 if self.motion > 0 else -1.0
            tip = d * (hl + reach * 0.18)
            base = d * hl
            w = reach * 0.12
            p.setBrush(QBrush(level))
            p.setPen(Qt.NoPen)
            p.drawPolygon(QPolygonF([QPointF(tip, 0), QPointF(base, w), QPointF(base, -w)]))
        else:
            p.setPen(_pen(level, 1.6))
            p.drawLine(QPointF(0, 0), QPointF(hl + reach * 0.12, 0))
        p.end()
