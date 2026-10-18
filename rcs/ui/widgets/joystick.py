"""Virtual joystick widget. Emits normalized (x, y) ∈ [-1, 1] on movement."""
from __future__ import annotations

import math

from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QMouseEvent, QPainter, QPen, QRadialGradient
from PySide6.QtWidgets import QWidget


class Joystick(QWidget):
    moved = Signal(float, float)
    released = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self._pos = QPointF(0.0, 0.0)      # knob offset in widget coords
        self._pressed = False
        self.setFocusPolicy(Qt.StrongFocus)

    def _radius(self) -> float:
        return min(self.width(), self.height()) / 2 - 10

    def _knob_r(self) -> float:
        return self._radius() * 0.28

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx, cy = self.width() / 2, self.height() / 2
        r = self._radius()

        # outer ring
        p.setPen(QPen(QColor("#262c3a"), 2))
        p.setBrush(QColor("#121520"))
        p.drawEllipse(QPointF(cx, cy), r, r)
        # crosshair
        p.setPen(QPen(QColor("#1a1d26"), 1))
        p.drawLine(QPointF(cx - r, cy), QPointF(cx + r, cy))
        p.drawLine(QPointF(cx, cy - r), QPointF(cx, cy + r))

        # knob
        kx, ky = cx + self._pos.x(), cy + self._pos.y()
        kr = self._knob_r()
        grad = QRadialGradient(QPointF(kx - kr * 0.3, ky - kr * 0.3), kr)
        grad.setColorAt(0, QColor("#4a7fd1"))
        grad.setColorAt(1, QColor("#2a66d4"))
        p.setBrush(QBrush(grad))
        p.setPen(QPen(QColor("#3a79e8"), 2))
        p.drawEllipse(QPointF(kx, ky), kr, kr)

    def _update_pos(self, pt: QPointF) -> None:
        cx, cy = self.width() / 2, self.height() / 2
        dx, dy = pt.x() - cx, pt.y() - cy
        r = self._radius()
        d = math.hypot(dx, dy)
        if d > r:
            dx = dx * r / d
            dy = dy * r / d
        self._pos = QPointF(dx, dy)
        self.update()
        nx = dx / r if r > 0 else 0.0
        ny = -dy / r if r > 0 else 0.0  # up is +
        self.moved.emit(nx, ny)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        self._pressed = True
        self._update_pos(event.position())

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._pressed:
            self._update_pos(event.position())

    def mouseReleaseEvent(self, _event: QMouseEvent) -> None:
        self._pressed = False
        self._pos = QPointF(0.0, 0.0)
        self.update()
        self.moved.emit(0.0, 0.0)
        self.released.emit()
