"""One measure over time, drawn with QPainter (no chart library to ship).

One series and one y axis per chart: two measures of a different scale get
two charts, never a second axis. Gaps in the data (None) stay gaps. Hovering
shows a crosshair and the value under the mouse. Long series are thinned to
the low and high of each pixel column, so a spike is never lost.
"""
from __future__ import annotations

import bisect
import math
import time
from typing import List, Optional, Sequence

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import ACCENT, BORDER, MUTED, PANEL, TEXT

GRID = "#202634"


def nice_ticks(lo: float, hi: float, n: int = 5) -> List[float]:
    if hi <= lo:
        hi = lo + 1.0
    raw = (hi - lo) / max(1, n)
    mag = 10 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw)
    start = math.ceil(lo / step) * step
    out, v = [], start
    while v <= hi + step * 1e-6:
        out.append(round(v, 10))
        v += step
    return out


_TIME_STEPS = (60, 300, 600, 900, 1800, 3600, 7200, 3 * 3600, 6 * 3600, 12 * 3600,
               86400, 2 * 86400, 7 * 86400, 14 * 86400, 30 * 86400)


def time_ticks(t0: float, t1: float, n: int = 6) -> List[float]:
    span = max(1.0, t1 - t0)
    step = next((s for s in _TIME_STEPS if span / s <= n), _TIME_STEPS[-1])
    off = time.localtime(t0).tm_gmtoff         # ticks on local hours / days
    first = math.ceil((t0 + off) / step) * step - off
    return [first + i * step for i in range(int((t1 - first) // step) + 1)]


def time_label(t: float, span: float) -> str:
    if span < 2 * 86400:
        return time.strftime("%H:%M", time.localtime(t))
    return time.strftime("%m-%d", time.localtime(t))


class TimeChart(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.ts: List[float] = []
        self.vs: List[Optional[float]] = []
        self.title = ""
        self.unit = ""
        self.points = False             # one dot per value (a trend over boots)
        self.empty_text = "No data"
        self.lines: list = []           # (value, label): reference lines, e.g. the low battery limit
        self.hover: Optional[int] = None
        self.color = QColor(ACCENT)

    def set_data(self, ts: Sequence[float], vs: Sequence[Optional[float]], title: str, unit: str,
                 points: bool = False, lines: Sequence = (), empty: str = "No data") -> None:
        self.ts, self.vs = list(ts), list(vs)
        self.title, self.unit, self.points = title, unit, points
        self.lines = list(lines)
        self.empty_text = empty
        self.hover = None
        self.update()

    def values(self) -> List[float]:
        return [v for v in self.vs if v is not None]

    # ---- geometry ----

    def _plot(self) -> QRectF:
        return QRectF(58, 34, max(10, self.width() - 74), max(10, self.height() - 34 - 28))

    def _ranges(self):
        vals = self.values() + [v for v, _l in self.lines]
        lo, hi = min(vals), max(vals)
        pad = (hi - lo) * 0.08 or max(abs(hi) * 0.05, 0.5)
        lo, hi = lo - pad, hi + pad
        t0, t1 = self.ts[0], self.ts[-1]
        if t1 - t0 < 60:
            t0, t1 = t0 - 30, t1 + 30
        return t0, t1, lo, hi

    def _map(self, r: QRectF, t0, t1, lo, hi):
        def mx(t):
            return r.left() + (t - t0) / (t1 - t0) * r.width()

        def my(v):
            return r.bottom() - (v - lo) / (hi - lo) * r.height()
        return mx, my

    def _path(self, r, mx, my) -> QPainterPath:
        """Low and high of each pixel column; a gap starts a new piece."""
        path = QPainterPath()
        col, lo_v, hi_v, first = None, None, None, None
        pen_down = False

        def flush():
            nonlocal pen_down
            if col is None:
                return
            x = col + 0.5
            if pen_down:
                path.lineTo(x, my(first))
            else:
                path.moveTo(x, my(first))
                pen_down = True
            if hi_v != lo_v:
                path.lineTo(x, my(hi_v))
                path.lineTo(x, my(lo_v))

        for t, v in zip(self.ts, self.vs):
            if v is None:
                flush()
                col, pen_down = None, False
                continue
            c = int(mx(t))
            if c != col:
                flush()
                col, lo_v, hi_v, first = c, v, v, v
            else:
                lo_v, hi_v = min(lo_v, v), max(hi_v, v)
        flush()
        return path

    # ---- drawing ----

    def paintEvent(self, _ev) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(PANEL))
        p.setPen(QColor(TEXT))
        f = QFont(self.font())
        f.setBold(True)
        p.setFont(f)
        p.drawText(QRectF(14, 8, self.width() - 28, 20), Qt.AlignLeft | Qt.AlignVCenter,
                   f"{self.title}" + (f" ({self.unit})" if self.unit else ""))
        p.setFont(self.font())
        if not self.values():
            p.setPen(QColor(MUTED))
            p.drawText(self.rect(), Qt.AlignCenter, self.empty_text)
            return
        r = self._plot()
        t0, t1, lo, hi = self._ranges()
        mx, my = self._map(r, t0, t1, lo, hi)
        small = QFont(self.font())
        small.setPointSizeF(max(7.0, self.font().pointSizeF() - 1.5))
        p.setFont(small)
        for v in nice_ticks(lo, hi):            # recessive grid, labels in muted ink
            y = my(v)
            p.setPen(QPen(QColor(GRID), 1))
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(0, y - 8, r.left() - 8, 16), Qt.AlignRight | Qt.AlignVCenter, f"{v:g}")
        span = t1 - t0
        for t in time_ticks(t0, t1):
            x = mx(t)
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(x - 40, r.bottom() + 4, 80, 18), Qt.AlignCenter, time_label(t, span))
        p.setPen(QPen(QColor(BORDER), 1))
        p.drawLine(QPointF(r.left(), r.bottom()), QPointF(r.right(), r.bottom()))
        for v, label in self.lines:
            y = my(v)
            p.setPen(QPen(QColor(MUTED), 1, Qt.DashLine))
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y))
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(r.left() + 4, y - 17, r.width() - 8, 16), Qt.AlignRight | Qt.AlignVCenter, label)
        p.setClipRect(r.adjusted(-6, -6, 6, 6))
        p.setPen(QPen(self.color, 2, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        p.setBrush(Qt.NoBrush)
        p.drawPath(self._path(r, mx, my))
        if self.points:
            p.setPen(QPen(QColor(PANEL), 2))        # a ring of the surface around each dot
            p.setBrush(self.color)
            for t, v in zip(self.ts, self.vs):
                if v is not None:
                    p.drawEllipse(QPointF(mx(t), my(v)), 4.5, 4.5)
        p.setClipping(False)
        self._draw_hover(p, r, mx, my, span)

    def _draw_hover(self, p: QPainter, r: QRectF, mx, my, span: float) -> None:
        i = self.hover
        if i is None or not 0 <= i < len(self.ts) or self.vs[i] is None:
            return
        x, y = mx(self.ts[i]), my(self.vs[i])
        p.setPen(QPen(QColor(MUTED), 1))
        p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))
        p.setPen(QPen(QColor(PANEL), 2))
        p.setBrush(self.color)
        p.drawEllipse(QPointF(x, y), 5, 5)
        fmt = "%Y-%m-%d %H:%M" if self.points else ("%H:%M:%S" if span < 2 * 86400 else "%m-%d %H:%M")
        text = f"{self.vs[i]:.3g} {self.unit}   {time.strftime(fmt, time.localtime(self.ts[i]))}"
        w = p.fontMetrics().horizontalAdvance(text) + 16
        bx = min(max(r.left(), x - w / 2), r.right() - w)
        box = QRectF(bx, r.top() - 2, w, 22)
        p.setPen(QPen(QColor(BORDER), 1))
        p.setBrush(QColor("#0f1115"))
        p.drawRoundedRect(box, 4, 4)
        p.setPen(QColor(TEXT))
        p.drawText(box, Qt.AlignCenter, text)

    # ---- hover ----

    def nearest(self, x: float) -> Optional[int]:
        if not self.values():
            return None
        r = self._plot()
        t0, t1, _lo, _hi = self._ranges()
        t = t0 + (x - r.left()) / r.width() * (t1 - t0)
        i = bisect.bisect_left(self.ts, t)
        best = None
        for j in range(max(0, i - 40), min(len(self.ts), i + 40)):     # nearest with a value
            if self.vs[j] is not None and (best is None or abs(self.ts[j] - t) < abs(self.ts[best] - t)):
                best = j
        return best

    def mouseMoveEvent(self, ev) -> None:
        self.hover = self.nearest(ev.position().x())
        self.update()

    def leaveEvent(self, _ev) -> None:
        self.hover = None
        self.update()
