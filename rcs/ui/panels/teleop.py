"""Teleop panel: virtual joystick + keyboard + latched E-stop."""
from __future__ import annotations

import math
import time

from PySide6.QtCore import QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QKeyEvent, QPainter
from PySide6.QtWidgets import (
    QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QVBoxLayout, QWidget,
)

from ...core.clearance import SLOW_DISTANCE, STOP_DISTANCE, Clearance
from ..widgets.joystick import Joystick

STALE_S = 1.0       # no scan for this long: the bars stop claiming anything


class _ClearanceBar(QWidget):
    """Horizontal bar: full when clear, empty at the robot; green, amber, red."""

    SPAN = 2.0      # m shown by a full bar

    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self.label = label
        self.value = None       # metres, math.inf = clear, None = no data
        self.setFixedHeight(26)

    def set_value(self, v) -> None:
        if v != self.value:
            self.value = v
            self.update()

    def color(self) -> str:
        if self.value is None:
            return "#5b6378"
        if self.value < STOP_DISTANCE:
            return "#ff6a6a"
        return "#ffb547" if self.value < SLOW_DISTANCE else "#39d98a"

    def text(self) -> str:
        if self.value is None:
            return "no lidar"
        return f"> {self.SPAN:.1f} m" if self.value > self.SPAN else f"{self.value:.2f} m"

    def paintEvent(self, _ev) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        name_w, val_w = 62, 62
        bar = QRectF(name_w, h / 2 - 5, max(10, w - name_w - val_w), 10)
        p.setPen(QColor("#c6cad6"))
        p.drawText(QRectF(0, 0, name_w, h), Qt.AlignVCenter | Qt.AlignLeft, self.label)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#1b2030"))
        p.drawRoundedRect(bar, 5, 5)
        col = QColor(self.color())
        if self.value is not None:
            frac = max(0.03, min(1.0, self.value / self.SPAN))
            p.setBrush(col)
            p.drawRoundedRect(QRectF(bar.x(), bar.y(), bar.width() * frac, bar.height()), 5, 5)
        p.setPen(col)
        p.drawText(QRectF(w - val_w, 0, val_w, h), Qt.AlignVCenter | Qt.AlignRight, self.text())


class TeleopPanel(QWidget):
    cmd_vel = Signal(float, float)       # (linear m/s, angular rad/s)
    estop_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lin_lim = 0.45
        self._ang_lim = 0.8
        self._jx = 0.0
        self._jy = 0.0
        self._key_lin = 0.0
        self._key_ang = 0.0
        self._estop = False
        self._clear_t = 0.0
        self.setFocusPolicy(Qt.StrongFocus)
        self._build()

        self._tx_timer = QTimer(self)
        self._tx_timer.setInterval(50)   # 20 Hz, inside the 0.5 s stale cutoff
        self._tx_timer.timeout.connect(self._tick)
        self._tx_timer.start()

    def _build(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(20)

        # --- joystick side ---
        js_box = QGroupBox("Drive")
        v = QVBoxLayout(js_box)
        v.setSpacing(14)
        hint = QLabel("Press inside the ring and drag. Up = forward, sideways = turn. Release to stop.")
        hint.setProperty("muted", True)
        hint.setAlignment(Qt.AlignCenter)
        hint.setWordWrap(True)
        v.addWidget(hint)
        v.addStretch(1)
        self.joy = Joystick()
        self.joy.setFixedSize(340, 340)
        self.joy.moved.connect(self._on_joy)
        self.joy.released.connect(lambda: self._on_joy(0.0, 0.0))
        v.addWidget(self.joy, 0, Qt.AlignHCenter)
        self.readout = QLabel()
        self.readout.setAlignment(Qt.AlignCenter)
        self.readout.setTextFormat(Qt.RichText)
        v.addWidget(self.readout)
        v.addStretch(1)
        root.addWidget(js_box, 1)
        self._show_readout(0.0, 0.0)

        # --- controls side ---
        side_w = QWidget()
        side_w.setFixedWidth(330)
        side = QVBoxLayout(side_w)
        side.setContentsMargins(0, 0, 0, 0)
        side.setSpacing(14)

        limits = QGroupBox("Speed limits")
        f = QFormLayout(limits)
        self.lin = QDoubleSpinBox(); self.lin.setRange(0.05, 2.0); self.lin.setSingleStep(0.05)
        self.lin.setSuffix(" m/s"); self.lin.setValue(0.45)
        self.ang = QDoubleSpinBox(); self.ang.setRange(0.1, 3.0); self.ang.setSingleStep(0.1)
        self.ang.setSuffix(" rad/s"); self.ang.setValue(0.8)
        self.lin.valueChanged.connect(lambda v: setattr(self, "_lin_lim", v))
        self.ang.valueChanged.connect(lambda v: setattr(self, "_ang_lim", v))
        f.addRow("Max linear", self.lin)
        f.addRow("Max angular", self.ang)
        side.addWidget(limits)

        cbox = QGroupBox("Clearance from the robot body")
        cv = QVBoxLayout(cbox)
        cv.setSpacing(4)
        self.bar_front = _ClearanceBar("Front")
        self.bar_rear = _ClearanceBar("Rear")
        self.bar_near = _ClearanceBar("Nearest")
        self.bar_near.setToolTip("Closest obstacle in any direction (matters when turning on the spot)")
        for b in (self.bar_front, self.bar_rear, self.bar_near):
            cv.addWidget(b)
        self.clear_note = QLabel("Display only: the app does not brake for you.")
        self.clear_note.setProperty("muted", True)
        self.clear_note.setWordWrap(True)
        cv.addWidget(self.clear_note)
        side.addWidget(cbox)

        kb = QGroupBox("Keyboard")
        kg = QGridLayout(kb)
        kg.setHorizontalSpacing(12)
        kg.setVerticalSpacing(8)
        key_css = ("background:#1f2533; border:1px solid #3a4256; border-bottom-width:2px;"
                   " border-radius:5px; padding:2px 8px; font-weight:600;")
        for r, (keys, action) in enumerate([("W  ↑", "forward"), ("S  ↓", "reverse"),
                                            ("A  ←", "turn left"), ("D  →", "turn right"),
                                            ("Space", "stop")]):
            k = QLabel(keys)
            k.setStyleSheet(key_css)
            k.setAlignment(Qt.AlignCenter)
            k.setFixedWidth(64)
            kg.addWidget(k, r, 0)
            kg.addWidget(QLabel(action), r, 1)
        tip = QLabel("Click this page once so it receives the keys.")
        tip.setProperty("muted", True)
        tip.setWordWrap(True)
        kg.addWidget(tip, 5, 0, 1, 2)
        side.addWidget(kb)

        self.estop_btn = QPushButton("● E-STOP")
        self.estop_btn.setProperty("kind", "danger")
        self.estop_btn.setMinimumHeight(64)
        self.estop_btn.setStyleSheet("font-size: 18px; font-weight: 700;")
        self.estop_btn.setCheckable(True)
        self.estop_btn.toggled.connect(self._set_estop)
        side.addWidget(self.estop_btn)
        side.addStretch(1)

        root.addWidget(side_w)

    # ---- clearance ----

    def set_clearance(self, c: Clearance) -> None:
        self._clear_t = time.monotonic()
        self.bar_front.set_value(c.front)
        self.bar_rear.set_value(c.rear)
        self.bar_near.set_value(c.nearest)

    def _check_clearance_age(self) -> None:
        if self._clear_t and time.monotonic() - self._clear_t > STALE_S:
            self._clear_t = 0.0
            for b in (self.bar_front, self.bar_rear, self.bar_near):
                b.set_value(None)

    # ---- input handlers ----

    def _on_joy(self, x: float, y: float) -> None:
        self._jx = x
        self._jy = y

    def keyPressEvent(self, event: QKeyEvent) -> None:
        k = event.key()
        if k in (Qt.Key_W, Qt.Key_Up):
            self._key_lin = 1.0
        elif k in (Qt.Key_S, Qt.Key_Down):
            self._key_lin = -1.0
        elif k in (Qt.Key_A, Qt.Key_Left):
            self._key_ang = 1.0
        elif k in (Qt.Key_D, Qt.Key_Right):
            self._key_ang = -1.0
        elif k == Qt.Key_Space:
            self._key_lin = 0.0
            self._key_ang = 0.0

    def keyReleaseEvent(self, event: QKeyEvent) -> None:
        k = event.key()
        if k in (Qt.Key_W, Qt.Key_Up, Qt.Key_S, Qt.Key_Down):
            self._key_lin = 0.0
        if k in (Qt.Key_A, Qt.Key_Left, Qt.Key_D, Qt.Key_Right):
            self._key_ang = 0.0

    # ---- estop ----

    def _set_estop(self, on: bool) -> None:
        """Slot for the panel's own button: update state and tell the window."""
        self.set_estop_state(on)
        self.estop_changed.emit(on)

    def set_estop_state(self, on: bool) -> None:
        """Update state and visuals only (no signal), for external callers."""
        self._estop = on
        self.estop_btn.blockSignals(True)
        self.estop_btn.setChecked(on)
        self.estop_btn.blockSignals(False)
        self.estop_btn.setText("● E-STOP ENGAGED (click to release)" if on else "● E-STOP")

    def is_estopped(self) -> bool:
        return self._estop

    # ---- streaming ----

    # Zero commands sent after the operator lets go, so the robot brakes
    # (0.3 s at 20 Hz). After that teleop goes silent: Nav2 publishes to the
    # same driver topic and a constant zero stream would override it.
    _STOP_TICKS = 6

    def _tick(self) -> None:
        self._check_clearance_age()
        if self._estop:
            lin, ang = 0.0, 0.0
            active = True               # E-stop keeps streaming zeros
        else:
            # joystick wins over keys when touched
            jx, jy = self._jx, self._jy
            kl, ka = self._key_lin, self._key_ang
            lin_n = jy if abs(jy) > 1e-3 else kl
            ang_n = -jx if abs(jx) > 1e-3 else ka
            lin = max(-1.0, min(1.0, lin_n)) * self._lin_lim
            ang = max(-1.0, min(1.0, ang_n)) * self._ang_lim
            active = abs(lin) > 1e-4 or abs(ang) > 1e-4
        self._show_readout(lin, ang)
        if active:
            self._stop_left = self._STOP_TICKS
            self.cmd_vel.emit(lin, ang)
        elif getattr(self, "_stop_left", 0) > 0:
            self._stop_left -= 1
            self.cmd_vel.emit(0.0, 0.0)

    def _show_readout(self, lin: float, ang: float) -> None:
        self.readout.setText(
            f'<span style="font-size:22px; font-weight:600;">{lin:+.2f}</span>'
            f'<span style="color:#8b93a7;"> m/s</span>&nbsp;&nbsp;&nbsp;&nbsp;'
            f'<span style="font-size:22px; font-weight:600;">{ang:+.2f}</span>'
            f'<span style="color:#8b93a7;"> rad/s</span>')

    def apply_limits(self, lin: float, ang: float) -> None:
        self.lin.setValue(lin)
        self.ang.setValue(ang)
