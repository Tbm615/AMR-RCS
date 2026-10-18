"""Teleop panel: virtual joystick + keyboard + latched E-stop."""
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import (
    QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QVBoxLayout, QWidget,
)

from ..widgets.joystick import Joystick


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
        self.setFocusPolicy(Qt.StrongFocus)
        self._build()

        self._tx_timer = QTimer(self)
        self._tx_timer.setInterval(50)   # 20 Hz, inside the 0.5 s stale cutoff
        self._tx_timer.timeout.connect(self._tick)
        self._tx_timer.start()

    def _build(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(16)

        # --- joystick side ---
        js_box = QGroupBox("Joystick (drag inside the ring)")
        v = QVBoxLayout(js_box)
        self.joy = Joystick()
        self.joy.moved.connect(self._on_joy)
        self.joy.released.connect(lambda: self._on_joy(0.0, 0.0))
        v.addWidget(self.joy)
        self.readout = QLabel("v = 0.00 m/s   ω = 0.00 rad/s")
        v.addWidget(self.readout, 0, Qt.AlignCenter)
        root.addWidget(js_box, 2)

        # --- controls side ---
        side = QVBoxLayout()

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

        kb = QGroupBox("Keyboard (focus must be on this panel)")
        kbv = QVBoxLayout(kb)
        kbv.addWidget(QLabel("W / ↑ — forward"))
        kbv.addWidget(QLabel("S / ↓ — reverse"))
        kbv.addWidget(QLabel("A / ← — turn left"))
        kbv.addWidget(QLabel("D / → — turn right"))
        kbv.addWidget(QLabel("Space — brake"))
        side.addWidget(kb)

        self.estop_btn = QPushButton("● E-STOP")
        self.estop_btn.setProperty("kind", "danger")
        self.estop_btn.setMinimumHeight(64)
        self.estop_btn.setStyleSheet("font-size: 18px; font-weight: 700;")
        self.estop_btn.setCheckable(True)
        self.estop_btn.toggled.connect(self._set_estop)
        side.addWidget(self.estop_btn)
        side.addStretch(1)

        root.addLayout(side, 1)

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
        self._estop = on
        self.estop_btn.setText("● E-STOP ENGAGED (click to release)" if on else "● E-STOP")
        self.estop_changed.emit(on)

    def is_estopped(self) -> bool:
        return self._estop

    # ---- streaming ----

    def _tick(self) -> None:
        if self._estop:
            lin, ang = 0.0, 0.0
        else:
            # joystick wins over keys when touched
            jx, jy = self._jx, self._jy
            kl, ka = self._key_lin, self._key_ang
            lin_n = jy if abs(jy) > 1e-3 else kl
            ang_n = -jx if abs(jx) > 1e-3 else ka
            lin = max(-1.0, min(1.0, lin_n)) * self._lin_lim
            ang = max(-1.0, min(1.0, ang_n)) * self._ang_lim
        self.readout.setText(f"v = {lin:+.2f} m/s   ω = {ang:+.2f} rad/s")
        self.cmd_vel.emit(lin, ang)

    def apply_limits(self, lin: float, ang: float) -> None:
        self.lin.setValue(lin)
        self.ang.setValue(ang)
