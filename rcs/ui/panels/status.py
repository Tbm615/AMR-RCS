"""Status/health dashboard panel."""
from __future__ import annotations

import math

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QFormLayout, QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QVBoxLayout, QWidget,
)

from ...core.ros_client import RosClient


def _fmt_hz(hz: float) -> str:
    return f"{hz:5.1f} Hz" if hz > 0.1 else "   —"


class _Stat(QFrame):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setStyleSheet("QFrame { background:#121520; border:1px solid #1a1d26; border-radius:6px; }")
        v = QVBoxLayout(self)
        v.setContentsMargins(14, 10, 14, 10)
        self.title = QLabel(title); self.title.setProperty("muted", True)
        self.value = QLabel("—"); self.value.setStyleSheet("font-size:22px; font-weight:600;")
        v.addWidget(self.title)
        v.addWidget(self.value)


class StatusPanel(QWidget):
    def __init__(self, ros: RosClient, parent=None):
        super().__init__(parent)
        self.ros = ros
        self._lin = 0.0
        self._ang = 0.0
        self._build()
        self.ros.odom_received.connect(self._on_odom)
        self.ros.estop_received.connect(self._on_estop_remote)

        t = QTimer(self); t.timeout.connect(self._refresh); t.start(500)

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(16)

        grid = QGridLayout()
        self.s_conn = _Stat("Connection")
        self.s_lin = _Stat("Linear velocity")
        self.s_ang = _Stat("Angular velocity")
        self.s_estop = _Stat("E-stop state")
        grid.addWidget(self.s_conn, 0, 0)
        grid.addWidget(self.s_lin, 0, 1)
        grid.addWidget(self.s_ang, 0, 2)
        grid.addWidget(self.s_estop, 0, 3)
        root.addLayout(grid)

        box = QGroupBox("Topic rates")
        f = QFormLayout(box)
        self.hz_labels = {}
        for key, label in [
            ("odom", "/odom"),
            ("scan", "scan (filtered)"),
            ("map", "/map"),
            ("local_costmap", "local costmap"),
            ("global_costmap", "global costmap"),
            ("plan", "/plan"),
            ("image", "camera image"),
            ("tf", "/tf"),
            ("rosout", "/rosout"),
        ]:
            lbl = QLabel("—"); lbl.setStyleSheet("font-family: monospace;")
            f.addRow(label, lbl)
            self.hz_labels[key] = lbl
        root.addWidget(box)
        root.addStretch(1)

    def _on_odom(self, m: dict) -> None:
        tw = m.get("twist", {}).get("twist", {})
        self._lin = float(tw.get("linear", {}).get("x", 0.0))
        self._ang = float(tw.get("angular", {}).get("z", 0.0))

    def _on_estop_remote(self, on: bool) -> None:
        self.s_estop.value.setText("ENGAGED" if on else "clear")
        self.s_estop.value.setStyleSheet(
            "font-size:22px; font-weight:600; color:" + ("#ff6a6a" if on else "#39d98a"))

    def set_connected(self, ok: bool) -> None:
        self.s_conn.value.setText("online" if ok else "offline")
        self.s_conn.value.setStyleSheet(
            "font-size:22px; font-weight:600; color:" + ("#39d98a" if ok else "#ff6a6a"))

    def _refresh(self) -> None:
        for key, lbl in self.hz_labels.items():
            st = self.ros.topic_stats.get(key)
            lbl.setText(_fmt_hz(st.hz) if st else "   —")
        self.s_lin.value.setText(f"{self._lin:+.2f} m/s")
        self.s_ang.value.setText(f"{self._ang:+.2f} rad/s")
