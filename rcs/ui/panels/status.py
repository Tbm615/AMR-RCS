"""Status page: the robot's health at a glance (one line per part, what is
wrong and what to do), speeds, E-STOP, and the rate of each topic. A
diagnostic bundle for support is saved from here."""
from __future__ import annotations

from typing import Callable, Optional

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)

from ...core import health
from ...core.ros_client import RosClient

LEVEL_COLOR = {"ok": "#39d98a", "warn": "#ffb547", "error": "#ff6a6a", "off": "#8b93a7"}

GREEN, RED, AMBER, MUTED = "#39d98a", "#ff6a6a", "#ffb547", "#8b93a7"


class _Card(QFrame):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setObjectName("Card")
        v = QVBoxLayout(self)
        v.setContentsMargins(16, 12, 16, 14)
        v.setSpacing(4)
        self.title = QLabel(title)
        self.title.setProperty("muted", True)
        self.value = QLabel("—")
        v.addWidget(self.title)
        v.addWidget(self.value)
        self.set("—", MUTED)

    def set(self, text: str, color: str = "#e6e8ee") -> None:
        self.value.setText(text)
        self.value.setStyleSheet(f"font-size:22px; font-weight:600; color:{color};")


# (stats key, label, expected to stream continuously)
_TOPICS = [
    ("odom", "Odometry  /odom", True),
    ("tf", "Transforms  /tf", True),
    ("scan", "Lidar  /scan_filtered", True),
    ("map", "Map  /map", False),
    ("global_costmap", "Global costmap", False),
    ("local_costmap", "Local costmap", True),
    ("plan", "Planned path  /plan", False),
    ("image", "Camera image (while its page is open)", True),
    ("depth", "Camera depth (while recording the 3D map)", True),
    ("rosout", "Logs  /rosout", False),
]


class StatusPanel(QWidget):
    bundle_requested = Signal()

    def __init__(self, ros: RosClient, parent=None):
        super().__init__(parent)
        self.ros = ros
        self.health_source: Optional[Callable[[], dict]] = None     # set by the main window
        self.last_checks: list = []
        self._lin = 0.0
        self._ang = 0.0
        self._connected = False
        self._build()
        self.ros.odom_received.connect(self._on_odom)
        self.ros.estop_received.connect(self._on_estop_remote)
        t = QTimer(self)
        t.timeout.connect(self._refresh)
        t.start(500)

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(16)

        cards = QGridLayout()
        cards.setSpacing(12)
        self.c_conn = _Card("Connection")
        self.c_lin = _Card("Linear speed")
        self.c_ang = _Card("Angular speed")
        self.c_estop = _Card("E-STOP")
        for i, c in enumerate((self.c_conn, self.c_lin, self.c_ang, self.c_estop)):
            cards.addWidget(c, 0, i)
        self.c_conn.set("offline", RED)
        self.c_estop.set("clear", GREEN)
        root.addLayout(cards)

        hb = QGroupBox("Health")
        hv = QVBoxLayout(hb)
        top = QHBoxLayout()
        self.h_summary = QLabel("—")
        self.h_summary.setStyleSheet("font-size:15px; font-weight:600;")
        top.addWidget(self.h_summary, 1)
        self.bundle_btn = QPushButton("Save diagnostic bundle…")
        self.bundle_btn.setToolTip("One zip for whoever helps you: the app's log, settings without passwords "
                                   "or sign-in tokens, events, crash reports and the robot's state now")
        self.bundle_btn.clicked.connect(self.bundle_requested.emit)
        top.addWidget(self.bundle_btn)
        hv.addLayout(top)
        self.h_grid = QGridLayout()
        self.h_grid.setHorizontalSpacing(14)
        self.h_grid.setVerticalSpacing(4)
        self.h_grid.setColumnStretch(3, 1)
        hv.addLayout(self.h_grid)
        self.h_rows: list = []
        root.addWidget(hb)

        box = QGroupBox("Data from the robot")
        g = QGridLayout(box)
        g.setHorizontalSpacing(16)
        g.setVerticalSpacing(8)
        hdr_topic = QLabel("Topic"); hdr_topic.setProperty("muted", True)
        hdr_rate = QLabel("Rate"); hdr_rate.setProperty("muted", True)
        g.addWidget(hdr_topic, 0, 1)
        g.addWidget(hdr_rate, 0, 2)
        self.rows = {}
        for r, (key, label, streaming) in enumerate(_TOPICS, start=1):
            dot = QLabel("●")
            name = QLabel(label)
            rate = QLabel("—")
            rate.setMinimumWidth(90)
            g.addWidget(dot, r, 0)
            g.addWidget(name, r, 1)
            g.addWidget(rate, r, 2)
            self.rows[key] = (dot, rate, streaming)
        g.setColumnStretch(3, 1)
        note = QLabel(f'<span style="color:{GREEN}">●</span> receiving &nbsp;&nbsp; '
                      f'<span style="color:{MUTED}">●</span> nothing received yet &nbsp;&nbsp; '
                      f'<span style="color:{RED}">●</span> stopped (nothing for 3 s)')
        note.setProperty("muted", True)
        g.addWidget(note, len(_TOPICS) + 1, 1, 1, 3)
        root.addWidget(box)
        root.addStretch(1)

    def _on_odom(self, m: dict) -> None:
        tw = m.get("twist", {}).get("twist", {})
        self._lin = float(tw.get("linear", {}).get("x", 0.0))
        self._ang = float(tw.get("angular", {}).get("z", 0.0))

    def _on_estop_remote(self, on: bool) -> None:
        self.set_estop(on)

    def set_estop(self, on: bool) -> None:
        self.c_estop.set("ENGAGED" if on else "clear", RED if on else GREEN)

    def set_connected(self, ok: bool) -> None:
        self._connected = ok
        self.c_conn.set("online" if ok else "offline", GREEN if ok else RED)

    def _refresh_health(self) -> None:
        if self.health_source is None:
            return
        cs = health.checks(self.health_source())
        self.last_checks = cs
        while len(self.h_rows) < len(cs):
            i = len(self.h_rows)
            row = (QLabel("●"), QLabel(), QLabel(), QLabel())
            row[3].setWordWrap(True)
            row[3].setProperty("muted", True)
            for j, w in enumerate(row):
                self.h_grid.addWidget(w, i, j, Qt.AlignTop)
            self.h_rows.append(row)
        for i, row in enumerate(self.h_rows):
            visible = i < len(cs)
            for w in row:
                w.setVisible(visible)
            if not visible:
                continue
            c = cs[i]
            row[0].setStyleSheet(f"color:{LEVEL_COLOR[c.level]};")
            row[1].setText(c.title)
            row[2].setText(c.text)
            row[2].setStyleSheet(f"color:{LEVEL_COLOR[c.level]};" if c.level in ("warn", "error") else "")
            row[3].setText(c.hint)
        word = health.summary(cs)
        n_bad = sum(1 for c in cs if c.level in ("warn", "error"))
        self.h_summary.setText({"ok": "All good", "warn": f"{n_bad} thing(s) to look at",
                                "error": f"{n_bad} problem(s)"}[word])
        self.h_summary.setStyleSheet(f"font-size:15px; font-weight:600; color:{LEVEL_COLOR[word]};")

    def _refresh(self) -> None:
        self._refresh_health()
        for key, (dot, rate, streaming) in self.rows.items():
            st = self.ros.topic_stats.get(key)
            if not st or st.last_t == 0:
                dot.setStyleSheet(f"color:{MUTED};")
                rate.setText("—")
            elif st.fresh(3.0) or not streaming:
                dot.setStyleSheet(f"color:{GREEN};")
                rate.setText(f"{st.hz:.1f} Hz" if streaming and st.hz > 0.1 else "received")
            else:
                dot.setStyleSheet(f"color:{RED};")
                rate.setText("stopped")
        if self._connected:
            self.c_lin.set(f"{self._lin:+.2f} m/s")
            self.c_ang.set(f"{self._ang:+.2f} rad/s")
