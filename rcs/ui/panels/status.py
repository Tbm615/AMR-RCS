"""Status/health dashboard panel."""
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget,
)

from ...core.ros_client import RosClient

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
    ("cloud", "Point cloud (while 3D view is open)", True),
    ("rosout", "Logs  /rosout", False),
]


class StatusPanel(QWidget):
    def __init__(self, ros: RosClient, parent=None):
        super().__init__(parent)
        self.ros = ros
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

    def _refresh(self) -> None:
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
