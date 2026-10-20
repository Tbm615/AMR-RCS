"""2D map panel wrapping MapView + sidebar controls."""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QVBoxLayout, QWidget,
)

from ..widgets.map_view import MapView


class Map2DPanel(QWidget):
    goal_requested = Signal(float, float, float)
    cancel_requested = Signal()
    set_initial_pose = Signal(float, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.view = MapView()
        self._build()
        self.view.goal_requested.connect(self.goal_requested.emit)
        self.view.pose_requested.connect(self.set_initial_pose.emit)

    def _build(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        root.addWidget(self.view, 1)

        side = QVBoxLayout()
        side.setSpacing(10)

        tools = QGroupBox("Tools")
        tv = QVBoxLayout(tools)
        self.mode = QComboBox()
        self.mode.addItems(["Set goal", "Add waypoint", "Set initial pose"])
        self.mode.currentIndexChanged.connect(self._mode_changed)
        tv.addWidget(self.mode)
        self.hint = QLabel()
        self.hint.setWordWrap(True)
        self.hint.setProperty("muted", True)
        tv.addWidget(self.hint)
        self._mode_changed(0)

        cancel = QPushButton("Cancel current goal")
        cancel.setProperty("kind", "danger")
        cancel.clicked.connect(self.cancel_requested.emit)
        tv.addWidget(cancel)
        side.addWidget(tools)

        overlay = QGroupBox("Overlays")
        ov = QVBoxLayout(overlay)
        self.ck_scan = QCheckBox("Lidar scan"); self.ck_scan.setChecked(True)
        self.ck_scan.toggled.connect(lambda b: self.view.scan_scatter.setVisible(b))
        self.ck_path = QCheckBox("Planned path"); self.ck_path.setChecked(True)
        self.ck_path.toggled.connect(lambda b: self.view.path_line.setVisible(b))
        self.ck_wp = QCheckBox("Waypoints"); self.ck_wp.setChecked(True)
        self.ck_wp.toggled.connect(self.view.show_waypoints)
        self.ck_gc = QCheckBox("Global costmap")
        self.ck_gc.toggled.connect(self.view.show_global_costmap)
        self.ck_lc = QCheckBox("Local costmap")
        self.ck_lc.toggled.connect(self.view.show_local_costmap)
        for w in (self.ck_scan, self.ck_path, self.ck_wp, self.ck_gc, self.ck_lc):
            ov.addWidget(w)
        side.addWidget(overlay)

        side.addStretch(1)
        wrap = QWidget()
        wrap.setLayout(side)
        wrap.setFixedWidth(270)
        root.addWidget(wrap)

    _HINTS = {
        0: "Press where the robot should go and drag toward the direction it should face; "
           "release to send. (Or click, then click again in the direction.) "
           "Right-click / Esc cancels. Right-drag pans, wheel zooms.",
        1: "Press and drag to add a waypoint with a heading to the mission open on the "
           "Missions page.",
        2: "Press where the robot really is and drag toward the direction it faces; "
           "release to set the initial pose.",
    }

    def _mode_changed(self, idx: int) -> None:
        self.view.set_click_mode({0: "goal", 1: "waypoint", 2: "pose"}[idx])
        if hasattr(self, "hint"):
            self.hint.setText(self._HINTS[idx])
