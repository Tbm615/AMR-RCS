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
        tv.addWidget(QLabel("Click to set goal (two-click: point + heading)"))
        self.mode = QComboBox()
        self.mode.addItems(["Set goal", "Add waypoint", "Set initial pose"])
        self.mode.currentIndexChanged.connect(self._mode_changed)
        tv.addWidget(self.mode)

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
        self.ck_wp.toggled.connect(lambda b: self.view.wp_scatter.setVisible(b))
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
        wrap.setFixedWidth(240)
        root.addWidget(wrap)

    def _mode_changed(self, idx: int) -> None:
        self.view.set_click_mode({0: "goal", 1: "waypoint", 2: "pose"}[idx])
