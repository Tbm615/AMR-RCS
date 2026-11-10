"""2D map panel wrapping MapView + sidebar controls."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFrame, QGroupBox, QHBoxLayout, QLabel, QPushButton, QScrollArea,
    QVBoxLayout, QWidget,
)

from ..widgets.map_view import MapView
from ..widgets.zones_box import ZonesBox


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
        self.mode.addItems(["Set goal", "Add waypoint", "Set initial pose", "Measure distance", "Draw zone"])
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
        self.last_pose_btn = QPushButton("Use last position")
        self.last_pose_btn.setToolTip("Set the initial pose to where the robot last was on this map, "
                                      "if it has not been moved since")
        self.last_pose_btn.setVisible(False)
        tv.addWidget(self.last_pose_btn)
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
        self.ck_pl = QCheckBox("Saved places"); self.ck_pl.setChecked(True)
        self.ck_pl.toggled.connect(self.view.show_places)
        self.ck_zones = QCheckBox("Zones"); self.ck_zones.setChecked(True)
        self.ck_zones.toggled.connect(self.view.show_zones)
        for w in (self.ck_scan, self.ck_path, self.ck_wp, self.ck_pl, self.ck_zones, self.ck_gc, self.ck_lc):
            ov.addWidget(w)
        side.addWidget(overlay)

        self.zones = ZonesBox()
        self.zones.draw_requested.connect(self.draw_zone)
        self.view.zone_drawn.connect(self._zone_drawn)
        side.addWidget(self.zones)

        side.addStretch(1)
        wrap = QWidget()
        wrap.setLayout(side)
        scroll = QScrollArea()          # a short window scrolls the side bar, nothing is cut off
        scroll.setWidget(wrap)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFixedWidth(300)
        root.addWidget(scroll)

    _HINTS = {
        0: "Press where the robot should go and drag toward the direction it should face; "
           "release to send. (Or click, then click again in the direction.) "
           "Right-click / Esc cancels. Right-drag pans, wheel zooms.",
        1: "Press and drag to add a waypoint with a heading to the mission open on the "
           "Missions page.",
        2: "Press where the robot really is and drag toward the direction it faces; "
           "release to set the initial pose.",
        3: "Press and drag between two points to measure the distance. Esc clears it.",
        4: "Click the zone's corners; double-click, Enter or a click on the first corner closes it. "
           "Right-click or Backspace takes the last corner back, Esc drops it.",
    }

    def _mode_changed(self, idx: int) -> None:
        self.view.set_click_mode({0: "goal", 1: "waypoint", 2: "pose", 3: "measure", 4: "zone"}[idx])
        if hasattr(self, "hint"):
            self.hint.setText(self._HINTS[idx])

    def draw_zone(self, kind: str) -> None:
        """Draw a keepout or slow zone on this map (the Draw zone tool)."""
        self.view.zone_kind = kind
        self.mode.setCurrentIndex(4)
        self.view.cancel_zone()
        self.view.setFocus()

    def _zone_drawn(self, points: list) -> None:
        if self.zones.add_keep.isEnabled():
            self.zones.add_drawn(self.view.zone_kind, points)
