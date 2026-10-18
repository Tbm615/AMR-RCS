"""Navigation panel: reuses MapView with costmaps on by default + Nav2 status."""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)

from ..widgets.map_view import MapView


class NavigationPanel(QWidget):
    goal_requested = Signal(float, float, float)
    cancel_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.view = MapView()
        self.view.show_global_costmap(True)
        self.view.show_local_costmap(True)
        self._build()
        self.view.goal_requested.connect(self.goal_requested.emit)

    def _build(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)
        root.addWidget(self.view, 1)

        side = QVBoxLayout()
        side.setSpacing(10)

        info = QGroupBox("Nav2 status")
        v = QVBoxLayout(info)
        self.state_lbl = QLabel("idle")
        self.dist_lbl = QLabel("—")
        self.eta_lbl = QLabel("—")
        v.addWidget(QLabel("State:")); v.addWidget(self.state_lbl)
        v.addWidget(QLabel("Distance remaining:")); v.addWidget(self.dist_lbl)
        v.addWidget(QLabel("Estimated time:")); v.addWidget(self.eta_lbl)
        side.addWidget(info)

        ctrl = QGroupBox("Controls")
        cv = QVBoxLayout(ctrl)
        cancel = QPushButton("Cancel current goal")
        cancel.setProperty("kind", "danger")
        cancel.clicked.connect(self.cancel_requested.emit)
        cv.addWidget(cancel)
        side.addWidget(ctrl)

        side.addStretch(1)
        wrap = QWidget()
        wrap.setLayout(side)
        wrap.setFixedWidth(240)
        root.addWidget(wrap)

    def set_status(self, status: str) -> None:
        self.state_lbl.setText(status)
        if status == "active":
            self.state_lbl.setStyleSheet("color:#ffb547; font-weight:600;")
        elif status == "succeeded":
            self.state_lbl.setStyleSheet("color:#39d98a; font-weight:600;")
        elif status in ("aborted", "canceled"):
            self.state_lbl.setStyleSheet("color:#ff6a6a; font-weight:600;")
        else:
            self.state_lbl.setStyleSheet("color:#8b93a7;")

    def set_feedback(self, fb: dict) -> None:
        dr = fb.get("distance_remaining")
        nt = fb.get("estimated_time_remaining", {})
        if dr is not None:
            self.dist_lbl.setText(f"{float(dr):.2f} m")
        if nt:
            sec = float(nt.get("sec", 0))
            self.eta_lbl.setText(f"{sec:.1f} s")
