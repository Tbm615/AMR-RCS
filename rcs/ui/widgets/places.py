"""Saved places box: go to a named place, save where the robot is, manage the list."""
from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QInputDialog, QListWidget, QListWidgetItem, QPushButton, QVBoxLayout,
)

from ...core.places import Place, PlaceStore


class PlacesBox(QGroupBox):
    go_requested = Signal(object)           # Place
    add_to_mission = Signal(object)         # Place
    changed = Signal()                      # the list of places on this map changed

    def __init__(self, store: PlaceStore, parent=None):
        super().__init__("Saved places", parent)
        self.store = store
        self.map_name = ""
        self.pose = None                    # (x, y, yaw) of the robot
        v = QVBoxLayout(self)
        self.list = QListWidget()
        self.list.setMinimumHeight(120)
        self.list.itemDoubleClicked.connect(lambda _i: self._go())
        self.list.itemSelectionChanged.connect(self._sync)
        v.addWidget(self.list)
        self.go_btn = QPushButton("Go")
        self.go_btn.setProperty("kind", "primary")
        self.go_btn.clicked.connect(self._go)
        v.addWidget(self.go_btn)
        r1 = QHBoxLayout()
        self.here_btn = QPushButton("Save robot position")
        self.here_btn.clicked.connect(self._ask_add)
        r1.addWidget(self.here_btn)
        v.addLayout(r1)
        r2 = QHBoxLayout()
        self.ren_btn = QPushButton("Rename"); self.ren_btn.clicked.connect(self._ask_rename)
        self.del_btn = QPushButton("Delete"); self.del_btn.clicked.connect(self.delete_selected)
        self.mis_btn = QPushButton("→ Mission")
        self.mis_btn.setToolTip("Add as a waypoint of the mission open on the Missions page")
        self.mis_btn.clicked.connect(self._to_mission)
        for b in (self.ren_btn, self.del_btn, self.mis_btn):
            r2.addWidget(b)
        v.addLayout(r2)
        self.refresh()

    def selected(self) -> Optional[Place]:
        it = self.list.currentItem()
        return self.store.get(it.data(Qt.UserRole)) if it is not None and it.isSelected() else None

    def _sync(self) -> None:
        on = self.selected() is not None
        for b in (self.go_btn, self.ren_btn, self.del_btn, self.mis_btn):
            b.setEnabled(on)
        self.here_btn.setEnabled(self.pose is not None)

    def refresh(self) -> None:
        keep = self.selected()
        self.list.clear()
        for p in self.store.on_map(self.map_name):
            it = QListWidgetItem(p.name)
            it.setData(Qt.UserRole, p.id)
            it.setToolTip(f"x {p.x:.2f}, y {p.y:.2f}" + (f" · map {p.map}" if p.map else ""))
            self.list.addItem(it)
            if keep is not None and keep.id == p.id:
                self.list.setCurrentItem(it)
        self._sync()
        self.changed.emit()

    def set_map(self, name: str) -> None:
        if name != self.map_name:
            self.map_name = name
            self.refresh()

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        had = self.pose is not None
        self.pose = (x, y, yaw)
        if not had:
            self._sync()

    def add_here(self, name: str) -> Optional[Place]:
        if self.pose is None or not name.strip():
            return None
        p = self.store.add(name.strip(), *self.pose, self.map_name)
        self.refresh()
        return p

    def _ask_add(self) -> None:
        name, ok = QInputDialog.getText(self, "Save place", "Name of this place:",
                                        text=f"Place {len(self.store.places) + 1}")
        if ok:
            self.add_here(name)

    def rename_selected(self, name: str) -> None:
        p = self.selected()
        if p and name.strip():
            p.name = name.strip()
            self.store.save()
            self.refresh()

    def _ask_rename(self) -> None:
        p = self.selected()
        if p:
            name, ok = QInputDialog.getText(self, "Rename place", "Name:", text=p.name)
            if ok:
                self.rename_selected(name)

    def delete_selected(self) -> None:
        p = self.selected()
        if p:
            self.store.remove(p.id)
            self.refresh()

    def _go(self) -> None:
        p = self.selected()
        if p:
            self.go_requested.emit(p)

    def _to_mission(self) -> None:
        p = self.selected()
        if p:
            self.add_to_mission.emit(p)
