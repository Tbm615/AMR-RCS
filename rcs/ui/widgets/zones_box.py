"""The keepout and speed zones of the map Nav2 runs on: a list to edit, kept
on the robot (rcs_launcher.py zones_get / zones_put). Shown on the 2D Map page."""
from __future__ import annotations

import copy

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QInputDialog, QLabel, QListWidget, QListWidgetItem, QPushButton, QSpinBox,
    QVBoxLayout,
)

from ... import i18n
from ...core import zones as zn
from ..theme import MUTED

AMBER = "#ffb547"


class ZonesBox(QGroupBox):
    draw_requested = Signal(str)            # "keepout" | "slow": draw one on the map
    changed = Signal()                      # the list or the selection changed: redraw the maps
    save_requested = Signal(str, object)    # map name, zones
    reload_requested = Signal(str)          # map name: ask the robot again

    def __init__(self, parent=None):
        super().__init__("Keepout and speed zones", parent)
        self.map_name = ""          # the map whose zones these are ("" = none)
        self.robot_zones: list = []  # as the robot has them
        self.zones: list = []       # as edited here
        self.loaded = False         # the robot has answered for map_name
        self.available = True       # the robot side knows zones
        self.can_edit = True        # the signed-in role may change them
        self.connected = False
        self.applied = None         # Nav2 uses the robot's zones now (None: not said)
        self.confirm_dialogs = True
        self._building = False

        v = QVBoxLayout(self)
        v.setSpacing(6)
        self.where = QLabel()
        self.where.setWordWrap(True)
        self.where.setProperty("muted", True)
        v.addWidget(self.where)
        self.list = QListWidget()
        self.list.setMaximumHeight(104)
        self.list.currentRowChanged.connect(self._selected)
        self.list.itemDoubleClicked.connect(lambda _it: self.rename())
        v.addWidget(self.list)

        row = QHBoxLayout()
        self.add_keep = QPushButton("+ Keepout")
        self.add_keep.setToolTip("Draw an area Nav2 must never drive into: click its corners on the map, "
                                 "double-click (or click the first corner) to close it")
        self.add_keep.clicked.connect(lambda: self.draw_requested.emit("keepout"))
        self.add_slow = QPushButton("+ Slow")
        self.add_slow.setToolTip("Draw an area where Nav2 drives at most the % next to it")
        self.add_slow.clicked.connect(lambda: self.draw_requested.emit("slow"))
        row.addWidget(self.add_keep)
        row.addWidget(self.add_slow)
        v.addLayout(row)

        row = QHBoxLayout()
        lbl = QLabel("Slow zone speed:")
        lbl.setProperty("muted", True)
        self.speed = QSpinBox()
        self.speed.setRange(zn.SPEED_MIN, zn.SPEED_MAX)
        self.speed.setSingleStep(10)
        self.speed.setValue(40)
        self.speed.setSuffix(" %")
        self.speed.setToolTip("Most speed in a slow zone, in percent of the usual top speed")
        self.speed.valueChanged.connect(self._speed_changed)
        row.addWidget(lbl)
        row.addWidget(self.speed, 1)
        v.addLayout(row)

        row = QHBoxLayout()
        self.rename_btn = QPushButton("Rename")
        self.rename_btn.clicked.connect(self.rename)
        self.delete_btn = QPushButton("Delete")
        self.delete_btn.clicked.connect(self.delete)
        self.undo_btn = QPushButton("Undo")
        self.undo_btn.setToolTip("Back to the zones the robot has")
        self.undo_btn.clicked.connect(self.undo)
        for w in (self.rename_btn, self.delete_btn, self.undo_btn):
            row.addWidget(w)
        v.addLayout(row)
        self.save_btn = QPushButton("Save to robot")
        self.save_btn.setProperty("kind", "primary")
        self.save_btn.clicked.connect(self.save)
        v.addWidget(self.save_btn)
        self.status = QLabel()
        self.status.setWordWrap(True)
        v.addWidget(self.status)
        self._refresh()

    # ---- state from the app ----

    def dirty(self) -> bool:
        return self.loaded and self.zones != self.robot_zones

    def set_map(self, name: str) -> None:
        """The map Nav2 runs on ("" = none, or SLAM). Unsaved changes of
        another map are dropped: they belong to that map."""
        if name == self.map_name:
            return
        self.map_name = name
        self.robot_zones, self.zones, self.loaded, self.applied = [], [], False, None
        self._fill()
        if name and self.connected and self.available:
            self.reload_requested.emit(name)

    def set_link(self, connected: bool, available: bool) -> None:
        was = (self.connected, self.available)
        self.connected, self.available = connected, available
        if (connected, available) != was:
            # (re)connected: ask again, another PC may have changed them; edits
            # not saved yet are kept (on_answer)
            if connected and available and self.map_name and (not was[0] or not self.loaded):
                self.reload_requested.emit(self.map_name)
            self._refresh()

    def set_can_edit(self, on: bool) -> None:
        self.can_edit = on
        self._refresh()

    def on_answer(self, ans: dict) -> None:
        """The robot's answer to zones_get / zones_put."""
        if ans.get("name") != self.map_name:
            return
        if not ans.get("ok", True):
            self._say(f"Robot: {ans.get('msg', 'refused')}", error=True)
            return
        try:
            robot = zn.check(ans.get("zones") or [])
        except ValueError as e:
            self._say(f"The robot's zones cannot be read: {e}", error=True)
            return
        keep_edits = self.dirty() and not ans.get("msg")     # a plain reload: keep what is being edited
        self.robot_zones, self.loaded = robot, True
        if ans.get("msg"):          # answer to our save
            self.applied = bool(ans.get("applied"))
        if not keep_edits:
            self.zones = copy.deepcopy(robot)
        self._fill()
        if ans.get("msg"):
            self._say(f"Robot: {ans['msg']}")

    # ---- editing ----

    def add_drawn(self, kind: str, points: list) -> dict:
        base = i18n.tr("Keepout" if kind == "keepout" else "Slow")     # names in the app's language
        z = {"id": zn.new_id(self.zones, kind), "name": zn.new_name(self.zones, kind, base), "kind": kind,
             "points": [[round(x, 3), round(y, 3)] for x, y in points]}
        if kind == "slow":
            z["speed"] = self.speed.value()
        self.zones.append(z)
        self._fill(select=z["id"])
        return z

    def selected(self) -> dict | None:
        r = self.list.currentRow()
        return self.zones[r] if 0 <= r < len(self.zones) else None

    def selected_id(self) -> str:
        z = self.selected()
        return z["id"] if z else ""

    def rename(self, name: str | None = None) -> None:
        z = self.selected()
        if z is None or not self.can_edit:
            return
        if name is None:
            name, ok = QInputDialog.getText(self, i18n.tr("Rename zone"), i18n.tr("Name:"), text=z["name"])
            if not ok:
                return
        z["name"] = name.strip()[:40] or z["name"]
        self._fill(select=z["id"])

    def delete(self) -> None:
        z = self.selected()
        if z is None or not self.can_edit:
            return
        self.zones.remove(z)
        self._fill()

    def save(self) -> None:
        if not (self.map_name and self.loaded and self.can_edit):
            return
        try:
            zones = zn.check(self.zones)
        except ValueError as e:
            self._say(str(e), error=True)
            return
        self.save_requested.emit(self.map_name, zones)
        self._say("Saving…")

    def undo(self) -> None:
        self.zones = copy.deepcopy(self.robot_zones)
        self._fill()

    def _selected(self, _row: int) -> None:
        if self._building:
            return
        z = self.selected()
        if z and z["kind"] == "slow":
            self._building = True
            self.speed.setValue(z["speed"])
            self._building = False
        self._refresh()
        self.changed.emit()

    def _speed_changed(self, v: int) -> None:
        if self._building:
            return
        z = self.selected()
        if z and z["kind"] == "slow" and self.can_edit:
            z["speed"] = int(v)
            self._fill(select=z["id"])

    # ---- display ----

    def _fill(self, select: str = "") -> None:
        keep = select or self.selected_id()
        self._building = True
        self.list.clear()
        for z in self.zones:
            kind = i18n.tr("keepout") if z["kind"] == "keepout" else f"{i18n.tr('slow')} {z['speed']} %"
            it = QListWidgetItem(("⛔ " if z["kind"] == "keepout" else "🐢 ") + f"{z['name']} · {kind}")
            it.setData(Qt.UserRole, z["id"])
            self.list.addItem(it)
            if z["id"] == keep:
                self.list.setCurrentItem(it)
        self._building = False
        self._refresh()
        self.changed.emit()

    def _say(self, text: str, error: bool = False) -> None:
        self.status.setText(text)
        self.status.setStyleSheet(f"color:{AMBER};" if error else "")

    def _refresh(self) -> None:
        usable = self.connected and self.available and bool(self.map_name) and self.loaded
        if not self.connected:
            where = "Connect to the robot to see the zones of its maps."
        elif not self.available:
            where = "The robot side is too old for zones: update it (robot/README.md)."
        elif not self.map_name:
            where = "Start Nav2 on a saved map (Robot programs page): zones belong to a map."
        elif not self.loaded:
            where = f"Zones of {self.map_name}: asking the robot…"
        else:
            n = len(self.robot_zones)
            use = {True: ", in use by Nav2", False: " (restart navigation to use them)"}.get(self.applied, "")
            where = f"Zones of {self.map_name}: {n} on the robot{use if n or self.applied is False else ''}."
        self.where.setText(where)
        edit = usable and self.can_edit
        for w in (self.add_keep, self.add_slow):
            w.setEnabled(edit)
        sel = self.selected()
        self.speed.setEnabled(edit)
        self.rename_btn.setEnabled(edit and sel is not None)
        self.delete_btn.setEnabled(edit and sel is not None)
        self.save_btn.setEnabled(edit and self.dirty())
        self.undo_btn.setEnabled(usable and self.dirty())
        if usable and not self.can_edit:
            self.status.setText("Signed in as viewer: you can see the zones, not change them.")
            self.status.setStyleSheet(f"color:{MUTED};")
        elif self.dirty():
            self._say("Changes not saved on the robot yet.", error=True)
        elif self.status.text().startswith("Changes not saved") or self.status.text().startswith("Signed in"):
            self._say("")
