"""Fleet page: every robot that is watched, on one screen.

A card per robot profile: the robot seen from above (its own model, turned the
way it faces, coloured by how it is), online or not, battery, programs, E-STOP,
motion.
Watching is per robot (the check box on its card) and remembered. E-STOP from
here only engages; it is released on that robot's own pages, by whoever
drives it.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QPushButton, QScrollArea, QVBoxLayout, QWidget,
)

from ...core.fleet import FleetMonitor, describe, heading, overall
from ..widgets.robot_shape import RobotIcon

COLORS = {"ok": "#39d98a", "warn": "#ffb547", "error": "#ff6a6a", "off": "#8b93a7"}
LINES = (("link", "Link"), ("battery", "Battery"), ("estop", "E-STOP"), ("programs", "Programs"),
         ("motion", "Motion"))


class RobotCard(QFrame):
    open_requested = Signal(str)
    estop_requested = Signal(str)
    watch_changed = Signal(str, bool)

    def __init__(self, profile, watched: bool, model_of=None, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("Card")
        self.rid = profile.id
        self.model_of = model_of
        v = QVBoxLayout(self)
        v.setContentsMargins(14, 12, 14, 12)
        head = QHBoxLayout()
        self.icon = RobotIcon(64)
        head.addWidget(self.icon)
        names = QVBoxLayout()
        top = QHBoxLayout()
        self.name = QLabel()
        self.name.setStyleSheet("font-size:16px; font-weight:600;")
        top.addWidget(self.name, 1)
        self.watch = QCheckBox("Watch")
        self.watch.setChecked(watched)
        self.watch.toggled.connect(lambda on: self.watch_changed.emit(self.rid, on))
        top.addWidget(self.watch)
        names.addLayout(top)
        self.addr = QLabel()
        self.addr.setProperty("muted", True)
        names.addWidget(self.addr)
        names.addStretch(1)
        head.addLayout(names, 1)
        v.addLayout(head)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        self.vals = {}
        for i, (key, title) in enumerate(LINES):
            t = QLabel(title)
            t.setProperty("muted", True)
            val = QLabel("—")
            val.setWordWrap(True)
            grid.addWidget(t, i, 0, Qt.AlignTop)
            grid.addWidget(val, i, 1)
            self.vals[key] = val
        v.addLayout(grid)
        self.note = QLabel("")
        self.note.setProperty("muted", True)
        self.note.setWordWrap(True)
        v.addWidget(self.note)
        row = QHBoxLayout()
        self.open_btn = QPushButton("Open")
        self.open_btn.setToolTip("Connect to this robot to drive it and see all its pages")
        self.open_btn.clicked.connect(lambda: self.open_requested.emit(self.rid))
        self.estop_btn = QPushButton("E-STOP")
        self.estop_btn.setProperty("kind", "danger")
        self.estop_btn.clicked.connect(lambda: self.estop_requested.emit(self.rid))
        row.addWidget(self.open_btn)
        row.addStretch(1)
        row.addWidget(self.estop_btn)
        v.addLayout(row)
        self.set_profile(profile)

    def set_profile(self, profile) -> None:
        self.name.setText(profile.name)
        self.addr.setText(profile.ws_url)
        lim = profile.limits
        model = self.model_of(profile) if self.model_of else None
        self.icon.set_model(model, lim.footprint_half_length, lim.footprint_half_width)

    def show_state(self, d, driven: bool) -> None:
        if d is None:
            for val in self.vals.values():
                val.setText("—")
                val.setStyleSheet(f"color:{COLORS['off']};")
            self.vals["link"].setText("not watched")
            self.icon.set_state(None, "off")
            self.icon.setToolTip("not watched")
            self.estop_btn.setEnabled(False)
            self.note.setText("This is the robot open in the app." if driven else "")
            return
        desc = describe(d)
        odom = d.get("odom") or {}
        fresh = d.get("_ages", {}).get("odom", 1e9) <= 5.0
        level = overall(desc)
        yaw = heading(d)
        self.icon.set_state(yaw, level, odom.get("v", 0.0) if fresh else 0.0, odom.get("w", 0.0) if fresh else 0.0)
        where = "on the map" if "map_pose" in d and d.get("_ages", {}).get("map_pose", 1e9) <= 5.0 else "by odometry"
        self.icon.setToolTip(f"facing {yaw * 57.29578:.0f}° {where}" if yaw is not None else "heading unknown")
        for key, _t in LINES:
            text, level = desc.get(key, ("—", "off"))
            self.vals[key].setText(text)
            self.vals[key].setStyleSheet(f"color:{COLORS[level]};")
        self.estop_btn.setEnabled(bool(d.get("_connected")))
        last = (d.get("gate") or {}).get("last_stop")
        notes = []
        if driven:
            notes.append("This is the robot open in the app.")
        if last:
            notes.append(f"Last stopped itself: {last.get('reason', '')}")
        self.note.setText(" ".join(notes))


class FleetPanel(QWidget):
    open_requested = Signal(str)        # robot id
    estop_one = Signal(str)             # robot id: engaged, for the event log
    estop_all_done = Signal(object)     # names it reached
    watched_changed = Signal(object)    # set of robot ids, to remember

    def __init__(self, profiles, monitor: FleetMonitor, watched: set, model_of=None, parent=None) -> None:
        super().__init__(parent)
        self.profiles, self.monitor = profiles, monitor
        self.model_of = model_of        # profile -> robot model, or None
        self.watched = set(watched)
        self.driven = ""                # id of the robot the app is connected to
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        top = QHBoxLayout()
        hint = QLabel("Robots you watch are followed here, each over a light connection of its own. "
                      "E-STOP here only engages; release it on that robot's own pages.")
        hint.setWordWrap(True)
        hint.setProperty("muted", True)
        top.addWidget(hint, 1)
        self.all_btn = QPushButton("E-STOP all watched")
        self.all_btn.setProperty("kind", "danger")
        self.all_btn.setMinimumHeight(38)
        self.all_btn.clicked.connect(self.estop_all)
        top.addWidget(self.all_btn)
        root.addLayout(top)
        self.result = QLabel("")
        root.addWidget(self.result)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        self.body = QWidget()
        self.grid = QGridLayout(self.body)
        self.grid.setSpacing(12)
        self.grid.setAlignment(Qt.AlignTop)
        scroll.setWidget(self.body)
        root.addWidget(scroll, 1)
        self.cards: dict = {}
        monitor.updated.connect(self.refresh)
        self.rebuild()

    def rebuild(self) -> None:
        """Cards for the robot profiles as they are now."""
        ids = {p.id for p in self.profiles.profiles}
        for rid in list(self.cards):
            if rid not in ids:
                self.cards.pop(rid).deleteLater()
        for p in self.profiles.profiles:
            card = self.cards.get(p.id)
            if card is None:
                card = RobotCard(p, p.id in self.watched, self.model_of)
                card.open_requested.connect(self.open_requested)
                card.estop_requested.connect(self.estop)
                card.watch_changed.connect(self._watch)
                self.cards[p.id] = card
            else:
                card.set_profile(p)
        for i, p in enumerate(self.profiles.profiles):
            self.grid.addWidget(self.cards[p.id], i // 3, i % 3)
        self.watched &= ids
        self.monitor.watch(self.profiles.profiles, self.watched)
        self.refresh()

    def _watch(self, rid: str, on: bool) -> None:
        (self.watched.add if on else self.watched.discard)(rid)
        self.monitor.watch(self.profiles.profiles, self.watched)
        self.watched_changed.emit(set(self.watched))
        self.refresh()

    def refresh(self) -> None:
        if not self.isVisible() and self.cards:
            return
        for rid, card in self.cards.items():
            card.show_state(self.monitor.snapshot(rid), rid == self.driven)
        self.all_btn.setEnabled(bool(self.watched))

    def estop(self, rid: str) -> None:
        w = self.monitor.watches.get(rid)
        ok = w is not None and w.estop()
        name = w.profile.name if w else rid
        self.result.setText(f"E-STOP sent to {name}" if ok else f"E-STOP could not reach {name}")
        self.result.setStyleSheet(f"color:{COLORS['error'] if not ok else COLORS['warn']};")
        if ok:
            self.estop_one.emit(rid)

    def estop_all(self) -> None:
        names = self.monitor.estop_all()
        missed = sorted({self.monitor.watches[r].profile.name for r in self.watched if r in self.monitor.watches}
                        - set(names))
        text = f"E-STOP sent to {', '.join(names) or 'no robot'}"
        if missed:
            text += f"; could not reach {', '.join(missed)}"
        self.result.setText(text)
        self.result.setStyleSheet(f"color:{COLORS['error'] if missed or not names else COLORS['warn']};")
        self.estop_all_done.emit(names)
