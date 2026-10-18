"""Mission panel: list of waypoints (checkpoints), loop route, executor."""
from __future__ import annotations

import math
import time
from typing import Optional

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QHeaderView,
    QInputDialog, QLabel, QLineEdit, QMessageBox, QPushButton, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from ...core.mission import Mission, MissionStore, Waypoint
from ..widgets.map_view import MapView


class MissionPanel(QWidget):
    goal_requested = Signal(float, float, float)
    cancel_requested = Signal()
    waypoints_changed = Signal(object)   # list[(x,y)] for map overlay

    def __init__(self, store: MissionStore, parent=None):
        super().__init__(parent)
        self.store = store
        self.view = MapView()
        self.view.set_click_mode("waypoint")
        self.view.pose_clicked.connect(self._on_map_click)
        self.cur: Optional[Mission] = None
        self._exec_idx: int = -1
        self._exec_running: bool = False
        self._exec_dwell_until: float = 0.0
        self._retries = 0
        self._nav_state = ""
        self._in_tol_since = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._build()
        self._reload_missions()

        self._exec_timer = QTimer(self)
        self._exec_timer.setInterval(200)
        self._exec_timer.timeout.connect(self._exec_tick)
        self._exec_timer.start()

    # ---- build ----

    def _build(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        root.addWidget(self.view, 1)

        side = QVBoxLayout()
        side.setSpacing(10)
        wrap = QWidget(); wrap.setLayout(side); wrap.setFixedWidth(360)

        mbox = QGroupBox("Mission")
        mv = QVBoxLayout(mbox)
        row = QHBoxLayout()
        self.mission_combo = QComboBox()
        self.mission_combo.currentIndexChanged.connect(self._load_selected)
        row.addWidget(self.mission_combo, 1)
        new = QPushButton("+"); new.setToolTip("New mission"); new.clicked.connect(self._new_mission)
        dup = QPushButton("Dup"); dup.clicked.connect(self._duplicate)
        delete = QPushButton("Del"); delete.clicked.connect(self._delete)
        row.addWidget(new); row.addWidget(dup); row.addWidget(delete)
        mv.addLayout(row)
        rename_row = QHBoxLayout()
        self.name_edit = QLineEdit()
        self.name_edit.editingFinished.connect(self._rename)
        rename_row.addWidget(QLabel("Name:")); rename_row.addWidget(self.name_edit, 1)
        mv.addLayout(rename_row)
        self.loop_ck = QCheckBox("Loop route (restart after last waypoint)")
        self.loop_ck.toggled.connect(self._loop_toggle)
        mv.addWidget(self.loop_ck)
        side.addWidget(mbox)

        wbox = QGroupBox("Waypoints — click map in waypoint mode, or 'Set here'")
        wv = QVBoxLayout(wbox)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["#", "Name", "x (m)", "y (m)", "yaw°"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.itemChanged.connect(self._on_table_edit)
        wv.addWidget(self.table)
        row2 = QHBoxLayout()
        here = QPushButton("Set here"); here.clicked.connect(self._add_here)
        up = QPushButton("↑"); up.clicked.connect(lambda: self._move(-1))
        down = QPushButton("↓"); down.clicked.connect(lambda: self._move(1))
        rm = QPushButton("Remove"); rm.clicked.connect(self._remove)
        for b in (here, up, down, rm):
            row2.addWidget(b)
        wv.addLayout(row2)
        side.addWidget(wbox)

        tolbox = QGroupBox("Default per-waypoint settings (new waypoints)")
        tv = QVBoxLayout(tolbox)
        r = QHBoxLayout()
        self.tol_sb = QDoubleSpinBox(); self.tol_sb.setRange(0.05, 5.0); self.tol_sb.setValue(0.25); self.tol_sb.setSuffix(" m")
        self.dwell_sb = QDoubleSpinBox(); self.dwell_sb.setRange(0.0, 60.0); self.dwell_sb.setValue(0.0); self.dwell_sb.setSuffix(" s")
        r.addWidget(QLabel("Tolerance:")); r.addWidget(self.tol_sb)
        r.addWidget(QLabel("Dwell:")); r.addWidget(self.dwell_sb)
        tv.addLayout(r)
        side.addWidget(tolbox)

        ebox = QGroupBox("Execute")
        ev = QVBoxLayout(ebox)
        self.status_lbl = QLabel("stopped"); self.status_lbl.setProperty("muted", True)
        ev.addWidget(self.status_lbl)
        er = QHBoxLayout()
        self.play_btn = QPushButton("▶ Play"); self.play_btn.setProperty("kind", "success")
        self.play_btn.clicked.connect(self._play)
        self.pause_btn = QPushButton("⏸ Pause"); self.pause_btn.clicked.connect(self._pause)
        self.stop_btn = QPushButton("■ Stop"); self.stop_btn.setProperty("kind", "danger")
        self.stop_btn.clicked.connect(self._stop)
        for b in (self.play_btn, self.pause_btn, self.stop_btn):
            er.addWidget(b)
        ev.addLayout(er)
        side.addWidget(ebox)

        side.addStretch(1)
        root.addWidget(wrap)

    # ---- bindings ----

    def _reload_missions(self) -> None:
        self.mission_combo.blockSignals(True)
        self.mission_combo.clear()
        for m in self.store.missions:
            self.mission_combo.addItem(f"{m.name}", m.id)
        self.mission_combo.blockSignals(False)
        if self.store.missions:
            self.mission_combo.setCurrentIndex(0)
            self._load_selected()
        else:
            self._new_mission()

    def _load_selected(self) -> None:
        mid = self.mission_combo.currentData()
        if not mid:
            return
        self.cur = self.store.get(mid)
        self.name_edit.setText(self.cur.name)
        self.loop_ck.blockSignals(True)
        self.loop_ck.setChecked(self.cur.loop)
        self.loop_ck.blockSignals(False)
        self._refresh_table()
        self._emit_wps()

    def _refresh_table(self) -> None:
        self.table.blockSignals(True)
        self.table.setRowCount(0)
        if self.cur:
            for i, wp in enumerate(self.cur.waypoints):
                self.table.insertRow(i)
                it0 = QTableWidgetItem(str(i + 1))
                it0.setFlags(it0.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(i, 0, it0)
                self.table.setItem(i, 1, QTableWidgetItem(wp.name))
                self.table.setItem(i, 2, QTableWidgetItem(f"{wp.x:.3f}"))
                self.table.setItem(i, 3, QTableWidgetItem(f"{wp.y:.3f}"))
                self.table.setItem(i, 4, QTableWidgetItem(f"{math.degrees(wp.yaw):.1f}"))
        self.table.blockSignals(False)

    def _emit_wps(self) -> None:
        pts = [(w.x, w.y) for w in self.cur.waypoints] if self.cur else []
        self.view.set_waypoints(pts)
        self.waypoints_changed.emit(pts)

    def _save(self) -> None:
        if self.cur:
            self.store.upsert(self.cur)

    # ---- edits ----

    def _new_mission(self) -> None:
        m = Mission.new(f"Mission {len(self.store.missions)+1}")
        self.store.upsert(m)
        self.cur = m
        self._reload_missions()
        self._set_combo_to(m.id)

    def _duplicate(self) -> None:
        if not self.cur:
            return
        m = Mission.new(self.cur.name + " (copy)")
        m.loop = self.cur.loop
        m.waypoints = [Waypoint(**{**w.__dict__, "id": w.id + "c"}) for w in self.cur.waypoints]
        self.store.upsert(m)
        self._reload_missions(); self._set_combo_to(m.id)

    def _delete(self) -> None:
        if not self.cur:
            return
        if QMessageBox.question(self, "Delete", f"Delete '{self.cur.name}'?") != QMessageBox.Yes:
            return
        self.store.remove(self.cur.id)
        self.cur = None
        self._reload_missions()

    def _set_combo_to(self, mid: str) -> None:
        for i in range(self.mission_combo.count()):
            if self.mission_combo.itemData(i) == mid:
                self.mission_combo.setCurrentIndex(i)
                return

    def _rename(self) -> None:
        if self.cur:
            self.cur.name = self.name_edit.text().strip() or self.cur.name
            self._save()
            idx = self.mission_combo.currentIndex()
            if idx >= 0:
                self.mission_combo.setItemText(idx, self.cur.name)

    def _loop_toggle(self, on: bool) -> None:
        if self.cur:
            self.cur.loop = on
            self._save()

    def _add_here(self) -> None:
        if not self.cur:
            return
        wp = Waypoint.new(f"wp{len(self.cur.waypoints)+1}", self._robot_x, self._robot_y)
        wp.tolerance = self.tol_sb.value()
        wp.dwell = self.dwell_sb.value()
        self.cur.waypoints.append(wp)
        self._save(); self._refresh_table(); self._emit_wps()

    def add_waypoint_at(self, x: float, y: float) -> None:
        self._on_map_click(x, y)

    def _on_map_click(self, x: float, y: float) -> None:
        if not self.cur:
            return
        wp = Waypoint.new(f"wp{len(self.cur.waypoints)+1}", x, y)
        wp.tolerance = self.tol_sb.value()
        wp.dwell = self.dwell_sb.value()
        self.cur.waypoints.append(wp)
        self._save(); self._refresh_table(); self._emit_wps()

    def _on_table_edit(self, item: QTableWidgetItem) -> None:
        if not self.cur:
            return
        row = item.row()
        col = item.column()
        if row >= len(self.cur.waypoints):
            return
        wp = self.cur.waypoints[row]
        try:
            if col == 1:
                wp.name = item.text()
            elif col == 2:
                wp.x = float(item.text())
            elif col == 3:
                wp.y = float(item.text())
            elif col == 4:
                wp.yaw = math.radians(float(item.text()))
        except ValueError:
            self._refresh_table()
            return
        self._save(); self._emit_wps()

    def _move(self, delta: int) -> None:
        if not self.cur:
            return
        r = self.table.currentRow()
        if r < 0:
            return
        nr = r + delta
        if nr < 0 or nr >= len(self.cur.waypoints):
            return
        self.cur.waypoints[r], self.cur.waypoints[nr] = self.cur.waypoints[nr], self.cur.waypoints[r]
        self._save(); self._refresh_table()
        self.table.setCurrentCell(nr, 0)
        self._emit_wps()

    def _remove(self) -> None:
        if not self.cur:
            return
        r = self.table.currentRow()
        if r < 0:
            return
        del self.cur.waypoints[r]
        self._save(); self._refresh_table(); self._emit_wps()

    # ---- robot pose feed-in (so "Set here" works) ----

    def update_robot_pose(self, x: float, y: float, yaw: float) -> None:
        self._robot_x = x
        self._robot_y = y
        self.view.set_robot_pose(x, y, yaw)

    # ---- executor ----

    def _play(self) -> None:
        if not self.cur or not self.cur.waypoints:
            return
        if self._exec_idx < 0:
            self._exec_idx = 0
        self._exec_running = True
        self._dispatch_current()

    def _pause(self) -> None:
        self._exec_running = False
        self.status_lbl.setText("paused")
        self.cancel_requested.emit()

    def stop_mission(self) -> None:
        if self._exec_running or self._exec_idx >= 0:
            self._stop()

    def _stop(self) -> None:
        self._exec_running = False
        self._exec_idx = -1
        self.status_lbl.setText("stopped")
        self.cancel_requested.emit()

    MAX_RETRIES = 3

    def on_nav_status(self, status: str) -> None:
        """Retry the current waypoint when Nav2 drops or aborts the goal."""
        if not self._exec_running or not self.cur or self._exec_idx < 0:
            return
        self._nav_state = status
        # "succeeded" while not at the waypoint happens when a goal preempts
        # one that was about to finish (Nav2 reports the old goal's success).
        if status not in ("no_response", "aborted", "succeeded"):
            return
        wp = self.cur.waypoints[self._exec_idx]
        if math.hypot(self._robot_x - wp.x, self._robot_y - wp.y) <= wp.tolerance:
            return                      # already there; the tick will advance
        self._retries += 1
        if self._retries > self.MAX_RETRIES:
            self._stop()
            self.status_lbl.setText(f"stopped: Nav2 could not reach {wp.name} ({status})")
            return
        self.status_lbl.setText(f"{status} at {wp.name}, retry {self._retries}/{self.MAX_RETRIES}…")
        QTimer.singleShot(2000, self._retry_current)

    def _retry_current(self) -> None:
        if self._exec_running and self._exec_idx >= 0:
            self._dispatch_current(reset_retries=False)

    def _dispatch_current(self, reset_retries: bool = True) -> None:
        if not self.cur:
            return
        if reset_retries:
            self._retries = 0
        self._nav_state = "pending"
        self._in_tol_since = 0.0
        wp = self.cur.waypoints[self._exec_idx]
        self.status_lbl.setText(f"going to {wp.name} ({self._exec_idx+1}/{len(self.cur.waypoints)})")
        self.goal_requested.emit(wp.x, wp.y, wp.yaw)

    def _exec_tick(self) -> None:
        if not self._exec_running or not self.cur or self._exec_idx < 0:
            return
        wp = self.cur.waypoints[self._exec_idx]
        d = math.hypot(self._robot_x - wp.x, self._robot_y - wp.y)
        if d > wp.tolerance:
            self._in_tol_since = 0.0
        elif self._exec_dwell_until == 0:
            # Let Nav2 finish its own goal before sending the next one:
            # preempting a goal that is about to complete confuses Nav2.
            # Fall back to tolerance alone if no status arrives for 5 s.
            now = time.monotonic()
            if not self._in_tol_since:
                self._in_tol_since = now
            if self._nav_state in ("active", "pending") and now - self._in_tol_since < 5.0:
                return
        if d <= wp.tolerance:
            if self._exec_dwell_until == 0:
                self._exec_dwell_until = time.monotonic() + wp.dwell
                self.status_lbl.setText(f"arrived {wp.name} — dwell {wp.dwell:.1f} s")
                return
            if time.monotonic() < self._exec_dwell_until:
                return
            self._exec_dwell_until = 0.0
            self._exec_idx += 1
            if self._exec_idx >= len(self.cur.waypoints):
                if self.cur.loop:
                    self._exec_idx = 0
                else:
                    self._exec_running = False
                    self._exec_idx = -1
                    self.status_lbl.setText("mission complete")
                    return
            self._dispatch_current()
