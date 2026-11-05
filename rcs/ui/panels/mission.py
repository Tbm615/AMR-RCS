"""Mission panel: list of waypoints (checkpoints), loop route, executor."""
from __future__ import annotations

import copy
import json
import math
import time
import uuid
from pathlib import Path
from typing import Optional

from PySide6.QtCore import QItemSelectionModel, Qt, QTimer, Signal
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QGroupBox,
    QHBoxLayout, QHeaderView, QInputDialog, QLabel, QLineEdit, QMessageBox, QPushButton,
    QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from ...core.mission import Mission, MissionStore, Waypoint
from ..widgets.map_view import MapView


class MissionPanel(QWidget):
    goal_requested = Signal(float, float, float)
    cancel_requested = Signal()
    waypoints_changed = Signal(object)   # list[(x,y)] for map overlay
    event = Signal(str, str)             # level, text — for the Events page

    def __init__(self, store: MissionStore, parent=None):
        super().__init__(parent)
        self.store = store
        self.view = MapView()
        self.view.set_click_mode("waypoint")
        self.view.waypoint_requested.connect(self._on_map_click)
        self.cur: Optional[Mission] = None
        self._exec_idx: int = -1
        self._exec_running: bool = False
        self._exec_dwell_until: float = 0.0
        self._retries = 0
        self._nav_state = ""
        self._in_tol_since = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._lap = 0                   # laps finished in this run
        self._undo_stack: list = []     # waypoint lists of the open mission
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
        lr = QHBoxLayout()
        self.loop_ck = QCheckBox("Loop route")
        self.loop_ck.setToolTip("Start again from the first waypoint after the last one")
        self.loop_ck.toggled.connect(self._loop_toggle)
        lr.addWidget(self.loop_ck)
        lr.addStretch(1)
        lr.addWidget(QLabel("Laps"))
        self.laps_sb = QSpinBox()
        self.laps_sb.setRange(0, 9999)
        self.laps_sb.setSpecialValueText("until stopped")
        self.laps_sb.setToolTip("How many times to run the route; 0 = until you stop it")
        self.laps_sb.valueChanged.connect(self._laps_changed)
        lr.addWidget(self.laps_sb)
        mv.addLayout(lr)
        fr = QHBoxLayout()
        imp = QPushButton("Import…"); imp.clicked.connect(self._import)
        exp = QPushButton("Export…"); exp.clicked.connect(self._export)
        imp.setToolTip("Load a mission from a file (e.g. from another PC)")
        exp.setToolTip("Save this mission to a file")
        fr.addWidget(imp); fr.addWidget(exp)
        mv.addLayout(fr)
        side.addWidget(mbox)

        wbox = QGroupBox("Waypoints — Ctrl/Shift-click to select several")
        wv = QVBoxLayout(wbox)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["#", "Name", "x (m)", "y (m)", "yaw°"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        # Ctrl/Shift-click or drag to select several rows
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.table.itemChanged.connect(self._on_table_edit)
        self.table.itemSelectionChanged.connect(self._on_selection)
        # Delete only while the table (not a cell editor) has focus
        dk = QShortcut(QKeySequence.Delete, self.table, self._remove)
        dk.setContext(Qt.WidgetShortcut)
        QShortcut(QKeySequence.Undo, self, self._undo).setContext(Qt.WidgetWithChildrenShortcut)
        wv.addWidget(self.table)
        row2 = QHBoxLayout()
        here = QPushButton("Set here"); here.clicked.connect(self._add_here)
        up = QPushButton("↑"); up.clicked.connect(lambda: self._move(-1))
        down = QPushButton("↓"); down.clicked.connect(lambda: self._move(1))
        rm = QPushButton("Remove"); rm.setToolTip("Remove the selected waypoints (Delete)")
        rm.clicked.connect(self._remove)
        clr = QPushButton("Clear all"); clr.clicked.connect(self._clear_all)
        self.undo_btn = QPushButton("Undo"); self.undo_btn.setToolTip("Undo the last change (Ctrl+Z)")
        self.undo_btn.clicked.connect(self._undo)
        self.undo_btn.setEnabled(False)
        for b in (here, up, down):
            row2.addWidget(b)
        wv.addLayout(row2)
        row3 = QHBoxLayout()
        self.sel_lbl = QLabel(""); self.sel_lbl.setProperty("muted", True)
        row3.addWidget(self.sel_lbl, 1)
        row3.addWidget(self.undo_btn); row3.addWidget(rm); row3.addWidget(clr)
        wv.addLayout(row3)
        side.addWidget(wbox)

        tolbox = QGroupBox("Waypoint settings")
        tv = QVBoxLayout(tolbox)
        r = QHBoxLayout()
        self.tol_sb = QDoubleSpinBox(); self.tol_sb.setRange(0.05, 5.0); self.tol_sb.setValue(0.25); self.tol_sb.setSuffix(" m")
        self.dwell_sb = QDoubleSpinBox(); self.dwell_sb.setRange(0.0, 60.0); self.dwell_sb.setValue(0.0); self.dwell_sb.setSuffix(" s")
        r.addWidget(QLabel("Tolerance:")); r.addWidget(self.tol_sb)
        r.addWidget(QLabel("Dwell:")); r.addWidget(self.dwell_sb)
        tv.addLayout(r)
        self.apply_btn = QPushButton("Apply to selected")
        self.apply_btn.setToolTip("New waypoints use these values; this button also sets them on the selected rows")
        self.apply_btn.clicked.connect(self._apply_to_selected)
        self.apply_btn.setEnabled(False)
        tv.addWidget(self.apply_btn)
        side.addWidget(tolbox)

        ebox = QGroupBox("Execute")
        ev = QVBoxLayout(ebox)
        self.status_lbl = QLabel("stopped"); self.status_lbl.setProperty("muted", True)
        ev.addWidget(self.status_lbl)
        er = QHBoxLayout()
        self.play_btn = QPushButton("▶ Play"); self.play_btn.setProperty("kind", "success")
        self.play_btn.clicked.connect(lambda: self._play())
        self.pause_btn = QPushButton("⏸ Pause"); self.pause_btn.clicked.connect(self._pause)
        self.stop_btn = QPushButton("■ Stop"); self.stop_btn.setProperty("kind", "danger")
        self.stop_btn.clicked.connect(self._stop)
        for b in (self.play_btn, self.pause_btn, self.stop_btn):
            er.addWidget(b)
        ev.addLayout(er)
        self.from_btn = QPushButton("▶ Start from selected waypoint")
        self.from_btn.clicked.connect(self._play_from_selected)
        self.from_btn.setEnabled(False)
        ev.addWidget(self.from_btn)
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
        self.laps_sb.blockSignals(True)
        self.laps_sb.setValue(self.cur.laps)
        self.laps_sb.setEnabled(self.cur.loop)
        self.laps_sb.blockSignals(False)
        self._undo_stack.clear()
        self.undo_btn.setEnabled(False)
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
                tip = f"tolerance {wp.tolerance:.2f} m · dwell {wp.dwell:.1f} s"
                for c in range(5):
                    self.table.item(i, c).setToolTip(tip)
        self.table.blockSignals(False)
        self._on_selection()

    def _emit_wps(self) -> None:
        pts = ([(w.x, w.y, f"{i + 1}  {w.name}", w.yaw) for i, w in enumerate(self.cur.waypoints)]
               if self.cur else [])
        self.view.set_waypoints(pts)
        self.view.set_waypoint_selection(self.selected_rows())
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
        self.laps_sb.setEnabled(on)
        if self.cur:
            self.cur.loop = on
            self._save()

    def _laps_changed(self, n: int) -> None:
        if self.cur:
            self.cur.laps = n
            self._save()

    # ---- undo ----

    UNDO_DEPTH = 30

    def _remember(self) -> None:
        """Call before changing the waypoints of the open mission."""
        self._undo_stack.append(copy.deepcopy(self.cur.waypoints))
        del self._undo_stack[:-self.UNDO_DEPTH]
        self.undo_btn.setEnabled(True)

    def _undo(self) -> None:
        if not self.cur or not self._undo_stack or self._blocked_while_running():
            return
        self.cur.waypoints = self._undo_stack.pop()
        self.undo_btn.setEnabled(bool(self._undo_stack))
        self._save(); self._refresh_table(); self._emit_wps()

    # ---- files ----

    def _export(self) -> None:
        if not self.cur:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export mission", f"{self.cur.name}.json", "Mission (*.json)")
        if path:
            self.export_to(path)

    def export_to(self, path: str) -> None:
        if not path.lower().endswith(".json"):
            path += ".json"
        try:
            Path(path).write_text(json.dumps(self.cur.to_dict(), indent=2))
        except OSError as e:
            QMessageBox.warning(self, "Export mission", f"Cannot write {path}:\n{e}")
            return
        self.status_lbl.setText(f"exported to {path}")

    def _import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Import mission", "", "Mission (*.json)")
        if path:
            self.import_from(path)

    def import_from(self, path: str) -> Optional[Mission]:
        try:
            m = Mission.from_dict(json.loads(Path(path).read_text()))
        except (OSError, ValueError, TypeError, AttributeError) as e:
            QMessageBox.warning(self, "Import mission", f"Cannot read {path}:\n{e}")
            return None
        m.id = str(uuid.uuid4())[:8]            # never overwrite a mission here
        names = {x.name for x in self.store.missions}
        base, n = m.name, 2
        while m.name in names:
            m.name = f"{base} ({n})"
            n += 1
        self.store.upsert(m)
        self._reload_missions()
        self._set_combo_to(m.id)
        return m

    def _add_here(self) -> None:
        if not self.cur:
            return
        wp = Waypoint.new(f"wp{len(self.cur.waypoints)+1}", self._robot_x, self._robot_y,
                          getattr(self, "_robot_yaw", 0.0))
        wp.tolerance = self.tol_sb.value()
        wp.dwell = self.dwell_sb.value()
        self._remember()
        self.cur.waypoints.append(wp)
        self._save(); self._refresh_table(); self._emit_wps()

    def add_waypoint_at(self, x: float, y: float, yaw: float = 0.0, name: str = "") -> None:
        self._on_map_click(x, y, yaw, name)

    def _on_map_click(self, x: float, y: float, yaw: float = 0.0, name: str = "") -> None:
        if not self.cur:
            return
        wp = Waypoint.new(name or f"wp{len(self.cur.waypoints)+1}", x, y)
        wp.yaw = yaw
        wp.tolerance = self.tol_sb.value()
        wp.dwell = self.dwell_sb.value()
        self._remember()
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
        before = copy.deepcopy(self.cur.waypoints)
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
        self._undo_stack.append(before)
        self.undo_btn.setEnabled(True)
        self._save(); self._emit_wps()

    def selected_rows(self) -> list:
        return sorted({i.row() for i in self.table.selectionModel().selectedRows()})

    def select_rows(self, rows) -> None:
        self.table.clearSelection()
        sm, model = self.table.selectionModel(), self.table.model()
        for r in rows:
            sm.select(model.index(r, 0), QItemSelectionModel.Select | QItemSelectionModel.Rows)
        if rows:
            self.table.setCurrentCell(rows[0], 1, QItemSelectionModel.NoUpdate)

    def _on_selection(self) -> None:
        rows = self.selected_rows()
        self.view.set_waypoint_selection(rows)
        self.apply_btn.setEnabled(bool(rows))
        self.from_btn.setEnabled(len(rows) == 1)
        self.sel_lbl.setText(f"{len(rows)} selected" if len(rows) > 1 else "")

    def _blocked_while_running(self) -> bool:
        """Reordering or removing waypoints under a running or paused mission
        would make the executor drive to the wrong one."""
        if self._exec_idx >= 0:
            self.status_lbl.setText("stop the mission before reordering or removing waypoints")
            return True
        return False

    def _move(self, delta: int) -> None:
        if not self.cur or self._blocked_while_running():
            return
        rows = self.selected_rows()
        n = len(self.cur.waypoints)
        if not rows or (delta < 0 and rows[0] == 0) or (delta > 0 and rows[-1] == n - 1):
            return
        self._remember()
        wps = self.cur.waypoints
        for r in (rows if delta < 0 else reversed(rows)):
            wps[r], wps[r + delta] = wps[r + delta], wps[r]
        self._save(); self._refresh_table(); self._emit_wps()
        self.select_rows([r + delta for r in rows])

    def _remove(self) -> None:
        if not self.cur or self._blocked_while_running():
            return
        rows = self.selected_rows()
        if not rows:
            return
        self._remember()
        for r in reversed(rows):
            del self.cur.waypoints[r]
        self._save(); self._refresh_table(); self._emit_wps()
        if self.cur.waypoints:      # keep a selection so Delete can be pressed again
            self.select_rows([min(rows[0], len(self.cur.waypoints) - 1)])

    def _clear_all(self) -> None:
        if not self.cur or not self.cur.waypoints or self._blocked_while_running():
            return
        if QMessageBox.question(self, "Clear all",
                                f"Remove all {len(self.cur.waypoints)} waypoints of '{self.cur.name}'?") \
                != QMessageBox.Yes:
            return
        self._remember()
        self.cur.waypoints = []
        self._save(); self._refresh_table(); self._emit_wps()

    def _apply_to_selected(self) -> None:
        if not self.cur:
            return
        rows = self.selected_rows()
        self._remember()
        for r in rows:
            self.cur.waypoints[r].tolerance = self.tol_sb.value()
            self.cur.waypoints[r].dwell = self.dwell_sb.value()
        self._save(); self._refresh_table()
        self.select_rows(rows)
        self.status_lbl.setText(f"tolerance {self.tol_sb.value():.2f} m, dwell {self.dwell_sb.value():.1f} s "
                                f"applied to {len(rows)} waypoint(s)")

    # ---- robot pose feed-in (so "Set here" works) ----

    def update_robot_pose(self, x: float, y: float, yaw: float, draw: bool = True) -> None:
        self._robot_x = x
        self._robot_y = y
        self._robot_yaw = yaw
        if draw:
            self.view.set_robot_pose(x, y, yaw)

    # ---- executor ----

    def _play_from_selected(self) -> None:
        rows = self.selected_rows()
        if len(rows) == 1 and not self._exec_running:
            self._play(rows[0])

    def _play(self, start: Optional[int] = None) -> None:
        if not self.cur or not self.cur.waypoints:
            return
        if start is not None:
            self._exec_idx = start
            self._lap = 0
        elif self._exec_idx < 0:
            self._exec_idx = 0
            self._lap = 0
        self._exec_running = True
        self.event.emit("info", f"'{self.cur.name}' started ({len(self.cur.waypoints)} waypoints)")
        self._dispatch_current()

    def _pause(self) -> None:
        if self._exec_running and self.cur:
            self.event.emit("info", f"'{self.cur.name}' paused")
        self._exec_running = False
        self.status_lbl.setText("paused")
        self.cancel_requested.emit()

    def stop_mission(self) -> None:
        if self._exec_running or self._exec_idx >= 0:
            self._stop()

    def _stop(self) -> None:
        if self._exec_idx >= 0 and self.cur:
            self.event.emit("info", f"'{self.cur.name}' stopped")
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
            self.event.emit("error", f"Nav2 could not reach {wp.name} ({status}), mission stopped")
            return
        self.status_lbl.setText(f"{status} at {wp.name}, retry {self._retries}/{self.MAX_RETRIES}…")
        self.event.emit("warn", f"{status} at {wp.name}, retry {self._retries}/{self.MAX_RETRIES}")
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
        lap = ""
        if self.cur.loop:
            lap = f" · lap {self._lap + 1}" + (f"/{self.cur.laps}" if self.cur.laps else "")
        self.status_lbl.setText(f"going to {wp.name} ({self._exec_idx+1}/{len(self.cur.waypoints)}){lap}")
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
                self.event.emit("info", f"arrived at {wp.name}")
                return
            if time.monotonic() < self._exec_dwell_until:
                return
            self._exec_dwell_until = 0.0
            self._exec_idx += 1
            if self._exec_idx >= len(self.cur.waypoints):
                self._lap += 1
                if self.cur.loop and not (self.cur.laps and self._lap >= self.cur.laps):
                    self._exec_idx = 0
                else:
                    self._exec_running = False
                    self._exec_idx = -1
                    self.status_lbl.setText("mission complete")
                    self.event.emit("info", f"'{self.cur.name}' complete")
                    return
            self._dispatch_current()
