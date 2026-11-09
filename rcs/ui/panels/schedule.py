"""Schedule page: missions that start by themselves, and a report of every run."""
from __future__ import annotations

import time
from typing import Optional

from PySide6.QtCore import QTime, Qt, QTimer, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFileDialog, QFormLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QPushButton, QSpinBox, QSplitter, QTableWidget,
    QTableWidgetItem, QTimeEdit, QVBoxLayout, QWidget,
)

from ...core.runs import RunLog
from ...core.schedule import DAYS, Schedule, ScheduleStore

RESULT_COLOR = {"complete": "#39d98a", "skipped": "#ffb547", "failed": "#ff6a6a",
                "interrupted": "#ffb547", "stopped": "#8b93a7", "running": "#3b82f6"}


def _dur(s: float) -> str:
    s = int(s or 0)
    return f"{s // 3600} h {s % 3600 // 60:02d} min" if s >= 3600 else f"{s // 60} min {s % 60:02d} s"


class ScheduleDialog(QDialog):
    def __init__(self, missions: list, s: Optional[Schedule], parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Schedule" if s else "New schedule")
        self.s = s
        form = QFormLayout(self)
        self.mission = QComboBox()
        for m in missions:
            self.mission.addItem(m.name, m.id)
        if s:
            self.mission.setCurrentIndex(max(0, self.mission.findData(s.mission_id)))
        form.addRow("Mission", self.mission)
        self.at = QTimeEdit()
        self.at.setDisplayFormat("HH:mm")
        h, m = (s.at if s else "08:00").split(":")
        self.at.setTime(QTime(int(h), int(m)))
        form.addRow("Start at", self.at)
        days = QHBoxLayout()
        self.days = []
        for i, d in enumerate(DAYS):
            c = QCheckBox(d)
            c.setChecked(i in (s.days if s else range(5)))
            days.addWidget(c)
            self.days.append(c)
        form.addRow("On", days)
        self.min_soc = QSpinBox()
        self.min_soc.setRange(0, 100)
        self.min_soc.setSuffix(" %")
        self.min_soc.setValue(s.min_soc if s else 30)
        self.min_soc.setToolTip("Below this charge the run is skipped. 0: do not check the battery")
        form.addRow("Battery at least", self.min_soc)
        self.enabled = QCheckBox("On")
        self.enabled.setChecked(s.enabled if s else True)
        form.addRow("", self.enabled)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)

    def result_schedule(self) -> Schedule:
        s = Schedule(id=self.s.id if self.s else Schedule.new("").id, mission_id=self.mission.currentData() or "")
        s.at = self.at.time().toString("HH:mm")
        s.days = [i for i, c in enumerate(self.days) if c.isChecked()]
        s.min_soc = self.min_soc.value()
        s.enabled = self.enabled.isChecked()
        return s


class SchedulePanel(QWidget):
    changed = Signal()

    def __init__(self, store: ScheduleStore, runs: RunLog, missions, parent=None) -> None:
        super().__init__(parent)
        self.store, self.runs, self.missions = store, runs, missions
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        hint = QLabel("Scheduled missions start only while this app is open and connected to the robot. "
                      "When the robot is not ready at that time, the run is skipped and the report says why.")
        hint.setWordWrap(True)
        hint.setProperty("muted", True)
        root.addWidget(hint)
        split = QSplitter(Qt.Vertical)
        root.addWidget(split, 1)

        top = QGroupBox("Schedules")
        tv = QVBoxLayout(top)
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["On", "Mission", "At", "Days", "Battery at least", "Next run"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        self.table.cellDoubleClicked.connect(lambda _r, _c: self._edit())
        tv.addWidget(self.table)
        row = QHBoxLayout()
        self.add_btn = QPushButton("Add…")
        self.add_btn.clicked.connect(self._add)
        self.edit_btn = QPushButton("Edit…")
        self.edit_btn.clicked.connect(self._edit)
        self.toggle_btn = QPushButton("On / off")
        self.toggle_btn.clicked.connect(self._toggle)
        self.del_btn = QPushButton("Delete")
        self.del_btn.clicked.connect(self._delete)
        for b in (self.add_btn, self.edit_btn, self.toggle_btn, self.del_btn):
            row.addWidget(b)
        row.addStretch(1)
        tv.addLayout(row)
        split.addWidget(top)

        bottom = QGroupBox("Run reports")
        bv = QVBoxLayout(bottom)
        self.reports = QTableWidget(0, 11)
        self.reports.setHorizontalHeaderLabels(["Started", "Mission", "By", "Result", "Length", "Waypoints",
                                                "Laps", "Retries", "Distance", "Battery", "Why"])
        self.reports.verticalHeader().setVisible(False)
        self.reports.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.reports.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.reports.setSelectionMode(QAbstractItemView.SingleSelection)
        self.reports.horizontalHeader().setSectionResizeMode(10, QHeaderView.Stretch)
        self.reports.itemSelectionChanged.connect(self._show_details)
        bv.addWidget(self.reports, 1)
        self.details = QLabel("")
        self.details.setWordWrap(True)
        self.details.setProperty("muted", True)
        self.details.setTextInteractionFlags(Qt.TextSelectableByMouse)
        bv.addWidget(self.details)
        row = QHBoxLayout()
        self.summary = QLabel("")
        row.addWidget(self.summary, 1)
        self.export_btn = QPushButton("Export CSV…")
        self.export_btn.clicked.connect(self._export)
        row.addWidget(self.export_btn)
        bv.addLayout(row)
        split.addWidget(bottom)
        split.setSizes([260, 420])

        runs.changed.connect(self.refresh_reports)
        self._clock = QTimer(self)          # "next run" moves on with the time
        self._clock.setInterval(30000)
        self._clock.timeout.connect(self.refresh)
        self._clock.start()
        self.refresh()
        self.refresh_reports()

    # ---- schedules ----

    def _mission_name(self, mid: str) -> str:
        m = self.missions.get(mid)
        return m.name if m else "(deleted mission)"

    def refresh(self) -> None:
        sel = self._selected()
        self.table.setRowCount(len(self.store.schedules))
        now = time.time()
        for r, s in enumerate(self.store.schedules):
            nxt = s.next_time(now) if s.enabled else None
            cells = ["on" if s.enabled else "off", self._mission_name(s.mission_id), s.at, s.days_text(),
                     f"{s.min_soc}%" if s.min_soc else "not checked",
                     time.strftime("%a %Y-%m-%d %H:%M", time.localtime(nxt)) if nxt else "—"]
            for c, text in enumerate(cells):
                item = QTableWidgetItem(text)
                item.setData(Qt.UserRole, s.id)
                if not s.enabled:
                    item.setForeground(QColor("#8b93a7"))
                self.table.setItem(r, c, item)
            if s.id == (sel.id if sel else None):
                self.table.selectRow(r)
        self.table.resizeColumnsToContents()
        self.table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)

    def _selected(self) -> Optional[Schedule]:
        rows = {i.row() for i in self.table.selectedItems()}
        if len(rows) != 1:
            return None
        item = self.table.item(rows.pop(), 0)
        return self.store.get(item.data(Qt.UserRole)) if item else None

    def _add(self) -> None:
        if not self.missions.missions:
            self.details.setText("Make a mission first, on the Missions page.")
            return
        dlg = ScheduleDialog(self.missions.missions, None, self)
        if dlg.exec() == QDialog.Accepted:
            self.save(dlg.result_schedule())

    def _edit(self) -> None:
        s = self._selected()
        if s is None:
            return
        dlg = ScheduleDialog(self.missions.missions, s, self)
        if dlg.exec() == QDialog.Accepted:
            self.save(dlg.result_schedule())

    def save(self, s: Schedule) -> None:
        self.store.upsert(s)
        self.refresh()
        self.changed.emit()

    def _toggle(self) -> None:
        s = self._selected()
        if s is not None:
            s.enabled = not s.enabled
            self.save(s)

    def _delete(self) -> None:
        s = self._selected()
        if s is not None:
            self.store.remove(s.id)
            self.refresh()
            self.changed.emit()

    # ---- reports ----

    def refresh_reports(self) -> None:
        runs = list(reversed(self.runs.runs))
        self.reports.setRowCount(len(runs))
        for r, run in enumerate(runs):
            batt = ""
            if run.soc_start is not None:
                batt = f"{run.soc_start:.0f}%" + (f" → {run.soc_end:.0f}%" if run.soc_end is not None else "")
            cells = [time.strftime("%Y-%m-%d %H:%M", time.localtime(run.start)), run.mission,
                     "schedule" if run.trigger == "schedule" else "by hand", run.result,
                     "" if run.result == "skipped" else _dur(run.duration()),
                     "" if run.result == "skipped" else f"{run.reached}",
                     "" if run.result == "skipped" else str(run.laps),
                     "" if run.result == "skipped" else str(run.retries),
                     "" if run.result == "skipped" else f"{run.distance_m:.0f} m", batt, run.reason]
            for c, text in enumerate(cells):
                item = QTableWidgetItem(text)
                item.setData(Qt.UserRole, run.id)
                if c == 3:
                    item.setForeground(QColor(RESULT_COLOR.get(run.result, "#e6e8ee")))
                self.reports.setItem(r, c, item)
        self.reports.resizeColumnsToContents()
        self.reports.horizontalHeader().setSectionResizeMode(10, QHeaderView.Stretch)
        done = [r for r in self.runs.runs if r.result != "running"]
        ok = sum(r.result == "complete" for r in done)
        self.summary.setText(f"{len(done)} runs: {ok} complete, "
                             f"{sum(r.result == 'skipped' for r in done)} skipped, "
                             f"{sum(r.result in ('failed', 'interrupted') for r in done)} failed or interrupted")

    def _show_details(self) -> None:
        rows = {i.row() for i in self.reports.selectedItems()}
        if len(rows) != 1:
            self.details.setText("")
            return
        rid = self.reports.item(rows.pop(), 0).data(Qt.UserRole)
        run = next((r for r in self.runs.runs if r.id == rid), None)
        if run is None:
            return
        lines = run.events[-30:]
        more = f"… {len(run.events) - 30} earlier\n" if len(run.events) > 30 else ""
        self.details.setText(f"{run.mission}: {run.result}" + (f" ({run.reason})" if run.reason else "")
                             + ("\n" + more + "\n".join(lines) if lines else ""))

    def _export(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Export run reports",
                                              f"runs_{time.strftime('%Y%m%d')}.csv", "CSV (*.csv)")
        if path:
            n = self.runs.export_csv(path)
            self.details.setText(f"{n} runs written to {path}")
