"""Events page: the run's history, newest first, with filters and CSV export."""
from __future__ import annotations

from typing import List

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView, QComboBox, QFileDialog, QHBoxLayout, QHeaderView, QLabel,
    QMessageBox, QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from ...core.events import Event, EventLog, write_csv

COLORS = {"info": "#c6cad6", "warn": "#ffb547", "error": "#ff6a6a"}
LEVEL_FILTERS = [("All events", ("info", "warn", "error")),
                 ("Warnings and errors", ("warn", "error")),
                 ("Errors only", ("error",))]


class EventsPanel(QWidget):
    def __init__(self, log: EventLog, parent=None):
        super().__init__(parent)
        self.log = log
        v = QVBoxLayout(self)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(6)

        bar = QHBoxLayout()
        self.level = QComboBox()
        for label, levels in LEVEL_FILTERS:
            self.level.addItem(label, levels)
        self.level.currentIndexChanged.connect(self.refresh)
        self.source = QComboBox()
        self.source.currentIndexChanged.connect(self.refresh)
        export = QPushButton("Export CSV…")
        export.clicked.connect(self._export)
        clear = QPushButton("Clear")
        clear.clicked.connect(self._clear)
        bar.addWidget(self.level)
        bar.addWidget(self.source)
        self.count = QLabel("")
        self.count.setProperty("muted", True)
        bar.addWidget(self.count, 1)
        bar.addWidget(export)
        bar.addWidget(clear)
        v.addLayout(bar)

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Time", "Source", "Event"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setShowGrid(False)
        h = self.table.horizontalHeader()
        h.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        h.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        h.setSectionResizeMode(2, QHeaderView.Stretch)
        v.addWidget(self.table, 1)

        log.added.connect(self._on_added)
        log.cleared.connect(self.refresh)
        self._sync_sources()
        self.refresh()

    def _sync_sources(self) -> None:
        cur = self.source.currentData()
        self.source.blockSignals(True)
        self.source.clear()
        self.source.addItem("All sources", "")
        for s in self.log.sources():
            self.source.addItem(s, s)
        i = self.source.findData(cur)
        self.source.setCurrentIndex(max(0, i))
        self.source.blockSignals(False)

    def shown(self) -> List[Event]:
        """Events passing the filters, oldest first."""
        levels, src = self.level.currentData(), self.source.currentData()
        return [e for e in self.log.events if e.level in levels and (not src or e.source == src)]

    def _insert(self, row: int, e: Event) -> None:
        self.table.insertRow(row)
        col = QColor(COLORS.get(e.level, COLORS["info"]))
        for c, text in enumerate((e.time_str(), e.source, e.text)):
            it = QTableWidgetItem(text)
            it.setForeground(col)
            if c == 2:
                it.setToolTip(text)
            self.table.setItem(row, c, it)

    def refresh(self, *_a) -> None:
        self._sync_sources()
        ev = self.shown()
        self.table.setUpdatesEnabled(False)
        self.table.setRowCount(0)
        for e in reversed(ev):
            self._insert(self.table.rowCount(), e)
        self.table.setUpdatesEnabled(True)
        self.count.setText(f"{len(ev)} of {len(self.log.events)}")

    def _on_added(self, e: Event) -> None:
        if self.source.findData(e.source) < 0:
            self._sync_sources()
        if e in self.shown()[-1:]:
            self._insert(0, e)
            while self.table.rowCount() > self.log.events.maxlen:
                self.table.removeRow(self.table.rowCount() - 1)
        self.count.setText(f"{self.table.rowCount()} of {len(self.log.events)}")

    def _export(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Export events", "rcs_events.csv", "CSV (*.csv)")
        if not path:
            return
        if not path.lower().endswith(".csv"):
            path += ".csv"
        try:
            n = write_csv(path, self.shown())
        except OSError as e:
            QMessageBox.warning(self, "Export events", f"Cannot write {path}:\n{e}")
            return
        self.count.setText(f"saved {n} events to {path}")

    def _clear(self) -> None:
        if QMessageBox.question(self, "Clear events", "Delete all events?") == QMessageBox.Yes:
            self.log.clear()
