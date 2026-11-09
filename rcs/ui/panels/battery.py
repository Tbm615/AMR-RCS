"""Battery page.

Now: what the robot reads from its BMS right now, and how the link is doing.
History: the robot's battery log (one file per boot, a row a second), as a
chart of one measure, one boot at a time or a trend over all boots, and a
table of the boots with how each one ended (shut down, or power lost).
"""
from __future__ import annotations

import time
from typing import Optional

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView, QButtonGroup, QComboBox, QFrame, QGridLayout, QGroupBox, QHBoxLayout,
    QHeaderView, QLabel, QMessageBox, QProgressBar, QPushButton, QSplitter, QTableWidget,
    QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget,
)

from ...core.battery import CRITICAL, LOW, link_text, live
from ...core.battery_history import METRICS, TRENDS, how_it_ended, series, trend
from ..widgets.chart import TimeChart

GREEN, RED, AMBER, MUTED = "#39d98a", "#ff6a6a", "#ffb547", "#8b93a7"
LEVEL = {"ok": GREEN, "warn": AMBER, "error": RED}
TREND = "__trend__"
FOLLOW_MS = 10000       # the running boot's log is fetched again this often while shown


def _dur(s: float) -> str:
    s = int(s or 0)
    return f"{s // 3600} h {s % 3600 // 60:02d} min" if s >= 3600 else f"{s // 60} min {s % 60:02d} s"


def _bytes(n) -> str:
    if n is None:
        return "?"
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if n < 1024 or unit == "TB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024


class _Value(QFrame):
    def __init__(self, title: str, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("Card")
        v = QVBoxLayout(self)
        v.setContentsMargins(14, 10, 14, 12)
        v.setSpacing(2)
        t = QLabel(title)
        t.setProperty("muted", True)
        self.value = QLabel("—")
        self.value.setStyleSheet("font-size:20px; font-weight:600;")
        v.addWidget(t)
        v.addWidget(self.value)


class BatteryPanel(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.logs = None
        self._rows: dict = {}           # boot name -> rows
        self._files: list = []
        self._selected = TREND
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        self.tabs = QTabWidget()
        outer.addWidget(self.tabs)
        self.tabs.addTab(self._build_now(), "Now")
        self.tabs.addTab(self._build_history(), "History")
        self.tabs.addTab(self._build_charge(), "Charging")
        self.tabs.currentChanged.connect(lambda _i: self._on_tab())
        self._follow = QTimer(self)
        self._follow.setInterval(FOLLOW_MS)
        self._follow.timeout.connect(self._follow_current)
        self._follow.start()

    # ---- Now ----

    def _build_now(self) -> QWidget:
        w = QWidget()
        root = QVBoxLayout(w)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(14)
        self.link = QLabel("")
        self.link.setWordWrap(True)
        root.addWidget(self.link)
        self.prev_boot = QLabel("")
        self.prev_boot.setWordWrap(True)
        self.prev_boot.setVisible(False)
        root.addWidget(self.prev_boot)

        top = QHBoxLayout()
        self.soc = QLabel("—")
        self.soc.setStyleSheet("font-size:56px; font-weight:700;")
        self.soc.setMinimumWidth(190)
        self.soc.setAlignment(Qt.AlignCenter)
        top.addWidget(self.soc)
        grid = QGridLayout()
        grid.setSpacing(10)
        self.v: dict = {}
        for i, (key, title) in enumerate((
                ("voltage", "Voltage"), ("current", "Current"), ("power", "Power"), ("delta", "Cell difference"),
                ("temp", "Temperature"), ("mos", "MOS temperature"), ("capacity", "Remaining"), ("cycles", "Cycles"),
                ("balance", "Balancing"), ("avg", "Cell average"), ("throughput", "Discharged in all"),
                ("runtime", "BMS running for"))):
            self.v[key] = _Value(title)
            grid.addWidget(self.v[key], i // 4, i % 4)
        top.addLayout(grid, 1)
        root.addLayout(top)

        self.mos = QLabel("")
        self.mos.setProperty("muted", True)
        self.mos.setWordWrap(True)
        root.addWidget(self.mos)

        box = QGroupBox("Cells (voltage · resistance)")
        self.cell_grid = QGridLayout(box)
        self.cell_grid.setHorizontalSpacing(16)
        self.bars: list = []
        root.addWidget(box, 1)
        return w

    def _cells(self, cells: list, res: list) -> None:
        if len(cells) != len(self.bars):
            for row in self.bars:
                for w in row:
                    w.deleteLater()
            self.bars = []
            per_col = (len(cells) + 1) // 2
            for i in range(len(cells)):
                name = QLabel(f"{i + 1}")
                name.setProperty("muted", True)
                bar = QProgressBar()
                bar.setRange(2800, 3650)
                bar.setTextVisible(False)
                bar.setFixedHeight(10)
                val = QLabel("")
                col = (i // per_col) * 3
                self.cell_grid.addWidget(name, i % per_col, col)
                self.cell_grid.addWidget(bar, i % per_col, col + 1)
                self.cell_grid.addWidget(val, i % per_col, col + 2)
                self.bars.append((name, bar, val))
        lo, hi = (min(cells), max(cells)) if cells else (0.0, 0.0)
        for i, ((_n, bar, val), c) in enumerate(zip(self.bars, cells)):
            bar.setValue(int(c * 1000))
            mark = " ▲" if c == hi and hi != lo else " ▼" if c == lo and hi != lo else ""
            ohm = f" · {res[i] * 1000:.0f} mΩ" if i < len(res) else ""
            val.setText(f"{c:.3f} V{ohm}{mark}")

    def show_status(self, b: Optional[dict]) -> None:
        text, level = link_text(b)
        self.link.setText(f'<span style="color:{LEVEL[level]}">●</span> {text}')
        self._show_log_info((b or {}).get("log"))
        r = (b or {}).get("reading")
        fresh = live(b or {})
        if not r:
            self.soc.setText("—")
            self.soc.setStyleSheet(f"font-size:56px; font-weight:700; color:{MUTED};")
            return
        soc = r["soc"]
        color = MUTED if not fresh else RED if soc <= CRITICAL else AMBER if soc <= LOW else GREEN
        self.soc.setText(f"{soc}%")
        self.soc.setStyleSheet(f"font-size:56px; font-weight:700; color:{color};")
        cur = r["current"]
        bal = {0: "off", 1: "charging side", 2: "discharging side"}.get(r.get("balancing"), "—")
        if r.get("balance_current"):
            bal += f" · {r['balance_current']:+.2f} A"
        sets = {"voltage": f"{r['voltage']:.2f} V", "current": f"{cur:+.2f} A", "power": f"{r['power']:.0f} W",
                "delta": f"{r['delta'] * 1000:.0f} mV", "temp": " / ".join(f"{t:.1f}" for t in r["temps"]) + " °C",
                "mos": f"{r['mos_temp']:.1f} °C", "capacity": f"{r['remaining_ah']:.1f} / {r['nominal_ah']:.0f} Ah",
                "cycles": str(r["cycles"]), "balance": bal if "balancing" in r else "—",
                "avg": f"{r['cell_avg']:.3f} V" if "cell_avg" in r else "—",
                "throughput": f"{r['total_cycle_ah']:.0f} Ah" if "total_cycle_ah" in r else "—",
                "runtime": f"{r['runtime_s'] / 86400:.0f} days" if "runtime_s" in r else "—"}
        for k, s in sets.items():
            self.v[k].value.setText(s)
            self.v[k].value.setStyleSheet("font-size:20px; font-weight:600;"
                                          + ("" if fresh else f" color:{MUTED};"))
        age = "" if fresh else f" · last reading {b['age']:.0f} s ago"
        alarms = ""
        if r.get("alarms") or r.get("wire_alarms"):
            alarms = f" · <span style='color:{AMBER}'>BMS alarm bits {r.get('alarms', 0):#x}" \
                     f" / wires {r.get('wire_alarms', 0):#x}</span>"
        self.mos.setText(f"Charging {'on' if r['charging'] else 'off'} · "
                         f"Discharging {'on' if r['discharging'] else 'off'} · health {r['soh']}%{age}{alarms}")
        self._cells(r["cells"], r.get("resistances") or [])

    def _show_log_info(self, log: Optional[dict]) -> None:
        """How the boot before this one ended: a power cut shows here."""
        prev = (log or {}).get("previous")
        disk = (log or {}).get("disk") or {}
        parts = []
        if prev:
            text = how_it_ended({**prev, "current": False})
            color = MUTED if prev.get("clean_end") else AMBER
            parts.append(f"<span style='color:{color}'>Last boot: {text}.</span>")
        if disk.get("low"):
            parts.append(f"<span style='color:{RED}'>The robot's disk is nearly full "
                         f"({_bytes(disk.get('free'))} left); the battery log takes "
                         f"{_bytes(disk.get('used'))} in {disk.get('files')} files. "
                         "Nothing is deleted by itself: delete old boots on History.</span>")
        self.prev_boot.setText(" ".join(parts))
        self.prev_boot.setVisible(bool(parts))

    # ---- History ----

    def _build_history(self) -> QWidget:
        w = QWidget()
        root = QVBoxLayout(w)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(10)
        bar = QHBoxLayout()
        bar.addWidget(QLabel("Boot:"))
        self.boot_box = QComboBox()
        self.boot_box.setMinimumWidth(300)
        self.boot_box.currentIndexChanged.connect(self._on_boot)
        bar.addWidget(self.boot_box, 1)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh)
        bar.addWidget(self.refresh_btn)
        self.delete_btn = QPushButton("Delete this boot's log…")
        self.delete_btn.setProperty("kind", "danger")
        self.delete_btn.clicked.connect(self._ask_delete)
        bar.addWidget(self.delete_btn)
        root.addLayout(bar)

        metrics = QHBoxLayout()
        self.measure_group = QButtonGroup(self)
        self.measure_group.setExclusive(True)
        self.measure_btns: dict = {}
        for key, (title, _u, _c) in METRICS.items():
            b = QPushButton(title.split(" (")[0])
            b.setCheckable(True)
            self.measure_group.addButton(b)
            b.clicked.connect(lambda _c=False, k=key: self._set_metric(k))
            metrics.addWidget(b)
            self.measure_btns[key] = b
        metrics.addStretch(1)
        root.addLayout(metrics)
        self.measure = "soc"
        self.measure_btns["soc"].setChecked(True)

        split = QSplitter(Qt.Vertical)
        top = QWidget()
        tv = QVBoxLayout(top)
        tv.setContentsMargins(0, 0, 0, 0)
        self.chart = TimeChart()
        tv.addWidget(self.chart, 1)
        self.hist_info = QLabel("Connect to the robot to see its battery log.")
        self.hist_info.setWordWrap(True)
        self.hist_info.setProperty("muted", True)
        tv.addWidget(self.hist_info)
        split.addWidget(top)
        self.boots = QTableWidget(0, 8)
        self.boots.setHorizontalHeaderLabels(["Started", "Length", "Charge", "Lowest V", "Max cell diff",
                                              "Hottest", "Reconnects", "How it ended"])
        self.boots.verticalHeader().setVisible(False)
        self.boots.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.boots.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.boots.setSelectionMode(QAbstractItemView.SingleSelection)
        self.boots.horizontalHeader().setSectionResizeMode(7, QHeaderView.Stretch)
        self.boots.cellDoubleClicked.connect(lambda row, _c: self.select_boot(self._boot_names[row]))
        self._boot_names: list = []
        split.addWidget(self.boots)
        split.setSizes([420, 220])
        root.addWidget(split, 1)
        self._fill_boots([])
        return w

    def set_logs(self, logs) -> None:
        self.logs = logs
        logs.listed.connect(self._on_listed)
        logs.loaded.connect(self._on_loaded)
        logs.deleted.connect(self._on_deleted)
        logs.failed.connect(lambda text: self.hist_info.setText(f"<span style='color:{AMBER}'>{text}</span>"))
        logs.progress.connect(lambda name, have, size: self.hist_info.setText(
            f"copying {name} from the robot: {_bytes(have)} of {_bytes(size)}") if size and have < size else None)

    def history_visible(self) -> bool:
        return self.isVisible() and self.tabs.currentIndex() == 1

    def _on_tab(self) -> None:
        if self.history_visible() and self.logs is not None and not self.logs.files:
            self.refresh()

    def refresh(self) -> None:
        if self.logs is None:
            return
        if not self.logs.ros.is_connected():
            self._fill_boots([{"name": n, "local_only": True} for n in self.logs.local_names()])
            self.hist_info.setText("Not connected: showing the copies on this PC.")
            return
        self.hist_info.setText("asking the robot for its battery log…")
        self.logs.refresh_list()

    def _on_listed(self, d: dict) -> None:
        files = list(d.get("files") or [])
        known = {f.get("name") for f in files}
        files += [{"name": n, "local_only": True} for n in self.logs.local_names() if n not in known]
        self._fill_boots(files)
        disk = d.get("disk") or {}
        self.hist_info.setText(f"{len(d.get('files') or [])} boots on the robot, {_bytes(disk.get('used'))}; "
                               f"{_bytes(disk.get('free'))} free on its disk. Kept until you delete them.")
        if self._selected not in (TREND, ""):
            self.select_boot(self._selected)
        else:
            self._draw()

    def _summary(self, name: str) -> dict:
        return next((f for f in self._files if f.get("name") == name), {"name": name})

    def _fill_boots(self, files: list) -> None:
        self._files = files
        newest = sorted(files, key=lambda f: f.get("name", ""), reverse=True)
        self._boot_names = [f["name"] for f in newest]
        self.boot_box.blockSignals(True)
        self.boot_box.clear()
        self.boot_box.addItem("All boots: trend", TREND)
        for f in newest:
            self.boot_box.addItem(self._boot_label(f), f["name"])
        i = self.boot_box.findData(self._selected)
        self.boot_box.setCurrentIndex(max(0, i))
        self.boot_box.blockSignals(False)
        self.boots.setRowCount(len(newest))
        for row, f in enumerate(newest):
            soc = ""
            if f.get("soc_start") is not None:
                soc = f"{f['soc_start']:.0f}% → {f.get('soc_end', 0):.0f}%"
            cells = [self._boot_label(f, short=True), _dur(f.get("duration_s", 0)) if f.get("rows") else "",
                     soc, f"{f['v_min']:.2f} V" if f.get("v_min") is not None else "",
                     f"{f['delta_max'] * 1000:.0f} mV" if f.get("delta_max") is not None else "",
                     f"{f['temp_max']:.1f} °C" if f.get("temp_max") is not None else "",
                     str(f.get("reconnects", "")) if f.get("rows") else "",
                     "only on this PC" if f.get("local_only") else how_it_ended(f)]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if col == 7 and f.get("rows") and not f.get("clean_end") and not f.get("current"):
                    item.setForeground(QColor(AMBER))
                self.boots.setItem(row, col, item)
        self.boots.resizeColumnsToContents()
        self.boots.horizontalHeader().setSectionResizeMode(7, QHeaderView.Stretch)
        self._update_buttons()

    def _boot_label(self, f: dict, short: bool = False) -> str:
        name = f.get("name", "")
        try:
            stamp = time.strptime(name.split("_")[1], "%Y%m%d-%H%M%S")
            when = time.strftime("%Y-%m-%d %H:%M", stamp)
        except (IndexError, ValueError):
            when = name
        if short:
            return when + (" (now)" if f.get("current") else "")
        tail = " · running now" if f.get("current") else (
            f" · {_dur(f.get('duration_s', 0))}" if f.get("rows") else " · on this PC only")
        return f"{when}{tail}"

    def _on_boot(self, _i: int) -> None:
        self._selected = self.boot_box.currentData() or TREND
        self._update_buttons()
        if self._selected != TREND and self.logs is not None:
            if self._selected not in self._rows:
                self._rows[self._selected] = self.logs.read_local(self._selected)
            if self.logs.ros.is_connected() and not self._summary(self._selected).get("local_only"):
                self.logs.load(self._selected)
        self._draw()

    def select_boot(self, name: str) -> None:
        i = self.boot_box.findData(name)
        if i >= 0:
            self.boot_box.setCurrentIndex(i)
            self._on_boot(i)

    # ---- going to charge by itself (rcs.core.charging) ----

    charge_changed = Signal(object)     # dict: ChargeConfig

    def _build_charge(self) -> QWidget:
        from PySide6.QtWidgets import QCheckBox, QComboBox, QFormLayout, QSpinBox
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(20, 20, 20, 20)
        hint = QLabel("Below the charge set here, a running mission pauses and the robot drives to its charging "
                      "place; charged again, the mission goes on from where it was. The robot must charge there "
                      "(a dock, or someone plugging it in): the app drives it there and waits. Driving it yourself, "
                      "sending a goal or starting a mission meanwhile hands it back to you.")
        hint.setWordWrap(True)
        hint.setProperty("muted", True)
        v.addWidget(hint)
        f = QFormLayout()
        self.ch_on = QCheckBox("Go to charge by itself")
        self.ch_low = QSpinBox()
        self.ch_low.setRange(5, 80)
        self.ch_low.setSuffix(" %")
        self.ch_resume = QSpinBox()
        self.ch_resume.setRange(10, 100)
        self.ch_resume.setSuffix(" %")
        self.ch_place = QComboBox()
        self.ch_place.setToolTip("A saved place (Navigation page) in front of the charger")
        f.addRow("", self.ch_on)
        f.addRow("Go below", self.ch_low)
        f.addRow("Back to work above", self.ch_resume)
        f.addRow("Charging place", self.ch_place)
        v.addLayout(f)
        self.ch_status = QLabel("")
        self.ch_status.setWordWrap(True)
        v.addWidget(self.ch_status)
        v.addStretch(1)
        for wdg in (self.ch_on,):
            wdg.toggled.connect(lambda _b: self._charge_edited())
        for wdg in (self.ch_low, self.ch_resume):
            wdg.valueChanged.connect(lambda _v: self._charge_edited())
        self.ch_place.currentIndexChanged.connect(lambda _i: self._charge_edited())
        self._ch_loading = False
        return w

    def set_charge(self, cfg: dict, places: list) -> None:
        """cfg: ChargeConfig as a dict; places: [(id, name)] of the current map."""
        self._ch_loading = True
        self.ch_on.setChecked(bool(cfg.get("on")))
        self.ch_low.setValue(int(cfg.get("low", 25)))
        self.ch_resume.setValue(int(cfg.get("resume", 90)))
        self.ch_place.clear()
        self.ch_place.addItem("— choose a saved place —", "")
        for pid, name in places:
            self.ch_place.addItem(name, pid)
        i = self.ch_place.findData(cfg.get("place", ""))
        self.ch_place.setCurrentIndex(max(0, i))
        self._ch_loading = False

    def charge_config(self) -> dict:
        return {"on": self.ch_on.isChecked(), "low": float(self.ch_low.value()),
                "resume": float(max(self.ch_resume.value(), self.ch_low.value() + 5)),
                "place": self.ch_place.currentData() or ""}

    def _charge_edited(self) -> None:
        if not self._ch_loading:
            self.charge_changed.emit(self.charge_config())

    def show_charge_status(self, text: str) -> None:
        self.ch_status.setText(text)

    def _update_buttons(self) -> None:
        s = self._summary(self._selected) if self._selected != TREND else {}
        self.delete_btn.setEnabled(self._selected != TREND and not s.get("current") and self.can_delete)
        self.delete_btn.setToolTip("The running boot's log cannot be deleted" if s.get("current")
                                   else "" if self.can_delete else "Admins only")

    can_delete = True       # False unless the signed-in user is an admin

    def set_can_delete(self, ok: bool) -> None:
        self.can_delete = ok
        self._update_buttons()

    def _set_metric(self, key: str) -> None:
        self.measure = key
        self.measure_btns[key].setChecked(True)
        self._draw()

    def _on_loaded(self, name: str, rows: list) -> None:
        self._rows[name] = rows
        if name == self._selected:
            self._draw()

    def _on_deleted(self, name: str) -> None:
        self._rows.pop(name, None)
        if self._selected == name:
            self._selected = TREND
        self._fill_boots([f for f in self._files if f.get("name") != name])
        self.hist_info.setText(f"deleted {name} on the robot and on this PC")
        self._draw()

    def _ask_delete(self) -> None:
        name = self._selected
        if name == TREND:
            return
        ok = QMessageBox.question(
            self, "Delete battery log",
            f"Delete the battery log of the boot {self._boot_label(self._summary(name), short=True)}, "
            "on the robot and on this PC?\n\nThis cannot be undone.") == QMessageBox.Yes
        if ok:
            self.confirm_delete(name)

    def confirm_delete(self, name: str) -> None:
        if self._summary(name).get("local_only"):
            try:
                self.logs.local(name).unlink()
            except (OSError, ValueError):
                pass
            self.logs.deleted.emit(name)
            return
        if self.logs is not None:
            self.logs.delete(name)

    def _follow_current(self) -> None:
        """The running boot grows by a row a second: fetch what is new."""
        if self.logs is None or not self.history_visible() or not self.logs.ros.is_connected():
            return
        s = self._summary(self._selected)
        if s.get("current") and not self.logs.busy:
            self.logs.load(self._selected)

    def _draw(self) -> None:
        m = self.measure
        if self._selected == TREND:
            title, unit, _key = TRENDS[m]
            ts, vs = trend([f for f in self._files if not f.get("local_only")], m)
            self.chart.set_data(ts, vs, title, unit, points=True,
                                empty="No boots yet. Press Refresh while connected.")
            return
        title, unit, _cols = METRICS[m]
        rows = self._rows.get(self._selected, [])
        ts, vs = series(rows, m)
        lines = [(LOW, "low"), (CRITICAL, "critical")] if m == "soc" else []
        self.chart.set_data(ts, vs, title, unit, lines=lines,
                            empty="No reading in this boot yet" if rows else "Loading…")
        s = self._summary(self._selected)
        if rows and s.get("rows"):
            self.hist_info.setText(f"{self._boot_label(s, short=True)}: {how_it_ended(s)} · "
                                   f"{len(rows)} rows, {s.get('live_rows', 0)} with the BMS heard")
