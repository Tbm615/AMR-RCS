"""Battery page: what the robot reads from its BMS, and how the link is doing."""
from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QProgressBar, QVBoxLayout, QWidget,
)

from ...core.battery import CRITICAL, LOW, link_text, live

GREEN, RED, AMBER, MUTED = "#39d98a", "#ff6a6a", "#ffb547", "#8b93a7"
LEVEL = {"ok": GREEN, "warn": AMBER, "error": RED}


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
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(14)
        self.link = QLabel("")
        self.link.setWordWrap(True)
        root.addWidget(self.link)

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
                ("temp", "Temperature"), ("mos", "MOS temperature"), ("capacity", "Remaining"), ("cycles", "Cycles"))):
            self.v[key] = _Value(title)
            grid.addWidget(self.v[key], i // 4, i % 4)
        top.addLayout(grid, 1)
        root.addLayout(top)

        self.mos = QLabel("")
        self.mos.setProperty("muted", True)
        root.addWidget(self.mos)

        box = QGroupBox("Cells")
        self.cell_grid = QGridLayout(box)
        self.cell_grid.setHorizontalSpacing(16)
        self.bars: list = []
        root.addWidget(box, 1)

    def _cells(self, cells: list) -> None:
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
        for (_n, bar, val), c in zip(self.bars, cells):
            bar.setValue(int(c * 1000))
            mark = " ▲" if c == hi and hi != lo else " ▼" if c == lo and hi != lo else ""
            val.setText(f"{c:.3f} V{mark}")

    def show_status(self, b: Optional[dict]) -> None:
        text, level = link_text(b)
        self.link.setText(f'<span style="color:{LEVEL[level]}">●</span> {text}')
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
        sets = {"voltage": f"{r['voltage']:.2f} V", "current": f"{cur:+.2f} A", "power": f"{r['power']:.0f} W",
                "delta": f"{r['delta'] * 1000:.0f} mV", "temp": " / ".join(f"{t:.1f}" for t in r["temps"]) + " °C",
                "mos": f"{r['mos_temp']:.1f} °C", "capacity": f"{r['remaining_ah']:.1f} / {r['nominal_ah']:.0f} Ah",
                "cycles": str(r["cycles"])}
        for k, s in sets.items():
            self.v[k].value.setText(s)
            self.v[k].value.setStyleSheet("font-size:20px; font-weight:600;"
                                          + ("" if fresh else f" color:{MUTED};"))
        age = "" if fresh else f" · last reading {b['age']:.0f} s ago"
        self.mos.setText(f"Charging {'on' if r['charging'] else 'off'} · "
                         f"Discharging {'on' if r['discharging'] else 'off'} · health {r['soh']}%{age}")
        self._cells(r["cells"])
