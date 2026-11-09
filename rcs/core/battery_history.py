"""The robot's battery log, on this PC: list the boots, fetch a boot's file,
read it as series for the chart (robot/battery_log.py writes them).

Fetched files are kept per robot (robots/<id>/battery_log/) and only what was
added since is asked for again, so the running boot is cheap to follow. Like
the robot's, these copies are never deleted by the app on its own: only when
an operator deletes that boot's log.

One request at a time, the next only after the answer; a lost answer is asked
for again.
"""
from __future__ import annotations

import base64
import binascii
import csv
import io
import re
import time
import uuid
import zlib
from pathlib import Path
from typing import Optional

from PySide6.QtCore import QObject, QTimer, Signal

ANSWER_MS = 8000
TRIES = 3
NAME_RE = re.compile(r"^battery_\d{8}-\d{6}_[0-9a-f]{8}\.csv$")

# what the chart can show: key -> (title, unit, column(s) of the log)
METRICS = {
    "soc": ("Charge", "%", ("soc",)),
    "voltage": ("Voltage", "V", ("voltage",)),
    "current": ("Current", "A", ("current",)),
    "delta": ("Cell difference", "mV", ("delta",)),
    "temp": ("Temperature (hottest sensor)", "°C", ("temp1", "temp2")),
    "mos": ("MOS temperature", "°C", ("mos_temp",)),
}
# the same over many boots, from each boot's summary
TRENDS = {
    "soc": ("Lowest charge in the boot", "%", "soc_min"),
    "voltage": ("Lowest voltage in the boot", "V", "v_min"),
    "current": ("Highest discharge in the boot", "A", "current_min"),
    "delta": ("Largest cell difference in the boot", "mV", "delta_max"),
    "temp": ("Hottest in the boot", "°C", "temp_max"),
    "mos": ("Hottest MOS in the boot", "°C", "mos_max"),
}


def parse(text: str) -> list:
    """Rows as dicts. A line cut short (power cut) is dropped."""
    rows, header = [], None
    for rec in csv.reader(io.StringIO(text)):
        if header is None:
            header = rec
        elif len(rec) == len(header):
            rows.append(dict(zip(header, rec)))
    return rows


def _f(v) -> Optional[float]:
    try:
        return float(v) if v not in ("", None) else None
    except (TypeError, ValueError):
        return None


def series(rows: list, metric: str):
    """(times, values) of one metric; values None where the BMS was not heard,
    so the chart leaves a gap instead of a line across it."""
    cols = METRICS[metric][2]
    ts, vs = [], []
    for r in rows:
        t = _f(r.get("unix"))
        if t is None:
            continue
        vals = [v for v in (_f(r.get(c)) for c in cols) if v is not None]
        v = max(vals) if vals else None
        if v is not None and metric == "delta":
            v *= 1000.0
        ts.append(t)
        vs.append(v)
    return ts, vs


def trend(files: list, metric: str):
    """One point per boot, oldest first."""
    title, unit, key = TRENDS[metric]
    ts, vs = [], []
    for s in sorted(files, key=lambda s: s.get("start") or 0):
        if s.get("start") is None or s.get(key) is None:
            continue
        v = float(s[key])
        if metric == "delta":
            v *= 1000.0
        ts.append(float(s["start"]))
        vs.append(v)
    return ts, vs


def how_it_ended(s: dict) -> str:
    """For the operator: was it shut down, or did the power go?"""
    if s.get("current"):
        return "running now"
    last = s.get("last") or {}
    when = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(last["unix"])) if last.get("unix") else "?"
    batt = ""
    if last.get("soc") is not None:
        batt = f"; battery then {last['soc']:.0f}%"
        if last.get("voltage") is not None:
            batt += f", {last['voltage']:.2f} V"
        if last.get("current") is not None:
            batt += f", {last['current']:+.1f} A"
    if s.get("clean_end"):
        return f"shut down at {when}{batt}"
    return f"no clean stop: power cut or crash at {when}{batt}"


class BatteryLogs(QObject):
    listed = Signal(object)             # {"files": [...], "current": name, "disk": {...}}
    loaded = Signal(str, object)        # name, rows
    deleted = Signal(str)
    failed = Signal(str)
    progress = Signal(str, int, int)    # name, bytes so far, size

    def __init__(self, ros, folder: Path, parent=None) -> None:
        super().__init__(parent)
        self.ros = ros
        self.folder = Path(folder)
        self.answer_ms = ANSWER_MS
        self.files: list = []
        self.current = ""
        self.disk: dict = {}
        self._req = ""
        self._ask: Optional[tuple] = None   # (op, kwargs) of the request in flight
        self._tries = 0
        self._queue: list = []
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._no_answer)
        ros.battery_log.connect(self._on_answer)
        ros.disconnected.connect(self._drop)

    def use(self, folder: Path) -> None:
        """Another robot's copies from now on."""
        self._drop()
        self.folder = Path(folder)
        self.files, self.current, self.disk = [], "", {}

    @property
    def busy(self) -> bool:
        return bool(self._req)

    # ---- local copies ----

    def local(self, name: str) -> Path:
        if not NAME_RE.match(name):
            raise ValueError("no such log")
        return self.folder / name

    def local_names(self) -> list:
        if not self.folder.is_dir():
            return []
        return sorted(p.name for p in self.folder.glob("battery_*.csv") if NAME_RE.match(p.name))

    def read_local(self, name: str) -> list:
        try:
            return parse(self.local(name).read_text(encoding="utf-8", errors="replace"))
        except (OSError, ValueError):
            return []

    # ---- requests ----

    def refresh_list(self) -> None:
        self._enqueue("log_list", {})

    def load(self, name: str) -> None:
        """Fetch what this PC does not have yet of that boot, then emit loaded."""
        if any(op == "log_get" and kw.get("name") == name for op, kw in self._queue) or \
                (self._req and self._ask[0] == "log_get" and self._ask[1].get("name") == name):
            return                      # on its way already
        try:
            path = self.local(name)
        except ValueError as e:
            self.failed.emit(str(e))
            return
        have = path.stat().st_size if path.exists() else 0
        remote = next((f for f in self.files if f.get("name") == name), None)
        if remote is not None and not remote.get("current") and remote.get("size") == have:
            self.loaded.emit(name, self.read_local(name))       # complete already
            return
        self._enqueue("log_get", {"name": name, "offset": have})

    def delete(self, name: str) -> None:
        self._enqueue("log_delete", {"name": name})

    def _enqueue(self, op: str, kw: dict) -> None:
        self._queue.append((op, kw))
        if not self._req:
            self._next()

    def _next(self) -> None:
        if not self._queue or not self.ros.is_connected():
            self._queue.clear()
            return
        self._ask = self._queue.pop(0)
        self._tries = 0
        self._send()

    def _send(self) -> None:
        op, kw = self._ask
        self._req = uuid.uuid4().hex[:12]
        self.ros.send_battery_cmd(op, req=self._req, **kw)
        self._timer.start(self.answer_ms)

    def _no_answer(self) -> None:
        self._tries += 1
        if self._tries >= TRIES:
            self._req = ""
            self._queue.clear()
            self.failed.emit("no answer from the robot's battery service: update it, see Battery in "
                             "robot/README.md")
            return
        self._send()

    def _drop(self) -> None:
        self._timer.stop()
        self._req, self._ask = "", None
        self._queue.clear()

    def _on_answer(self, ans: dict) -> None:
        if not self._req or ans.get("req") != self._req:
            return                      # an answer to a request we repeated
        self._timer.stop()
        self._req = ""
        op, kw = self._ask
        if "error" in ans:
            self.failed.emit(str(ans["error"]))
        elif op == "log_list":
            self.files = [f for f in ans.get("files") or [] if isinstance(f, dict)]
            self.current = str(ans.get("current", ""))
            self.disk = ans.get("disk") or {}
            self.listed.emit({"files": self.files, "current": self.current, "disk": self.disk})
        elif op == "log_delete":
            name = str(ans.get("deleted", ""))
            try:
                self.local(name).unlink()
            except (OSError, ValueError):
                pass
            self.files = [f for f in self.files if f.get("name") != name]
            self.disk = ans.get("disk") or self.disk
            self.deleted.emit(name)
        else:
            self._on_chunk(kw["name"], ans)
        self._next()

    def _on_chunk(self, name: str, ans: dict) -> bool:
        """True when this boot's file is complete here (or failed); else the
        next piece is the next request."""
        try:
            data = zlib.decompress(base64.b64decode(str(ans.get("data", "")), validate=True))
            offset, size = int(ans.get("offset", -1)), int(ans.get("size", 0))
        except (binascii.Error, zlib.error, TypeError, ValueError):
            self.failed.emit("the battery log arrived damaged, try again")
            return True
        path = self.local(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        have = path.stat().st_size if path.exists() else 0
        if offset != have:
            self.failed.emit("the battery log changed while it was copied, try again")
            return True
        with open(path, "ab") as f:
            f.write(data)
        have += len(data)
        self.progress.emit(name, have, size)
        if not ans.get("eof") and data:
            self._queue.insert(0, ("log_get", {"name": name, "offset": have}))
            return False
        self.loaded.emit(name, self.read_local(name))
        return True
