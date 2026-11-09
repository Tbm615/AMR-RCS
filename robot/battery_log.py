"""The battery log on the robot: one CSV file per boot, one row every second.

  $RCS_WS/battery_log/battery_<boot time>_<boot id>.csv

Each row goes to the disk before the next second (flush + fsync), so after a
power cut the file holds everything up to the last second or so, and the last
row says how the battery was right then. A row is written every second whether
or not the BMS answers: then the values are empty and the state column says
why, so the file has no holes in time.

A clean stop (the robot shutting down, or the service being stopped) writes a
last row with the note "stopped". A file that ends without it ended in a power
cut or a crash.

Files are kept for good: nothing here ever deletes one by itself. Only an
operator does, from the app (log_delete) or by hand; the running boot's file
cannot be deleted. Free disk space is reported so the app can warn in time.

Every row has all the BMS reports (each cell, its resistance, temperatures,
balancing, alarms, capacity, cycles, health, switches) plus the link's state.
Once a minute the BMS's raw frame is added too, so the file can be decoded
again later if a field turns out to be read wrong.

Also answers the app's requests for these files (LogExchange). No ROS here, so
tests/mock_rosbridge.py and the tests run this very code.

  app: {"op": "log_list", "req": "..."}
  bot: {"req", "files": [summary, ...], "current": name}
  app: {"op": "log_get", "req", "name", "offset"}
  bot: {"req", "name", "offset", "size", "data": base64(zlib(bytes)), "eof"}
  app: {"op": "log_delete", "req", "name"}
  bot: {"req", "deleted": name}
  bot: {"req", "error": "..."}
"""
from __future__ import annotations

import base64
import csv
import io
import json
import os
import re
import shutil
import threading
import time
import uuid
import zlib
from pathlib import Path
from typing import Callable, Optional

FIELDS = ["unix", "time", "uptime_s", "state", "rssi", "age_s", "soc", "voltage", "current", "power",
          "delta", "cell_min", "cell_max", "cell_avg", "cell_min_no", "cell_max_no",
          "temp1", "temp2", "mos_temp", "remaining_ah", "nominal_ah", "cycles", "total_cycle_ah",
          "soh", "charging", "discharging", "balancing", "balance_current", "alarms", "wire_alarms",
          "runtime_s", "reconnects", "connected_s", "bad_frames", "error", "cells", "resistances",
          "note", "frame"]
FRESH_S = 15.0              # an older reading is not written as if it were current
FRAME_EVERY_S = 60.0        # the raw frame goes in this often
LOW_DISK = 2 * 1024 ** 3    # free space below this is reported as a warning
CHUNK = 512 * 1024          # bytes of CSV per answer, before compression
NAME_RE = re.compile(r"^battery_\d{8}-\d{6}_[0-9a-f]{8}\.csv$")
STOPPED = "stopped"


def boot_id() -> str:
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip().replace("-", "")[:8]
    except OSError:
        return uuid.uuid4().hex[:8]


def uptime() -> float:
    try:
        return float(Path("/proc/uptime").read_text().split()[0])
    except (OSError, ValueError, IndexError):
        return time.monotonic()


def _num(v) -> str:
    return "" if v is None else str(v)


def _float(row: dict, key: str) -> Optional[float]:
    try:
        v = row.get(key, "")
        return float(v) if v not in ("", None) else None
    except ValueError:
        return None


def read_rows(path: Path) -> list:
    """Rows of one file. A last line cut short by a power cut is dropped."""
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []
    return parse(text)


def parse(text: str) -> list:
    rows = []
    reader = csv.reader(io.StringIO(text))
    header = None
    for rec in reader:
        if header is None:
            header = rec
            continue
        if len(rec) != len(header):
            continue
        rows.append(dict(zip(header, rec)))
    return rows


def summarize(rows: list, name: str = "") -> dict:
    """How one boot went, from its rows."""
    out = {"name": name, "rows": len(rows), "start": None, "end": None, "duration_s": 0.0,
           "clean_end": False, "live_rows": 0}
    if not rows:
        return out
    t = [_float(r, "unix") for r in rows]
    t = [v for v in t if v is not None]
    if t:
        out["start"], out["end"] = t[0], t[-1]
    ups = [_float(r, "uptime_s") for r in rows]
    ups = [v for v in ups if v is not None]
    out["duration_s"] = round(ups[-1] - ups[0], 1) if len(ups) > 1 else 0.0
    live = [r for r in rows if r.get("soc") not in ("", None)]
    out["live_rows"] = len(live)

    def col(key):
        return [v for v in (_float(r, key) for r in live) if v is not None]

    soc, volt, cur = col("soc"), col("voltage"), col("current")
    delta = col("delta")
    temps = col("temp1") + col("temp2")
    mos = col("mos_temp")
    if soc:
        out.update(soc_start=soc[0], soc_end=soc[-1], soc_min=min(soc))
    if volt:
        out.update(v_min=min(volt), v_max=max(volt))
    if cur:
        out.update(current_min=min(cur), current_max=max(cur))
    if delta:
        out["delta_max"] = max(delta)
    if temps:
        out["temp_max"] = max(temps)
    if mos:
        out["mos_max"] = max(mos)
    for key in ("cycles", "soh"):
        v = col(key)
        if v:
            out[key] = v[-1]
    rec = col("reconnects")
    out["reconnects"] = int(rec[-1] - rec[0]) if len(rec) > 1 else 0
    last = rows[-1]
    out["clean_end"] = last.get("note", "").startswith(STOPPED)
    lv = live[-1] if live else {}
    out["last"] = {"unix": _float(last, "unix"), "state": last.get("state", ""),
                   "soc": _float(lv, "soc"), "voltage": _float(lv, "voltage"),
                   "current": _float(lv, "current"), "unix_live": _float(lv, "unix")}
    return out


class BootLog:
    """Writes this boot's file. Thread safe: the watchdog writes its last row
    from another thread."""

    def __init__(self, folder: Path, boot: str = "", clock: Callable[[], float] = time.time,
                 up: Callable[[], float] = uptime) -> None:
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.boot = (boot or boot_id())[:8]
        self.clock, self.up = clock, up
        self._lock = threading.Lock()
        self.rows = 0
        self._frame_t = -1e9
        mine = sorted(self.folder.glob(f"battery_*_{self.boot}.csv"))
        if mine:                    # the service restarted during this boot
            self.path = mine[-1]
            new = False
        else:
            started = self.clock() - self.up()
            stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(started))
            self.path = self.folder / f"battery_{stamp}_{self.boot}.csv"
            new = True
        self.previous = self._previous()
        torn = False
        if not new and self.path.stat().st_size:
            with open(self.path, "rb") as f:     # killed in the middle of a row
                f.seek(-1, os.SEEK_END)
                torn = f.read(1) != b"\n"
        self._f = open(self.path, "a", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        if torn:
            self._f.write("\n")         # that row is dropped when read, the next ones are whole
        if new:
            self._w.writerow(FIELDS)
            self._sync()

    @property
    def name(self) -> str:
        return self.path.name

    def files(self) -> list:
        return sorted((p for p in self.folder.glob("battery_*.csv") if NAME_RE.match(p.name)),
                      key=lambda p: p.name)

    def _previous(self) -> Optional[dict]:
        """How the boot before this one ended: the reason this log exists."""
        older = [p for p in self.files() if p != self.path]
        return summary_of(older[-1]) if older else None

    def disk(self) -> dict:
        """How much the log takes, and what is left: it is never pruned."""
        used = 0
        for p in self.files():
            try:
                used += p.stat().st_size
            except OSError:
                pass
        try:
            free = shutil.disk_usage(self.folder).free
        except OSError:
            free = None
        return {"files": len(self.files()), "used": used, "free": free,
                "low": free is not None and free < LOW_DISK}

    def _sync(self) -> None:
        self._f.flush()
        try:
            os.fsync(self._f.fileno())
        except OSError:
            pass

    def write(self, st: dict, note: str = "", frame: str = "") -> None:
        now = self.clock()
        r = st.get("reading") or {}
        age = st.get("age")
        fresh = bool(r) and age is not None and age <= FRESH_S
        cells = (r.get("cells") or []) if fresh else []
        temps = (r.get("temps") or [None, None]) if fresh else [None, None]
        temps = list(temps) + [None] * (2 - len(temps))

        def v(key):
            return _num(r.get(key)) if fresh else ""

        def flag(key):
            return ("1" if r.get(key) else "0") if fresh else ""

        raw = ""
        if frame and fresh and now - self._frame_t >= FRAME_EVERY_S:
            raw, self._frame_t = frame, now

        row = [f"{now:.1f}", time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)),
               f"{self.up():.1f}", st.get("state", ""), _num(st.get("rssi")), _num(age),
               v("soc"), v("voltage"), v("current"), v("power"), v("delta"),
               _num(min(cells)) if cells else "", _num(max(cells)) if cells else "",
               v("cell_avg"), v("cell_min_no"), v("cell_max_no"),
               _num(temps[0]), _num(temps[1]), v("mos_temp"), v("remaining_ah"), v("nominal_ah"),
               v("cycles"), v("total_cycle_ah"), v("soh"), flag("charging"), flag("discharging"),
               v("balancing"), v("balance_current"), v("alarms"), v("wire_alarms"), v("runtime_s"),
               _num(st.get("reconnects")), _num(st.get("connected_s")), _num(st.get("bad_frames")),
               st.get("error", "") or "", ";".join(str(c) for c in cells),
               ";".join(str(c) for c in (r.get("resistances") or [])) if fresh else "", note, raw]
        with self._lock:
            if self._f.closed:
                return
            self._w.writerow(row)
            self._sync()
            self.rows += 1

    def close(self, note: str = STOPPED) -> None:
        """The last row of a clean stop."""
        self.write({"state": "stopped"}, note)
        with self._lock:
            self._f.close()


def summary_of(path: Path, current: bool = False) -> dict:
    """A finished file's summary is kept next to it: those files never change."""
    path = Path(path)
    side = path.with_suffix(".summary.json")
    size = path.stat().st_size if path.exists() else 0
    if not current:
        try:
            s = json.loads(side.read_text())
            if s.get("size") == size:
                return s
        except (OSError, ValueError):
            pass
    s = summarize(read_rows(path), path.name)
    s["size"] = size
    s["current"] = current
    if not current:
        try:
            side.write_text(json.dumps(s))
        except OSError:
            pass
    return s


class LogExchange:
    """Answers the app's log_list and log_get. Only files of the log folder can
    be read, by name: nothing else on the robot."""

    def __init__(self, log: BootLog) -> None:
        self.log = log

    def handle(self, cmd: dict) -> dict:
        req = str(cmd.get("req", ""))
        try:
            if cmd.get("op") == "log_list":
                return {"req": req, "current": self.log.name, "disk": self.log.disk(),
                        "files": [summary_of(p, p == self.log.path) for p in self.log.files()]}
            if cmd.get("op") == "log_get":
                return self._get(req, cmd)
            if cmd.get("op") == "log_delete":
                return self._delete(req, cmd)
            raise ValueError("unknown request")
        except Exception as e:      # a bad request must not touch the logging
            return {"req": req, "error": str(e) or type(e).__name__}

    def _path(self, cmd: dict) -> Path:
        name = str(cmd.get("name", ""))
        if not NAME_RE.match(name):
            raise ValueError("no such log")
        path = self.log.folder / name
        if not path.is_file():
            raise ValueError("no such log")
        return path

    def _delete(self, req: str, cmd: dict) -> dict:
        """Only ever asked for by an operator, one file at a time."""
        path = self._path(cmd)
        if path == self.log.path:
            raise ValueError("this is the running boot's log: it cannot be deleted")
        path.unlink()
        try:
            path.with_suffix(".summary.json").unlink()
        except OSError:
            pass
        return {"req": req, "deleted": path.name, "disk": self.log.disk()}

    def _get(self, req: str, cmd: dict) -> dict:
        path = self._path(cmd)
        name = path.name
        offset = int(cmd.get("offset", 0))
        size = path.stat().st_size
        if not 0 <= offset <= size:
            raise ValueError("offset past the end")
        with open(path, "rb") as f:
            f.seek(offset)
            data = f.read(CHUNK)
            while b"\n" not in data and offset + len(data) < size:
                more = f.read(4096)         # a row longer than a chunk: to its end
                if not more:
                    break
                data += more
        if offset + len(data) < size or path == self.log.path:
            # only whole lines: the app parses what it has so far
            cut = data.rfind(b"\n") + 1
            if cut:
                data = data[:cut]
        return {"req": req, "name": name, "offset": offset, "size": size,
                "data": base64.b64encode(zlib.compress(data)).decode(),
                "eof": offset + len(data) >= size}


class Watchdog(threading.Thread):
    """Ends the process if the service stops going round. Runs in a thread of
    its own, so a hang of the event loop cannot stop it."""

    def __init__(self, log: BootLog, timeout: float = 20.0, exit_fn=os._exit) -> None:
        super().__init__(daemon=True, name="battery-watchdog")
        self.log, self.timeout, self.exit_fn = log, timeout, exit_fn
        self.beat = time.monotonic()
        self.last_status: dict = {}
        self.fired = False

    def alive(self, status: dict) -> None:
        self.beat = time.monotonic()
        self.last_status = status

    def check(self) -> bool:
        if time.monotonic() - self.beat <= self.timeout:
            return False
        self.fired = True
        st = dict(self.last_status, state="hung")
        self.log.write(st, f"watchdog: the service hung for {self.timeout:.0f} s, restarting it")
        self.exit_fn(3)
        return True

    def run(self) -> None:
        while not self.fired:
            time.sleep(1.0)
            self.check()
