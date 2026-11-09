"""For support: a zip of what helps to understand a problem, crash reports,
and knowing that the app did not close properly last time.

The bundle holds the app's log, its settings and robot profiles with every
sign-in token taken out, the open robot's event log and state, crash reports,
and a snapshot of the robot as the app sees it (health checks, topic rates,
launcher, E-STOP gate, battery). Nothing is sent anywhere: the operator saves
the file and gives it to whoever helps.
"""
from __future__ import annotations

import json
import os
import platform
import sys
import tempfile
import time
import traceback
import zipfile
from pathlib import Path
from typing import Iterable, Optional, Tuple

from .. import __version__
from .profile import CONFIG_DIR, PROFILES_PATH, SETTINGS_PATH

CRASHES = CONFIG_DIR / "crashes"
RUNNING = CONFIG_DIR / "running.json"
LOG = Path(tempfile.gettempdir()) / "rcs.log"
LOG_TAIL = 2 * 1024 * 1024
SECRET_KEYS = ("token", "password")


def redact(obj):
    """A copy with every secret value replaced."""
    if isinstance(obj, dict):
        return {k: ("<removed>" if any(w in str(k).lower() for w in SECRET_KEYS) and v else redact(v))
                for k, v in obj.items()}
    if isinstance(obj, list):
        return [redact(v) for v in obj]
    return obj


def about() -> dict:
    try:
        from PySide6 import __version__ as qt
    except ImportError:                 # pragma: no cover
        qt = "?"
    return {"app": __version__, "python": sys.version.split()[0], "qt": qt, "os": platform.platform(),
            "machine": platform.machine(), "frozen": bool(getattr(sys, "frozen", False)),
            "created": time.strftime("%Y-%m-%d %H:%M:%S")}


# ---- did the app close properly? ----

def start_session() -> Optional[dict]:
    """Note that the app runs. Returns the previous run's note if it never
    closed (crash, killed, power cut), else None."""
    previous = None
    try:
        previous = json.loads(RUNNING.read_text())
    except (OSError, ValueError):
        pass
    if isinstance(previous, dict) and _alive(previous.get("pid")):
        previous = None                 # still running (this process, or a second window): not a crash
    try:
        RUNNING.write_text(json.dumps({"pid": os.getpid(), "started": time.time(), "app": __version__}))
    except OSError:
        pass
    return previous if isinstance(previous, dict) else None


def _alive(pid) -> bool:
    if not isinstance(pid, int) or pid <= 0:
        return False
    if pid == os.getpid():
        return True
    if os.name == "nt":
        import ctypes
        h = ctypes.windll.kernel32.OpenProcess(0x1000, False, pid)     # PROCESS_QUERY_LIMITED_INFORMATION
        if not h:
            return False
        code = ctypes.c_ulong()
        ok = ctypes.windll.kernel32.GetExitCodeProcess(h, ctypes.byref(code))
        ctypes.windll.kernel32.CloseHandle(h)
        return bool(ok) and code.value == 259                          # STILL_ACTIVE
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def end_session() -> None:
    try:
        RUNNING.unlink()
    except OSError:
        pass


def record_crash(exc_type, value, tb) -> Optional[Path]:
    """A crash report of its own, kept with the settings (the log in the temp
    folder may be gone after a reboot)."""
    try:
        CRASHES.mkdir(parents=True, exist_ok=True)
        path = CRASHES / f"crash-{time.strftime('%Y%m%d-%H%M%S')}.txt"
        text = "".join(traceback.format_exception(exc_type, value, tb))
        path.write_text(f"RCS {__version__} on {platform.platform()}\n{time.ctime()}\n\n{text}", encoding="utf-8")
        reports = sorted(CRASHES.glob("crash-*.txt"))
        for old in reports[:-50]:       # the last 50 are enough
            old.unlink(missing_ok=True)
        return path
    except OSError:
        return None


def crash_reports() -> list:
    return sorted(CRASHES.glob("crash-*.txt")) if CRASHES.exists() else []


# ---- the bundle ----

def _json_file(path: Path):
    try:
        return redact(json.loads(path.read_text(encoding="utf-8")))
    except (OSError, ValueError):
        return None


def make_bundle(path, live: Optional[dict] = None, files: Iterable[Tuple[str, Path]] = ()) -> Path:
    """Write the zip; returns its path. live: what the app sees now (already
    free of secrets is not assumed: it is redacted too)."""
    path = Path(path)
    if path.suffix.lower() != ".zip":
        path = path.with_name(path.name + ".zip")
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("about.json", json.dumps(about(), indent=2))
        for name, src in (("settings.json", SETTINGS_PATH), ("robots.json", PROFILES_PATH)):
            data = _json_file(src)
            if data is not None:
                z.writestr(name, json.dumps(data, indent=2))
        if live is not None:
            z.writestr("live.json", json.dumps(redact(live), indent=2, default=str))
        try:
            size = LOG.stat().st_size
            with open(LOG, "rb") as f:
                f.seek(max(0, size - LOG_TAIL))
                z.writestr("rcs.log", f.read())
        except OSError:
            pass
        for p in crash_reports()[-20:]:
            z.write(p, f"crashes/{p.name}")
        for arc, src in files:
            src = Path(src)
            if src.is_file():
                if src.suffix == ".json":
                    data = _json_file(src)
                    if data is not None:
                        z.writestr(arc, json.dumps(data, indent=2))
                        continue
                z.write(src, arc)
    return path
