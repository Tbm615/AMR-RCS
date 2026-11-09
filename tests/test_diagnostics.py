"""Health checks, the diagnostic bundle (no secrets in it), crash reports and an
app that did not close properly. Needs nothing.

Run:
    python tests/test_diagnostics.py
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = os.environ["APPDATA"] = tempfile.mkdtemp(prefix="rcs_diag_cfg_")

from PySide6.QtWidgets import QApplication  # noqa: E402

app = QApplication(sys.argv)
from rcs.core import diagnostics, health  # noqa: E402
from rcs.ui.main_window import MainWindow  # noqa: E402

PASS = FAIL = 0
TMP = Path(tempfile.mkdtemp(prefix="rcs_diag_tmp_"))


def check(name, cond, extra=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def level(cs, key):
    return next((c.level for c in cs if c.key == key), None)


def test_checks() -> None:
    print("\n[health checks]")
    cs = health.checks({})
    check("offline: one error, what to do", len(cs) == 1 and cs[0].level == "error" and cs[0].hint)
    base = {"connected": True, "launcher_online": True,
            "launcher": {"rcs": "0.6.0b1", "stacks": {"bringup": {"state": "running"}, "nav": {"state": "running"}}},
            "ages": {"odom": 0.1, "scan": 0.2, "battery": 1.0}, "localized": True,
            "gate": {"engaged": False, "stops": 0},
            "battery": {"reading": {"soc": 80, "voltage": 49.1}, "log": {"disk": {"free": 50e9}}}}
    cs = health.checks(base)
    check("all good", health.summary(cs) == "ok", str([(c.key, c.level) for c in cs if c.level != "ok"]))
    cs = health.checks({**base, "launcher_online": False})
    check("no launcher: said, with the fix", level(cs, "launcher") == "error")
    cs = health.checks({**base, "ages": {"odom": 0.1, "scan": 9.0}})
    check("lidar stopped while the base runs", level(cs, "scan") == "error")
    cs = health.checks({**base, "estop": True})
    check("E-STOP engaged is a problem", level(cs, "estop") == "error" and health.summary(cs) == "error")
    cs = health.checks({**base, "gate": None})
    check("no E-STOP service is a warning", level(cs, "estop") == "warn")
    cs = health.checks({**base, "battery": {"reading": {"soc": 12, "voltage": 46.0}, "log": {"disk": {"free": 1e9}}}})
    check("low battery and a full disk", level(cs, "battery") == "error" and level(cs, "disk") == "warn")
    cs = health.checks({**base, "localized": False})
    check("Nav2 up, not localized: set the initial pose", level(cs, "loc") == "warn")
    cs = health.checks({**base, "robot_note": "The robot side is v0.5.0, update it"})
    check("old robot side said", level(cs, "robotside") == "warn")
    cs = health.checks({**base, "gateway": True, "role": ""})
    check("a gateway and no sign-in", level(cs, "signin") == "warn")


def test_app() -> None:
    print("\n[the app]")
    diagnostics.RUNNING.write_text(json.dumps({"pid": 999999, "started": time.time() - 3600, "app": "0.5.0"}))
    w = MainWindow()
    w.show()
    check("an unclean last run is told", any("did not close properly" in e.text for e in w.events.events))
    check("this run noted", json.loads(diagnostics.RUNNING.read_text())["pid"] == os.getpid())
    w.p_status.health_source = lambda: {"connected": True, "launcher_online": False}
    w.p_status._refresh()
    texts = [r[1].text() for r in w.p_status.h_rows if not r[1].isHidden()]
    check("Status page lists the checks", "Connection" in texts and "Robot launcher" in texts, str(texts))
    check("and sums them up", "problem" in w.p_status.h_summary.text(), w.p_status.h_summary.text())
    w.p_status.health_source = w.health_context
    w.p_status._refresh()
    check("its own context works offline", w.p_status.last_checks and w.p_status.last_checks[0].key == "link")

    prof = w.profiles.active()
    prof.user, prof.token, prof.role = "ann", "secret-token-123", "admin"
    w.profiles.upsert(prof)
    w._log("warn", "Test", "something to find in the bundle")
    from rcs.core.profile import save_settings
    save_settings({"sound": True, "password": "not this either"})
    try:
        raise RuntimeError("boom for the report")
    except RuntimeError:
        crash = diagnostics.record_crash(*sys.exc_info())
    check("crash report written", crash is not None and "boom for the report" in crash.read_text())
    out = w.save_bundle(str(TMP / "diag"))
    check("bundle saved", out is not None and out.suffix == ".zip" and out.exists())
    with zipfile.ZipFile(out) as z:
        names = set(z.namelist())
        blob = b"".join(z.read(n) for n in names)
        about = json.loads(z.read("about.json"))
        live = json.loads(z.read("live.json"))
    for n in ("about.json", "settings.json", "robots.json", "live.json", "robot/events.csv"):
        check(f"has {n}", n in names, str(sorted(names)))
    check("has the crash report", any(n.startswith("crashes/") for n in names))
    check("no sign-in token anywhere in it", b"secret-token-123" not in blob)
    check("no password either", b"not this either" not in blob)
    check("the version and system", about.get("app") and about.get("os"))
    check("the health checks as they are", live.get("health") and live["health"][0]["key"] == "link")
    check("the event log", b"something to find in the bundle" in blob)
    w.close()
    check("closing properly clears the note", not diagnostics.RUNNING.exists())


def main() -> int:
    test_checks()
    test_app()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
