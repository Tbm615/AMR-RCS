"""Versions: the robot side older than the app, a newer release on GitHub.
No network: GitHub's answer is made up here.

Run:
    python tests/test_updates.py
"""
from __future__ import annotations

import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = os.environ["APPDATA"] = tempfile.mkdtemp(prefix="rcs_upd_cfg_")

from PySide6.QtWidgets import QApplication  # noqa: E402

app = QApplication(sys.argv)
from rcs import __version__  # noqa: E402
from rcs.core import versions  # noqa: E402
from rcs.core.profile import load_settings  # noqa: E402
from rcs.ui.main_window import MainWindow  # noqa: E402

PASS = FAIL = 0


def check(name, cond, extra=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def pump(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.01)


def main() -> int:
    print("\n[versions]")
    p = versions.parse
    check("beta before release", p("0.6.0b1") < p("0.6.0") < p("0.6.1") < p("0.7.0b1"))
    check("tags read like versions", p("v0.6.0-beta.2") == p("0.6.0b2") and p("v0.5.0") == p("0.5.0"))
    check("nonsense is oldest", p("hello") < p("0.0.1"))
    rel = [{"tag_name": "v0.5.0"}, {"tag_name": "v0.6.0-beta.1", "prerelease": True},
           {"tag_name": "v0.6.0-beta.3", "prerelease": True}, {"tag_name": "v0.7.0", "draft": True}]
    check("a stable app is not offered betas", versions.newest(rel, "0.5.0") is None)
    check("a beta app is offered the newest beta", versions.newest(rel, "0.6.0b1")["tag_name"] == "v0.6.0-beta.3")
    check("nothing newer: nothing", versions.newest(rel, "0.6.0b3") is None)
    check("drafts never", versions.newest([{"tag_name": "v9.0.0", "draft": True}], "0.5.0") is None)

    print("\n[the robot side]")
    note = versions.robot_side_note
    check("old robot side (no version) told", "older" in note({}))
    check("too old told", "update it" in note({"rcs": "0.5.0"}))
    check("same: nothing", note({"rcs": __version__}) == "")
    check("newer robot side: update the app", "update the app" in note({"rcs": "99.0.0"}))
    w = MainWindow()
    w.show()
    w.p_robot._connected = True
    w.ros._mark("launcher")
    w.p_robot._on_state({"version": 1, "stacks": {}, "maps": [], "time": time.time()})
    pump(0.3)
    check("Robot programs says the robot side is old", "older than" in w.p_robot.banner.text(), w.p_robot.banner.text()[-160:])

    print("\n[a new release]")
    real = versions.check
    versions.check = lambda timeout=6.0: {"tag_name": "v99.0.0", "html_url": "https://example.invalid/r"}
    w.settings.pop("update_checked", None)
    w.check_updates()
    check("offered in the status bar", wait(lambda: "v99.0.0" in w.sb_update.text()), w.sb_update.text())
    check("with its page", "https://example.invalid/r" in w.sb_update.text())
    check("remembered", load_settings().get("update_offer", {}).get("tag") == "v99.0.0")
    calls = []
    versions.check = lambda timeout=6.0: calls.append(1)
    w.check_updates()
    pump(0.5)
    check("not asked again the same day", not calls)
    w.p_conn.update_ck.setChecked(False)
    w.settings.pop("update_checked", None)
    w.check_updates()
    pump(0.5)
    check("switched off: never asked", not calls and load_settings().get("check_updates") is False)
    versions.check = real
    w.close()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


def wait(cond, sec=5.0):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    return False


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
