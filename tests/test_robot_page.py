"""Robot programs page against the mock rosbridge's fake launcher.

Run:
    python tests/test_robot_page.py

Clicks through the page like an operator: start bringup, start navigation
with SLAM, save the map, restart on the saved map, stop the base. The mock
speaks the same JSON protocol as robot/rcs_launcher.py.
"""
from __future__ import annotations

import asyncio
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_robotpage_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]

from PySide6.QtWidgets import QApplication

from rcs.core.profile import RobotProfile
from rcs.ui import theme
from rcs.ui.main_window import MainWindow
from tests import mock_rosbridge

PORT = 9192
app = QApplication(sys.argv)
PASS = FAIL = 0


def check(name: str, cond: bool, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def wait_until(cond, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    return False


def main() -> int:
    threading.Thread(target=lambda: asyncio.run(
        mock_rosbridge.main_async("127.0.0.1", PORT, duration=40)), daemon=True).start()
    time.sleep(0.6)

    theme.apply(app)
    w = MainWindow()
    w.show()
    rp = w.p_robot
    rp.confirm_dialogs = False
    w.btns["robot"].click()
    check("asks to connect first", "Connect to a robot first" in rp.banner.text())
    check("Start disabled while offline", not rp.base.start_btn.isEnabled())

    prof = RobotProfile.new(name="mock")
    prof.host, prof.port = "127.0.0.1", PORT
    w.profiles.upsert(prof)
    w._on_connect(prof)
    check("connected", wait_until(w.ros.is_connected, 10))
    check("launcher online", wait_until(lambda: "online" in rp.banner.text(), 5))
    check("saved maps listed", wait_until(lambda: rp.map_combo.count() == 2, 3),
          f"{[rp.map_combo.itemText(i) for i in range(rp.map_combo.count())]}")

    state = lambda k: rp._state.get("stacks", {}).get(k, {}).get("state")   # noqa: E731
    check("navigation Start disabled until the base runs", not rp.nav.start_btn.isEnabled())
    check("Save map disabled without navigation", not rp.save_btn.isEnabled())

    rp.base.start_btn.click()
    check("bringup running", wait_until(lambda: state("bringup") == "running", 5))
    check("result shown", wait_until(lambda: "bringup starting" in rp.result.text(), 3))
    check("navigation Start enabled", wait_until(rp.nav.start_btn.isEnabled, 3))

    rp.rb_slam.setChecked(True)
    rp.nav.start_btn.click()
    check("navigation (SLAM) running", wait_until(lambda: state("nav") == "running", 5))
    check("SLAM mode sent", rp._state["stacks"]["nav"]["args"].get("mode") == "slam")
    check("Start becomes Restart", wait_until(lambda: rp.nav.start_btn.text() == "Restart", 3))
    check("'Open 2D Map' shown", wait_until(rp.goto_map.isVisible, 3))

    rp.map_name.setText("bad name!")
    rp.save_btn.click()
    check("invalid map name rejected locally", "letters, digits" in rp.result.text())
    rp.map_name.setText("office_1")
    check("Save map enabled", wait_until(rp.save_btn.isEnabled, 3))
    rp.save_btn.click()
    check("map saved", wait_until(lambda: "map saved" in rp.result.text(), 5), rp.result.text())
    check("saved map appears in the list", wait_until(lambda: rp.map_combo.findText("office_1") >= 0, 3))

    rp.rb_map.setChecked(True)
    rp.map_combo.setCurrentText("office_1")
    wait_until(rp.nav.start_btn.isEnabled, 3)
    rp.nav.start_btn.click()
    check("restarted on the saved map",
          wait_until(lambda: rp._state["stacks"]["nav"]["args"].get("map") == "office_1", 5))

    rp.goto_map.click()
    app.processEvents()
    check("'Open 2D Map' switches page", w.stack.currentWidget() is w.p_map)
    w.btns["robot"].click()

    wait_until(rp.base.stop_btn.isEnabled, 3)
    rp.base.stop_btn.click()
    check("stopping the base stops navigation too",
          wait_until(lambda: state("bringup") == "stopped" and state("nav") == "stopped", 5))

    w.ros.disconnect_robot()
    check("page resets on disconnect",
          wait_until(lambda: "Connect to a robot first" in rp.banner.text(), 5))
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    rc = main()
    print(f"exit code: {rc}")
    sys.stdout.flush(); sys.stderr.flush()
    if sys.platform == "win32":
        import ctypes
        from ctypes import wintypes
        k32 = ctypes.windll.kernel32
        k32.GetCurrentProcess.restype = wintypes.HANDLE
        k32.TerminateProcess.argtypes = [wintypes.HANDLE, wintypes.UINT]
        k32.TerminateProcess(k32.GetCurrentProcess(), rc)
    os._exit(rc)
