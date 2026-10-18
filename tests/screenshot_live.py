"""Take the documentation screenshots from the real app driving real Nav2.

Start the simulated robot first (sim/run_sim.sh), then:
    QT_QPA_PLATFORM=offscreen python tests/screenshot_live.py
Writes full-window PNGs to docs/screenshots/.
"""
from __future__ import annotations

import os
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_shot_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from PySide6.QtWidgets import QApplication

app = QApplication(sys.argv)
from rcs.core.mission import Mission, Waypoint
from rcs.ui import theme
from rcs.ui.main_window import MainWindow

OUT = ROOT / "docs" / "screenshots"
OUT.mkdir(parents=True, exist_ok=True)
START = (3.55, -5.10, -1.64)
DOCK, PICK, DROP = (3.55, -5.10, -1.64), (3.30, -2.60, 1.57), (3.30, -8.55, -1.57)


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.01)


def wait_until(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    return False


def shot(w, page, name):
    w.btns[page].click()
    pump(0.6)
    pm = w.grab()
    path = OUT / f"{name}.png"
    pm.save(str(path))
    print(f"  wrote {path.relative_to(ROOT)}", flush=True)


def main() -> int:
    theme.apply(app)
    w = MainWindow()
    w.resize(1440, 900)
    w.show()

    prof = w.profiles.active()
    prof.host, prof.port = "127.0.0.1", 9090
    w.profiles.upsert(prof)
    w.p_conn._reload()
    w._on_connect(prof)
    assert wait_until(w.ros.is_connected, 15), "rosbridge not reachable - start sim/run_sim.sh"
    assert wait_until(lambda: w.p_map.view.map_img.image is not None, 20)
    w.p_map.set_initial_pose.emit(*START)
    assert wait_until(lambda: w.tf.pose_xyyaw("map", "base_link") is not None, 30)
    pump(4)

    # drive toward the pick-up point, take the navigation shots on the way
    statuses = []
    w.ros.nav_status.connect(statuses.append)
    for _ in range(5):
        statuses.clear()
        w.p_map.goal_requested.emit(*PICK)
        if wait_until(lambda: "active" in statuses, 8):
            break
    pump(4)
    for v in (w.p_map.view, w.p_nav.view):
        v.center_on_robot(9.0)
    w.p_map.ck_lc.setChecked(True)
    shot(w, "map", "map")
    shot(w, "nav", "nav")
    shot(w, "logs", "logs")
    shot(w, "status", "status")
    shot(w, "conn", "conn")
    w.p_map.cancel_requested.emit()
    wait_until(lambda: "canceled" in statuses, 8)
    pump(1)
    shot(w, "teleop", "teleop")

    # a looping delivery route, shown while it runs
    m = Mission.new("Delivery loop")
    m.loop = True
    m.waypoints = [Waypoint.new("Pick-up", *PICK), Waypoint.new("Drop-off", *DROP),
                   Waypoint.new("Dock", *DOCK)]
    for wp in m.waypoints:
        wp.tolerance, wp.dwell = 0.3, 2.0
    w.missions.upsert(m)
    w.p_mission._reload_missions()
    w.p_mission._set_combo_to(m.id)
    w.p_mission.play_btn.click()
    # shoot mid-way to the second stop, so the route and the path are visible
    wait_until(lambda: w.p_mission._exec_idx == 1
               and abs(w._robot_y - PICK[1]) > 2.0, 120)
    w.p_mission.view.getPlotItem().vb.setRange(xRange=(-1.5, 8.5), yRange=(-10.5, -0.5), padding=0)
    shot(w, "mission", "mission")
    w.p_mission.stop_btn.click()
    pump(1)

    w.ros.disconnect_robot()
    pump(0.5)
    return 0


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    os._exit(rc)
