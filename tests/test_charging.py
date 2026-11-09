"""Going to charge by itself: below the set charge the mission pauses and the
robot drives to its charging place; charged, the mission goes on. Battery and
Nav2 are made up here.

Run:
    python tests/test_charging.py
"""
from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = os.environ["APPDATA"] = tempfile.mkdtemp(prefix="rcs_chg_cfg_")

from rcs.core.charging import MAX_TRIES, RETRY_S, ChargeConfig, ChargeKeeper  # noqa: E402

PASS = FAIL = 0


def check(name, cond, extra=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def test_keeper() -> None:
    print("\n[the rules]")
    t = [0.0]
    k = ChargeKeeper(ChargeConfig(on=True, low=25, resume=90, place="dock"), clock=lambda: t[0])
    check("above the charge: nothing", k.step(60, -5, True, "active", False, True) == [])
    check("E-STOP: waits", k.step(20, -5, True, "active", True, True) == [] and k.state == "idle")
    acts = k.step(24, -5, True, "active", False, True)
    check("below: pause the mission and go", acts == ["pause_mission", "go_charger"] and k.state == "going", str(acts))
    check("on the way: nothing more", k.step(23, -5, False, "active", False, True) == [])
    check("there", k.step(23, -5, False, "succeeded", False, True) == ["arrived"] and k.state == "charging")
    k.step(40, 8.0, False, "succeeded", False, True)
    check("charging seen from the current", "charging" in k.why, k.why)
    acts = k.step(90, 8.0, False, "succeeded", False, True)
    check("charged: the mission goes on", acts == ["charged", "resume_mission"] and k.state == "idle", str(acts))
    k = ChargeKeeper(ChargeConfig(on=True, low=25, resume=90, place="dock"), clock=lambda: t[0])
    check("no mission running: only go", k.step(20, -5, False, "", False, True) == ["go_charger"])
    tries = 1
    for _ in range(MAX_TRIES + 1):
        t[0] += RETRY_S + 1
        acts = k.step(20, -5, False, "aborted", False, True)
        if "go_charger" in acts:
            tries += 1
        if "failed" in acts:
            break
    check("Nav2 fails: tried again, then gives up and says so", tries == MAX_TRIES and "failed" in acts
          and k.state == "idle", f"{tries} {acts}")
    k = ChargeKeeper(ChargeConfig(on=True, low=25, resume=90, place="dock"), clock=lambda: t[0])
    k.step(20, -5, True, "active", False, True)
    k.take_over()
    check("the operator takes over: it stops, no mission resumed later",
          k.state == "idle" and not k.resume_mission)
    check("off: nothing", ChargeKeeper(ChargeConfig(on=False, place="dock")).step(5, -5, True, "", False, True) == [])
    check("no place: nothing", ChargeKeeper(ChargeConfig(on=True)).step(5, -5, True, "", False, False) == [])
    c = ChargeConfig.from_dict({"on": True, "low": 95, "resume": 10, "place": 3})
    check("odd settings clamped", c.low == 80 and c.resume >= c.low + 5 and c.place == "3", str(c))


def test_app() -> None:
    print("\n[in the app]")
    import time
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    from rcs.core.mission import Mission, Waypoint
    from rcs.core.robot_data import RobotState
    from rcs.ui.main_window import MainWindow
    w = MainWindow()
    w.ros.is_connected = lambda: True
    place = w.place_store.add("Dock", 1.0, 2.0, 3.14, "")
    w.p_nav.places.refresh()
    w._show_charge()
    check("the place is offered", w.p_battery.ch_place.findData(place.id) >= 0)
    w.p_battery.ch_place.setCurrentIndex(w.p_battery.ch_place.findData(place.id))
    w.p_battery.ch_low.setValue(30)
    w.p_battery.ch_on.setChecked(True)
    check("saved with the robot", RobotState(w.robot_state.path).get("charging", {}).get("place") == place.id
          and w.charger.cfg.on and w.charger.cfg.low == 30)
    ms = Mission.new("rounds")
    ms.waypoints = [Waypoint.new("A", 5, 5, 0), Waypoint.new("B", 6, 6, 0)]
    w.missions.upsert(ms)
    w.p_mission._reload_missions()
    w.p_mission._set_combo_to(ms.id)
    goals = []
    real = w.ros.send_nav_goal
    w.ros.send_nav_goal = lambda x, y, yaw=0.0: goals.append((x, y))
    def feed(soc, cur):                 # what the robot's battery service sends every second
        w._battery = {"reading": {"soc": soc, "voltage": 47.0, "current": cur}, "age": 0.5, "state": "connected"}
        w.ros._mark("battery")
    w.p_mission._play()
    check("mission running", w.p_mission._exec_running and goals[-1] == (5, 5), str(goals))
    feed(25, -4.0)
    w._check_sensors()
    check("low battery: mission paused", not w.p_mission._exec_running)
    check("and off to the dock", goals[-1] == (1.0, 2.0), str(goals[-2:]))
    check("said in the event log", any(e.source == "Charging" and "going to charge" in e.text for e in w.events.events))
    w._nav_state = "succeeded"
    feed(25, -4.0)
    w._check_sensors()
    check("arrived", w.charger.state == "charging")
    feed(95, 6.0)
    w._check_sensors()
    check("charged: the mission goes on where it was", w.p_mission._exec_running and goals[-1] == (5, 5), str(goals))
    feed(20, -4.0)
    w._nav_state = "active"
    w._check_sensors()
    w._send_goal(9.0, 9.0, 0.0)               # the operator sends their own goal
    check("an operator's goal takes over", w.charger.state == "idle" and not w.charger.resume_mission)
    w.ros.send_nav_goal = real
    w.close()


def main() -> int:
    test_keeper()
    test_app()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
