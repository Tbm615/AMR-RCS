"""The robot's own E-STOP: the gate's rules (robot/estop_gate.py), and the app
with it, against the mock rosbridge running that very gate.

Run:
    python tests/test_estop.py

Needs no ROS: tests/test_estop_ros.py runs the real node (rcs_safety.py).
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "robot"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_estop_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]

from PySide6.QtWidgets import QApplication

import estop_gate
from estop_gate import Gate
from rcs.core.mission import Mission, Waypoint
from rcs.core.profile import RobotProfile
from rcs.ui.main_window import MainWindow
from tests import mock_rosbridge

app = QApplication(sys.argv)
PASS = FAIL = 0
PORT = 9193
TMP = Path(tempfile.mkdtemp(prefix="rcs_estop_"))


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def wait(cond, timeout=8.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    return bool(cond())


def test_gate() -> None:
    print("\n[the gate]")
    t = [100.0]
    path = TMP / "estop.json"
    g = Gate(path, clock=lambda: t[0], wall=lambda: 1_700_000_000 + t[0])
    check("commands pass when clear", g.filter(0.4, 0.2) == (0.4, 0.2) and not g.blocking())
    check("engaging cancels Nav2", g.estop(True, "/amr/estop") == ["cancel_nav"])
    check("then nothing moves", g.filter(0.4, 0.2) == (0.0, 0.0) and g.blocking())
    check("engaging again does nothing more", g.estop(True) == [])
    t[0] += 0.5
    check("just engaged: not cancelled twice at once", g.tick(nav_active=True) == [])
    t[0] += 0.5
    check("a goal Nav2 still runs is cancelled again", g.tick(nav_active=True) == ["cancel_nav"])
    check("but not in a burst", g.tick(nav_active=True) == [])
    t[0] += 1.0
    check("once a second", g.tick(nav_active=True) == ["cancel_nav"])
    check("kept through a restart", Gate(path).engaged and "restart" in Gate(path).by)
    g.estop(False)
    check("released only when told", not g.blocking() and not Gate(path).engaged)
    path.write_text("{not json")
    check("a broken file is no E-STOP and no crash", not Gate(path).engaged)

    print("\n[losing the app]")
    g.heartbeat({"id": "pc1", "host": "office-pc", "driving": True})
    g.heartbeat({"id": "pc2", "host": "watcher", "driving": False})
    t[0] += estop_gate.HEARTBEAT_S - 0.5
    check("heard recently: nothing", g.tick() == [] and g.state()["apps"] == 2)
    g.heartbeat({"id": "pc2", "host": "watcher", "driving": False})
    t[0] += 1.0
    check("the driving app goes silent: the robot stops", g.tick() == ["cancel_nav"] and g.blocking())
    st = g.state()
    check("and says why", st["stops"] == 1 and "office-pc" in st["last_stop"]["reason"], f"{st}")
    check("a short hold, not a latch", not st["engaged"])
    t[0] += estop_gate.HOLD_S + 0.1
    check("then commands pass again (the gamepad works)", not g.blocking())
    t[0] += estop_gate.HEARTBEAT_S + 1
    check("an app that was only watching stops nothing", g.tick() == [] and g.state()["apps"] == 0)
    g.heartbeat({"id": "pc3", "driving": True})
    g.heartbeat({"id": "pc3", "bye": True})
    t[0] += estop_gate.HEARTBEAT_S + 1
    check("an app closed on purpose stops nothing", g.tick() == [] and g.stops == 1)
    g.heartbeat({"driving": True})
    check("a heartbeat with no id is ignored", g.state()["apps"] == 0)


def mission() -> Mission:
    m = Mission.new("loop")
    m.loop = True
    m.waypoints = [Waypoint.new("A", 3.0, 0.0), Waypoint.new("B", 3.0, 3.0)]
    return m


def test_app() -> None:
    print("\n[the app with the robot's gate]")
    mock_rosbridge.ROBOT = mock_rosbridge.FakeRobot("bot", 70)
    threading.Thread(target=lambda: asyncio.run(mock_rosbridge.main_async("127.0.0.1", PORT, 60)),
                     daemon=True).start()
    time.sleep(0.6)
    w = MainWindow()
    w.show()
    prof = RobotProfile.new(name="bot")
    prof.host, prof.port = "127.0.0.1", PORT
    w.profiles.upsert(prof)
    w._on_connect(prof)
    gate = mock_rosbridge.ROBOT.gate
    check("connected and the robot's gate heard", wait(lambda: w.gate_state.get("gate")), f"{w.gate_state}")
    check("the app's heartbeat reaches it", wait(lambda: gate.state()["apps"] == 1))
    check("watching only: not driving", not any(c["driving"] for c in gate.clients.values()))
    m = mission()
    w.missions.upsert(m)
    w.p_mission.reload()
    w.p_mission.open_mission(m.id)
    w.p_mission._play(0, 0)
    check("a mission running: the robot is told", wait(lambda: any(c["driving"] for c in gate.clients.values())))

    print("\n[E-STOP from the app is held by the robot]")
    w._set_estop(True)
    check("engaged on the robot", wait(lambda: gate.engaged))
    check("Nav2 cancelled there", mock_rosbridge.ROBOT.cancels >= 1)
    check("the run report says E-STOP", w.runs.runs[-1].result == "stopped" and w.runs.runs[-1].reason == "E-STOP")
    w._set_estop(False)
    check("released from the app", wait(lambda: not gate.engaged))
    time.sleep(0.6)
    app.processEvents()
    check("an old state does not engage it again", not w._estop_on)

    print("\n[E-STOP engaged on the robot by someone else]")
    gate.estop(True, "/amr/estop")
    check("the app follows", wait(lambda: w._estop_on))
    gate.estop(False)
    w._set_estop(False)

    print("\n[the app is lost while driving]")
    w.p_mission._play(0, 0)
    check("driving again", wait(lambda: any(c["driving"] for c in gate.clients.values())))
    stops = gate.stops
    w.ros._beat.stop()          # as if the Wi-Fi went: no more heartbeats
    check("the robot stops by itself", wait(lambda: gate.stops == stops + 1, 6.0), f"{gate.state()}")
    w.ros._beat.start()
    check("the app is told why", wait(lambda: any("stopped itself" in e.text for e in w.events.events)))
    check("and so is the run report", any("stopped itself" in e for e in w.runs.active.events)
          if w.runs.active else False)

    print("\n[the connection drops during a mission]")
    w.ros.disconnect_robot()
    check("closed on purpose: goodbye said", wait(lambda: gate.state()["apps"] == 0))
    w._on_connect(prof)
    check("back", wait(lambda: w.ros.is_connected()))
    w.p_mission._play(0, 0)
    w.ros._on_closed(w.ros._gen)        # as roslibpy reports a lost socket
    check("mission paused, not left running blind", wait(lambda: not w.p_mission._exec_running))
    check("said in the events", any("mission paused" in e.text for e in w.events.events))

    print("\n[E-STOP engaged while the robot could not hear it]")
    gate.estop(False)
    w._estop_on = False
    w._set_estop(True)              # published, but say the robot missed it
    gate.estop(False)
    w._gate_seen = False            # as after a reconnect
    check("told again on the first state after reconnecting", wait(lambda: gate.engaged))
    w._set_estop(False)
    w.close()


def main() -> int:
    test_gate()
    test_app()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    os._exit(code)
