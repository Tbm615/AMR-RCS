"""End-to-end verification against the mock rosbridge server.

Run:
    python tests/test_e2e.py

Launches the mock server in a thread, starts a QApplication with the full
MainWindow, drives teleop + goal publishing, and asserts the server received
the right messages. No display required (QT_QPA_PLATFORM=offscreen).
"""
from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import asyncio
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from rcs.ui import theme
from rcs.ui.main_window import MainWindow
from rcs.core.profile import RobotProfile
from tests import mock_rosbridge


PORT = 9191


def _start_mock():
    def run():
        asyncio.run(mock_rosbridge.main_async("127.0.0.1", PORT, duration=20))
    t = threading.Thread(target=run, daemon=True)
    t.start()
    time.sleep(0.6)   # let the socket bind
    return t


def _profile() -> RobotProfile:
    p = RobotProfile.new(name="mock")
    p.host = "127.0.0.1"
    p.port = PORT
    return p


def main() -> int:
    _start_mock()

    app = QApplication(sys.argv)
    theme.apply(app)
    w = MainWindow()
    w.show()

    # swap the first profile for one pointed at the mock
    prof = _profile()
    w.profiles.profiles[0] = prof
    w.profiles.active_id = prof.id
    w.profiles.save()

    results = {"ticks": 0, "goal_sent": False, "cmdvel_sent": False, "done": False}

    def after_connect():
        # we're connected (or trying); send a teleop command, then a goal
        w.ros.publish_cmd_vel(0.2, 0.1)
        results["cmdvel_sent"] = True
        QTimer.singleShot(500, send_goal)

    def send_goal():
        w.ros.send_nav_goal(1.5, 0.8, 1.57)
        results["goal_sent"] = True
        QTimer.singleShot(2000, finish)

    def finish():
        results["done"] = True
        w.ros.disconnect()
        QTimer.singleShot(300, app.quit)

    def tick():
        results["ticks"] += 1
        if w.ros.is_connected() and results["ticks"] == 10:
            after_connect()
        if results["ticks"] > 150:   # safety net
            app.quit()

    w.ros.set_profile(prof)
    w.ros.connect()

    poll = QTimer()
    poll.timeout.connect(tick)
    poll.start(100)

    # overall watchdog in case something hangs
    QTimer.singleShot(15000, app.quit)
    app.exec()

    # ---- verify ----
    pubs = mock_rosbridge.HUB.received_publishes
    cmd = [p for p in pubs if p["topic"] == "/cmd_vel_safe"]
    goal = [p for p in pubs if p["topic"] == "/goal_pose"]

    # subscribe traffic landed too?
    subs = set()
    for topic, s in mock_rosbridge.HUB.subs_by_topic.items():
        if s:
            subs.add(topic)

    stats = w.ros.topic_stats

    ok = True
    def check(name, cond):
        nonlocal ok
        print(f"  {'OK' if cond else 'FAIL'}  {name}")
        if not cond:
            ok = False

    print("\n--- E2E results ---")
    check("connected to mock", w.ros.is_connected() or results["done"])
    check("teleop cmd_vel published to server", len(cmd) >= 1)
    # The teleop panel streams at 20 Hz (zeros when idle); our one-shot
    # publish_cmd_vel(0.2, 0.1) is one of many publishes, not the last.
    if cmd:
        hit = any(abs(float(c["msg"]["linear"]["x"]) - 0.2) < 0.05 and
                  abs(float(c["msg"]["angular"]["z"]) - 0.1) < 0.05 for c in cmd)
        check("  at least one cmd_vel matched (0.2, 0.1)", hit)
    check("nav goal published to /goal_pose", len(goal) >= 1)
    if goal:
        pos = goal[-1]["msg"]["pose"]["position"]
        check("  goal x matches", abs(float(pos["x"]) - 1.5) < 0.05)
        check("  goal y matches", abs(float(pos["y"]) - 0.8) < 0.05)
    check("map subscribed", "/map" in subs or stats.get("map") is not None)
    check("scan rate > 0", (stats.get("scan") and stats["scan"].hz > 1.0) if stats.get("scan") else False)
    check("odom rate > 0", (stats.get("odom") and stats["odom"].hz > 1.0) if stats.get("odom") else False)
    check("tf rate > 0", (stats.get("tf") and stats["tf"].hz > 1.0) if stats.get("tf") else False)
    check("map rate > 0", (stats.get("map") and stats["map"].hz > 0.1) if stats.get("map") else False)

    print(f"\n{'ALL CHECKS PASSED' if ok else 'FAILURES'}")
    return 0 if ok else 1


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush(); sys.stderr.flush()
    # Skip interpreter teardown: Qt/PySide object destruction order at exit
    # can segfault after all checks already ran.
    os._exit(rc)
