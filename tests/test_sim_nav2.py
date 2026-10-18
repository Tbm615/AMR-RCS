"""Integration test: the real RCS MainWindow driving a real ROS 2 Nav2 stack
through a real rosbridge_server (start it with sim/run_sim.sh first).

Checks, in order:
  1. connect, receive /map, robot pose from TF (map->odom->base_link)
  2. lidar alignment: scan points projected with the app's laser pose
     (from /tf_static base_link->laser_frame) land on occupied map cells
  3. goal sent through the 2D-map panel reaches Nav2 and the robot arrives
  4. cancel stops navigation
  5. E-stop while navigating: goal canceled, robot stops, /amr/estop published
  6. a looping mission visits its waypoints in order and wraps around

Run:  QT_QPA_PLATFORM=offscreen python tests/test_sim_nav2.py
"""
from __future__ import annotations

import math
import os
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# keep the operator's real profiles/missions untouched
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_sim_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from PySide6.QtWidgets import QApplication

app = QApplication(sys.argv)
from rcs.core.mission import Mission, Waypoint
from rcs.ui import theme
from rcs.ui.main_window import MainWindow

HOST = os.environ.get("RCS_SIM_HOST", "127.0.0.1")
PORT = int(os.environ.get("RCS_SIM_PORT", "9090"))
START = (3.55, -5.10)
GOAL_A = (3.30, -8.55, -1.64)
GOAL_B = (3.30, -2.60, 1.57)

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  OK   {name}" + (f"  ({detail})" if detail else ""), flush=True)
    else:
        FAIL += 1
        print(f"  FAIL {name}" + (f"  ({detail})" if detail else ""), flush=True)


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


def main() -> int:
    theme.apply(app)
    w = MainWindow()
    w.show()

    statuses, scans, odom_v, estop_echo = [], [], [0.0], []
    w.ros.nav_status.connect(statuses.append)
    w.ros.scan_received.connect(lambda m: scans.append(m))
    w.ros.odom_received.connect(
        lambda m: odom_v.__setitem__(0, abs(m["twist"]["twist"]["linear"]["x"])
                                     + abs(m["twist"]["twist"]["angular"]["z"])))
    w.ros.estop_received.connect(estop_echo.append)

    def pose():
        return w._robot_x, w._robot_y, w._robot_yaw

    def dist_to(x, y):
        rx, ry, _ = pose()
        return math.hypot(rx - x, ry - y)

    prof = w.profiles.active()
    prof.host, prof.port = HOST, PORT
    w.profiles.upsert(prof)
    w._on_connect(prof)

    print("\n[1] connect, map, pose")
    check("connected to rosbridge", wait_until(w.ros.is_connected, 15))
    check("map received", wait_until(lambda: w.p_map.view.map_img.image is not None, 20))
    # Like an operator would: localize the robot from the app (2D map panel
    # "Set initial pose"), so AMCL starts publishing map->odom.
    w.p_map.set_initial_pose.emit(START[0], START[1], -1.64)
    have_tf = wait_until(lambda: w.tf.pose_xyyaw("map", "base_link") is not None, 30)
    check("map->base_link available from TF", have_tf)
    check("robot pose matches start", wait_until(lambda: dist_to(*START) < 0.3, 10),
          "pose=(%.2f, %.2f)" % pose()[:2])

    print("\n[2] lidar alignment")
    laser = w.tf.pose_xyyaw("base_link", "laser_frame")
    check("base_link->laser_frame received from /tf_static",
          laser is not None and abs(laser[0] - 0.40) < 1e-3,
          f"laser={laser}")
    check("scan stream arriving", wait_until(lambda: len(scans) >= 3, 10))
    if scans and w.p_map.view.map_img.image is not None:
        m = scans[-1]
        r = np.array([np.inf if v is None else v for v in m["ranges"]], dtype=np.float32)
        a = m["angle_min"] + np.arange(r.size) * m["angle_increment"]
        ok = np.isfinite(r) & (r < m["range_max"])
        lx, ly, lyaw = w._laser_x, w._laser_y, w._laser_yaw
        px = lx + r[ok] * np.cos(a[ok] + lyaw)
        py = ly + r[ok] * np.sin(a[ok] + lyaw)
        grid = w.p_map.view.map_img.image.T            # (h, w), row 0 = lowest y
        ox, oy = w.p_map.view._map_origin
        res = w.p_map.view._map_res
        occ = (grid >= 65) & (grid <= 100)
        near = occ.copy()
        for dr in (-2, -1, 0, 1, 2):                    # 2-cell tolerance
            for dc in (-2, -1, 0, 1, 2):
                near |= np.roll(np.roll(occ, dr, 0), dc, 1)
        cols = ((px - ox) / res).astype(int)
        rows = ((py - oy) / res).astype(int)
        inside = (rows >= 0) & (rows < near.shape[0]) & (cols >= 0) & (cols < near.shape[1])
        frac = float(near[rows[inside], cols[inside]].mean()) if inside.any() else 0.0
        check("scan points land on map walls (>=90%)", frac >= 0.9,
              f"{frac:.1%} of {inside.sum()} points")

    print("\n[3] goal from the 2D map panel")
    # Nav2's navigation lifecycle only finishes activating once AMCL provides
    # map->odom, so the first goal can be dropped. The app must say so instead
    # of claiming it is navigating; the operator then simply resends.
    for attempt in range(1, 6):
        statuses.clear()
        w.p_map.goal_requested.emit(*GOAL_A)
        if wait_until(lambda: any(s in statuses for s in ("active", "no_response")) and "pending" in statuses
                      and (len(statuses) > 1), 8) and "no_response" not in statuses:
            break
        print(f"       attempt {attempt}: {statuses} -> resend", flush=True)
    check("goal acknowledged by Nav2 (pending -> active)",
          statuses[:1] == ["pending"] and "active" in statuses, f"statuses={statuses[:4]}")
    arrived = wait_until(lambda: "succeeded" in statuses, 150)
    check("Nav2 reported succeeded", arrived, f"statuses={sorted(set(statuses))}")
    check("robot at goal A (<0.3 m)", dist_to(*GOAL_A[:2]) < 0.3,
          "pose=(%.2f, %.2f)" % pose()[:2])

    print("\n[4] cancel")
    statuses.clear()
    w.p_map.goal_requested.emit(*GOAL_B)
    moving = wait_until(lambda: odom_v[0] > 0.05, 30)
    check("robot starts moving toward goal B", moving)
    pump(2.0)
    w.p_map.cancel_requested.emit()
    check("Nav2 reported canceled", wait_until(lambda: "canceled" in statuses, 10),
          f"statuses={sorted(set(statuses))}")
    check("robot stopped after cancel", wait_until(lambda: odom_v[0] < 0.01, 5),
          f"v={odom_v[0]:.3f}")

    print("\n[5] E-stop while navigating")
    statuses.clear()
    estop_echo.clear()
    w.p_map.goal_requested.emit(*GOAL_B)
    check("robot moving again", wait_until(lambda: odom_v[0] > 0.05, 30))
    pump(1.5)
    w.sidebar_estop.click()
    app.processEvents()
    check("E-stop engaged in UI", w.sidebar_estop.isChecked() and w.p_teleop.is_estopped())
    check("goal canceled by E-stop", wait_until(lambda: "canceled" in statuses, 10),
          f"statuses={sorted(set(statuses))}")
    check("robot stopped within 2 s", wait_until(lambda: odom_v[0] < 0.01, 2),
          f"v={odom_v[0]:.3f}")
    check("/amr/estop=true published", wait_until(lambda: True in estop_echo, 5))
    pump(2.0)
    check("robot stays stopped while engaged", odom_v[0] < 0.01, f"v={odom_v[0]:.3f}")
    w.sidebar_estop.click()
    app.processEvents()
    check("E-stop released", not w.sidebar_estop.isChecked() and not w.p_teleop.is_estopped())
    check("/amr/estop=false published", wait_until(lambda: False in estop_echo, 5))

    print("\n[6] looping mission")
    m = Mission.new("sim loop")
    m.loop = True
    m.waypoints = [Waypoint.new("B", *GOAL_B), Waypoint.new("A", *GOAL_A)]
    for wp in m.waypoints:
        wp.tolerance = 0.35
        wp.dwell = 0.5
    w.missions.upsert(m)
    w.p_mission._reload_missions()
    w.p_mission._set_combo_to(m.id)
    dispatched = []
    w.p_mission.goal_requested.connect(lambda x, y, yaw: dispatched.append((round(x, 2), round(y, 2), pose())))
    w.p_mission.play_btn.click()
    wrapped = wait_until(lambda: len(dispatched) >= 3, 300)
    check("mission dispatched B, A, then wrapped to B", wrapped,
          f"{[(d[0], d[1]) for d in dispatched]}")
    if len(dispatched) >= 3:
        order = [(d[0], d[1]) for d in dispatched[:3]]
        check("waypoint order B -> A -> B",
              order == [GOAL_B[:2], GOAL_A[:2], GOAL_B[:2]], f"{order}")
        at_b = math.hypot(dispatched[1][2][0] - GOAL_B[0], dispatched[1][2][1] - GOAL_B[1])
        at_a = math.hypot(dispatched[2][2][0] - GOAL_A[0], dispatched[2][2][1] - GOAL_A[1])
        check("robot was at B when it moved on", at_b < 0.35, f"{at_b:.2f} m")
        check("robot was at A when it moved on", at_a < 0.35, f"{at_a:.2f} m")
    w.p_mission.stop_btn.click()
    check("mission stop cancels navigation", wait_until(lambda: odom_v[0] < 0.01, 10))

    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    w.ros.disconnect_robot()
    pump(0.5)
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(rc)
