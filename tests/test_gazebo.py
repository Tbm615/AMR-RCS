"""The whole robot side in Gazebo, driven by the real app (sim/gazebo/run_gazebo.sh
--test): the Hinson model, the real rosbridge, rcs_launcher.py, scan filter, Nav2
and CBF filter. Where the robot really is comes from Gazebo, so this checks the
app against the truth, not against itself.

Covers what the fake base of test_sim_nav2.py cannot: Robot programs, camera and
3D map, clearance bars on real obstacles, speed steps, a lost connection, SLAM.
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import faulthandler  # noqa: E402
import signal  # noqa: E402
faulthandler.register(signal.SIGUSR1, all_threads=True)    # kill -USR1 <pid>: where is it stuck
# keep the operator's real profiles/missions untouched
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_gz_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

app = QApplication(sys.argv)
from rcs.core.mission import Mission, Waypoint
from rcs.ui import theme
from rcs.ui.main_window import MainWindow

CTL = str(ROOT / "sim" / "gazebo" / "sim_ctl.py")
MAP = "test_rooms"                  # sim/gazebo/make_test_map.py
START = (3.0, 4.0, 0.0)             # Warehouse
GOAL_A = (7.2, 11.0, 1.57)          # corridor, through the Warehouse door
GOAL_B = (19.0, 4.7, 0.0)           # Columns, through the door between the rooms
FAR = (24.0, 17.0, 0.0)             # Workshop, past what is not on the map
PLACE = (10.0, 5.0)
# not on the map: (x, y, radius it takes up)
LOOSE = ((13.0, 10.3, 0.5), (10.0, 7.2, 0.57), (24.0, 5.0, 0.3), (18.8, 15.0, 0.3))
TRUTH = os.path.join(tempfile.mkdtemp(prefix="rcs_gz_truth_"), "truth")

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name}" + (f"  ({detail})" if detail else ""), flush=True)


def note(text):
    print(f"  ...  {text}", flush=True)


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


def truth():
    """Where the robot really is, from Gazebo: (x, y, yaw)."""
    try:
        return tuple(float(v) for v in Path(TRUTH).read_text().split())
    except (OSError, ValueError):
        return None


def kill_bridge():
    """rosbridge ignores SIGTERM: it keeps the port with ROS shut down, the app
    connects to it again and gets nothing. SIGINT stops it."""
    pattern = "rosbridge_websocket_launch|rosbridge_server/rosbridge_websocket|rosapi/rosapi_node"
    subprocess.run(["pkill", "-INT", "-f", pattern], check=False)
    end = time.monotonic() + 8
    while time.monotonic() < end and subprocess.run(["pgrep", "-f", pattern], capture_output=True).returncode == 0:
        time.sleep(0.1)
    subprocess.run(["pkill", "-KILL", "-f", pattern], check=False)


def ctl(*args):
    subprocess.run(["python3", CTL, *map(str, args)], check=True, timeout=30, stdout=subprocess.DEVNULL)


def main() -> int:
    theme.apply(app)
    w = MainWindow()
    w.resize(1400, 900)
    w.show()
    watcher = subprocess.Popen(["python3", CTL, "truth", TRUTH])

    statuses, scans, frames, odom = [], [], [], [0.0, 0.0]
    clear = [None]
    w.ros.nav_status.connect(statuses.append)
    w.ros.scan_received.connect(scans.append)
    w.ros.image_received.connect(frames.append)

    def on_odom(m):
        t = m["twist"]["twist"]
        odom[0], odom[1] = t["linear"]["x"], t["angular"]["z"]
    w.ros.odom_received.connect(on_odom)
    real_set = w.p_teleop.set_clearance
    w.p_teleop.set_clearance = lambda c: (clear.__setitem__(0, c), real_set(c))[1]

    def stack(name):
        return w.p_robot._state.get("stacks", {}).get(name, {}).get("state")

    def moving():
        return abs(odom[0]) + abs(odom[1])

    def err_to(x, y):
        t = truth()
        return math.hypot(t[0] - x, t[1] - y) if t else 99.0

    def app_error():
        t = truth()
        return math.hypot(t[0] - w._robot_x, t[1] - w._robot_y) if t else 99.0

    def scan_on_walls(m, dx=0.0, dy=0.0, dth=0.0):
        """Share of the lidar points, placed by the app (moved by dx, dy, dth),
        within 2 cells of a wall."""
        grid = w.p_map.view.map_img.image
        if grid is None:
            return 0.0, 0
        r = np.array([np.inf if v is None else v for v in m["ranges"]], dtype=np.float32)
        a = m["angle_min"] + np.arange(r.size) * m["angle_increment"]
        ok = np.isfinite(r) & (r < m["range_max"])
        px = w._laser_x + dx + r[ok] * np.cos(a[ok] + w._laser_yaw + dth)
        py = w._laser_y + dy + r[ok] * np.sin(a[ok] + w._laser_yaw + dth)
        grid = grid.T
        ox, oy = w.p_map.view._map_origin
        res = w.p_map.view._map_res
        occ = (grid >= 65) & (grid <= 100)
        near = occ.copy()
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                near |= np.roll(np.roll(occ, dr, 0), dc, 1)
        cols, rows = ((px - ox) / res).astype(int), ((py - oy) / res).astype(int)
        inside = (rows >= 0) & (rows < near.shape[0]) & (cols >= 0) & (cols < near.shape[1])
        return (float(near[rows[inside], cols[inside]].mean()) if inside.any() else 0.0), int(inside.sum())

    def send_goal(goal):
        """Nav2 can drop a first goal right after activation; the operator resends."""
        for _ in range(5):
            statuses.clear()
            w.p_map.goal_requested.emit(*goal)
            if wait_until(lambda: "active" in statuses, 8):
                return True
        return False

    prof = w.profiles.active()
    prof.host, prof.port = "127.0.0.1", 9090
    w.profiles.upsert(prof)
    w._on_connect(prof)

    print("\n[1] Robot programs: start the robot from the app")
    w._show("robot")
    check("connected", wait_until(w.ros.is_connected, 20))
    check("launcher online", wait_until(lambda: w.p_robot._launcher_online(), 15))
    check("saved maps listed", wait_until(lambda: w.p_robot.map_combo.findText(MAP) >= 0, 10))
    w.p_robot.map_combo.setCurrentText(MAP)
    check("map preview shown", wait_until(lambda: MAP in w.p_robot._previews if hasattr(w.p_robot, "_previews")
                                          else w.p_robot.preview.isVisible(), 10))
    w.p_robot.base.start_btn.click()
    check("bringup running", wait_until(lambda: stack("bringup") == "running", 60), str(stack("bringup")))
    check("robot in Gazebo", wait_until(lambda: truth() is not None, 30), str(truth()))
    w.p_robot.nav.start_btn.click()
    check("Nav2 running", wait_until(lambda: stack("nav") == "running", 90), str(stack("nav")))

    print("\n[2] Localize from the app, compare with the truth")
    w._show("map")
    check("map received", wait_until(lambda: w.p_map.view.map_img.image is not None, 30))
    pump(6)
    for _ in range(4):
        w.p_map.set_initial_pose.emit(*START)
        if wait_until(lambda: w.tf.pose_xyyaw("map", "base_link") is not None, 8):
            break
    check("app shows the robot where it really is (<0.2 m)", wait_until(lambda: app_error() < 0.2, 15),
          f"app ({w._robot_x:.2f}, {w._robot_y:.2f}) truth {truth()}")
    check("scan arrives", wait_until(lambda: len(scans) >= 3, 10))
    m = scans[-1]
    check("real lidar size", len(m["ranges"]) == 4995, str(len(m["ranges"])))
    # the lidar placed through /tf_static: 0.4 m ahead of base_link
    lx = w._robot_x + 0.4 * math.cos(w._robot_yaw)
    ly = w._robot_y + 0.4 * math.sin(w._robot_yaw)
    check("lidar placed from /tf_static (0.4 m ahead of the robot)",
          math.hypot(w._laser_x - lx, w._laser_y - ly) < 0.01 and abs(w._laser_yaw - w._robot_yaw) < 0.01,
          f"laser ({w._laser_x:.3f}, {w._laser_y:.3f}) expected ({lx:.3f}, {ly:.3f})")

    print("\n[3] Teleop: speed steps, against the real speed")
    w._show("teleop")
    t = w.p_teleop
    for i, (label, share) in enumerate((("Slow", 0.25), ("Full", 1.0))):
        step = 0 if share < 1 else 2
        t.set_speed(step)
        before = truth()
        QTest.keyPress(t, Qt.Key_W)
        pump(1.6)
        v = odom[0]
        QTest.keyRelease(t, Qt.Key_W)
        expected = prof.limits.max_linear * share
        check(f"{label}: wheels at {expected:.2f} m/s", abs(v - expected) < 0.05, f"{v:.3f}")
        check(f"{label}: stops on key release", wait_until(lambda: moving() < 0.01, 3), f"{moving():.3f}")
        after = truth()
        heading = math.atan2(after[1] - before[1], after[0] - before[0])
        check(f"{label}: went where the robot faces", abs(math.sin(heading - before[2])) < 0.1)
    t.set_speed(1)

    print("\n[3b] Turning on the spot: about the robot's centre, odometry agrees with the truth")
    # the wheels touch the floor at +-0.195 m, as wheel_separation says: off-centre
    # contacts (the URDF before v0.6) made the centre circle and odometry drift
    t.set_speed(2)
    before, ob = truth(), w.tf.pose_xyyaw("odom", "base_link")
    QTest.keyPress(t, Qt.Key_A)
    pump(2.5)
    QTest.keyRelease(t, Qt.Key_A)
    wait_until(lambda: moving() < 0.01, 3)
    pump(0.5)
    # the last transforms of the turn can come a little late: wait until still
    last = [None]

    def settled():
        now = w.tf.pose_xyyaw("odom", "base_link")
        ok = now is not None and last[0] is not None and abs(now[2] - last[0][2]) < 0.005
        last[0] = now
        return ok
    for _ in range(15):
        pump(0.3)
        if settled():
            break
    after, oa = truth(), w.tf.pose_xyyaw("odom", "base_link")

    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))
    moved = math.hypot(after[0] - before[0], after[1] - before[1])
    turned = wrap(after[2] - before[2])
    check("really turned", abs(turned) > 1.0, f"{math.degrees(turned):.0f}°")
    check("turned about its centre (moved < 3 cm)", moved < 0.03,
          f"{moved * 100:.1f} cm for {math.degrees(turned):.0f}°")
    if ob is not None and oa is not None:
        err = wrap(turned - wrap(oa[2] - ob[2]))
        check("odometry turn matches the truth (< 5°)", abs(err) < math.radians(5),
              f"truth {math.degrees(turned):.1f}°, odom {math.degrees(wrap(oa[2] - ob[2])):.1f}°")
    else:
        check("odometry available", False, "no odom -> base_link")
    t.set_speed(1)

    print("\n[3c] The scan on the map's walls, now that AMCL has seen the robot move and turn")
    pump(2)
    scans.clear()
    wait_until(lambda: len(scans) >= 2, 5)
    frac, n = scan_on_walls(scans[-1]) if scans else (0.0, 0)
    # AMCL is good to a few cm and a few degrees: the app's placement is right
    # when a nudge within that lands the scan on the walls
    best = max(((scan_on_walls(scans[-1], dx, dy, math.radians(dth))[0], dx, dy, dth)
                for dx in (-0.1, 0.0, 0.1) for dy in (-0.1, 0.0, 0.1) for dth in (-3, -2, -1, 0, 1, 2, 3)),
               key=lambda b: b[0]) if scans else (0.0, 0, 0, 0)
    t_now = truth()
    yaw_err = math.degrees(math.atan2(math.sin(w._robot_yaw - t_now[2]), math.cos(w._robot_yaw - t_now[2]))) \
        if t_now else 99.0
    check("scan lands on the map's walls (>=90%, within AMCL's error)", best[0] >= 0.9,
          f"as placed {frac:.1%} of {n}; best {best[0]:.1%} at dx {best[1]:+.1f} dy {best[2]:+.1f} "
          f"dyaw {best[3]:+d}°; app {app_error():.3f} m and {yaw_err:+.1f}° from the truth")

    print("\n[4] Clearance bars on a real obstacle")
    x, y, yaw = truth()
    gap = 1.20                       # front edge of the robot to the box
    ahead = 0.45 + gap + 0.25
    ctl("box", "probe", x + ahead * math.cos(yaw), y + ahead * math.sin(yaw))
    check("box seen at the right distance", wait_until(lambda: clear[0] and abs(clear[0].front - gap) < 0.12, 8),
          f"front {clear[0].front if clear[0] else None}, placed at {gap}")
    note(f"rear bar: {clear[0].rear if clear[0] else None} (lidar sits at the front of the body)")
    ctl("remove", "probe")
    check("bar clears when the box goes", wait_until(lambda: clear[0].front > gap + 0.5, 8), f"{clear[0].front:.2f}")

    print("\n[5] Goal, by the truth; Esc; E-STOP")
    w._show("map")
    check("goal accepted", send_goal(GOAL_A), str(statuses[:4]))
    check("Nav2 succeeded", wait_until(lambda: "succeeded" in statuses, 180), str(sorted(set(statuses))))
    check("really at the goal (<0.3 m)", err_to(*GOAL_A[:2]) < 0.3, f"{err_to(*GOAL_A[:2]):.3f} m")
    check("app agrees with the truth (<0.2 m)", app_error() < 0.2, f"{app_error():.3f} m")
    check("moving to B", send_goal(GOAL_B) and wait_until(lambda: moving() > 0.05, 20))
    pump(2)
    w.on_escape()
    check("Esc cancels the goal", wait_until(lambda: "canceled" in statuses, 10), str(sorted(set(statuses))))
    check("robot stopped", wait_until(lambda: moving() < 0.01, 5), f"{moving():.3f}")
    check("moving again", send_goal(GOAL_B) and wait_until(lambda: moving() > 0.1, 20))
    pump(2)
    before, t0 = truth(), time.monotonic()
    w._set_estop(True)
    stopped = wait_until(lambda: moving() < 0.01, 5)
    took = time.monotonic() - t0
    check("E-STOP stops the robot", stopped, f"{took:.2f} s, {err_to(before[0], before[1]):.2f} m")
    pump(2)
    check("stays stopped", moving() < 0.01, f"{moving():.3f}")
    w._set_estop(False)
    pump(1)

    print("\n[5b] Across the building, past what is not on the map")
    closest = [99.0]

    def watch_gap():
        p = truth()
        closest[0] = min(closest[0], min(math.hypot(p[0] - x, p[1] - y) - r for x, y, r in LOOSE))
        return "succeeded" in statuses or "aborted" in statuses
    check("goal accepted", send_goal(FAR), str(statuses[:4]))
    wait_until(watch_gap, 240)
    check("reached the far room", "succeeded" in statuses, str(sorted(set(statuses))))
    check("really there (<0.3 m)", err_to(*FAR[:2]) < 0.3, f"{err_to(*FAR[:2]):.3f} m")
    check("never touched an unmapped object", closest[0] > 0.3, f"centre came within {closest[0]:.2f} m of one")
    check("back home", send_goal(GOAL_A) and wait_until(lambda: "succeeded" in statuses, 240),
          str(sorted(set(statuses))))

    print("\n[6] Mission: 2 laps")
    ms = Mission.new("gazebo laps")
    ms.waypoints = [Waypoint.new("B", *GOAL_B), Waypoint.new("A", *GOAL_A)]
    for wp in ms.waypoints:
        wp.tolerance = 0.35
    ms.loop = True
    ms.laps = 2
    w.missions.upsert(ms)
    w.p_mission._reload_missions()
    w.p_mission._set_combo_to(ms.id)
    w._show("mission")
    sent = []
    w.p_mission.goal_requested.connect(lambda gx, gy, _yaw: sent.append((round(gx, 2), round(gy, 2))))
    w.p_mission.play_btn.click()
    def legs():
        """The goals sent, a retry of the same waypoint counted once."""
        return [g for i, g in enumerate(sent) if i == 0 or g != sent[i - 1]]
    check("all 4 legs sent", wait_until(lambda: len(legs()) >= 4, 420), str(sent))
    check("mission ended by itself", wait_until(lambda: not w.p_mission._exec_running, 180))
    check("order B A B A", legs()[:4] == [GOAL_B[:2], GOAL_A[:2]] * 2,
          f"{legs()[:4]} ({len(sent) - len(legs())} retries)")
    check("really at the last waypoint", err_to(*GOAL_A[:2]) < 0.4, f"{err_to(*GOAL_A[:2]):.3f} m")

    print("\n[7] Saved place, then Use last position after restarting Nav2")
    w._show("nav")
    place = w.place_store.add("Mid", *PLACE, 0.0, MAP)
    statuses.clear()
    w._go_to_place(place)
    check("place reached", wait_until(lambda: "succeeded" in statuses, 180) and err_to(*PLACE) < 0.3,
          f"{err_to(*PLACE):.3f} m")
    pump(1.5)
    w._show("robot")
    w.p_robot.nav.stop_btn.click()
    check("Nav2 stopped", wait_until(lambda: stack("nav") == "stopped", 40), str(stack("nav")))
    w.p_robot.nav.start_btn.click()
    check("Nav2 back", wait_until(lambda: stack("nav") == "running", 90), str(stack("nav")))
    pump(8)
    check("Use last position offered", wait_until(lambda: w.p_robot.last_pose_btn.isVisible(), 10))
    w.p_robot.last_pose_btn.click()
    check("robot found again (<0.3 m)", wait_until(lambda: app_error() < 0.3, 15), f"{app_error():.3f} m")

    print("\n[8] Camera and 3D map")
    w._show("cam")
    frames.clear()
    check("camera frames arrive", wait_until(lambda: len(frames) >= 10, 15), str(len(frames)))
    check("3D map recorded while the other pages were open (Record on every page)",
          len(w.ros.cloud_map) > 1000, f"{len(w.ros.cloud_map)} points")
    w._show("3d")
    w.ros.cloud_map.clear()
    w.p_3d.rec_btn.setChecked(True)
    # how much depends on what is in view (3 cm voxels): a wall a metre away is a few thousand
    check("3D map fills up", wait_until(lambda: len(w.ros.cloud_map) > 2000, 30), str(len(w.ros.cloud_map)))
    xyz = w.ros.cloud_map.snapshot()[0]
    if w.p_map.view.map_img.image is None:
        check("3D points against the map", False, "no map")
    elif len(xyz) == 0:
        check("3D points between floor and wall top", False, "no points")
    else:
        z = xyz[:, 2]
        check("3D points between floor and wall top", z.min() > -0.2 and z.max() < 2.3,
              f"z {z.min():.2f} .. {z.max():.2f}")
        grid = w.p_map.view.map_img.image.T
        ox, oy = w.p_map.view._map_origin
        res = w.p_map.view._map_res
        occ = (grid >= 65) & (grid <= 100)
        # what the camera sees is the world: the map's walls and the objects
        # that are only in Gazebo (LOOSE), which are as solid
        gx, gy = np.meshgrid(ox + (np.arange(occ.shape[1]) + 0.5) * res, oy + (np.arange(occ.shape[0]) + 0.5) * res)
        for lx_, ly_, lr in LOOSE:
            occ |= np.hypot(gx - lx_, gy - ly_) <= lr + res
        near = occ.copy()
        for dr in range(-3, 4):
            for dc in range(-3, 4):
                near |= np.roll(np.roll(occ, dr, 0), dc, 1)
        high = xyz[z > 0.4]
        cols, rows = ((high[:, 0] - ox) / res).astype(int), ((high[:, 1] - oy) / res).astype(int)
        inside = (rows >= 0) & (rows < near.shape[0]) & (cols >= 0) & (cols < near.shape[1])
        frac = float(near[rows[inside], cols[inside]].mean()) if inside.any() else 0.0

        def shifted(dx, dy, dth):
            c, s_ = math.cos(dth), math.sin(dth)
            px, py = high[:, 0] - w._robot_x, high[:, 1] - w._robot_y
            hx, hy = w._robot_x + dx + px * c - py * s_, w._robot_y + dy + px * s_ + py * c
            cc, rr = ((hx - ox) / res).astype(int), ((hy - oy) / res).astype(int)
            ok = (rr >= 0) & (rr < near.shape[0]) & (cc >= 0) & (cc < near.shape[1])
            return float(near[rr[ok], cc[ok]].mean()) if ok.any() else 0.0
        best = max(((shifted(dx, dy, math.radians(dth)), dx, dy, dth) for dx in np.arange(-0.6, 0.61, 0.1)
                    for dy in np.arange(-0.6, 0.61, 0.1) for dth in range(-10, 11, 2)), key=lambda b: b[0]) \
            if len(high) else (0, 0, 0, 0)
        check("3D points above 0.4 m sit on walls and objects (>=80%)", frac >= 0.8,
              f"{frac:.1%} of {inside.sum()}, built in {w.ros.cloud_world!r}, app {app_error():.2f} m from the "
              f"truth; best nearby {best[0]:.0%} at dx {best[1]:+.1f} dy {best[2]:+.1f} dyaw {best[3]:+d}°")
    w.p_3d.set_view_mode("fusion") if hasattr(w.p_3d, "set_view_mode") else None
    pump(1)

    print("\n[9] Connection lost: the robot's E-STOP gate stops it by itself")
    w._show("teleop")
    t.set_speed(2)
    QTest.keyPress(t, Qt.Key_W)
    pump(1.0)
    p0, t0 = truth(), time.monotonic()
    kill_bridge()
    last, still = truth(), t0
    while time.monotonic() - t0 < 4:
        pump(0.05)
        now = truth()
        if math.hypot(now[0] - last[0], now[1] - last[1]) > 0.002:
            still = time.monotonic()
        last = now
    QTest.keyRelease(t, Qt.Key_W)
    check("teleop: robot stops by itself", still - t0 < 1.5, f"{still - t0:.2f} s, {err_to(*p0[:2]):.2f} m")
    bridge = subprocess.Popen(["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml", "port:=9090"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    check("app reconnects by itself", wait_until(w.ros.is_connected, 40))
    t.set_speed(1)
    pump(3)
    w._show("map")
    check("goal after the link came back", send_goal(GOAL_B))
    pump(3)
    check("driving under Nav2", wait_until(lambda: abs(odom[0]) > 0.1, 20),
          f"{odom[0]:.3f} m/s, Nav2 said {statuses[-6:]}, app {app_error():.2f} m from the truth")
    kill_bridge()
    p0, t0 = truth(), time.monotonic()
    last, still = p0, t0
    while time.monotonic() - t0 < 7:
        pump(0.05)
        now = truth()
        turned = abs(math.atan2(math.sin(now[2] - last[2]), math.cos(now[2] - last[2])))
        if math.hypot(now[0] - last[0], now[1] - last[1]) > 0.002 or turned > 0.005:
            still = time.monotonic()
        last = now
    went = err_to(*p0[:2])
    check("Nav2 goal: robot stops by itself when the app is lost (< 5 s)", still - t0 < 5.0,
          f"still after {still - t0:.2f} s, went {went:.2f} m")
    bridge.wait(timeout=10)
    bridge = subprocess.Popen(["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml", "port:=9090"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    check("reconnects again", wait_until(w.ros.is_connected, 40))
    pump(2)
    w.ros.cancel_nav_goal()
    wait_until(lambda: moving() < 0.01, 10)

    print("\n[10] SLAM: build a map, save it, see it in the list")
    w._show("robot")
    w.p_robot.nav.stop_btn.click()
    check("Nav2 stopped", wait_until(lambda: stack("nav") == "stopped", 40), str(stack("nav")))
    w.p_robot.rb_slam.setChecked(True)
    w.p_robot.nav.start_btn.click()
    check("SLAM running", wait_until(lambda: stack("nav") == "running", 90), str(stack("nav")))
    pump(10)
    w.p_robot.map_name.setText("gazebo_test")
    w.p_robot.save_btn.click()
    check("map saved", wait_until(lambda: "gazebo_test" in w.p_robot._state.get("maps", []), 45),
          w.p_robot.result.text()[:80])
    w.p_robot.rb_map.setChecked(True)
    w.p_robot.map_combo.setCurrentText("gazebo_test")
    pump(3)

    print("\n[11] Stop from the app")
    w.p_robot._send("stop", stack="bringup")
    check("everything stopped", wait_until(lambda: stack("bringup") == "stopped" and stack("nav") == "stopped", 40),
          f"{stack('bringup')} {stack('nav')}")
    pump(3)
    left = subprocess.run(["pgrep", "-c", "gzserver"], capture_output=True, text=True).stdout.strip()
    check("Gazebo closed with it", left == "0", left)

    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    watcher.kill()
    bridge.kill()
    w.ros.disconnect_robot()
    pump(0.5)
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
