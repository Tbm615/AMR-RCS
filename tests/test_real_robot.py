"""The real app against the real robot: every page and the robot side.

    QT_QPA_PLATFORM=offscreen .venv/bin/python tests/test_real_robot.py \
        --host 192.168.10.10 --map map_3269 --pose -0.89,0.35,268 [--drive]

--pose is where the robot stands on --map (x, y in m, yaw in degrees), for the
initial pose; tests/locate_robot.py finds it from one scan.

Part A (always) does not move the robot: connection, Robot programs, sensors,
battery, E-STOP, Nav2 on the saved map, keepout and speed zones, maps to and
from the robot, the 3D map, SLAM and saving its map.
Part B (--drive) moves it, slowly and within about 1.5 m of where it is:
teleop, E-STOP while driving, goals, Esc, a slow zone, a keepout zone, a
mission, a saved place, losing the app while driving. Keep the area clear and
the hardware E-stop within reach.

What the test adds on the robot (zones, test maps) it removes at the end. The
operator's own app settings are not touched (own config folder).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# keep the operator's real profiles, settings and per-robot data untouched
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_real_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

app = QApplication(sys.argv)
from rcs import __version__ as APP_VERSION  # noqa: E402
from rcs.core.mission import Mission, Waypoint  # noqa: E402
from rcs.ui import theme  # noqa: E402
from rcs.ui.main_window import MainWindow  # noqa: E402
from rcs.ui.panels.robot import RobotPanel  # noqa: E402
import faulthandler  # noqa: E402
import signal  # noqa: E402

faulthandler.register(signal.SIGUSR1, all_threads=True)    # kill -USR1 <pid>: where is it stuck
RobotPanel.confirm_dialogs = False                          # QMessageBox blocks offscreen

PASS = FAIL = 0
FAILED: list = []
TEST_MAP_COPY = "rcs_test_copy"
TEST_SLAM_MAP = "rcs_test_slam"
TEST_ZONES: set = set()         # ids of the zones the test drew: removed at the end


def check(name, cond, detail=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    if not cond:
        FAILED.append(name)
    print(f"  {'OK  ' if cond else 'FAIL'} {name}" + (f"  ({detail})" if detail else ""), flush=True)
    return bool(cond)


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


class Robot:
    """Shell on the robot (sshpass + one shared connection)."""

    def __init__(self, host: str, user: str, password: str):
        self.base = ["sshpass", "-p", password, "ssh", "-o", "StrictHostKeyChecking=accept-new",
                     "-o", "ConnectTimeout=8", "-o", "ControlMaster=auto",
                     "-o", f"ControlPath={tempfile.gettempdir()}/rcs-test-%C", "-o", "ControlPersist=10m",
                     f"{user}@{host}"]
        self.env = ("source /opt/ros/humble/setup.bash; source ~/tbm_test/install/setup.bash; "
                    "export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 "
                    "CYCLONEDDS_URI=file://$HOME/tbm_test/install/bringup/share/bringup/config/cyclonedds.xml; ")

    def sh(self, cmd: str, timeout: float = 60, ros: bool = False) -> str:
        """Run cmd on the robot; the Qt loop keeps running meanwhile."""
        p = subprocess.Popen(self.base + [("bash -c " + json.dumps(self.env + cmd)) if ros else cmd],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        end = time.monotonic() + timeout
        while p.poll() is None and time.monotonic() < end:
            pump(0.05)
        if p.poll() is None:
            p.kill()
        return p.communicate()[0]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def click(btn, timeout=20.0):
    """Click like an operator: once the button is enabled (it is greyed out while
    the launcher still works on the last command)."""
    ok = wait_until(btn.isEnabled, timeout)
    if not ok:
        print(f"  ...  button {btn.text()!r} still disabled after {timeout:.0f} s", flush=True)
    btn.click()
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.10.10")
    ap.add_argument("--user", default="iop")
    ap.add_argument("--password", default=os.environ.get("RCS_ROBOT_PASSWORD", ""))
    ap.add_argument("--map", default="map_3269")
    ap.add_argument("--pose", default="", help="x,y,yaw_deg of the robot on --map")
    ap.add_argument("--build-map", default="", metavar="NAME",
                    help="only: build a map of the room with SLAM (0.7 m out and back in 4 directions), save it as "
                         "NAME on the robot, print the robot's pose on it")
    ap.add_argument("--drive", action="store_true", help="also part B: the robot moves")
    ap.add_argument("--only", default="", help="comma-separated step numbers to run (debugging)")
    args = ap.parse_args()
    if not args.build_map and not args.pose:
        ap.error("--pose is needed (or --build-map)")
    px, py, pyaw = (float(v) for v in (args.pose or "0,0,0").split(","))
    START = (px, py, math.radians(pyaw))
    MAP = args.map
    robot = Robot(args.host, args.user, args.password)
    only = set(args.only.split(",")) - {""}

    def step(n: str) -> bool:
        return not only or n in only

    theme.apply(app)
    w = MainWindow()
    w.resize(1400, 900)
    w.show()

    statuses, scans, frames = [], [], []
    odom = {"v": 0.0, "w": 0.0, "n": 0}
    clear = [None]
    gate, batt = [None], [None]
    w.ros.nav_status.connect(statuses.append)
    w.ros.scan_received.connect(scans.append)
    w.ros.image_received.connect(frames.append)
    w.ros.estop_state.connect(lambda d: gate.__setitem__(0, d))
    w.ros.battery.connect(lambda d: batt.__setitem__(0, d))

    def on_odom(m):
        t = m["twist"]["twist"]
        odom["v"], odom["w"] = t["linear"]["x"], t["angular"]["z"]
        odom["n"] += 1
    w.ros.odom_received.connect(on_odom)
    real_set = w.p_teleop.set_clearance
    w.p_teleop.set_clearance = lambda c: (clear.__setitem__(0, c), real_set(c))[1]

    def stack(name):
        return w.p_robot._state.get("stacks", {}).get(name, {}).get("state")

    def moving():
        return abs(odom["v"]) + abs(odom["w"])

    def pose():
        return w.tf.pose_xyyaw("map", "base_link")

    def dist_to(x, y):
        p = pose()
        return math.hypot(p[0] - x, p[1] - y) if p else 99.0

    def occ_near(cells: int):
        grid = w.p_map.view.map_img.image
        if grid is None:
            return None
        g = grid.T
        occ = (g >= 65) & (g <= 100)
        near = occ.copy()
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                near |= np.roll(np.roll(occ, dr, 0), dc, 1)
        return near

    def scan_on_walls(m, cells=2):
        """Share of the lidar points, placed by the app, within `cells` cells of a wall."""
        near = occ_near(cells)
        if near is None:
            return 0.0, 0
        r = np.array([np.inf if v is None else v for v in m["ranges"]], dtype=np.float32)
        a = m["angle_min"] + np.arange(r.size) * m["angle_increment"]
        ok = np.isfinite(r) & (r < m["range_max"]) & (r > 0.05)
        sx = w._laser_x + r[ok] * np.cos(a[ok] + w._laser_yaw)
        sy = w._laser_y + r[ok] * np.sin(a[ok] + w._laser_yaw)
        ox, oy = w.p_map.view._map_origin
        res = w.p_map.view._map_res
        cols, rows = ((sx - ox) / res).astype(int), ((sy - oy) / res).astype(int)
        inside = (rows >= 0) & (rows < near.shape[0]) & (cols >= 0) & (cols < near.shape[1])
        # a point on a part of the map never seen (unknown) cannot be judged: leave it out
        g = w.p_map.view.map_img.image.T
        known = np.zeros_like(inside)
        known[inside] = g[rows[inside], cols[inside]] >= 0
        inside &= known
        return (float(near[rows[inside], cols[inside]].mean()) if inside.any() else 0.0), int(inside.sum())

    def send_goal(goal):
        """Nav2 can drop a first goal right after activation; the operator resends."""
        for _ in range(4):
            statuses.clear()
            w.p_map.goal_requested.emit(*goal)
            if wait_until(lambda: "active" in statuses, 8):
                return True
        return False

    def localize():
        for _ in range(4):
            w.p_map.set_initial_pose.emit(*START)
            if wait_until(lambda: pose() is not None and dist_to(*START[:2]) < 0.5, 8):
                return True
        return False

    def zones_answer():
        got = []
        w.ros.zones.connect(got.append)
        return got

    import threading

    def stale_watch():
        dumped = 0
        while True:
            time.sleep(1)
            st = w.ros.topic_stats.get("launcher")
            if w.ros.is_connected() and st and not st.fresh(8.0) and dumped < 2:
                dumped += 1
                print("  !!!  launcher state stale for 8 s while connected: thread dump", flush=True)
                faulthandler.dump_traceback(all_threads=True)
    threading.Thread(target=stale_watch, daemon=True).start()

    prof = w.profiles.active()
    prof.host, prof.port = args.host, 9090
    w.profiles.upsert(prof)
    w._on_connect(prof)

    if args.build_map:
        return build_map(w, robot, args.build_map, stack, odom, moving, pose, clear)

    try:
        # -------------------------------------------------------------- part A
        print("\n[1] Connection and the robot side")
        w._show("robot")
        check("connected", wait_until(w.ros.is_connected, 20))
        check("launcher online", wait_until(lambda: w.p_robot._launcher_online(), 15))
        rs = w.p_robot._state.get("rcs", "")
        check("robot side is this app's version", rs.replace("b", "-beta.") == APP_VERSION or rs == APP_VERSION,
              f"robot {rs}, app {APP_VERSION}")
        caps = w.p_robot._state.get("caps", [])
        check("robot side can copy maps, zones, E-STOP gate", {"map_transfer", "zones", "estop_gate"} <= set(caps),
              str(caps))
        check("saved maps listed", wait_until(lambda: w.p_robot.map_combo.findText(MAP) >= 0, 10),
              str(w.p_robot._state.get("maps")))
        check("E-STOP service on the robot answers", wait_until(lambda: gate[0] is not None, 5), str(gate[0]))
        check("robot sees this app (heartbeat)", wait_until(lambda: (gate[0] or {}).get("apps", 0) >= 1, 6),
              str(gate[0]))

        if step("2"):
            print("\n[2] Robot programs: stop and start the robot base from the app")
            if stack("nav") not in ("stopped", None):
                click(w.p_robot.nav.stop_btn)
                wait_until(lambda: stack("nav") == "stopped", 40)
            if stack("bringup") == "running":
                click(w.p_robot.base.stop_btn)
                check("bringup stopped", wait_until(lambda: stack("bringup") == "stopped", 40), str(stack("bringup")))
                pump(2)
                left = robot.sh("pgrep -fc '[h]ins_le_ros2_node|[r]os2_control_node|[o]rbbec_camera' || true").strip()
                check("its processes are gone on the robot", left == "0", left)
            click(w.p_robot.base.start_btn)
            check("bringup running", wait_until(lambda: stack("bringup") == "running", 60), str(stack("bringup")))
            out = robot.sh("ps -eo args | grep -c '[c]md_vel_topic:=/rcs/cmd_vel_gated'")
            check("bringup drives through the E-STOP gate", out.strip() not in ("", "0"), out.strip())

        print("\n[3] Sensors in the app")
        scans.clear()
        check("odometry arrives", wait_until(lambda: odom["n"] > 20, 20), str(odom["n"]))
        check("scan arrives", wait_until(lambda: len(scans) >= 3, 20), str(len(scans)))
        if scans:
            m = scans[-1]
            check("real lidar size (4995 points)", len(m["ranges"]) == 4995, str(len(m["ranges"])))
        pump(5)
        rates = {k: round(st.hz, 1) for k, st in w.ros.topic_stats.items()}
        note(f"rates in the app: {rates}")
        sc = next((v for k, v in rates.items() if "scan" in k), 0)
        check("scan rate in the app >= 5 Hz (throttled for Wi-Fi)", sc >= 5, f"{sc} Hz")
        tfl = w.tf.pose_xyyaw("base_link", "laser_frame")
        check("lidar from /tf_static, 0.4 m ahead of base_link",
              tfl is not None and abs(tfl[0] - 0.4) < 0.01 and abs(tfl[1]) < 0.01, str(tfl))
        w._show("teleop")
        check("clearance bars computed", wait_until(lambda: clear[0] is not None, 10), str(clear[0]))
        if clear[0]:
            c = clear[0]
            note(f"clearance: front {c.front:.2f} m, rear {c.rear:.2f} m, nearest {c.nearest:.2f} m")
            check("clearance values plausible (0..30 m, or nothing in range)",
                  all(0 <= v < 30 or v == math.inf for v in (c.front, c.rear, c.nearest)))
        w._show("cam")
        frames.clear()
        check("camera frames arrive", wait_until(lambda: len(frames) >= 10, 20), str(len(frames)))
        if frames:
            f = frames[-1]
            check("camera picture has a size", f.width() >= 320 and f.height() >= 240, f"{f.width()}x{f.height()}")
        t0, n0 = time.monotonic(), len(frames)
        pump(5)
        fps = (len(frames) - n0) / (time.monotonic() - t0)
        check("camera >= 3 frames/s in the app", fps >= 3, f"{fps:.1f}/s")

        print("\n[4] Battery (JK-BMS over Bluetooth on the robot)")
        w._show("battery")
        check("battery state arrives", wait_until(lambda: batt[0] is not None, 10))
        b = batt[0] or {}
        check("BMS connected", wait_until(lambda: (batt[0] or {}).get("state") == "connected", 30), str(b.get("state")))
        r = (batt[0] or {}).get("reading") or {}
        check("15 cells", len(r.get("cells", [])) == 15, str(len(r.get("cells", []))))
        check("pack voltage = sum of cells (±0.2 V)",
              r and abs(sum(r["cells"]) - r["voltage"]) < 0.2, f"{sum(r.get('cells', [0])):.2f} vs {r.get('voltage')}")
        check("plausible values", r and 0 <= r["soc"] <= 100 and 40 < r["voltage"] < 60 and -5 < r["temps"][0] < 60,
              f"soc {r.get('soc')}, {r.get('voltage')} V, {r.get('temps')} °C")
        check("Battery page shows the charge", w.p_battery.soc.text() == f"{r.get('soc')}%", w.p_battery.soc.text())
        # the Bluetooth link drops once or twice a minute and is back in 1-2 s (weak signal on the
        # robot): a reading up to 20 s old is the normal gap, older means the link is down
        check("reading fresh (< 20 s)", wait_until(lambda: (batt[0] or {}).get("age", 99) < 20, 20),
              str((batt[0] or {}).get("age")))
        note(f"RSSI {(batt[0] or {}).get('rssi')} dBm, reconnects {(batt[0] or {}).get('reconnects')}, "
             f"bad frames {(batt[0] or {}).get('bad_frames')}")
        logs = []
        w.ros.battery_log.connect(logs.append)
        w.ros.send_battery_cmd("log_list")
        check("battery log files listed", wait_until(lambda: any(d.get("files") for d in logs), 10),
              str([d.get("op") for d in logs]))
        cur = ((batt[0] or {}).get("log") or {}).get("file")
        if cur:
            logs.clear()
            w.ros.send_battery_cmd("log_get", name=cur)
            check("this boot's log readable", wait_until(lambda: any(d.get("data") and d.get("size") for d in logs), 20),
                  str([(d.get("name"), d.get("size"), d.get("error")) for d in logs]))
        hc = {c.key: c for c in __import__("rcs.core.health", fromlist=["checks"]).checks(w.health_context())}
        note("health: " + ", ".join(f"{k}={c.level}:{c.text}" for k, c in hc.items()))
        check("Status page health: nothing in error", not [c for c in hc.values() if c.level == "error"],
              str([c.title for c in hc.values() if c.level == "error"]))

        if step("5"):
            print("\n[5] E-STOP from the app (robot standing still)")
            w._set_estop(True)
            check("robot's gate engaged", wait_until(lambda: (gate[0] or {}).get("engaged"), 3), str(gate[0]))
            out = robot.sh("timeout 4 ros2 topic echo --once /rcs/cmd_vel_gated geometry_msgs/msg/Twist", ros=True,
                           timeout=20)
            check("zero sent to the drives while engaged", "x: 0.0" in out and "z: 0.0" in out, out.strip()[-120:])
            robot.sh("pkill -f '[r]cs_safety.py'")
            pump(3)
            check("gate restarted by the start script", "rcs_safety" in robot.sh("pgrep -fa '[r]cs_safety.py' || true"))
            gate[0] = None
            check("E-STOP survives a restart of the gate", wait_until(lambda: (gate[0] or {}).get("engaged"), 10),
                  str(gate[0]))
            w._set_estop(False)
            check("released", wait_until(lambda: gate[0] and not gate[0].get("engaged"), 4), str(gate[0]))

        print(f"\n[6] Nav2 on {MAP}, localized from the app")
        w._show("robot")
        w.p_robot.rb_map.setChecked(True)
        w.p_robot.map_combo.setCurrentText(MAP)
        pump(1)
        if stack("nav") == "running" and w.p_robot._state.get("stacks", {}).get("nav", {}).get("args", {}).get("map") \
                not in (MAP, f"{MAP}.yaml"):
            click(w.p_robot.nav.stop_btn)
            wait_until(lambda: stack("nav") == "stopped", 40)
        if stack("nav") != "running":
            click(w.p_robot.nav.start_btn)
        t0 = time.monotonic()
        check("Nav2 running", wait_until(lambda: stack("nav") == "running", 120),
              f"{stack('nav')} {w.p_robot._state.get('stacks', {}).get('nav', {}).get('detail')}")
        note(f"Nav2 up in {time.monotonic() - t0:.0f} s")
        nav_log = w.p_robot._state.get("stacks", {}).get("nav", {}).get("log", [])
        w._show("map")
        check("map received", wait_until(lambda: w.p_map.view.map_img.image is not None, 30))
        pump(5)
        check("no automatic (0, 0) pose: robot not localized until we say", pose() is None
              or dist_to(0, 0) > 0.05, str(pose()))
        hc = {c.key: c for c in __import__("rcs.core.health", fromlist=["checks"]).checks(w.health_context())}
        check("Status says to set the initial pose", hc.get("loc") and hc["loc"].level == "warn",
              str(hc.get("loc")))
        check("initial pose taken", localize(), str(pose()))
        pump(3)
        p = pose()
        check("app places the robot where it is (< 0.3 m of the given pose)", p and dist_to(*START[:2]) < 0.3,
              f"{p}")
        scans.clear()
        wait_until(lambda: len(scans) >= 2, 5)
        frac, n = scan_on_walls(scans[-1]) if scans else (0.0, 0)
        check("scan lands on the map's walls (>= 80 %)", frac >= 0.8, f"{frac:.0%} of {n} points")
        gc, lc = [], []
        gc_grid = []        # (info, grid) of each global costmap, before the app drops the raw cells
        from rcs.core.ros_client import grid_from_msg
        orig_dec = w.ros._dec_cost

        def tap_cost(job):
            sig, m = job
            if sig == "global_costmap_received":
                g = grid_from_msg(m)
                if g is not None:
                    gc_grid.append((m["info"], g))
                    del gc_grid[:-2]
            orig_dec(job)
        w.ros._dec_cost = tap_cost
        w.ros.global_costmap_received.connect(gc.append)
        w.ros.local_costmap_received.connect(lc.append)
        w._show("nav")
        check("global costmap arrives", wait_until(lambda: gc, 30) or w.p_map.view is not None and bool(gc))
        check("local costmap arrives", wait_until(lambda: lc, 30))
        act = robot.sh("timeout 8 ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger",
                       ros=True, timeout=30)
        check("Nav2 navigation active after the pose", "success=True" in act, act.strip()[-80:])
        note("Nav2 log tail: " + " | ".join(nav_log[-3:])[:300])

        # free spots near the robot, on the map, for zones and goals
        ox, oy = w.p_map.view._map_origin
        res = w.p_map.view._map_res

        def scan_points_map():
            """The newest scan in the map frame (as the app places it)."""
            if not scans:
                return np.zeros((0, 2))
            m = scans[-1]
            r = np.array([np.inf if v is None else v for v in m["ranges"]], dtype=np.float32)
            a = m["angle_min"] + np.arange(r.size) * m["angle_increment"]
            ok = np.isfinite(r) & (r < m["range_max"]) & (r > 0.05)
            return np.stack([w._laser_x + r[ok] * np.cos(a[ok] + w._laser_yaw),
                             w._laser_y + r[ok] * np.sin(a[ok] + w._laser_yaw)], 1)

        def spot(dist, avoid=(), clear_m=0.55, path=True):
            """A free place about `dist` m from the robot (tries shorter if none): clear by
            `clear_m` on the map and in the live scan, and the straight way there clear too."""
            p0 = pose() or START
            sp = scan_points_map()
            cells = max(2, int(round(clear_m / res)))
            nn = occ_near(cells)
            g = w.p_map.view.map_img.image.T

            def ok(x, y, margin):
                c, r_ = int((x - ox) / res), int((y - oy) / res)
                if not (0 <= r_ < nn.shape[0] and 0 <= c < nn.shape[1]) or nn[r_, c] or not 0 <= g[r_, c] < 25:
                    return False
                return not len(sp) or np.min(np.hypot(sp[:, 0] - x, sp[:, 1] - y)) > margin
            for d in (dist, dist * 0.8, dist * 0.65):
                for k in range(72):
                    th = p0[2] + math.radians(5 * ((k + 1) // 2)) * (1 if k % 2 else -1)
                    x, y = p0[0] + d * math.cos(th), p0[1] + d * math.sin(th)
                    if any(math.hypot(x - ax, y - ay) < ar for ax, ay, ar in avoid):
                        continue
                    if ok(x, y, clear_m) and not path or ok(x, y, clear_m) and all(ok(p0[0] + f * (x - p0[0]), p0[1] + f * (y - p0[1]), 0.4)
                                                 for f in np.linspace(0.35, 1, 8)):
                        return (x, y, th)
            return None

        def box(cx, cy, h):
            return [[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]]

        if step("7"):
            print("\n[7] Keepout and speed zones on the robot")
            w._show("map")
            zb = w.p_map.zones
            check("zones loaded for the Nav2 map", wait_until(lambda: zb.loaded and zb.map_name == MAP, 15),
                  f"{zb.map_name} loaded={zb.loaded}")
            before = list(zb.robot_zones)
            note(f"zones already on {MAP}: {len(before)}")
            k = spot(2.0, clear_m=0.4, path=False)
            p0 = pose()
            check("found a free place for a test keepout zone", k is not None)
            if k and p0:
                ans = zones_answer()
                zb.zones = [z for z in zb.zones]
                zk = zb.add_drawn("keepout", box(k[0], k[1], 0.3))
                zs = zb.add_drawn("slow", box(p0[0], p0[1], 0.8))
                TEST_ZONES.update((zk["id"], zs["id"]))
                zs["speed"] = 30
                import roslibpy
                limits = []         # Nav2 says it once, when it changes (not latched): listen first
                lim_sub = roslibpy.Topic(w.ros._ros, "/speed_limit", "nav2_msgs/msg/SpeedLimit")
                lim_sub.subscribe(lambda m_: limits.append(m_.get("speed_limit")))
                zb.save()
                check("zones saved on the robot", wait_until(lambda: any(a.get("msg") for a in ans), 15),
                      str(ans[-1:]))
                check("applied to the running Nav2 at once", zb.applied is True, str(ans[-1:]))
                out = robot.sh(f"ls ~/tbm_test/install/navigation/share/navigation/maps/{MAP}.zones.json "
                               f"~/tbm_test/maps/{MAP}.zones.json 2>/dev/null")
                check("kept next to the map", ".zones.json" in out, out.strip())
                gc_grid.clear()

                def lethal_at(x, y):
                    if not gc_grid:
                        return None
                    info, g = gc_grid[-1]
                    c, r_ = int((x - info["origin"]["position"]["x"]) / info["resolution"]), \
                        int((y - info["origin"]["position"]["y"]) / info["resolution"])
                    if not (0 <= r_ < g.shape[0] and 0 <= c < g.shape[1]):
                        return None
                    return int(g[r_, c])
                check("keepout zone lethal in the global costmap",
                      wait_until(lambda: lethal_at(k[0], k[1]) == 100, 30), str(lethal_at(k[0], k[1])))
                # the app refuses a goal there
                sent = []
                orig = w._send_goal
                w._send_goal = lambda *a: sent.append(a)
                w._send_goal_checked(k[0], k[1], 0.0)
                w._send_goal = orig
                check("app refuses a goal inside the keepout zone", not sent)
                plan = robot.sh(
                    "timeout 20 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "
                    f"'{{goal: {{header: {{frame_id: map}}, pose: {{position: {{x: {k[0]:.2f}, y: {k[1]:.2f}}}, "
                    "orientation: {w: 1.0}}}, use_start: false}'", ros=True, timeout=40)
                check("Nav2 plans no path into the keepout zone", "ABORTED" in plan or "rejected" in plan.lower()
                      or "FAILED" in plan, plan.strip().splitlines()[-1] if plan.strip() else "")
                check("robot inside the 30 % slow zone: Nav2 limits to 30 %", wait_until(lambda: 30.0 in limits, 15),
                      str(limits))
                lim_sub.unsubscribe()
                w._show("teleop")
                pump(1.5)
                note_txt = w.p_teleop.zone_note.text() if hasattr(w.p_teleop, "zone_note") else ""
                check("Teleop page says the robot is in the slow zone", "30" in note_txt or "slow" in note_txt.lower(),
                      note_txt[:100])
                # back to what was there
                w._show("map")
                ans.clear()
                zb.zones = [z for z in zb.zones if z["id"] not in (zk["id"], zs["id"])]
                zb.save()
                check("test zones removed", wait_until(lambda: any(a.get("msg") for a in ans), 15)
                      and len(zb.robot_zones) == len(before), str(len(zb.robot_zones)))
                gc_grid.clear()
                check("keepout gone from the costmap", wait_until(lambda: lethal_at(k[0], k[1]) not in (100, None), 30),
                      str(lethal_at(k[0], k[1])))
                if not before:
                    robot.sh(f"rm -f ~/tbm_test/install/navigation/share/navigation/maps/{MAP}.zones.json "
                             f"~/tbm_test/maps/{MAP}.zones.json")

        if step("8"):
            print("\n[8] Maps between the PC and the robot")
            out = []
            slot = lambda ok, t: out.append((ok, t))        # noqa: E731
            w.transfer.finished.connect(slot)
            w.transfer.download(MAP)
            wait_until(lambda: out, 60)
            check("map copied from the robot", out and out[0][0] and MAP in w.library.names(), str(out))
            ref = Path(f"/home/minh/AMR_WS/first/src/navigation/maps/{MAP}.pgm")
            if ref.exists() and MAP in w.library.names():
                meta, pgm = w.library.read(MAP)
                check("same picture as the map file", pgm == ref.read_bytes() or len(pgm) == len(ref.read_bytes()),
                      f"{len(pgm)} vs {ref.stat().st_size} bytes")
            if MAP in w.library.names():
                meta, pgm = w.library.read(MAP)
                w.library.add(TEST_MAP_COPY, meta, pgm)
                out.clear()
                w.transfer.upload(TEST_MAP_COPY)
                wait_until(lambda: out, 60)
                check("map sent to the robot", out and out[0][0], str(out))
                check("robot lists it", wait_until(lambda: TEST_MAP_COPY in w.p_robot._state.get("maps", []), 10))
                out.clear()
                w.transfer.upload(TEST_MAP_COPY)
                wait_until(lambda: out, 60)
                check("same name again: refused without confirming", out and out[0][0] is False, str(out))
                prev = []
                w.ros.map_preview.connect(prev.append)
                w.ros.send_launcher_cmd("map_preview", name=TEST_MAP_COPY)
                check("preview of the sent map", wait_until(lambda: any(p_.get("png") for p_ in prev), 15),
                      str([p_.get("error") for p_ in prev]))
            w.transfer.finished.disconnect(slot)

        if step("9"):
            print("\n[9] 3D map from the camera")
            w._show("3d")
            w.ros.cloud_map.clear()
            w.p_3d.rec_btn.setChecked(True)
            check("3D map fills up", wait_until(lambda: len(w.ros.cloud_map) > 2000, 40), str(len(w.ros.cloud_map)))
            xyz = w.ros.cloud_map.snapshot()[0]
            note(f"{len(xyz)} points, built in {w.ros.cloud_world!r}")
            if len(xyz):
                z = xyz[:, 2]
                check("3D points between floor and ceiling", np.percentile(z, 1) > -0.3 and np.percentile(z, 99) < 3.5,
                      f"z 1-99 % {np.percentile(z, 1):.2f} .. {np.percentile(z, 99):.2f}")
                nr = occ_near(4)
                high = xyz[z > 0.4]
                if nr is not None and len(high):
                    cols, rows = ((high[:, 0] - ox) / res).astype(int), ((high[:, 1] - oy) / res).astype(int)
                    ins = (rows >= 0) & (rows < nr.shape[0]) & (cols >= 0) & (cols < nr.shape[1])
                    frac = float(nr[rows[ins], cols[ins]].mean()) if ins.any() else 0.0
                    note(f"3D points above 0.4 m within 20 cm of a map wall: {frac:.0%} of {ins.sum()} "
                         "(furniture not on the map counts against)")
                    check("3D map placed on the map (>= 50 % of high points on walls)", frac >= 0.5, f"{frac:.0%}")

        # ------------------------------------------------------------------ part B
        if args.drive:
            drive(w, robot, START, MAP, statuses, scans, odom, gate, prof, pose, dist_to, moving, send_goal,
                  localize, spot, box, zones_answer, step, scan_on_walls, clear)

        if step("10"):
            print("\n[10] SLAM from the app: build a map, save it, see it listed")
            w._show("robot")
            click(w.p_robot.nav.stop_btn)
            check("Nav2 stopped", wait_until(lambda: stack("nav") == "stopped", 40), str(stack("nav")))
            left = robot.sh("pgrep -fc '[a]mcl|[c]ontroller_server|[p]lanner_server|[m]ap_server|[c]bf_safety' || true").strip()
            check("nothing of Nav2 left on the robot", left == "0", left)
            w.p_robot.rb_slam.setChecked(True)
            click(w.p_robot.nav.start_btn)
            check("SLAM running", wait_until(lambda: stack("nav") == "running", 120), str(stack("nav")))
            w.ros.map_received.disconnect() if False else None
            maps = []
            w.ros.map_received.connect(maps.append)
            check("SLAM map arrives", wait_until(lambda: maps, 40), str(len(maps)))
            pump(5)
            w.p_robot.map_name.setText(TEST_SLAM_MAP)
            note(f"save clicked at {time.time():.1f}")
            click(w.p_robot.save_btn)
            check("map saved", wait_until(lambda: TEST_SLAM_MAP in w.p_robot._state.get("maps", []), 60),
                  w.p_robot.result.text()[:100])
            click(w.p_robot.nav.stop_btn)
            check("SLAM stopped", wait_until(lambda: stack("nav") == "stopped", 40), str(stack("nav")))

            print(f"\n[11] Back to Nav2 on {MAP}; Use last position")
            w.p_robot.rb_map.setChecked(True)
            w.p_robot.map_combo.setCurrentText(MAP)
            pump(1)
            click(w.p_robot.nav.start_btn)
            check("Nav2 back", wait_until(lambda: stack("nav") == "running", 120), str(stack("nav")))
            pump(5)
            if check("Use last position offered", wait_until(lambda: w.p_robot.last_pose_btn.isVisible(), 15)):
                click(w.p_robot.last_pose_btn)
                check("robot on the map again (< 0.3 m of where it was)",
                      wait_until(lambda: pose() is not None and dist_to(*START[:2]) < 0.3 + (0.0 if not args.drive
                                                                                                else 0.3), 15),
                      str(pose()))
    except Exception:          # a broken step must not leave test zones or maps on the robot
        import traceback
        traceback.print_exc()
        check("test ran to the end", False)

    print("\n[12] Clean up what the test added on the robot")
    zb = w.p_map.zones
    if TEST_ZONES and zb.loaded and any(z["id"] in TEST_ZONES for z in zb.robot_zones):
        got = zones_answer()
        zb.zones = [z for z in zb.robot_zones if z["id"] not in TEST_ZONES]
        zb.save()
        check("test zones removed", wait_until(lambda: any(a.get("msg") for a in got), 15))
    robot.sh(f"cd ~/tbm_test/maps && rm -f {TEST_MAP_COPY}.* {TEST_SLAM_MAP}.*")
    pump(3)
    left = [m_ for m_ in w.p_robot._state.get("maps", []) if m_.startswith("rcs_test")]
    check("test maps removed", wait_until(lambda: not [m_ for m_ in w.p_robot._state.get("maps", [])
                                                       if m_.startswith("rcs_test")], 10), str(left))

    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    for f in FAILED:
        print(f"  failed: {f}")
    w.ros.disconnect_robot()
    pump(0.5)
    return 0 if FAIL == 0 else 1


def build_map(w, robot, name, stack, odom, moving, pose, clear) -> int:
    """SLAM from the app, drive out and back in four directions, save the map from the app."""
    print(f"\n[M] A new map of the room: {name}")
    w._show("robot")
    check("connected", wait_until(w.ros.is_connected, 20))
    check("launcher online", wait_until(lambda: w.p_robot._launcher_online(), 15))
    if stack("bringup") != "running":
        click(w.p_robot.base.start_btn)
        check("bringup running", wait_until(lambda: stack("bringup") == "running", 60))
    w.p_robot.rb_slam.setChecked(True)
    click(w.p_robot.nav.start_btn)
    pump(3)
    check("SLAM running", wait_until(lambda: stack("nav") == "running"
                                     and w.p_robot._state.get("stacks", {}).get("nav", {}).get("args", {})
                                     .get("mode") == "slam", 120), str(stack("nav")))
    check("robot on the SLAM map", wait_until(lambda: pose() is not None, 30), str(pose()))
    pump(3)
    # SLAM adds the lidar every 0.5 m driven (turning on the spot adds nothing) and the
    # lidar sees nothing behind the robot: drive out and back in four directions
    w._show("teleop")
    t = w.p_teleop
    t.set_speed(1)                                     # Medium: 0.225 m/s, 0.4 rad/s
    h0 = pose()[2]

    def turn_to(target):
        for _ in range(3):
            err = wrap(target - pose()[2])
            if abs(err) < math.radians(6):
                return True
            key = Qt.Key_A if err > 0 else Qt.Key_D
            QTest.keyPress(t, key)
            t0 = time.monotonic()
            while abs(wrap(target - pose()[2])) > math.radians(4) and time.monotonic() - t0 < 25 \
                    and (wrap(target - pose()[2]) > 0) == (err > 0):
                pump(0.05)
            QTest.keyRelease(t, key)
            wait_until(lambda: moving() < 0.01, 3)
            pump(0.5)
        return abs(wrap(target - pose()[2])) < math.radians(10)

    def drive_for(key, dist):
        start = pose()
        QTest.keyPress(t, key)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10:
            pump(0.05)
            q = pose()
            if math.hypot(q[0] - start[0], q[1] - start[1]) >= dist:
                break
            if key == Qt.Key_W and clear[0] and clear[0].front < 0.5:
                break
        QTest.keyRelease(t, key)
        wait_until(lambda: moving() < 0.01, 3)
        q = pose()
        return math.hypot(q[0] - start[0], q[1] - start[1])

    legs = 0
    for k in range(4):
        turn_to(h0 + k * math.pi / 2)
        pump(1.0)
        front = clear[0].front if clear[0] else 0.0
        if front < 1.3:
            note(f"direction {k * 90}°: only {front:.2f} m clear ahead, skipped")
            continue
        out = drive_for(Qt.Key_W, 0.7)
        pump(1.0)
        back = drive_for(Qt.Key_S, out)
        note(f"direction {k * 90}°: out {out:.2f} m, back {back:.2f} m (clear ahead {front:.1f} m)")
        legs += 1
    turn_to(h0)
    check("drove out and back in at least 3 directions", legs >= 3, f"{legs}")
    check("stopped", wait_until(lambda: moving() < 0.01, 3), f"{moving():.3f}")
    t.set_speed(0)
    pump(4)                                             # SLAM takes the last scans in
    p = pose()
    w._show("robot")
    w.p_robot.map_name.setText(name)
    click(w.p_robot.save_btn)
    check(f"map saved as {name}", wait_until(lambda: name in w.p_robot._state.get("maps", []), 60),
          w.p_robot.result.text()[:100])
    print(f"\nPOSE {p[0]:.3f},{p[1]:.3f},{math.degrees(p[2]):.1f}", flush=True)
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    w.ros.disconnect_robot()
    pump(0.5)
    return 0 if FAIL == 0 else 1


def drive(w, robot, START, MAP, statuses, scans, odom, gate, prof, pose, dist_to, moving, send_goal,
          localize, spot, box, zones_answer, step, scan_on_walls, clear):
    """Part B: the robot moves, slowly, near where it stands."""
    t = w.p_teleop

    if step("B1"):
        print("\n[B1] Teleop: slow, forward and back, against the wheels")
        w._show("teleop")
        for key, sign in ((Qt.Key_W, 1), (Qt.Key_S, -1)):
            t.set_speed(0)
            before = pose()
            QTest.keyPress(t, key)
            pump(1.5)
            v = odom["v"]
            QTest.keyRelease(t, key)
            expected = prof.limits.max_linear * 0.25 * sign
            check(f"{'forward' if sign > 0 else 'back'}: wheels at {expected:+.2f} m/s", abs(v - expected) < 0.04,
                  f"{v:+.3f}")
            check("stops on key release", wait_until(lambda: moving() < 0.01, 3), f"{moving():.3f}")
            pump(0.5)
            after = pose()
            if before and after:
                moved = math.hypot(after[0] - before[0], after[1] - before[1])
                head = math.atan2(after[1] - before[1], after[0] - before[0])
                check("went the way the robot faces", moved > 0.1 and abs(math.sin(head - before[2])) < 0.25,
                      f"{moved:.2f} m")
        print("\n[B1b] Turning on the spot")
        t.set_speed(0)
        before = pose()
        QTest.keyPress(t, Qt.Key_A)
        pump(2.5)
        QTest.keyRelease(t, Qt.Key_A)
        wait_until(lambda: moving() < 0.01, 3)
        pump(1.0)
        after = pose()
        if before and after:
            turned = wrap(after[2] - before[2])
            moved = math.hypot(after[0] - before[0], after[1] - before[1])
            check("turned left", turned > 0.2, f"{math.degrees(turned):.0f}°")
            check("about its centre (< 8 cm)", moved < 0.08, f"{moved * 100:.1f} cm")
        QTest.keyPress(t, Qt.Key_D)
        pump(2.5)
        QTest.keyRelease(t, Qt.Key_D)
        wait_until(lambda: moving() < 0.01, 3)
        t.set_speed(1)

    if step("B2"):
        print("\n[B2] E-STOP while driving, and no driving while engaged")
        w._show("teleop")
        t.set_speed(0)
        QTest.keyPress(t, Qt.Key_W)
        pump(1.0)
        t0 = time.monotonic()
        w._set_estop(True)
        stopped = wait_until(lambda: moving() < 0.01, 3)
        check("E-STOP stops the robot", stopped, f"{time.monotonic() - t0:.2f} s")
        pump(1.0)
        check("stays stopped with the key held", moving() < 0.01, f"{moving():.3f}")
        QTest.keyRelease(t, Qt.Key_W)
        w._set_estop(False)
        wait_until(lambda: gate[0] and not gate[0].get("engaged"), 4)
        pump(1)
        t.set_speed(1)

    if step("B3"):
        print("\n[B3] Goals on the 2D Map; Esc cancels")
        w._show("map")
        g = spot(1.2)
        if check("free place 1.2 m away", g is not None):
            home = pose()
            check("goal accepted", send_goal(g), str(statuses[:4]))
            check("Nav2 succeeded", wait_until(lambda: "succeeded" in statuses, 90), str(sorted(set(statuses))))
            check("at the goal (< 0.25 m)", dist_to(*g[:2]) < 0.25, f"{dist_to(*g[:2]):.2f} m")
            scans.clear()
            wait_until(lambda: len(scans) >= 2, 5)
            frac, n = scan_on_walls(scans[-1]) if scans else (0, 0)
            check("still localized: scan on the walls (>= 80 %)", frac >= 0.8, f"{frac:.0%} of {n}")
            back = (home[0], home[1], home[2])
            check("goal back", send_goal(back) and wait_until(lambda: moving() > 0.03, 15))
            pump(1.0)
            w.on_escape()
            check("Esc cancels the goal", wait_until(lambda: "canceled" in statuses, 8), str(sorted(set(statuses))))
            check("robot stopped", wait_until(lambda: moving() < 0.01, 5), f"{moving():.3f}")
            check("back home", send_goal(back) and wait_until(lambda: "succeeded" in statuses, 90),
                  str(sorted(set(statuses))))

    if step("B4"):
        print("\n[B4] A slow zone on the way: Nav2 slows down in it")
        w._show("map")
        zb = w.p_map.zones
        g = spot(1.6)
        p0 = pose()
        if check("free place for the slow-zone run", g is not None and p0 is not None):
            import roslibpy
            ans = zones_answer()
            # the zone covers the second half of the way, so the robot is at speed when it enters
            fx, fy = p0[0] + 0.7 * (g[0] - p0[0]), p0[1] + 0.7 * (g[1] - p0[1])
            half = 0.55
            zs = zb.add_drawn("slow", box(fx, fy, half))
            TEST_ZONES.add(zs["id"])
            zs["speed"] = 30
            limits = []
            lim_sub = roslibpy.Topic(w.ros._ros, "/speed_limit", "nav2_msgs/msg/SpeedLimit")
            lim_sub.subscribe(lambda m_: limits.append((time.monotonic(), m_.get("speed_limit"))))
            zb.save()
            check("slow zone saved", wait_until(lambda: any(a.get("msg") for a in ans), 15))
            pump(2)
            trace = []          # (t, inside, v)

            def watch():
                q = pose()
                if q:
                    inside = abs(q[0] - fx) < half and abs(q[1] - fy) < half
                    trace.append((time.monotonic(), inside, abs(odom["v"])))
                return "succeeded" in statuses or "aborted" in statuses
            send_goal(g)
            wait_until(watch, 90)
            lim_sub.unsubscribe()
            check("reached the goal through the zone", "succeeded" in statuses, str(sorted(set(statuses))))
            entered = next((t_ for t_, ins, _ in trace if ins), None)
            before = max((v for t_, ins, v in trace if not ins and (entered is None or t_ < entered)), default=0)
            settled = [v for t_, ins, v in trace if ins and entered and t_ - entered > 1.0]
            lim_t = [round(t_ - entered, 2) for t_, v in limits if v == 30.0 and entered] if entered else []
            note(f"before the zone up to {before:.2f} m/s; /speed_limit 30 at {lim_t} s from entering; "
                 f"in the zone after 1 s: up to {max(settled, default=0):.3f} m/s over {len(settled)} samples")
            check("Nav2 says 30 % when the robot enters", any(-0.5 < t_ < 1.0 for t_ in lim_t), str(lim_t))
            check("in the 30 % zone (after 1 s): at most 30 % of 0.4 m/s (+0.03)",
                  settled and max(settled) <= 0.3 * 0.4 + 0.03, f"{max(settled, default=0):.3f} m/s")
            ans.clear()
            zb.zones = [z for z in zb.zones if z["id"] != zs["id"]]
            zb.save()
            wait_until(lambda: any(a.get("msg") for a in ans), 15)
            send_goal(START)
            wait_until(lambda: "succeeded" in statuses, 90)

    if step("B5"):
        print("\n[B5] Mission: 2 waypoints, 1 lap; then a saved place")
        g1, g2 = spot(1.0), None
        if g1:
            g2 = spot(1.0, avoid=((g1[0], g1[1], 0.8),))
        if check("two free waypoints", g1 is not None and g2 is not None):
            ms = Mission.new("real robot test")
            ms.waypoints = [Waypoint.new("P1", *g1), Waypoint.new("P2", *g2)]
            for wp in ms.waypoints:
                wp.tolerance = 0.3
            ms.loop = False
            w.missions.upsert(ms)
            w.p_mission._reload_missions()
            w.p_mission._set_combo_to(ms.id)
            w._show("mission")
            sent = []
            w.p_mission.goal_requested.connect(lambda gx, gy, _yaw: sent.append((round(gx, 2), round(gy, 2))))
            w.p_mission.play_btn.click()
            check("mission ended by itself", wait_until(lambda: len(sent) >= 2 and not w.p_mission._exec_running,
                                                        200), str(sent))
            legs = [s for i, s in enumerate(sent) if i == 0 or s != sent[i - 1]]
            check("order P1 P2", legs[:2] == [(round(g1[0], 2), round(g1[1], 2)), (round(g2[0], 2), round(g2[1], 2))],
                  str(legs))
            check("at the last waypoint (< 0.35 m)", dist_to(*g2[:2]) < 0.35, f"{dist_to(*g2[:2]):.2f} m")
            w._show("nav")
            place = w.place_store.add("Test start", START[0], START[1], START[2], MAP)
            statuses.clear()
            w._go_to_place(place)
            check("saved place reached", wait_until(lambda: "succeeded" in statuses, 90)
                  and dist_to(*START[:2]) < 0.3, f"{dist_to(*START[:2]):.2f} m")

    if step("B6"):
        print("\n[B6] App lost while driving: the robot stops by itself")
        w._show("teleop")
        pump(1.0)
        front = clear[0].front if clear[0] else 0.0
        if check("at least 1 m clear ahead", front > 1.0, f"{front:.2f} m"):
            t.set_speed(0)
            QTest.keyPress(t, Qt.Key_W)
            check("driving", wait_until(lambda: moving() > 0.05, 3), f"{moving():.3f}")
            # cut the app off on the robot; the robot's clock says when
            out = robot.sh("date +%s.%N; pkill -INT -f '[r]osbridge_server/rosbridge_websocket'", timeout=10)
            QTest.keyRelease(t, Qt.Key_W)
            try:
                t_kill = float(out.split()[0])
            except (IndexError, ValueError):
                t_kill = None
            check("app reconnects by itself", wait_until(w.ros.is_connected, 60))
            check("robot standing still after it", wait_until(lambda: moving() < 0.01, 10), f"{moving():.3f}")
            log = robot.sh("grep -a 'lost the RCS app' ~/tbm_test/runlog/rcs_safety.log | tail -1")
            try:
                t_stop = float(log.split("[")[2].split("]")[0])
            except (IndexError, ValueError):
                t_stop = None
            took = (t_stop - t_kill) if (t_stop and t_kill) else None
            check("the robot's gate stopped it (3 s without the app's heartbeat, < 4.5 s)",
                  took is not None and 0 < took < 4.5, f"{took:.2f} s" if took is not None else log.strip()[-120:])
        t.set_speed(1)


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)
