"""Keepout and speed zones on real ROS 2: robot/rcs_launcher.py keeps them next
to the map, starts Nav2 with the costmap filters, and changes them live.

Run (needs ROS 2 Humble and the built workspace, no robot):
    bash -c "source /opt/ros/humble/setup.bash && source ../install/setup.bash && python3 tests/test_zones_ros.py"

Everything runs on its own ROS domain, on loopback only. The robot is
sim/fake_base.py on a copy of map23_04 behind the robot's E-STOP gate
(rcs_safety.py); bringup is a stand-in (fake_base gives /odom and the scan).
"""
from __future__ import annotations

import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).parent
ROBOT = HERE.parent / "robot"
AMR_WS = Path(os.environ.get("AMR_WS", HERE.parent.parent))
os.environ.update(ROS_DOMAIN_ID="81", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
os.environ.pop("CYCLONEDDS_URI", None)

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav_msgs.msg import Odometry
    import tf2_ros
    from nav2_msgs.msg import SpeedLimit
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
except ImportError:
    print("needs ROS 2 and Nav2: source /opt/ros/humble/setup.bash first")
    sys.exit(2)

PASS = FAIL = 0
TMP = Path(tempfile.mkdtemp(prefix="rcs_zones_ros_"))
X, Y, YAW = 3.55, -5.10, -1.64
MAP = "map23_04"


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def box(cx, cy, r):
    return [[cx - r, cy - r], [cx + r, cy - r], [cx + r, cy + r], [cx - r, cy + r]]


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("zones_probe")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd = self.create_publisher(String, "/rcs/launcher/cmd", 10)
        self.pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.state: dict = {}
        self.answers: list = []
        self.speed: list = []
        self.grid = None
        self.create_subscription(String, "/rcs/launcher/state",
                                 lambda m: self.state.update(json.loads(m.data)), latched)
        self.create_subscription(String, "/rcs/launcher/zones", lambda m: self.answers.append(json.loads(m.data)), 10)
        self.create_subscription(SpeedLimit, "/speed_limit", lambda m: self.speed.append(m.speed_limit), 10)
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 lambda m: setattr(self, "amcl", (round(m.pose.pose.position.x, 2),
                                                                  round(m.pose.pose.position.y, 2))), latched)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: setattr(self, "grid", m), latched)
        self.plan = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.vx = 0.0
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "vx", m.twist.twist.linear.x), 20)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self._n = 0

    def spin(self, sec: float, until=None) -> bool:
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until and until():
                return True
        return False

    def send(self, op: str, **kw) -> dict:
        self._n += 1
        cid = f"z{self._n}"
        self.cmd.publish(String(data=json.dumps({"id": cid, "op": op, **kw})))
        if op.startswith("zones"):
            self.spin(30, lambda: any(a.get("id") == cid for a in self.answers))
            return next((a for a in self.answers if a.get("id") == cid), {})
        self.spin(15, lambda: (self.state.get("result") or {}).get("id") == cid)
        return self.state.get("result") or {}

    def nav_state(self) -> str:
        return self.state.get("stacks", {}).get("nav", {}).get("state", "")

    def set_pose(self) -> None:
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.pose.pose.position.x, m.pose.pose.position.y = X, Y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(YAW / 2), math.cos(YAW / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.0685
        self.pose.publish(m)

    def cost(self, x: float, y: float):
        g = self.grid
        if g is None:
            return None
        i = g.info
        c, r = int((x - i.origin.position.x) / i.resolution), int((y - i.origin.position.y) / i.resolution)
        return g.data[r * i.width + c] if 0 <= c < i.width and 0 <= r < i.height else None

    def path_to(self, x: float, y: float):
        """(status, path length in points) of Nav2's planner to (x, y): it never moves the robot."""
        if not self.plan.wait_for_server(timeout_sec=10):
            return None, 0
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x, g.goal.pose.position.y = x, y
        g.goal.pose.orientation.w = 1.0
        fut = self.plan.send_goal_async(g)
        self.spin(10, fut.done)
        if not fut.done() or not fut.result().accepted:
            return "rejected", 0
        res = fut.result().get_result_async()
        self.spin(20, res.done)
        if not res.done():
            return "no answer", 0
        return res.result().status, len(res.result().result.path.poses)


    def drive_to(self, x: float, y: float, timeout: float = 120.0):
        """Drive there with Nav2; (status, [(map y, speed), ...] on the way)."""
        if not self.nav.wait_for_server(timeout_sec=10):
            return None, []
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = 0.7071, 0.7071
        fut = self.nav.send_goal_async(g)
        self.spin(10, fut.done)
        if not fut.done() or not fut.result().accepted:
            return "rejected", []
        res = fut.result().get_result_async()
        track, end = [], time.time() + timeout
        while time.time() < end and not res.done():
            self.spin(0.1)
            try:
                tr = self.tf_buf.lookup_transform("map", "base_link", rclpy.time.Time())
                track.append((tr.transform.translation.y, abs(self.vx)))
            except tf2_ros.TransformException:
                pass
        return (res.result().status if res.done() else "running"), track


def start(args: list, name: str, **env) -> subprocess.Popen:
    return subprocess.Popen(args, env={**os.environ, **env}, start_new_session=True,
                            stdout=open(TMP / f"{name}.log", "a"), stderr=subprocess.STDOUT)


def stop(p: subprocess.Popen) -> None:
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(15)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def main() -> int:
    share = AMR_WS / "install" / "navigation" / "share" / "navigation"
    if not (share / "config" / "nav2_smac_mppi.yml").exists():
        print(f"needs the built workspace ({share})")
        return 2
    ws = TMP / "ws"
    cfg = ws / "install" / "navigation" / "share" / "navigation" / "config"
    shutil.copytree(share / "config", cfg)              # Nav2 and CBF parameters, as on the robot
    (ws / "maps").mkdir()
    for ext in ("yaml", "pgm"):
        shutil.copy(share / "maps" / f"{MAP}.{ext}", ws / "maps")
    rclpy.init()
    probe = Probe()
    base = start([sys.executable, "-u", str(HERE.parent / "sim" / "fake_base.py"), "--map", str(ws / "maps" / f"{MAP}.yaml"),
                  "--x", str(X), "--y", str(Y), "--yaw", str(YAW), "--cmd-topic", "/rcs/cmd_vel_gated"], "fake_base")
    gate = start([sys.executable, "-u", str(ROBOT / "rcs_safety.py")], "rcs_safety",
                 RCS_ESTOP_FILE=str(TMP / "estop.json"))          # Nav2's commands reach the base through it
    launcher = start([sys.executable, "-u", str(ROBOT / "rcs_launcher.py")], "launcher", RCS_WS=str(ws),
                     RCS_ZONES_DIR=str(TMP / "zones"), RCS_BRINGUP_CMD="bash -c 'sleep 600' cmd_vel_topic:=/rcs/cmd_vel_gated")
    try:
        print("\n[zones kept on the robot]")
        check("launcher up, says it has zones", probe.spin(15, lambda: "zones" in probe.state.get("caps", [])),
              str(probe.state.get("caps")))
        zones = [{"id": "k1", "name": "Pallet", "kind": "keepout", "points": box(3.3, -7.0, 0.3)},
                 {"id": "s1", "name": "Dock", "kind": "slow", "speed": 30, "points": box(X, Y, 0.8)}]
        a = probe.send("zones_put", name=MAP, zones=zones)
        check("saved", a.get("ok") and len(a.get("zones", [])) == 2 and not a.get("applied"), str(a)[:200])
        check("next to the map", (ws / "maps" / f"{MAP}.zones.json").exists())
        a = probe.send("zones_get", name=MAP)
        check("read back", [z["name"] for z in a.get("zones", [])] == ["Pallet", "Dock"])
        a = probe.send("zones_put", name="nowhere", zones=zones)
        check("an unknown map is refused", a.get("ok") is False and "not found" in a.get("msg", ""), str(a))
        a = probe.send("zones_put", name=MAP, zones=[{"kind": "keepout", "points": [[0, 0]]}])
        check("bad zones refused, the kept ones stay", a.get("ok") is False
              and len(probe.send("zones_get", name=MAP).get("zones", [])) == 2, a.get("msg", ""))

        print("\n[Nav2 started by the launcher, with the zones]")
        r = probe.send("start", stack="bringup")
        probe.spin(3)
        r = probe.send("start", stack="nav", mode="map", map=MAP)
        check("navigation starting", r.get("ok"), str(r))
        check("running", probe.spin(120, lambda: probe.nav_state() == "running"), probe.nav_state())
        end = time.time() + 90
        while time.time() < end and probe.grid is None:
            probe.set_pose()
            probe.spin(2, lambda: probe.grid is not None)
        check("localized, global costmap", probe.grid is not None, f"AMCL {probe.amcl}")
        probe.spin(6)
        check("keepout zone lethal for Nav2", probe.cost(3.3, -7.0) == 100, str(probe.cost(3.3, -7.0)))
        check("next to it not", probe.cost(3.3, -6.0) not in (100, None), str(probe.cost(3.3, -6.0)))
        check("slow zone: speed limit 30 % where the robot is", probe.spin(20, lambda: 30.0 in probe.speed),
              str(probe.speed[-3:]))
        probe.spin(5)
        ok = probe.amcl is not None and abs(probe.amcl[0] - X) < 0.3 and abs(probe.amcl[1] - Y) < 0.3
        check("the pose set stays (no (0, 0) from a launch file)", ok, f"AMCL {probe.amcl}")
        sys.path.insert(0, str(ROBOT))
        os.environ["RCS_WS"], os.environ["RCS_ZONES_DIR"] = str(ws), str(TMP / "zones_cmd")
        import rcs_launcher
        cmds = rcs_launcher._nav_commands("map", str(ws / "maps" / f"{MAP}.yaml"))
        flat = " ".join(" ".join(c) for c in cmds)
        check("Nav2 started without robot_navigation.launch.py's initial pose",
              "robot_navigation" not in flat and "nav_map.launch.py" in flat and "keepout_mask:=" in flat,
              flat[:160])
        st = probe.path_to(3.3, -7.0)
        check("Nav2 plans no path into the keepout zone", st[0] in (GoalStatus.STATUS_ABORTED, "rejected"), str(st))
        st = probe.path_to(3.3, -6.2)
        check("but next to it", st[0] == GoalStatus.STATUS_SUCCEEDED and st[1] > 5, f"{st} AMCL {probe.amcl}")

        print("\n[changed while Nav2 runs]")
        probe.speed.clear()
        a = probe.send("zones_put", name=MAP, zones=[{"id": "k1", "name": "Pallet", "kind": "keepout",
                                                       "points": box(3.3, -3.6, 0.3)}])
        check("saved and in use at once", a.get("ok") and a.get("applied") is True, a.get("msg", ""))
        check("new keepout lethal", probe.spin(15, lambda: probe.cost(3.3, -3.6) == 100), str(probe.cost(3.3, -3.6)))
        check("the old one gone", probe.cost(3.3, -7.0) != 100, str(probe.cost(3.3, -7.0)))
        check("slow zone gone: no limit", probe.spin(15, lambda: 0.0 in probe.speed), str(probe.speed[-3:]))

        print("\n[a slow zone added while Nav2 runs, and driven through]")
        a = probe.send("zones_put", name=MAP, zones=[{"id": "s1", "name": "Hall", "kind": "slow", "speed": 30,
                                                       "points": [[2.0, -4.6], [5.0, -4.6], [5.0, -3.4], [2.0, -3.4]]}])
        check("in use at once", a.get("applied") is True, a.get("msg", ""))
        st, track = probe.drive_to(3.3, -2.0)
        inside = [v for y, v in track if -4.45 < y < -3.55]        # 15 cm in from its edges
        after = [v for y, v in track if y > -3.2]
        check("goal through the zone reached", st == GoalStatus.STATUS_SUCCEEDED, str(st))
        check("at most 30 % of 0.4 m/s inside", inside and max(inside) <= 0.4 * 0.30 + 0.02,
              f"max {max(inside or [0]):.3f} m/s over {len(inside)} samples")
        check("faster once out of it", after and max(after) > 0.2, f"max {max(after or [0]):.3f} m/s")

        print("\n[stopped]")
        r = probe.send("stop", stack="nav")
        check("navigation stopped", probe.spin(40, lambda: probe.nav_state() == "stopped"), probe.nav_state())
        probe.spin(3)
        left = subprocess.run(["pgrep", "-f", "costmap_filters.launch|keepout_mask_server|speed_filter_info_server"],
                              capture_output=True, text=True).stdout.split()
        check("the zone filters stopped with it", not left, str(left))
    finally:
        stop(launcher)
        stop(gate)
        stop(base)
        probe.destroy_node()
        rclpy.try_shutdown()
    print(f"\n--- {PASS} passed, {FAIL} failed ---  (logs in {TMP})", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
