"""The robot's E-STOP node (robot/rcs_safety.py) and the launcher's use of it,
on real ROS 2: what reaches the drives, Nav2 cancelled, the latch through a
restart, losing the app, bringup refused without the gate.

Run (needs ROS 2 Humble on this PC, no robot):
    bash -c "source /opt/ros/humble/setup.bash && python3 tests/test_estop_ros.py"

Everything runs on its own ROS domain, on loopback only: it cannot see or
touch a real robot. Bringup is never really started (RCS_BRINGUP_CMD is a sleep).
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).parent
ROBOT = HERE.parent / "robot"
os.environ.update(ROS_DOMAIN_ID="79", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
os.environ.pop("CYCLONEDDS_URI", None)

try:
    import rclpy
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
except ImportError:
    print("needs ROS 2: source /opt/ros/humble/setup.bash first")
    sys.exit(2)

PASS = FAIL = 0
TMP = Path(tempfile.mkdtemp(prefix="rcs_estop_ros_"))


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("estop_probe")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.estop = self.create_publisher(Bool, "/amr/estop", 10)
        self.beat = self.create_publisher(String, "/rcs/app/heartbeat", 10)
        self.launcher = self.create_publisher(String, "/rcs/launcher/cmd", 10)
        self.out: list = []
        self.state: dict = {}
        self.launcher_state: dict = {}
        self.cancels = 0
        self.create_subscription(Twist, "/rcs/cmd_vel_gated", lambda m: self.out.append(
            (time.monotonic(), m.linear.x, m.angular.z)), 50)
        self.create_subscription(String, "/rcs/estop/state", lambda m: setattr(self, "state", json.loads(m.data)),
                                 latched)
        self.create_subscription(String, "/rcs/launcher/state",
                                 lambda m: setattr(self, "launcher_state", json.loads(m.data)), 10)
        self.create_service(CancelGoal, "/navigate_to_pose/_action/cancel_goal", self._cancel)

    def _cancel(self, _req, res):
        self.cancels += 1
        return res

    def spin(self, seconds: float, until=None) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if until is not None and until():
                return True
        return bool(until()) if until else True

    def drive(self, v: float, seconds: float) -> list:
        """Send v for a while; what came out at the other end."""
        start = time.monotonic()
        t = Twist()
        t.linear.x = v
        while time.monotonic() - start < seconds:
            self.cmd.publish(t)
            self.spin(0.05)
        return [o for o in self.out if o[0] >= start]

    def heartbeat(self, driving: bool, bye: bool = False) -> None:
        self.beat.publish(String(data=json.dumps({"id": "test-app", "host": "test-pc", "driving": driving,
                                                  **({"bye": True} if bye else {})})))


def start(script: str, **env) -> subprocess.Popen:
    return subprocess.Popen([sys.executable, "-u", str(ROBOT / script)], env={**os.environ, **env},
                            start_new_session=True, stdout=open(TMP / f"{script}.log", "a"),
                            stderr=subprocess.STDOUT)


def stop(p: subprocess.Popen) -> None:
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(5)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        os.killpg(p.pid, signal.SIGKILL)


def main() -> int:
    rclpy.init()
    probe = Probe()
    estop_file = str(TMP / "estop.json")
    ws = TMP / "ws"
    (ws / "maps").mkdir(parents=True)
    # a stand-in for bringup that drives from the gate, as the default does
    launcher = start("rcs_launcher.py", RCS_WS=str(ws),
                     RCS_BRINGUP_CMD="bash -c 'sleep 30' cmd_vel_topic:=/rcs/cmd_vel_gated")
    gate = None
    try:
        print("\n[no gate: bringup refused]")
        probe.spin(4.0, lambda: probe.launcher_state)
        probe.launcher.publish(String(data=json.dumps({"id": "c1", "op": "start", "stack": "bringup"})))
        ok = probe.spin(6.0, lambda: (probe.launcher_state.get("result") or {}).get("id") == "c1")
        res = probe.launcher_state.get("result") or {}
        check("the launcher refuses to start the drives unguarded", ok and not res.get("ok")
              and "E-STOP service" in res.get("msg", ""), f"{res}")
        check("and says it has the gate", "estop_gate" in probe.launcher_state.get("caps", []))
        sys.path.insert(0, str(ROBOT))
        import rcs_launcher
        check("bringup's drives follow the gate", "cmd_vel_topic:=/rcs/cmd_vel_gated" in rcs_launcher.BRINGUP_CMD,
              f"{rcs_launcher.BRINGUP_CMD}")

        print("\n[the gate]")
        gate = start("rcs_safety.py", RCS_ESTOP_FILE=estop_file)
        check("up", probe.spin(10.0, lambda: probe.state.get("gate")), f"{probe.state}")
        probe.launcher.publish(String(data=json.dumps({"id": "c2", "op": "start", "stack": "bringup"})))
        probe.spin(6.0, lambda: (probe.launcher_state.get("result") or {}).get("id") == "c2")
        res = probe.launcher_state.get("result") or {}
        check("with the gate up, bringup starts", res.get("ok"), f"{res}")
        probe.launcher.publish(String(data=json.dumps({"id": "c3", "op": "stop", "stack": "bringup"})))
        out = probe.drive(0.3, 1.0)
        check("commands pass when clear", out and all(abs(o[1] - 0.3) < 1e-6 for o in out[-5:]), f"{out[-3:]}")

        print("\n[E-STOP]")
        probe.estop.publish(Bool(data=True))
        check("engaged", probe.spin(3.0, lambda: probe.state.get("engaged")))
        check("Nav2 cancelled", probe.spin(3.0, lambda: probe.cancels >= 1))
        out = probe.drive(0.3, 1.0)
        check("nothing gets through", out and all(o[1] == 0.0 for o in out), f"{[o[1] for o in out][:5]}")
        rate = len([o for o in out if o[1] == 0.0]) / 1.0
        check("zero sent on its own, beating any other publisher", rate > 30, f"{rate:.0f}/s")
        probe.out.clear()
        stop(gate)
        probe.spin(0.5, lambda: False)
        check("stopping, the gate's last word is zero", probe.out and probe.out[-1][1] == 0.0
              and probe.out[-1][2] == 0.0, f"{probe.out[-1:]}")
        probe.out.clear()
        probe.drive(0.3, 0.5)
        check("gate down: nothing reaches the drives at all", not probe.out, f"{probe.out[:3]}")
        gate = start("rcs_safety.py", RCS_ESTOP_FILE=estop_file)
        probe.state = {}
        check("restarted: still engaged", probe.spin(10.0, lambda: probe.state.get("gate"))
              and probe.state.get("engaged"), f"{probe.state}")
        out = probe.drive(0.3, 0.8)
        check("and still nothing gets through", out and all(o[1] == 0.0 for o in out))
        probe.estop.publish(Bool(data=False))
        check("released", probe.spin(3.0, lambda: not probe.state.get("engaged")))
        out = probe.drive(0.3, 0.8)
        check("moving again", out and abs(out[-1][1] - 0.3) < 1e-6)

        print("\n[losing the app]")
        n = probe.cancels
        for _ in range(4):
            probe.heartbeat(True)
            probe.drive(0.3, 0.25)
        check("a driving app heard: commands pass", abs(probe.out[-1][1] - 0.3) < 1e-6)
        probe.out.clear()
        t0 = time.monotonic()
        out = probe.drive(0.3, 5.0)                     # no more heartbeats
        stopped = [o for o in out if o[1] == 0.0]
        check("silent for 3 s: the robot stops", stopped and 2.5 < stopped[0][0] - t0 < 4.5,
              f"{(stopped[0][0] - t0) if stopped else None}")
        check("Nav2 cancelled", probe.cancels > n)
        check("and says why", "test-pc" in (probe.state.get("last_stop") or {}).get("reason", ""), f"{probe.state}")
        check("then commands pass again", abs(out[-1][1] - 0.3) < 1e-6)
        n = probe.cancels
        probe.heartbeat(True)
        probe.heartbeat(True, bye=True)
        probe.drive(0.3, 4.5)
        check("an app that says goodbye stops nothing", probe.cancels == n and abs(probe.out[-1][1] - 0.3) < 1e-6)
    finally:
        for p in (launcher, gate):
            if p is not None:
                stop(p)
        probe.destroy_node()
        rclpy.try_shutdown()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
