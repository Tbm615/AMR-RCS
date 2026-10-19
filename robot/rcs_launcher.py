#!/usr/bin/env python3
"""RCS launcher: lets the RCS app start and stop the robot's ROS programs.

Runs on the robot next to rosbridge (see start_rcs_robot.sh). The app talks
to it through two std_msgs/String topics carrying JSON:

  /rcs/launcher/cmd    app -> robot
      {"id": "...", "op": "start", "stack": "bringup"}
      {"id": "...", "op": "start", "stack": "nav", "mode": "map", "map": "map_3269"}
      {"id": "...", "op": "start", "stack": "nav", "mode": "slam"}
      {"id": "...", "op": "stop",  "stack": "bringup" | "nav"}
      {"id": "...", "op": "save_map", "name": "office"}

  /rcs/launcher/state  robot -> app, 1 Hz and on every change
      {"version": 1, "stacks": {"bringup": {...}, "nav": {...}},
       "maps": [...], "result": {"id", "ok", "msg"}}

Stacks:
  bringup  drives, lidar, scan filter, camera (bringup.launch.py)
  nav      mode "map":  robot_navigation.launch.py on a saved map (AMCL)
           mode "slam": slam_toolbox + Nav2 + CBF filter, builds the map live

Environment: RCS_WS is the colcon workspace (default ~/tbm_test). Saved maps
go to $RCS_WS/maps; logs to $RCS_WS/runlog/rcs_<stack>.log.
"""
from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

WS = Path(os.environ.get("RCS_WS", str(Path.home() / "tbm_test"))).expanduser()
NAV_SHARE = WS / "install" / "navigation" / "share" / "navigation"
MAP_DIRS = [NAV_SHARE / "maps", WS / "maps"]
SAVED_MAPS = WS / "maps"
LOG_DIR = WS / "runlog"
PID_FILE = LOG_DIR / "rcs_launcher.pids"
LOG_MAX_BYTES = 20 * 1024 * 1024
# The HINS lidar driver can print this hundreds of times a second.
DROP_LINES = ("null data",)


def _maps() -> dict:
    """name -> yaml path. Saved maps override packaged ones of the same name."""
    out = {}
    for d in MAP_DIRS:
        if d.is_dir():
            for y in sorted(d.glob("*.yaml")):
                out[y.stem] = str(y)
    return out


def _nav_commands(mode: str, map_yaml: str = "") -> list:
    cfg = NAV_SHARE / "config"
    if mode == "map":
        return [["ros2", "launch", "navigation", "robot_navigation.launch.py",
                 f"map:={map_yaml}", "use_cbf:=true", "rviz:=false"]]
    # SLAM: same Nav2 + CBF chain as robot_navigation.launch.py, slam_toolbox
    # in place of AMCL and the map server.
    return [
        ["ros2", "launch", "slam", "slamtoolbox.launch.xml"],
        ["ros2", "launch", "nav2_bringup", "navigation_launch.py",
         f"params_file:={cfg / 'nav2_smac_mppi.yml'}"],
        ["ros2", "run", "navigation", "cbf_safety_filter.py", "--ros-args",
         "--params-file", str(cfg / "cbf_params.yaml"),
         "-r", "cmd_vel_raw:=/cmd_vel", "-r", "cmd_vel:=/cmd_vel_safe",
         "-r", "scan:=/scan_filtered", "-r", "odom:=/odom"],
    ]


class Stack:
    """A group of processes started and stopped together."""

    def __init__(self, name: str):
        self.name = name
        self.procs: list = []
        self.state = "stopped"     # stopped | starting | running | stopping | failed
        self.detail = ""
        self.since = time.time()
        self.args: dict = {}
        self.tail: deque = deque(maxlen=40)
        self._lock = threading.Lock()
        self._log_path = LOG_DIR / f"rcs_{name}.log"

    # -- lifecycle --

    def start(self, cmds: list, args: dict) -> None:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self.tail.clear()
            self.args = args
            self.procs = []
            log = open(self._log_path, "a", buffering=1)
            log.write(f"\n=== {time.ctime()} start {args}\n")
            for cmd in cmds:
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, bufsize=1,
                                     start_new_session=True, errors="replace")
                self.procs.append(p)
                threading.Thread(target=self._pump, args=(p, log), daemon=True).start()
            self._set("starting")
        _save_pids()

    def _pump(self, p: subprocess.Popen, log) -> None:
        written = 0
        for line in p.stdout:
            if any(s in line for s in DROP_LINES):
                continue
            self.tail.append(line.rstrip()[:300])
            try:
                if written > LOG_MAX_BYTES:
                    log.seek(0); log.truncate(); written = 0
                log.write(line)
                written += len(line)
            except Exception:
                pass

    def stop(self) -> None:
        with self._lock:
            procs = list(self.procs)
            if not procs:
                self._set("stopped")
                return
            self._set("stopping")
        _signal_groups(procs)
        with self._lock:
            self.procs = []
            self._set("stopped")
        _save_pids()

    def poll(self) -> None:
        """Mark the stack failed when one of its processes exits by itself."""
        with self._lock:
            if self.state not in ("starting", "running"):
                return
            dead = [p for p in self.procs if p.poll() is not None]
            if not dead:
                return
            codes = ", ".join(str(p.returncode) for p in dead)
            alive = [p for p in self.procs if p.poll() is None]
        # half a stack is worse than none; off the executor thread
        threading.Thread(target=_signal_groups, args=(alive,), daemon=True).start()
        with self._lock:
            self.procs = []
            self._set("failed", f"a process exited (code {codes}); see the log below")
        _save_pids()

    def _set(self, state: str, detail: str = "") -> None:
        self.state, self.detail, self.since = state, detail, time.time()

    def running(self) -> bool:
        return self.state in ("starting", "running")

    def pgids(self) -> list:
        return [p.pid for p in self.procs if p.poll() is None]

    def to_dict(self) -> dict:
        return {"state": self.state, "detail": self.detail, "since": self.since,
                "args": self.args, "log": list(self.tail)[-15:]}


def _signal_groups(procs: list) -> None:
    """SIGINT (like Ctrl-C, lets ros2 launch shut down cleanly), then escalate."""
    for sig, wait in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        alive = [p for p in procs if p.poll() is None]
        if not alive:
            return
        for p in alive:
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                pass
        end = time.time() + wait
        while time.time() < end and any(p.poll() is None for p in alive):
            time.sleep(0.2)


STACKS = {"bringup": Stack("bringup"), "nav": Stack("nav")}


def _save_pids() -> None:
    try:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        PID_FILE.write_text(" ".join(str(g) for s in STACKS.values() for g in s.pgids()))
    except Exception:
        pass


def _kill_leftovers() -> None:
    """Process groups left by a previous launcher (crash or restart) are
    stopped, so the app never sees "stopped" while the robot is still driven."""
    try:
        pgids = [int(x) for x in PID_FILE.read_text().split()]
    except Exception:
        return
    for g in pgids:
        try:
            os.killpg(g, signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass
    time.sleep(5)
    for g in pgids:
        try:
            os.killpg(g, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    PID_FILE.unlink(missing_ok=True)


class Launcher(Node):
    def __init__(self):
        super().__init__("rcs_launcher")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, "/rcs/launcher/state", latched)
        self.create_subscription(String, "/rcs/launcher/cmd", self._on_cmd, 10)
        self.result = {"id": "", "ok": True, "msg": ""}
        self.busy = ""             # description of the command being executed
        self._lock = threading.Lock()
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"RCS launcher ready, workspace {WS}")

    # -- state --

    def _tick(self) -> None:
        for s in STACKS.values():
            s.poll()
        self._update_ready()
        self.publish_state()

    def _update_ready(self) -> None:
        """starting -> running once the stack's key topics have publishers."""
        b, n = STACKS["bringup"], STACKS["nav"]
        if b.state == "starting" and self.count_publishers("/odom") and \
                self.count_publishers("/scan_filtered"):
            b._set("running")
        if n.state == "starting" and \
                self.count_publishers("/navigate_to_pose/_action/status") and \
                self.count_publishers("/map"):
            n._set("running")

    def publish_state(self) -> None:
        stacks = {k: s.to_dict() for k, s in STACKS.items()}
        # Started by hand (ssh, a terminal) rather than from the app.
        if STACKS["bringup"].state in ("stopped", "failed") and self.count_publishers("/odom"):
            stacks["bringup"]["external"] = True
        if STACKS["nav"].state in ("stopped", "failed") and \
                self.count_publishers("/navigate_to_pose/_action/status"):
            stacks["nav"]["external"] = True
        msg = {"version": 1, "time": time.time(), "stacks": stacks,
               "maps": sorted(_maps()), "busy": self.busy, "result": self.result}
        self.pub.publish(String(data=json.dumps(msg)))

    # -- commands --

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(cmd, dict):
            return
        with self._lock:
            if self.busy:
                self._finish(cmd, False, f"busy: {self.busy}")
                return
            self.busy = f"{cmd.get('op')} {cmd.get('stack', cmd.get('name', ''))}".strip()
        self.publish_state()
        threading.Thread(target=self._run, args=(cmd,), daemon=True).start()

    def _run(self, cmd: dict) -> None:
        try:
            ok, text = self._execute(cmd)
        except Exception as e:      # never let a bad command kill the launcher
            ok, text = False, f"error: {e}"
        with self._lock:
            self.busy = ""
        self._finish(cmd, ok, text)

    def _finish(self, cmd: dict, ok: bool, text: str) -> None:
        self.result = {"id": str(cmd.get("id", "")), "ok": ok, "msg": text}
        self.get_logger().info(f"{cmd.get('op')} {cmd.get('stack', '')}: {text}")
        self.publish_state()

    def _execute(self, cmd: dict):
        op, name = cmd.get("op"), cmd.get("stack")
        if op == "save_map":
            return self._save_map(str(cmd.get("name", "")))
        if name not in STACKS:
            return False, f"unknown stack {name!r}"
        st = STACKS[name]
        if op == "stop":
            if name == "bringup":
                STACKS["nav"].stop()      # nav without the base is useless
            if not st.running() and st.state != "failed":
                return True, f"{name} was not running"
            st.stop()
            return True, f"{name} stopped"
        if op != "start":
            return False, f"unknown op {op!r}"
        if name == "bringup":
            if st.running():
                return True, "bringup is already running"
            if self.count_publishers("/odom"):
                return False, "bringup is already running outside the app (stop it there first)"
            st.start([["ros2", "launch", "bringup", "bringup.launch.py"]], {})
            return True, "bringup starting"
        # nav
        if not STACKS["bringup"].running() and not self.count_publishers("/odom"):
            return False, "start bringup first"
        mode = cmd.get("mode", "map")
        args = {"mode": mode}
        if mode == "map":
            maps = _maps()
            m = cmd.get("map", "")
            if m not in maps:
                return False, f"map {m!r} not found on the robot"
            args["map"] = m
            cmds = _nav_commands("map", maps[m])
        elif mode == "slam":
            cmds = _nav_commands("slam")
        else:
            return False, f"unknown mode {mode!r}"
        if st.running():
            st.stop()                    # switching map or mode = restart
        elif self.count_publishers("/navigate_to_pose/_action/status"):
            return False, "Nav2 is already running outside the app (stop it there first)"
        st.start(cmds, args)
        return True, "navigation starting" + (
            " - set the initial pose on the 2D Map page" if mode == "map" else "")

    def _save_map(self, name: str):
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", name):
            return False, "map name: letters, digits, - and _ only"
        if not self.count_publishers("/map"):
            return False, "no /map to save (start navigation first)"
        SAVED_MAPS.mkdir(parents=True, exist_ok=True)
        out = SAVED_MAPS / name
        r = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(out),
             "--ros-args", "-p", "map_subscribe_transient_local:=true"],
            capture_output=True, text=True, timeout=40)
        if r.returncode == 0 and out.with_suffix(".yaml").exists():
            return True, f"map saved as {name}"
        lines = (r.stdout + r.stderr).strip().splitlines()
        return False, "map_saver failed: " + (lines[-1] if lines else f"code {r.returncode}")

    def shutdown(self) -> None:
        for s in STACKS.values():
            s.stop()


def main() -> None:
    _kill_leftovers()
    rclpy.init()
    node = Launcher()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()          # never leave the base running unmanaged
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
