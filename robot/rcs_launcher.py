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
      {"id": "...", "op": "map_preview", "name": "office"}
      {"id": "...", "op": "map_get" | "map_put", ...}     see map_exchange.py
      {"id": "...", "op": "zones_get", "name": "office"}
      {"id": "...", "op": "zones_put", "name": "office", "zones": [...]}   see zone_masks.py

  /rcs/launcher/state  robot -> app, 1 Hz and on every change
      {"version": 1, "stacks": {"bringup": {...}, "nav": {...}},
       "maps": [...], "map_mtimes": {name: mtime}, "caps": ["map_transfer", "estop_gate"],
       "camera": {"color": bool, "depth": bool, "publishers": n},
       "result": {"id", "ok", "msg"}}

  /rcs/launcher/preview  robot -> app, answer to map_preview
      {"name": "office", "png": "<base64>", "width_m", "height_m", "resolution",
       "mtime", "saved": true}   or   {"name": "office", "error": "..."}

  /rcs/launcher/map_data  robot -> app, answers to map_get and map_put

  /rcs/launcher/zones  robot -> app, answers to zones_get and zones_put
      {"id", "name", "ok", "msg", "zones": [...], "applied": bool (Nav2 uses them now)}

Stacks:
  bringup  drives, lidar, scan filter, camera (bringup.launch.py)
  nav      mode "map":  nav_map.launch.py: Nav2 + AMCL + CBF filter on a saved map
                        (as robot_navigation.launch.py, without its initial pose), with
                        the map's keepout and speed zones (costmap_filters.launch.py)
           mode "slam": slam_toolbox + Nav2 + CBF filter, builds the map live

Environment: RCS_WS is the colcon workspace (default ~/tbm_test). Saved maps
go to $RCS_WS/maps; logs to $RCS_WS/runlog/rcs_<stack>.log.
"""
from __future__ import annotations

import json
import os
import re
import shlex
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
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped

from camera_check import camera_usb
from map_exchange import MapExchange
from map_preview import map_preview
import zone_masks

WS = Path(os.environ.get("RCS_WS", str(Path.home() / "tbm_test"))).expanduser()
NAV_SHARE = WS / "install" / "navigation" / "share" / "navigation"
MAP_DIRS = [NAV_SHARE / "maps", WS / "maps"]
SAVED_MAPS = WS / "maps"
LOG_DIR = WS / "runlog"
PID_FILE = LOG_DIR / "rcs_launcher.pids"
LOG_MAX_BYTES = 20 * 1024 * 1024
# The simulator (sim/gazebo/run_gazebo.sh) sets these; unset on the robot.
# The drives follow the E-STOP gate (rcs_safety.py), not /cmd_vel_safe: every
# command passes it. RCS_ESTOP_GATE=0 starts bringup the old way, unguarded; so
# does an RCS_BRINGUP_CMD without cmd_vel_topic:=/rcs/cmd_vel_gated (Gazebo sim).
GATE = os.environ.get("RCS_ESTOP_GATE", "1") != "0"
ROBOT_SIDE = "0.6.0b2"      # the release of these robot/ files: the app says when it is older than itself
GATED_TOPIC = "/rcs/cmd_vel_gated"
BRINGUP_CMD = shlex.split(os.environ.get(
    "RCS_BRINGUP_CMD",
    "ros2 launch bringup bringup.launch.py" + (f" cmd_vel_topic:={GATED_TOPIC}" if GATE else "")))
SIM_TIME = os.environ.get("RCS_USE_SIM_TIME") == "1"
# Nav2 now and then never finishes its lifecycle bring-up (a lost service
# answer at startup, seen with FastDDS): after this long it is restarted once.
NAV_UP_TIMEOUT_S = float(os.environ.get("RCS_NAV_UP_TIMEOUT", "60"))
# Nav2 is up when its lifecycle managers say so (their topics exist already when
# a node is only configured, e.g. when its activation hung)
MANAGERS = ("lifecycle_manager_localization", "lifecycle_manager_navigation", "lifecycle_manager_zones")
NAV_RESTARTS = 2            # automatic restarts before the stack shows failed
# On a saved map Nav2's navigation half finishes activating only once AMCL has
# a pose (its costmaps wait for map -> base_link): it may wait for the
# operator as long as it likes, but not this long after a pose came.
NAV_ACTIVE_AFTER_POSE_S = float(os.environ.get("RCS_NAV_ACTIVE_AFTER_POSE", "45"))
# what Nav2's lifecycle manager says when it gives up: restart at once
ABORTED = ("Failed to bring up all requested nodes",)
HERE = Path(__file__).resolve().parent
# masks and Nav2 parameters with the zone filters, rewritten at every Nav2 start
ZONES_DIR = Path(os.environ.get("RCS_ZONES_DIR", str(Path.home() / ".ros" / "rcs_zones"))).expanduser()
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


def _mtime(path: str) -> float:
    try:
        return os.path.getmtime(path)
    except OSError:
        return 0.0


def _zone_setup(map_yaml: str):
    """(Nav2 params file with the zone filters, masks) for a map, or (None, why)."""
    try:
        masks = zone_masks.write_masks(map_yaml, zone_masks.read_zones(map_yaml), ZONES_DIR)
        params = zone_masks.params_with_filters(NAV_SHARE / "config" / "nav2_smac_mppi.yml",
                                                ZONES_DIR / "nav2_params.yaml")
        return params, masks
    except Exception as e:      # no zones is better than no navigation
        return None, f"zones off: {e}"


def _nav_commands(mode: str, map_yaml: str = "") -> list:
    """The processes of the nav stack. On a saved map: nav_map.launch.py, one
    launch as robot_navigation.launch.py (map server + AMCL, Nav2, CBF filter)
    with the map's zones, without its initial pose: that (0, 0, 0) comes a few
    seconds after the start and can overwrite the pose the operator has just
    set. The operator sets it (or "Use last position"); the app says so."""
    cfg = NAV_SHARE / "config"
    sim = ["use_sim_time:=true"] if SIM_TIME else []
    cbf = ["ros2", "run", "navigation", "cbf_safety_filter.py", "--ros-args",
           "--params-file", str(cfg / "cbf_params.yaml"),
           "-r", "cmd_vel_raw:=/cmd_vel", "-r", "cmd_vel:=/cmd_vel_safe",
           "-r", "scan:=/scan_filtered", "-r", "odom:=/odom",
           *(["-p", "use_sim_time:=true"] if SIM_TIME else [])]
    if mode == "map":
        params, masks = _zone_setup(map_yaml)
        cmd = ["ros2", "launch", str(HERE / "nav_map.launch.py"), f"map:={map_yaml}",
               f"params_file:={params or cfg / 'nav2_smac_mppi.yml'}",
               f"cbf_params_file:={cfg / 'cbf_params.yaml'}", *sim]
        if params is not None:
            cmd += [f"keepout_mask:={masks['keepout']}", f"speed_mask:={masks['speed']}"]
        return [cmd]
    # SLAM: slam_toolbox in place of AMCL and the map server
    return [
        ["ros2", "launch", "slam", "slamtoolbox.launch.xml", *sim],
        ["ros2", "launch", "nav2_bringup", "navigation_launch.py",
         f"params_file:={cfg / 'nav2_smac_mppi.yml'}", *sim],
        cbf,
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
        self.cmds: list = []
        self.stagger = 0.0
        self.retries = 0            # automatic restarts since the operator's start
        self._gen = 0               # bumped by stop(): a start still staggering gives up
        self.aborted = False        # a lifecycle manager gave up (from the log)
        self.starts = 0             # every start, automatic ones too
        self.started = 0.0

    # -- lifecycle --

    def start(self, cmds: list, args: dict, stagger: float = 0.0, retry: bool = False) -> None:
        """Start the processes, stagger seconds apart (the lock is not held
        while waiting, so the state keeps going out)."""
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self.tail.clear()
            self.args, self.cmds, self.stagger = args, cmds, stagger
            if not retry:
                self.retries = 0
            self.aborted = False
            self.starts += 1
            self.started = time.time()
            self.procs = []
            gen = self._gen
            log = open(self._log_path, "a", buffering=1)
            log.write(f"\n=== {time.ctime()} start {args}{' (automatic restart)' if retry else ''}\n")
            self._set("starting", f"restarted ({self.retries}): it did not come up" if retry else "")
        for i, cmd in enumerate(cmds):
            if i and stagger:
                time.sleep(stagger)
            with self._lock:
                if gen != self._gen:            # stopped meanwhile
                    return
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, bufsize=1,
                                     start_new_session=True, errors="replace")
                self.procs.append(p)
            threading.Thread(target=self._pump, args=(p, log, self.starts), daemon=True).start()
            _save_pids()

    def _pump(self, p: subprocess.Popen, log, start: int) -> None:
        written = 0
        for line in p.stdout:
            if any(s in line for s in DROP_LINES):
                continue
            # only this start's processes: a stopped one says such things while it dies
            if start == self.starts and any(s in line for s in ABORTED):
                self.aborted = True
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
            self._gen += 1
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


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def _signal_groups(procs: list) -> None:
    """SIGINT (like Ctrl-C, lets ros2 launch shut down cleanly), then escalate,
    to the whole process group: a child left behind by its parent goes too."""
    groups = [p.pid for p in procs]
    for sig, wait in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        alive = [g for g in groups if _group_alive(g)]
        if not alive:
            break
        for g in alive:
            try:
                os.killpg(g, sig)
            except ProcessLookupError:
                pass
        end = time.time() + wait
        while time.time() < end and any(_group_alive(g) for g in alive):
            for p in procs:
                p.poll()            # reap the parents, so their group can empty
            time.sleep(0.2)
    for p in procs:
        p.poll()


STACKS = {"bringup": Stack("bringup"), "nav": Stack("nav")}


def _nav_runs_on(name: str) -> bool:
    nav = STACKS["nav"]
    return nav.running() and nav.args.get("map") == name


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
        self.preview_pub = self.create_publisher(String, "/rcs/launcher/preview", 5)
        self.map_data_pub = self.create_publisher(String, "/rcs/launcher/map_data", 5)
        self.zones_pub = self.create_publisher(String, "/rcs/launcher/zones", 5)
        self._active_cli = {m: self.create_client(Trigger, f"/{m}/is_active") for m in MANAGERS}
        self._active: dict = {}             # manager -> it said it is active (this start)
        self._active_fut: dict = {}
        self._active_key = None             # which start of the nav stack they are about
        self._pose_t = 0.0                  # last /amcl_pose: AMCL has a pose
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 lambda _m: setattr(self, "_pose_t", time.time()), 10)
        self._load_map_clients: dict = {}      # made here, not from a worker thread
        try:
            from nav2_msgs.srv import LoadMap
            for server in ("keepout_mask_server", "speed_mask_server"):
                self._load_map_clients[server] = self.create_client(LoadMap, f"/{server}/load_map")
        except ImportError:                     # no Nav2 here: zones are kept, never applied live
            pass
        self.exchange = MapExchange(_maps, SAVED_MAPS, _nav_runs_on)
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
        """starting -> running once the stack's key topics have publishers and,
        for Nav2, its lifecycle managers say it is active. Nav2 that does not
        come up is restarted (_nav_not_up)."""
        b, n = STACKS["bringup"], STACKS["nav"]
        if b.state == "starting" and self.count_publishers("/odom") and \
                self.count_publishers("/scan_filtered"):
            b._set("running")
        if n.state not in ("starting", "running") or not n.procs or len(n.procs) != len(n.cmds):
            return
        self._ask_active(n)
        nav_ok = self._active.get("lifecycle_manager_navigation", False)
        others = all(self._active.get(m) for m in self._managers(n) if m != "lifecycle_manager_navigation")
        posed = self._waits_for_pose(n)
        if n.state == "starting" and others and (nav_ok or posed) and \
                self.count_publishers("/navigate_to_pose/_action/status") and \
                self.count_publishers("/map"):
            n._set("running")
        now = time.time()
        why = ""
        if n.aborted:
            why = "its bring-up was aborted"
        elif n.state == "starting" and now - n.since > NAV_UP_TIMEOUT_S:
            why = f"not up in {NAV_UP_TIMEOUT_S:.0f} s"
        elif n.state == "running" and posed and not nav_ok and self._pose_t > n.started and \
                now - self._pose_t_first(n) > NAV_ACTIVE_AFTER_POSE_S:
            why = f"not active {NAV_ACTIVE_AFTER_POSE_S:.0f} s after the robot got its pose"
        if why:
            with self._lock:
                if self.busy:
                    return
                self.busy = "restart nav"
            threading.Thread(target=self._nav_not_up, args=(why,), daemon=True).start()

    def _waits_for_pose(self, n: "Stack") -> bool:
        """Nav2 on a saved map: its navigation half waits for AMCL's pose."""
        flat = " ".join(" ".join(c) for c in n.cmds)
        return "nav_map.launch.py" in flat or "localization_launch.py" in flat

    def _pose_t_first(self, n: "Stack") -> float:
        """When AMCL first had a pose in this start of the nav stack."""
        if getattr(self, "_first_pose", (None, 0.0))[0] != n.starts:
            self._first_pose = (n.starts, 0.0)
        if not self._first_pose[1] and self._pose_t > n.started:
            self._first_pose = (n.starts, self._pose_t)
        return self._first_pose[1] or time.time()

    def _managers(self, n: "Stack") -> tuple:
        """The lifecycle managers this start of the nav stack has."""
        flat = " ".join(" ".join(c) for c in n.cmds)
        out = ["lifecycle_manager_navigation"]
        if "localization_launch.py" in flat or "nav_map.launch.py" in flat:
            out.append("lifecycle_manager_localization")
        if "costmap_filters.launch.py" in flat or "keepout_mask:=" in flat:
            out.append("lifecycle_manager_zones")
        return tuple(out)

    def _ask_active(self, n: "Stack") -> None:
        """Ask the nav stack's lifecycle managers whether they are active, until
        each says yes (answers come on the executor), from scratch at every start."""
        if n.starts != self._active_key:
            self._active_key, self._active, self._active_fut = n.starts, {}, {}
        for m in self._managers(n):
            if self._active.get(m):
                continue
            fut = self._active_fut.get(m)
            if fut is not None and fut.done():
                res = fut.result()
                self._active[m] = bool(res is not None and res.success)
                fut = None
                self._active_fut.pop(m, None)
            if fut is None and not self._active.get(m) and self._active_cli[m].service_is_ready():
                self._active_fut[m] = self._active_cli[m].call_async(Trigger.Request())

    def _nav_not_up(self, why: str) -> None:
        """Nav2 has not come up: restart it (NAV_RESTARTS times), then give up and say so."""
        n = STACKS["nav"]
        try:
            n.stop()
            if n.retries < NAV_RESTARTS:
                n.retries += 1
                self.get_logger().warn(f"Nav2 did not come up ({why}): restarting it ({n.retries}/{NAV_RESTARTS})")
                n.start(n.cmds, n.args, stagger=n.stagger, retry=True)
            else:
                n._set("failed", f"Nav2 did not come up {NAV_RESTARTS + 1} times ({why}): see the log below")
        finally:
            with self._lock:
                self.busy = ""
            self.publish_state()

    def publish_state(self) -> None:
        stacks = {k: s.to_dict() for k, s in STACKS.items()}
        # Started by hand (ssh, a terminal) rather than from the app.
        if STACKS["bringup"].state in ("stopped", "failed") and self.count_publishers("/odom"):
            stacks["bringup"]["external"] = True
        if STACKS["nav"].state in ("stopped", "failed") and \
                self.count_publishers("/navigate_to_pose/_action/status"):
            stacks["nav"]["external"] = True
        maps = _maps()
        msg = {"version": 1, "rcs": ROBOT_SIDE, "time": time.time(), "stacks": stacks,
               "maps": sorted(maps), "map_mtimes": {k: _mtime(v) for k, v in maps.items()},
               "caps": ["map_transfer", "zones"] + (["estop_gate"] if GATED_TOPIC in " ".join(BRINGUP_CMD) else []), "busy": self.busy, "result": self.result,
               "camera": {**camera_usb(), "publishers": self.count_publishers("/camera/depth/image_raw")}}
        self.pub.publish(String(data=json.dumps(msg)))

    # -- commands --

    def _on_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(cmd, dict):
            return
        if cmd.get("op") == "map_preview":      # read-only: fine while a stack is starting
            threading.Thread(target=self._send_preview, args=(str(cmd.get("name", "")),),
                             daemon=True).start()
            return
        if cmd.get("op") in ("map_get", "map_put"):     # files only: fine while a stack runs
            threading.Thread(target=self._exchange_map, args=(cmd,), daemon=True).start()
            return
        if cmd.get("op") in ("zones_get", "zones_put"):     # files, and a reload if Nav2 uses them
            threading.Thread(target=self._zones, args=(cmd,), daemon=True).start()
            return
        with self._lock:
            if self.busy:
                self._finish(cmd, False, f"busy: {self.busy}")
                return
            self.busy = f"{cmd.get('op')} {cmd.get('stack', cmd.get('name', ''))}".strip()
        self.publish_state()
        threading.Thread(target=self._run, args=(cmd,), daemon=True).start()

    def _send_preview(self, name: str) -> None:
        path = _maps().get(name)
        try:
            if path is None:
                raise ValueError("map not found on the robot")
            out = {"name": name, "saved": Path(path).parent == SAVED_MAPS, **map_preview(path)}
        except Exception as e:      # a bad map file must not touch the launcher
            out = {"name": name, "error": str(e)}
        self.preview_pub.publish(String(data=json.dumps(out)))

    def _exchange_map(self, cmd: dict) -> None:
        self.map_data_pub.publish(String(data=json.dumps(self.exchange.handle(cmd))))

    def _zones(self, cmd: dict) -> None:
        name = str(cmd.get("name", ""))
        out = {"id": str(cmd.get("id", "")), "name": name, "ok": True, "msg": "", "zones": [], "applied": False}
        try:
            path = _maps().get(name)
            if path is None:
                raise ValueError(f"map {name!r} not found on the robot")
            if cmd.get("op") == "zones_put":
                zones = zone_masks.write_zones(path, cmd.get("zones"))
                out["msg"] = f"{len(zones)} zone(s) saved on the robot"
                if _nav_runs_on(name):
                    out["applied"], why = self._reload_zones(path, zones)
                    out["msg"] += ", in use now" if out["applied"] else f"; {why}"
            out["zones"] = zone_masks.read_zones(path)
        except Exception as e:      # a bad request must not touch the launcher
            out.update(ok=False, msg=str(e))
        self.zones_pub.publish(String(data=json.dumps(out)))
        if cmd.get("op") == "zones_put":
            self.get_logger().info(f"zones of {name}: {out['msg']}")

    def _reload_zones(self, map_yaml: str, zones: list):
        """New masks into the running filters (their map servers reload the files)."""
        try:
            masks = zone_masks.write_masks(map_yaml, zones, ZONES_DIR)
        except Exception as e:
            return False, f"not applied: {e}"
        for server, key in (("keepout_mask_server", "keepout"), ("speed_mask_server", "speed")):
            cli = self._load_map_clients.get(server)
            if cli is None or not cli.wait_for_service(timeout_sec=3.0):
                return False, "Nav2 runs without zone filters: restart navigation to use them"
            fut = cli.call_async(cli.srv_type.Request(map_url=masks[key]))
            end = time.monotonic() + 10.0          # main() spins the node: the answer comes there
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.05)
            if not fut.done() or fut.result() is None or fut.result().result != 0:      # RESULT_SUCCESS
                return False, f"not applied: {server} did not load the new mask"
        return True, ""

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
            if GATED_TOPIC in " ".join(BRINGUP_CMD) and not self.count_publishers(GATED_TOPIC):
                return False, ("the robot's E-STOP service (rcs_safety.py) is not running, so the "
                               "drives would not move: see robot/README.md")
            st.start([BRINGUP_CMD], {})
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
             "--ros-args", "-p", "map_subscribe_transient_local:=true",
             *(["-p", "use_sim_time:=true"] if SIM_TIME else [])],
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
