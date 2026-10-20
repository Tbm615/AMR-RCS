"""rosbridge client wrapped as a QObject with signals.

Threads:
  - roslibpy's network thread (the twisted reactor) parses the JSON and runs
    the subscription callbacks. Those never touch Qt: cheap messages go into
    a locked mailbox, heavy ones (maps, costmaps, scans, camera, depth) are
    handed to the decoder thread.
  - the decoder thread turns heavy messages into numpy arrays / QImages and
    builds the 3D map, then drops the result into the mailbox.
  - a QTimer on the GUI thread drains the mailbox and emits the signals.

Connecting never blocks the GUI: roslibpy reconnects on its own (backoff
capped at a few seconds) and every (re)connect re-subscribes from scratch.
The twisted reactor is never stopped — it cannot be restarted, which used to
leave the app unable to reconnect after a disconnect.

Nothing here uses rclpy — the operator PC does not need ROS 2 installed.
"""
from __future__ import annotations

import base64
import json
import logging
import math
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

import numpy as np

try:
    import roslibpy
except ImportError:  # pragma: no cover
    roslibpy = None

from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QImage

from . import cloud_map as cm
from .profile import RobotProfile
from .tf_cache import TFCache, stamp_sec

LOG = logging.getLogger("rcs.ros")

# Topics of the robot-side launcher (robot/rcs_launcher.py).
LAUNCHER_CMD = "/rcs/launcher/cmd"
LAUNCHER_STATE = "/rcs/launcher/state"


# ----- small helpers ---------------------------------------------------------

@dataclass
class TopicStats:
    hz: float = 0.0
    last_t: float = 0.0
    _recent: deque = field(default_factory=lambda: deque(maxlen=20))

    def mark(self) -> None:
        now = time.monotonic()
        self._recent.append(now)
        self.last_t = now
        if len(self._recent) >= 2:
            span = self._recent[-1] - self._recent[0]
            self.hz = (len(self._recent) - 1) / span if span > 0 else 0.0

    def fresh(self, timeout: float = 2.0) -> bool:
        return self.last_t > 0 and (time.monotonic() - self.last_t) < timeout


def quat_to_yaw(q: Dict[str, float]) -> float:
    x, y, z, w = q.get("x", 0), q.get("y", 0), q.get("z", 0), q.get("w", 1)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    return math.atan2(siny, cosy)


def yaw_to_quat(yaw: float) -> Dict[str, float]:
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}


def grid_from_msg(msg: dict) -> Optional[np.ndarray]:
    """OccupancyGrid data -> (h, w) int8 array, or None if malformed."""
    info = msg.get("info", {})
    w, h = int(info.get("width", 0)), int(info.get("height", 0))
    data = msg.get("data")
    if w <= 0 or h <= 0 or data is None:
        return None
    if isinstance(data, str):          # some rosbridge builds send int8[] as base64
        a = np.frombuffer(base64.b64decode(data), dtype=np.int8)
    else:
        a = np.fromiter(data, dtype=np.int8, count=len(data)) if not isinstance(data, np.ndarray) else data
    if a.size != w * h:
        return None
    return a.reshape(h, w)


def cost_rgba(grid: np.ndarray) -> np.ndarray:
    """Nav2 costs -> RGBA. 100 lethal, 99 inscribed, 1..98 inflation, 0 free,
    -1 unknown. Inflation fades in with cost so low-cost space stays readable;
    obstacles (inscribed/lethal) are solid magenta."""
    c = grid.astype(np.int16)
    h, w = c.shape
    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    infl = (c > 0) & (c < 99)
    rgba[infl, 0], rgba[infl, 1], rgba[infl, 2] = 70, 130, 255
    rgba[infl, 3] = np.clip(c[infl] * 1.6, 0, 150).astype(np.uint8)
    rgba[c >= 99] = (230, 40, 140, 210)
    return rgba


def scan_points(msg: dict):
    """LaserScan -> (xs, ys) in the laser frame, valid returns only."""
    ranges = msg.get("ranges") or []
    if not len(ranges):
        return np.empty(0, np.float32), np.empty(0, np.float32)
    a0 = float(msg.get("angle_min", 0.0))
    da = float(msg.get("angle_increment", 0.0))
    rmin = float(msg.get("range_min", 0.0))
    rmax = float(msg.get("range_max", 100.0))
    # rosbridge encodes inf/NaN (no return) as JSON null -> None
    arr = np.array([np.inf if r is None else r for r in ranges], dtype=np.float32)
    ang = a0 + np.arange(arr.size, dtype=np.float32) * da
    ok = np.isfinite(arr) & (arr >= rmin) & (arr <= rmax)
    arr, ang = arr[ok], ang[ok]
    return arr * np.cos(ang), arr * np.sin(ang)


class _Decoder(threading.Thread):
    """Single worker with a coalescing queue: only the newest job per key is
    kept, so a slow decode never builds a backlog."""

    def __init__(self) -> None:
        super().__init__(name="rcs-decoder", daemon=True)
        self._cv = threading.Condition()
        self._jobs: Dict[str, tuple] = {}
        self.busy_s = 0.0              # total time spent decoding (for tests)

    def submit(self, key: str, fn: Callable, arg: Any) -> None:
        with self._cv:
            self._jobs.pop(key, None)            # move to the back of the queue
            self._jobs[key] = (fn, arg)
            self._cv.notify()

    def clear(self) -> None:
        with self._cv:
            self._jobs.clear()

    def run(self) -> None:
        while True:
            with self._cv:
                while not self._jobs:
                    self._cv.wait()
                key = next(iter(self._jobs))
                fn, arg = self._jobs.pop(key)
            t0 = time.perf_counter()
            try:
                fn(arg)
            except Exception:  # noqa: BLE001
                LOG.exception("decoding %s failed", key)
            self.busy_s += time.perf_counter() - t0


# ----- the client ------------------------------------------------------------

class RosClient(QObject):
    """Qt-friendly wrapper around roslibpy.Ros.

    Signals are only ever emitted on the GUI thread (see _post/_drain), so
    every connected slot, including plain lambdas, runs on the GUI thread.
    Emitting Qt signals straight from roslibpy's threads corrupted the heap
    under high message rates (TF/odom at 50 Hz).
    """

    connected = Signal()
    disconnected = Signal()
    connection_error = Signal(str)

    odom_received = Signal(object)
    scan_received = Signal(object)       # LaserScan dict + "_xy": (xs, ys) in the laser frame
    map_received = Signal(object)        # OccupancyGrid dict + "_grid": (h, w) int8
    global_costmap_received = Signal(object)   # + "_rgba"
    local_costmap_received = Signal(object)    # + "_rgba"
    plan_received = Signal(object)
    image_received = Signal(object)      # QImage (decoded camera frame)
    cloud_updated = Signal(object)       # dict: stats of the last 3D-map frame
    rosout_received = Signal(object)
    tf_received = Signal(object)         # TF changed (already in self.tf); payload None
    estop_received = Signal(bool)
    launcher_state = Signal(object)      # dict from the robot's rcs_launcher.py
    nav_status = Signal(str)             # "idle" | "active" | "succeeded" | "aborted" | "canceled"

    CONNECT_HINT_MS = 6000

    def __init__(self, profile: Optional[RobotProfile] = None, parent=None):
        super().__init__(parent)
        self.profile = profile
        self._ros: Optional["roslibpy.Ros"] = None
        self._gen = 0                    # bumps per connect; stale callbacks compare it
        self._subs: Dict[str, "roslibpy.Topic"] = {}
        self._pubs: Dict[str, "roslibpy.Topic"] = {}
        self._topic_lock = threading.RLock()
        self.topic_stats: Dict[str, TopicStats] = {}
        self.tf = TFCache()
        self.cloud_map = cm.CloudMap(voxel=0.03)
        # mailbox filled by network/decoder threads, drained on the GUI thread
        self._lock = threading.Lock()
        self._events: deque = deque(maxlen=5000)     # ordered, every message
        self._latest: Dict[str, tuple] = {}           # coalesced, newest only
        self._pump = QTimer(self)
        self._pump.setInterval(15)
        self._pump.timeout.connect(self._drain)
        self._pump.start()
        self._decoder = _Decoder()
        self._decoder.start()
        self._want_connected = False
        self._connect_hint = QTimer(self)
        self._connect_hint.setSingleShot(True)
        self._connect_hint.timeout.connect(self._hint_not_connected)
        self._seen_goals: set = set()
        self._goal_acked = True
        self._nav_goal_id: Optional[str] = None
        # Camera, depth and point cloud are heavy: only subscribed while a page
        # showing them is open (or the 3D map is recording).
        #   image: camera page      map3d: 3D map recording
        self._streams_on: Dict[str, bool] = {"image": False, "map3d": False}
        self.map3d_source = "rgbd"       # "rgbd" (depth + color images) | "cloud" (PointCloud2)
        self.map3d_range = 4.0           # m, farther depth is too noisy to keep
        self._color_ring: deque = deque(maxlen=12)    # (stamp, jpeg bytes) for RGB-D pairing
        self._depth_info: Optional[dict] = None       # last CameraInfo of the depth stream
        self.map3d_frames = 0

    # ---- thread hand-off ----

    _NOARG = object()

    def _post(self, signal_name: str, payload: Any = _NOARG, coalesce: bool = False) -> None:
        """Queue a signal emission for the GUI thread. With coalesce=True only
        the newest payload per signal is kept (for high-rate state topics)."""
        with self._lock:
            if coalesce:
                self._latest[signal_name] = (signal_name, payload)
            else:
                self._events.append((signal_name, payload))

    def _drain(self) -> None:
        with self._lock:
            if not self._events and not self._latest:
                return
            items = list(self._events) + list(self._latest.values())
            self._events.clear()
            self._latest.clear()
        for name, payload in items:
            sig = getattr(self, name)
            try:
                if payload is self._NOARG:
                    sig.emit()
                else:
                    sig.emit(payload)
            except Exception:  # noqa: BLE001  one bad slot must not kill the pump
                LOG.exception("handler for %s failed", name)

    # ---- lifecycle ----

    def set_profile(self, profile: RobotProfile) -> None:
        was = self._want_connected
        if was:
            self.disconnect_robot()
        self.profile = profile
        if was:
            self.connect_robot()

    def is_connected(self) -> bool:
        ros = self._ros
        return ros is not None and bool(ros.is_connected)

    def connect_robot(self) -> None:
        """Start connecting; returns at once. `connected` fires when the socket
        is up, and roslibpy keeps retrying until disconnect_robot()."""
        if roslibpy is None:
            self.connection_error.emit("roslibpy is not installed")
            return
        if not self.profile:
            self.connection_error.emit("no robot profile selected")
            return
        if self._ros is not None:
            self._drop_connection()
        self._want_connected = True
        self._gen += 1
        gen = self._gen
        try:
            ros = roslibpy.Ros(host=self.profile.host, port=self.profile.port,
                               is_secure=self.profile.use_ssl)
        except Exception as e:  # noqa: BLE001
            self.connection_error.emit(str(e))
            return
        fac = ros.factory
        # retry quickly and forever (twisted's default backoff grows to an hour)
        fac.initialDelay, fac.maxDelay, fac.factor, fac.maxRetries = 0.5, 3.0, 1.6, None
        ros.on("ready", lambda *_: self._on_ready(gen))
        ros.on("close", lambda *_: self._on_closed(gen))
        self._ros = ros
        ros.factory.manager.run()        # starts the reactor thread once; non-blocking
        self._connect_hint.start(self.CONNECT_HINT_MS)

    def _hint_not_connected(self) -> None:
        if self._want_connected and not self.is_connected() and self.profile:
            self.connection_error.emit(
                f"no answer from {self.profile.ws_url} yet — still trying. "
                "Is rosbridge running on the robot and this PC on its network?")

    def _drop_connection(self) -> None:
        ros, self._ros = self._ros, None
        self._gen += 1
        with self._topic_lock:
            self._subs.clear()
            self._pubs.clear()
        self._decoder.clear()
        if ros is None:
            return
        try:
            ros.factory.stopTrying()     # ReconnectingClientFactory: no more retries
        except Exception:  # noqa: BLE001
            pass
        # close() sends the close frame from the reactor thread; never terminate():
        # a stopped twisted reactor cannot be started again.
        threading.Thread(target=self._close_quietly, args=(ros,), daemon=True).start()

    @staticmethod
    def _close_quietly(ros) -> None:
        try:
            ros.close(timeout=3)
        except Exception:  # noqa: BLE001
            pass

    def disconnect_robot(self) -> None:
        self._want_connected = False
        self._connect_hint.stop()
        self._drop_connection()
        self.disconnected.emit()

    def _on_ready(self, gen: int) -> None:          # network thread
        if gen != self._gen:
            return
        self._subscribe_defaults()
        self._post("connected")

    def _on_closed(self, gen: int) -> None:         # network thread
        if gen != self._gen:
            return
        self._post("disconnected")

    # ---- subscriptions ----

    def _mark(self, key: str) -> None:
        st = self.topic_stats.get(key)
        if st is None:
            st = self.topic_stats[key] = TopicStats()
        st.mark()

    def _subscribe_defaults(self) -> None:
        if not self.profile or not self._ros:
            return
        t = self.profile.topics
        with self._topic_lock:
            # a reconnect starts from a clean slate on the server side
            ros = self._ros
            for name in list(self._subs):
                try:
                    ros.off(name)
                except Exception:  # noqa: BLE001
                    pass
            self._subs.clear()
            self._pubs.clear()
            self._subscribe(t.odom, "nav_msgs/msg/Odometry",
                            lambda m: (self._mark("odom"), self._post("odom_received", m, coalesce=True)),
                            throttle_rate=50)
            self._subscribe(t.scan, "sensor_msgs/msg/LaserScan",
                            lambda m: (self._mark("scan"), self._decoder.submit("scan", self._dec_scan, m)),
                            throttle_rate=100)
            self._subscribe(t.map, "nav_msgs/msg/OccupancyGrid",
                            lambda m: (self._mark("map"), self._decoder.submit("map", self._dec_map, m)))
            self._subscribe(t.global_costmap, "nav_msgs/msg/OccupancyGrid",
                            lambda m: (self._mark("global_costmap"),
                                       self._decoder.submit("gcost", self._dec_cost,
                                                            ("global_costmap_received", m))),
                            throttle_rate=1000)
            self._subscribe(t.local_costmap, "nav_msgs/msg/OccupancyGrid",
                            lambda m: (self._mark("local_costmap"),
                                       self._decoder.submit("lcost", self._dec_cost,
                                                            ("local_costmap_received", m))),
                            throttle_rate=500)
            self._subscribe(t.plan, "nav_msgs/msg/Path",
                            lambda m: (self._mark("plan"), self._post("plan_received", m, coalesce=True)),
                            throttle_rate=200)
            self._subscribe(t.rosout, "rcl_interfaces/msg/Log",
                            lambda m: (self._mark("rosout"), self._post("rosout_received", m)))
            self._subscribe(t.estop_topic, "std_msgs/msg/Bool",
                            lambda m: self._post("estop_received", bool(m.get("data", False))))
            self._subscribe("/tf", "tf2_msgs/msg/TFMessage", self._on_tf)
            # Static transforms (e.g. base_link->laser_frame from the URDF) only
            # ever arrive on /tf_static.
            self._subscribe("/tf_static", "tf2_msgs/msg/TFMessage", self._on_tf)
            self._subscribe(LAUNCHER_STATE, "std_msgs/msg/String", self._on_launcher_state)
            if t.nav_action:
                self._subscribe(f"{t.nav_action}/_action/status", "action_msgs/msg/GoalStatusArray",
                                self._on_nav_status)
            self._sync_streams()

    def _on_tf(self, m: dict) -> None:              # network thread; cheap
        self._mark("tf")
        self.tf.ingest(m)
        self._post("tf_received", None, coalesce=True)

    # ---- decoder jobs (decoder thread) ----

    def _dec_scan(self, m: dict) -> None:
        m["_xy"] = scan_points(m)
        self._post("scan_received", m, coalesce=True)

    def _dec_map(self, m: dict) -> None:
        g = grid_from_msg(m)
        if g is None:
            return
        m["_grid"] = g
        m.pop("data", None)                          # drop the big list early
        self._post("map_received", m, coalesce=True)

    def _dec_cost(self, job) -> None:
        sig, m = job
        g = grid_from_msg(m)
        if g is None:
            return
        m["_rgba"] = cost_rgba(g)
        m.pop("data", None)
        self._post(sig, m, coalesce=True)

    def _dec_image(self, data: bytes) -> None:
        img = QImage.fromData(data)
        if not img.isNull():
            self._post("image_received", img, coalesce=True)

    def _map_frame(self) -> str:
        return self.profile.topics.map_frame if self.profile else "map"

    def _dec_depth(self, m: dict) -> None:
        """RGB-D frame -> points in the map frame -> 3D map."""
        info = self._depth_info
        if info is None:
            self._post("cloud_updated", {"error": "waiting for the depth camera_info"}, coalesce=True)
            return
        fmt = str(m.get("format", ""))
        depth = cm.decode_compressed_depth(m) if "format" in m else cm.decode_raw_depth(m)
        if depth is None:
            self._post("cloud_updated", {"error": f"cannot decode depth image ({fmt or m.get('encoding')})"},
                       coalesce=True)
            return
        hdr = m.get("header", {})
        stamp = stamp_sec(hdr)
        frame = (hdr.get("frame_id") or info.get("header", {}).get("frame_id") or "").lstrip("/")
        tf = self.tf.matrix(self._map_frame(), frame, stamp)
        if tf is None:
            self._post("cloud_updated", {"error": f"no TF from {self._map_frame()} to {frame or '?'}"},
                       coalesce=True)
            return
        color = None
        if self._color_ring:
            cst, jpeg = min(self._color_ring, key=lambda c: abs(c[0] - stamp))
            if abs(cst - stamp) < 0.25 or not stamp:
                color = cm.decode_jpeg_rgb(jpeg)
        # decimate so one frame is ~20k points whatever the resolution
        step = max(1, int(round(math.sqrt(depth.size / 20000))))
        pts, rgb = cm.project_depth(depth, info.get("k") or info.get("K"), color,
                                    step=step, max_range=self.map3d_range)
        self._integrate(cm.transform(pts, tf), rgb, tf, frame)

    def _dec_cloud(self, m: dict) -> None:
        out = cm.decode_pointcloud2(m)
        if out is None:
            self._post("cloud_updated", {"error": "cannot decode the PointCloud2"}, coalesce=True)
            return
        xyz, rgb = out
        hdr = m.get("header", {})
        frame = (hdr.get("frame_id") or "").lstrip("/")
        if frame == self._map_frame():
            # already a map (e.g. RTAB-Map's assembled cloud): take it as is
            self.cloud_map.replace(xyz, rgb)
            self.map3d_frames += 1
            self._post("cloud_updated", {"points": len(self.cloud_map), "frame_points": len(xyz),
                                         "frame": frame, "replaced": True}, coalesce=True)
            return
        tf = self.tf.matrix(self._map_frame(), frame, stamp_sec(hdr))
        if tf is None:
            self._post("cloud_updated", {"error": f"no TF from {self._map_frame()} to {frame or '?'}"},
                       coalesce=True)
            return
        d = np.linalg.norm(xyz, axis=1)
        keep = d < self.map3d_range
        self._integrate(cm.transform(xyz[keep], tf), rgb[keep] if rgb is not None else None, tf, frame)

    def _integrate(self, xyz, rgb, tf, frame: str) -> None:
        added = self.cloud_map.integrate(xyz, rgb)
        self.map3d_frames += 1
        self._post("cloud_updated", {"points": len(self.cloud_map), "frame_points": len(xyz),
                                     "added": added, "frame": frame, "colored": rgb is not None,
                                     "full": self.cloud_map.full,
                                     "sensor": tuple(float(v) for v in tf[:3, 3])}, coalesce=True)

    # ---- launcher ----

    def _on_launcher_state(self, m: dict) -> None:
        try:
            st = json.loads(m.get("data", ""))
        except (TypeError, ValueError):
            return
        if isinstance(st, dict):
            self._mark("launcher")
            self._post("launcher_state", st, coalesce=True)

    def send_launcher_cmd(self, op: str, **kw) -> str:
        """Ask the robot's launcher to start/stop a stack or save the map.
        Returns the command id, echoed back in launcher_state["result"]."""
        cmd_id = uuid.uuid4().hex[:12]
        pub = self._advertise(LAUNCHER_CMD, "std_msgs/msg/String")
        if pub is not None:
            pub.publish(roslibpy.Message({"data": json.dumps({"id": cmd_id, "op": op, **kw})}))
        return cmd_id

    # ---- on-demand streams ----

    def set_stream_active(self, key: str, on: bool) -> None:
        """Subscribe to (on) or drop (off) an on-demand stream: "image" (camera
        page) or "map3d" (3D map recording). "cloud" is an alias of "map3d"."""
        if key == "cloud":
            key = "map3d"
        if self._streams_on.get(key) == on:
            return
        self._streams_on[key] = on
        if self._ros and self.is_connected():
            with self._topic_lock:
                self._sync_streams()

    def set_map3d_source(self, source: str) -> None:
        if source == self.map3d_source:
            return
        self.map3d_source = source
        if self._ros and self.is_connected():
            with self._topic_lock:
                self._sync_streams()

    def _sync_streams(self) -> None:
        if not self.profile or not self._ros:
            return
        t, lim = self.profile.topics, self.profile.limits
        rec = self._streams_on["map3d"]
        rgbd = rec and self.map3d_source == "rgbd"
        depth_type = ("sensor_msgs/msg/CompressedImage" if t.camera_depth.endswith("compressedDepth")
                      else "sensor_msgs/msg/Image")
        want = {
            # color frames feed both the camera page and the RGB-D colouring
            t.camera_image: (self._streams_on["image"] or rgbd, "sensor_msgs/msg/CompressedImage",
                             self._on_image, lim.image_throttle_ms),
            t.camera_depth: (rgbd, depth_type,
                             lambda m: (self._mark("depth"), self._decoder.submit("depth", self._dec_depth, m)),
                             lim.cloud_throttle_ms),
            t.camera_info: (rgbd, "sensor_msgs/msg/CameraInfo", self._on_depth_info, 2000),
            t.camera_cloud: (rec and self.map3d_source == "cloud", "sensor_msgs/msg/PointCloud2",
                             lambda m: (self._mark("cloud"), self._decoder.submit("cloud", self._dec_cloud, m)),
                             lim.cloud_throttle_ms),
        }
        stat_key = {t.camera_image: "image", t.camera_depth: "depth", t.camera_cloud: "cloud",
                    t.camera_info: "depth_info"}
        for name, (on, msg_type, cb, throttle) in want.items():
            if not name:
                continue
            if on and name not in self._subs:
                self._subscribe(name, msg_type, cb, throttle_rate=throttle)
            elif not on and name in self._subs:
                try:
                    self._subs.pop(name).unsubscribe()
                except Exception:  # noqa: BLE001
                    pass
                self.topic_stats.pop(stat_key.get(name, name), None)

    def _on_depth_info(self, m: dict) -> None:
        self._mark("depth_info")
        self._depth_info = m

    def _on_image(self, m: dict) -> None:            # network thread
        self._mark("image")
        data = m.get("data")
        try:
            raw = base64.b64decode(data) if isinstance(data, str) else bytes(data or b"")
        except Exception:  # noqa: BLE001
            return
        if self._streams_on["map3d"] and self.map3d_source == "rgbd":
            self._color_ring.append((stamp_sec(m.get("header", {})), raw))
        if self._streams_on["image"]:
            self._decoder.submit("image", self._dec_image, raw)

    # action_msgs/GoalStatus codes
    _NAV_STATUS = {1: "active", 2: "active", 3: "canceling",
                   4: "succeeded", 5: "canceled", 6: "aborted"}

    def _on_nav_status(self, m: dict) -> None:
        sl = m.get("status_list") or []
        if not sl:
            return
        latest = max(sl, key=lambda s: (s.get("goal_info", {}).get("stamp", {}).get("sec", 0),
                                        s.get("goal_info", {}).get("stamp", {}).get("nanosec", 0)))
        uuid = tuple(latest.get("goal_info", {}).get("goal_id", {}).get("uuid", []) or [])
        if uuid and uuid not in self._seen_goals:
            self._seen_goals.add(uuid)
            self._goal_acked = True          # Nav2 accepted a new goal
        status = self._NAV_STATUS.get(int(latest.get("status", 0)))
        if status:
            self._post("nav_status", status)

    def publish_estop(self, on: bool) -> None:
        if not self.profile or not self.profile.topics.estop_topic:
            return
        pub = self._advertise(self.profile.topics.estop_topic, "std_msgs/msg/Bool")
        if pub is not None:
            pub.publish(roslibpy.Message({"data": bool(on)}))

    def _subscribe(
        self,
        name: str,
        msg_type: str,
        cb: Callable[[dict], Any],
        throttle_rate: int = 0,
    ) -> None:
        if not self._ros or not name:
            return
        # reconnect_on_close=False: _on_ready re-subscribes everything itself
        topic = roslibpy.Topic(self._ros, name, msg_type, throttle_rate=throttle_rate,
                               queue_size=1, reconnect_on_close=False)
        topic.subscribe(cb)
        self._subs[name] = topic

    # ---- publishing ----

    def _advertise(self, name: str, msg_type: str) -> Optional["roslibpy.Topic"]:
        if not self._ros or not name or not self.is_connected():
            return None
        with self._topic_lock:
            pub = self._pubs.get(name)
            if pub is None:
                pub = roslibpy.Topic(self._ros, name, msg_type, queue_size=1,
                                     reconnect_on_close=False)
                pub.advertise()
                self._pubs[name] = pub
        return pub

    def publish_cmd_vel(self, lin: float, ang: float) -> None:
        if not self.profile:
            return
        pub = self._advertise(self.profile.topics.cmd_vel, "geometry_msgs/msg/Twist")
        if pub is None:
            return
        pub.publish(roslibpy.Message({
            "linear": {"x": float(lin), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(ang)},
        }))

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        if not self.profile or not self._ros:
            return
        pub = self._advertise(self.profile.topics.initial_pose,
                              "geometry_msgs/msg/PoseWithCovarianceStamped")
        if pub is None:
            return
        pub.publish(roslibpy.Message({
            "header": {"frame_id": self.profile.topics.map_frame, "stamp": {"sec": 0, "nanosec": 0}},
            "pose": {
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": yaw_to_quat(yaw),
                },
                "covariance": [0.25, 0, 0, 0, 0, 0,
                               0, 0.25, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0.07],
            },
        }))

    # ---- nav2 goal ----
    #
    # Publishing to /goal_pose (PoseStamped) is what RViz does and is the most
    # robust way to drive Nav2 through rosbridge — the ROS 2 action protocol
    # over rosbridge still has rough edges in several roslibpy releases.
    # Nav2's bt_navigator subscribes to /goal_pose via nav2_bringup; it works
    # out of the box in Humble. We still try the action for cancel.

    def send_nav_goal(self, x: float, y: float, yaw: float = 0.0) -> None:
        if not self.profile or not self._ros:
            return
        pub = self._advertise(self.profile.topics.goal_pose, "geometry_msgs/msg/PoseStamped")
        if pub is None:
            return
        pub.publish(roslibpy.Message({
            "header": {"frame_id": self.profile.topics.map_frame,
                       "stamp": {"sec": 0, "nanosec": 0}},
            "pose": {
                "position": {"x": float(x), "y": float(y), "z": 0.0},
                "orientation": yaw_to_quat(float(yaw)),
            },
        }))
        # /goal_pose is fire-and-forget; Nav2 drops it if bt_navigator isn't
        # active yet. Report "pending" until the action status shows a new
        # goal, and "no_response" if that doesn't happen.
        self._goal_acked = False
        self.nav_status.emit("pending")
        QTimer.singleShot(self.GOAL_ACK_TIMEOUT_MS, self._check_goal_ack)

    GOAL_ACK_TIMEOUT_MS = 4000

    def _check_goal_ack(self) -> None:
        if not self._goal_acked:
            self.nav_status.emit("no_response")

    def cancel_nav_goal(self) -> None:
        """Send empty goal to cancel via the Nav2 BT navigator's cancel service.

        rosbridge exposes action cancellation as a service call; we try the
        documented endpoint and swallow failures (the user also has the
        physical E-stop in the sidebar as a hard brake).
        """
        if self._ros and self.profile:
            try:
                service = roslibpy.Service(
                    self._ros,
                    self.profile.topics.cancel_nav,
                    "action_msgs/srv/CancelGoal",
                )
                service.call(roslibpy.ServiceRequest({"goal_info": {
                    "goal_id": {"uuid": [0] * 16}, "stamp": {"sec": 0, "nanosec": 0}
                }}), lambda _r: None, lambda _e: None)
            except Exception:
                pass
        self.nav_status.emit("canceled")

    # ---- generic service call ----

    def call_service(self, name: str, srv_type: str, args: dict, cb: Callable[[dict], Any]) -> None:
        if not self._ros:
            return
        service = roslibpy.Service(self._ros, name, srv_type)
        service.call(roslibpy.ServiceRequest(args), cb, lambda e: cb({"error": str(e)}))

    # ---- topic discovery ----

    def topics_list(self, cb: Callable[[list], Any]) -> None:
        if not self._ros:
            cb([])
            return
        self._ros.get_topics(cb, lambda e: cb([]))
