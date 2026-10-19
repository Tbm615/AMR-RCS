"""rosbridge client wrapped as a QObject with signals.

roslibpy delivers callbacks on its own network threads. Those threads never
touch Qt: they drop messages into a locked mailbox, and a QTimer on the GUI
thread drains it and emits the signals. Nothing here uses rclpy — the
operator PC does not need ROS 2 installed.
"""
from __future__ import annotations

import json
import math
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

try:
    import roslibpy
except ImportError:  # pragma: no cover
    roslibpy = None

from PySide6.QtCore import QObject, QTimer, Signal, Slot

from .profile import RobotProfile

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
    _request_reconnect = Signal()   # internal: hop IO thread -> GUI thread

    odom_received = Signal(object)
    scan_received = Signal(object)
    map_received = Signal(object)
    global_costmap_received = Signal(object)
    local_costmap_received = Signal(object)
    plan_received = Signal(object)
    image_received = Signal(object)      # compressed JPEG (bytes)
    cloud_received = Signal(object)
    rosout_received = Signal(object)
    tf_received = Signal(object)
    estop_received = Signal(bool)
    launcher_state = Signal(object)      # dict from the robot's rcs_launcher.py
    nav_status = Signal(str)             # "idle" | "active" | "succeeded" | "aborted" | "canceled"

    def __init__(self, profile: Optional[RobotProfile] = None, parent=None):
        super().__init__(parent)
        self.profile = profile
        self._ros: Optional["roslibpy.Ros"] = None
        self._subs: Dict[str, "roslibpy.Topic"] = {}
        self._pubs: Dict[str, "roslibpy.Topic"] = {}
        self.topic_stats: Dict[str, TopicStats] = {}
        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setInterval(2000)
        self._reconnect_timer.timeout.connect(self._try_reconnect)
        # start() on QTimer must be called on the thread that owns it (the GUI
        # thread). roslibpy's close/error callbacks fire on its IO thread, so
        # we bounce through a queued-connection signal.
        self._request_reconnect.connect(self._reconnect_timer.start)
        # mailbox filled by network threads, drained on the GUI thread
        self._lock = threading.Lock()
        self._events: deque = deque(maxlen=5000)     # ordered, every message
        self._latest: Dict[str, tuple] = {}           # coalesced, newest only
        self._pump = QTimer(self)
        self._pump.setInterval(15)
        self._pump.timeout.connect(self._drain)
        self._pump.start()
        self._want_connected = False
        self._seen_goals: set = set()
        self._goal_acked = True
        self._nav_goal_id: Optional[str] = None
        # Camera and point cloud are heavy (the depth cloud is ~1 MB/frame):
        # only subscribed while a page showing them is open.
        self._streams_on: Dict[str, bool] = {"image": False, "cloud": False}

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
            if payload is self._NOARG:
                sig.emit()
            else:
                sig.emit(payload)

    # ---- lifecycle ----

    def set_profile(self, profile: RobotProfile) -> None:
        was_connected = self.is_connected()
        if was_connected:
            self.disconnect_robot()
        self.profile = profile
        if was_connected:
            self.connect_robot()

    def is_connected(self) -> bool:
        return self._ros is not None and self._ros.is_connected

    def connect_robot(self) -> None:
        if roslibpy is None:
            self.connection_error.emit("roslibpy is not installed")
            return
        if not self.profile:
            self.connection_error.emit("no robot profile selected")
            return
        self._want_connected = True
        self._open()

    def _open(self) -> None:
        try:
            self._ros = roslibpy.Ros(host=self.profile.host, port=self.profile.port, is_secure=self.profile.use_ssl)
            self._ros.on_ready(self._on_ready, run_in_thread=True)
            self._ros.on("close", lambda *_: self._on_closed())
            self._ros.on("error", lambda e, *_: self._on_error(str(e)))
            self._ros.run()
        except Exception as e:  # noqa: BLE001
            self._on_error(str(e))

    def disconnect_robot(self) -> None:
        self._want_connected = False
        self._reconnect_timer.stop()
        if self._ros:
            try:
                for t in list(self._subs.values()):
                    try:
                        t.unsubscribe()
                    except Exception:
                        pass
                for t in list(self._pubs.values()):
                    try:
                        t.unadvertise()
                    except Exception:
                        pass
                self._subs.clear()
                self._pubs.clear()
                self._ros.terminate()
            except Exception:
                pass
            self._ros = None
        self.disconnected.emit()

    def _try_reconnect(self) -> None:
        if not self._want_connected:
            self._reconnect_timer.stop()
            return
        if self.is_connected():
            self._reconnect_timer.stop()
            return
        self._open()

    def _on_ready(self) -> None:
        self._post("connected")
        self._subscribe_defaults()

    def _on_closed(self) -> None:
        self._post("disconnected")
        if self._want_connected:
            self._post("_request_reconnect")

    def _on_error(self, msg: str) -> None:
        self._post("connection_error", msg)
        if self._want_connected:
            self._post("_request_reconnect")

    # ---- subscriptions ----

    def _mark(self, key: str) -> None:
        self.topic_stats.setdefault(key, TopicStats()).mark()

    def _subscribe_defaults(self) -> None:
        if not self.profile or not self._ros:
            return
        t = self.profile.topics
        self._subscribe(t.odom, "nav_msgs/msg/Odometry",
                        lambda m: (self._mark("odom"), self._post("odom_received", m, coalesce=True)),
                        throttle_rate=50)
        self._subscribe(t.scan, "sensor_msgs/msg/LaserScan",
                        lambda m: (self._mark("scan"), self._post("scan_received", m, coalesce=True)),
                        throttle_rate=100)
        self._subscribe(t.map, "nav_msgs/msg/OccupancyGrid",
                        lambda m: (self._mark("map"), self._post("map_received", m, coalesce=True)),
                        latch=True)
        self._subscribe(t.global_costmap, "nav_msgs/msg/OccupancyGrid",
                        lambda m: (self._mark("global_costmap"), self._post("global_costmap_received", m, coalesce=True)),
                        throttle_rate=1000, latch=True)
        self._subscribe(t.local_costmap, "nav_msgs/msg/OccupancyGrid",
                        lambda m: (self._mark("local_costmap"), self._post("local_costmap_received", m, coalesce=True)),
                        throttle_rate=500)
        self._subscribe(t.plan, "nav_msgs/msg/Path",
                        lambda m: (self._mark("plan"), self._post("plan_received", m, coalesce=True)))
        self._sync_streams()
        self._subscribe(t.rosout, "rcl_interfaces/msg/Log",
                        lambda m: (self._mark("rosout"), self._post("rosout_received", m)))
        self._subscribe(t.estop_topic, "std_msgs/msg/Bool",
                        lambda m: self._post("estop_received", bool(m.get("data", False))),
                        latch=True)
        self._subscribe("/tf", "tf2_msgs/msg/TFMessage",
                        lambda m: (self._mark("tf"), self._post("tf_received", m)))
        # Static transforms (e.g. base_link->laser_frame from the URDF) only
        # ever arrive on /tf_static.
        self._subscribe("/tf_static", "tf2_msgs/msg/TFMessage",
                        lambda m: self._post("tf_received", m))
        self._subscribe(LAUNCHER_STATE, "std_msgs/msg/String", self._on_launcher_state)
        if t.nav_action:
            self._subscribe(f"{t.nav_action}/_action/status", "action_msgs/msg/GoalStatusArray",
                            self._on_nav_status)

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

    def set_stream_active(self, key: str, on: bool) -> None:
        """Subscribe to (on) or drop (off) an on-demand stream: "image" or "cloud"."""
        if self._streams_on.get(key) == on:
            return
        self._streams_on[key] = on
        if self._ros and self.is_connected():
            self._sync_streams()

    def _sync_streams(self) -> None:
        if not self.profile or not self._ros:
            return
        t, lim = self.profile.topics, self.profile.limits
        streams = {
            "image": (t.camera_image, "sensor_msgs/msg/CompressedImage",
                      lambda m: (self._mark("image"), self._on_image(m)), lim.image_throttle_ms),
            "cloud": (t.camera_cloud, "sensor_msgs/msg/PointCloud2",
                      lambda m: (self._mark("cloud"), self._post("cloud_received", m, coalesce=True)),
                      lim.cloud_throttle_ms),
        }
        for key, (name, msg_type, cb, throttle) in streams.items():
            if not name:
                continue
            if self._streams_on[key] and name not in self._subs:
                self._subscribe(name, msg_type, cb, throttle_rate=throttle)
            elif not self._streams_on[key] and name in self._subs:
                try:
                    self._subs.pop(name).unsubscribe()
                except Exception:
                    pass
                self.topic_stats.pop(key, None)

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
        latch: bool = False,
    ) -> None:
        if not self._ros or not name:
            return
        topic = roslibpy.Topic(self._ros, name, msg_type, throttle_rate=throttle_rate, queue_size=1)
        topic.subscribe(cb)
        self._subs[name] = topic

    def _on_image(self, m: dict) -> None:
        import base64
        data = m.get("data")
        if isinstance(data, str):
            try:
                self._post("image_received", base64.b64decode(data), coalesce=True)
            except Exception:
                pass
        elif isinstance(data, (bytes, bytearray)):
            self._post("image_received", bytes(data), coalesce=True)

    # ---- publishing ----

    def _advertise(self, name: str, msg_type: str) -> Optional["roslibpy.Topic"]:
        if not self._ros or not name:
            return None
        pub = self._pubs.get(name)
        if pub is None:
            pub = roslibpy.Topic(self._ros, name, msg_type, queue_size=1)
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
