"""rosbridge client wrapped as a QObject with signals.

Everything in this module runs on roslibpy's own IO thread; it posts into Qt
by emitting signals (queued, thread-safe). Nothing here uses rclpy — the
operator PC does not need ROS 2 installed.
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

try:
    import roslibpy
except ImportError:  # pragma: no cover
    roslibpy = None

from PySide6.QtCore import QObject, QTimer, Signal, Slot

from .profile import RobotProfile


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

    All subscription callbacks arrive from the roslibpy IO thread. We re-emit
    them as Qt signals (which are queued across threads via Qt::AutoConnection
    + the receiver's thread affinity), so UI slots run safely on the GUI thread.
    """

    connected = Signal()
    disconnected = Signal()
    connection_error = Signal(str)
    _request_reconnect = Signal()   # internal: hop IO thread -> GUI thread

    odom_received = Signal(dict)
    scan_received = Signal(dict)
    map_received = Signal(dict)
    global_costmap_received = Signal(dict)
    local_costmap_received = Signal(dict)
    plan_received = Signal(dict)
    image_received = Signal(bytes)      # compressed JPEG
    cloud_received = Signal(dict)
    rosout_received = Signal(dict)
    tf_received = Signal(dict)
    estop_received = Signal(bool)
    nav_feedback = Signal(dict)
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
        self._want_connected = False
        self._nav_goal_id: Optional[str] = None

    # ---- lifecycle ----

    def set_profile(self, profile: RobotProfile) -> None:
        was_connected = self.is_connected()
        if was_connected:
            self.disconnect()
        self.profile = profile
        if was_connected:
            self.connect()

    def is_connected(self) -> bool:
        return self._ros is not None and self._ros.is_connected

    def connect(self) -> None:
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

    def disconnect(self) -> None:
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
        self.connected.emit()
        self._subscribe_defaults()

    def _on_closed(self) -> None:
        self.disconnected.emit()
        if self._want_connected:
            self._request_reconnect.emit()   # cross-thread safe

    def _on_error(self, msg: str) -> None:
        self.connection_error.emit(msg)
        if self._want_connected:
            self._request_reconnect.emit()

    # ---- subscriptions ----

    def _mark(self, key: str) -> None:
        self.topic_stats.setdefault(key, TopicStats()).mark()

    def _subscribe_defaults(self) -> None:
        if not self.profile or not self._ros:
            return
        t = self.profile.topics
        self._subscribe(t.odom, "nav_msgs/msg/Odometry",
                        lambda m: (self._mark("odom"), self.odom_received.emit(m)))
        self._subscribe(t.scan, "sensor_msgs/msg/LaserScan",
                        lambda m: (self._mark("scan"), self.scan_received.emit(m)),
                        throttle_rate=100)
        self._subscribe(t.map, "nav_msgs/msg/OccupancyGrid",
                        lambda m: (self._mark("map"), self.map_received.emit(m)),
                        latch=True)
        self._subscribe(t.global_costmap, "nav_msgs/msg/OccupancyGrid",
                        lambda m: (self._mark("global_costmap"), self.global_costmap_received.emit(m)),
                        throttle_rate=1000, latch=True)
        self._subscribe(t.local_costmap, "nav_msgs/msg/OccupancyGrid",
                        lambda m: (self._mark("local_costmap"), self.local_costmap_received.emit(m)),
                        throttle_rate=500)
        self._subscribe(t.plan, "nav_msgs/msg/Path",
                        lambda m: (self._mark("plan"), self.plan_received.emit(m)))
        self._subscribe(t.camera_image, "sensor_msgs/msg/CompressedImage",
                        lambda m: (self._mark("image"), self._on_image(m)),
                        throttle_rate=self.profile.limits.image_throttle_ms)
        if t.camera_cloud:
            self._subscribe(t.camera_cloud, "sensor_msgs/msg/PointCloud2",
                            lambda m: (self._mark("cloud"), self.cloud_received.emit(m)),
                            throttle_rate=self.profile.limits.cloud_throttle_ms)
        self._subscribe(t.rosout, "rcl_interfaces/msg/Log",
                        lambda m: (self._mark("rosout"), self.rosout_received.emit(m)))
        self._subscribe(t.estop_topic, "std_msgs/msg/Bool",
                        lambda m: self.estop_received.emit(bool(m.get("data", False))),
                        latch=True)
        self._subscribe("/tf", "tf2_msgs/msg/TFMessage",
                        lambda m: (self._mark("tf"), self.tf_received.emit(m)))

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
                self.image_received.emit(base64.b64decode(data))
            except Exception:
                pass
        elif isinstance(data, (bytes, bytearray)):
            self.image_received.emit(bytes(data))

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
        self.nav_status.emit("active")

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
