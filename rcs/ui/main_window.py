"""Main window — sidebar + stacked panels, central controller for signals."""
from __future__ import annotations

import math
import time

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QAction, QIcon
from PySide6.QtWidgets import (
    QButtonGroup, QFrame, QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QStackedWidget, QStatusBar, QVBoxLayout, QWidget,
)

from .. import APP_NAME, __version__
from ..core.mission import MissionStore
from ..core.profile import ProfileStore, RobotProfile
from ..core.ros_client import RosClient, quat_to_yaw
from ..core.tf_cache import TFCache
from .panels.camera import CameraPanel
from .panels.connection import ConnectionPanel
from .panels.logs import LogsPanel
from .panels.map2d import Map2DPanel
from .panels.mission import MissionPanel
from .panels.navigation import NavigationPanel
from .panels.status import StatusPanel
from .panels.teleop import TeleopPanel
from .panels.view3d import View3DPanel


class _NavButton(QPushButton):
    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self.setObjectName("NavButton")
        self.setCheckable(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(42)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME} — Robot Control System")
        self.resize(1400, 880)

        self.profiles = ProfileStore()
        self.missions = MissionStore()
        self.ros = RosClient(profile=self.profiles.active())
        self.tf = TFCache()

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._laser_x = 0.0
        self._laser_y = 0.0
        self._laser_yaw = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_v = 0.0
        self._plan_xy: list = []
        self._nav_state = ""

        self._build()
        self._wire()

    # ---- construct UI ----

    def _build(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        side = QFrame(); side.setObjectName("Sidebar")
        side.setFixedWidth(200)
        sv = QVBoxLayout(side)
        sv.setContentsMargins(0, 12, 0, 12)
        sv.setSpacing(0)

        brand = QLabel("RCS")
        brand.setStyleSheet("color:#3b82f6; font-size:22px; font-weight:700; padding:10px 19px 0 19px;")
        sv.addWidget(brand)
        sub = QLabel(f"Robot Control System · v{__version__}")
        sub.setStyleSheet("padding: 0 19px 16px 19px; color:#5b6378; font-size:11px;")
        sv.addWidget(sub)

        self.nav_group = QButtonGroup(self)
        self.nav_group.setExclusive(True)

        self.btns = {}
        for key, label in [
            ("conn", "Connection"),
            ("status", "Status"),
            ("teleop", "Teleop"),
            ("map", "2D Map"),
            ("nav", "Navigation"),
            ("mission", "Missions"),
            ("3d", "3D View"),
            ("cam", "Camera"),
            ("logs", "Logs"),
        ]:
            b = _NavButton(label)
            self.nav_group.addButton(b)
            sv.addWidget(b)
            self.btns[key] = b
        sv.addStretch(1)

        est_frame = QFrame()
        est_frame.setObjectName("EstopBar")
        est_frame.setStyleSheet("QFrame#EstopBar { border-top: 1px solid #1a1d26; }")
        esv = QVBoxLayout(est_frame)
        esv.setContentsMargins(10, 10, 10, 10)
        self.sidebar_estop = QPushButton("● E-STOP")
        self.sidebar_estop.setProperty("kind", "danger")
        self.sidebar_estop.setCheckable(True)
        self.sidebar_estop.setMinimumHeight(44)
        self.sidebar_estop.setStyleSheet("font-weight:700; font-size:14px;")
        esv.addWidget(self.sidebar_estop)
        sv.addWidget(est_frame)

        root.addWidget(side)

        # stacked content area
        self.stack = QStackedWidget()
        root.addWidget(self.stack, 1)

        # instantiate panels
        self.p_conn = ConnectionPanel(self.profiles)
        self.p_status = StatusPanel(self.ros)
        self.p_teleop = TeleopPanel()
        self.p_map = Map2DPanel()
        self.p_nav = NavigationPanel()
        self.p_mission = MissionPanel(self.missions)
        self.p_3d = View3DPanel()
        self.p_cam = CameraPanel()
        self.p_logs = LogsPanel()

        self._pages = {
            "conn": self.p_conn,
            "status": self.p_status,
            "teleop": self.p_teleop,
            "map": self.p_map,
            "nav": self.p_nav,
            "mission": self.p_mission,
            "3d": self.p_3d,
            "cam": self.p_cam,
            "logs": self.p_logs,
        }
        for p in self._pages.values():
            self.stack.addWidget(p)

        for key, btn in self.btns.items():
            btn.clicked.connect(lambda _c=False, k=key: self._show(k))
        self.btns["conn"].setChecked(True)

        # status bar
        sb = QStatusBar()
        self.setStatusBar(sb)
        self.sb_conn = QLabel("● offline"); self.sb_conn.setStyleSheet("color:#ff6a6a;")
        self.sb_robot = QLabel("—")
        self.sb_pose = QLabel("pose: —")
        self.sb_estop = QLabel("● estop: clear"); self.sb_estop.setStyleSheet("color:#39d98a;")
        sb.addPermanentWidget(self.sb_robot)
        sb.addPermanentWidget(QLabel("   |   "))
        sb.addPermanentWidget(self.sb_conn)
        sb.addPermanentWidget(QLabel("   |   "))
        sb.addPermanentWidget(self.sb_pose)
        sb.addPermanentWidget(QLabel("   |   "))
        sb.addPermanentWidget(self.sb_estop)

    def _show(self, key: str) -> None:
        self.stack.setCurrentWidget(self._pages[key])

    # ---- wire signals ----

    def _wire(self) -> None:
        # connection panel ↔ ros client
        self.p_conn.connect_requested.connect(self._on_connect)
        self.p_conn.disconnect_requested.connect(self.ros.disconnect_robot)
        self.p_conn.profile_changed.connect(self._on_profile_changed)

        self.ros.connected.connect(self._on_ros_connected)
        self.ros.disconnected.connect(self._on_ros_disconnected)
        self.ros.connection_error.connect(self._on_ros_error)

        # ros → panels
        self.ros.odom_received.connect(self._on_odom)
        self.ros.scan_received.connect(self._on_scan)
        self.ros.map_received.connect(self._on_map)
        self.ros.global_costmap_received.connect(self._on_global_costmap)
        self.ros.local_costmap_received.connect(self._on_local_costmap)
        self.ros.plan_received.connect(self._on_plan)
        self.ros.image_received.connect(self.p_cam.set_frame)
        self.ros.cloud_received.connect(self.p_3d.update_pointcloud)
        self.ros.rosout_received.connect(self.p_logs.append_log)
        self.ros.tf_received.connect(self._on_tf)
        self.ros.nav_status.connect(self._on_nav_state)
        self.ros.nav_status.connect(self.p_nav.set_status)
        self.ros.estop_received.connect(self._on_estop_remote)

        # teleop → ros
        self.p_teleop.cmd_vel.connect(self.ros.publish_cmd_vel)
        self.p_teleop.estop_changed.connect(self._set_estop)
        self.sidebar_estop.toggled.connect(self._set_estop)

        # map / nav / mission → ros
        self.p_map.goal_requested.connect(self.ros.send_nav_goal)
        self.p_map.cancel_requested.connect(self.ros.cancel_nav_goal)
        self.p_map.set_initial_pose.connect(self.ros.publish_initial_pose)
        self.p_nav.goal_requested.connect(self.ros.send_nav_goal)
        self.p_nav.cancel_requested.connect(self.ros.cancel_nav_goal)
        self.p_mission.goal_requested.connect(self.ros.send_nav_goal)
        self.p_mission.cancel_requested.connect(self.ros.cancel_nav_goal)
        self.p_mission.waypoints_changed.connect(self.p_map.view.set_waypoints)
        self.ros.nav_status.connect(self.p_mission.on_nav_status)
        # "Add waypoint" mode on the 2D Map page appends to the open mission
        self.p_map.view.pose_clicked.connect(self.p_mission.add_waypoint_at)
        self.p_mission.waypoints_changed.connect(self.p_nav.view.set_waypoints)

    def _on_connect(self, profile: RobotProfile) -> None:
        self.ros.set_profile(profile)
        self.ros.connect_robot()
        self.sb_robot.setText(f"{profile.name} ({profile.ws_url})")
        self._apply_profile_to_panels(profile)

    def _on_profile_changed(self, profile: RobotProfile) -> None:
        self._apply_profile_to_panels(profile)

    def _apply_profile_to_panels(self, profile: RobotProfile) -> None:
        self.p_teleop.apply_limits(profile.limits.max_linear, profile.limits.max_angular)
        self.p_map.view.set_robot_footprint(
            profile.limits.footprint_half_length, profile.limits.footprint_half_width)
        self.p_nav.view.set_robot_footprint(
            profile.limits.footprint_half_length, profile.limits.footprint_half_width)
        self.p_mission.view.set_robot_footprint(
            profile.limits.footprint_half_length, profile.limits.footprint_half_width)
        self.p_3d.set_robot_footprint(
            profile.limits.footprint_half_length, profile.limits.footprint_half_width)

    def _on_ros_connected(self) -> None:
        self.sb_conn.setText("● connected"); self.sb_conn.setStyleSheet("color:#39d98a;")
        self.p_conn.set_connected(True, self.ros.profile.ws_url if self.ros.profile else "")
        self.p_status.set_connected(True)

    def _on_ros_disconnected(self) -> None:
        self.sb_conn.setText("● offline"); self.sb_conn.setStyleSheet("color:#ff6a6a;")
        self.p_conn.set_connected(False)
        self.p_status.set_connected(False)

    def _on_ros_error(self, msg: str) -> None:
        self.p_conn.set_connected(False, msg)

    # ---- ros message handlers (on GUI thread via queued Qt signals) ----

    def _on_odom(self, m: dict) -> None:
        p = m.get("pose", {}).get("pose", {})
        pos = p.get("position", {})
        self._odom_x = float(pos.get("x", 0.0))
        self._odom_y = float(pos.get("y", 0.0))
        self._odom_yaw = quat_to_yaw(p.get("orientation", {}))
        self._odom_v = float(m.get("twist", {}).get("twist", {}).get("linear", {}).get("x", 0.0))
        self._update_robot_pose()

    def _on_tf(self, m: dict) -> None:
        self.tf.ingest(m)
        self._update_robot_pose()

    def _update_robot_pose(self) -> None:
        if not self.ros.profile:
            return
        p = self.ros.profile.topics
        pose = self.tf.pose_xyyaw(p.map_frame, p.base_frame)
        if pose is None:
            pose = self.tf.pose_xyyaw(p.odom_frame, p.base_frame)
        if pose is None:
            pose = (self._odom_x, self._odom_y, self._odom_yaw)
        self._robot_x, self._robot_y, self._robot_yaw = pose

        laser_pose = self.tf.pose_xyyaw(p.map_frame, p.laser_frame)
        if laser_pose is None:
            laser_pose = self.tf.pose_xyyaw(p.odom_frame, p.laser_frame)
        if laser_pose is None:
            # robot URDF usually mounts the lidar forward of base_link
            off = self.ros.profile.limits.laser_offset_x
            laser_pose = (
                pose[0] + off * math.cos(pose[2]),
                pose[1] + off * math.sin(pose[2]),
                pose[2],
            )
        self._laser_x, self._laser_y, self._laser_yaw = laser_pose

        self.p_map.view.set_robot_pose(*pose)
        self.p_nav.view.set_robot_pose(*pose)
        self.p_mission.update_robot_pose(*pose)
        self.p_3d.update_robot_pose(*pose)
        self.sb_pose.setText(f"pose: x={pose[0]:+.2f} y={pose[1]:+.2f} yaw={pose[2]:+.2f}")
        now = time.monotonic()
        if now - getattr(self, "_progress_t", 0.0) > 0.2:      # 5 Hz is plenty
            self._progress_t = now
            self._update_progress()

    def _on_scan(self, m: dict) -> None:
        self.p_map.view.set_scan(m, self._laser_x, self._laser_y, self._laser_yaw)
        self.p_nav.view.set_scan(m, self._laser_x, self._laser_y, self._laser_yaw)

    def _on_map(self, m: dict) -> None:
        self.p_map.view.set_occupancy(m)
        self.p_nav.view.set_occupancy(m)
        self.p_mission.view.set_occupancy(m)

    def _frame_in_map(self, m: dict):
        """Pose of a message's header frame in the map frame, or None if TF
        doesn't know it yet (the local costmap usually lives in odom)."""
        frame = (m.get("header", {}).get("frame_id") or "").lstrip("/")
        map_frame = self.ros.profile.topics.map_frame if self.ros.profile else "map"
        if not frame or frame == map_frame:
            return (0.0, 0.0, 0.0)
        return self.tf.pose_xyyaw(map_frame, frame)

    def _on_global_costmap(self, m: dict) -> None:
        pose = self._frame_in_map(m)
        if pose is None:
            return
        self.p_map.view.set_global_costmap(m, pose)
        self.p_nav.view.set_global_costmap(m, pose)

    def _on_local_costmap(self, m: dict) -> None:
        pose = self._frame_in_map(m)
        if pose is None:
            return
        self.p_map.view.set_local_costmap(m, pose)
        self.p_nav.view.set_local_costmap(m, pose)

    def _on_plan(self, m: dict) -> None:
        self.p_map.view.set_path(m)
        self.p_nav.view.set_path(m)
        self.p_mission.view.set_path(m)
        poses = m.get("poses", [])
        self._plan_xy = [(p["pose"]["position"]["x"], p["pose"]["position"]["y"]) for p in poses]
        self._update_progress()

    # Distance/time remaining are computed here from /plan and the robot pose:
    # rosbridge (Humble) cannot subscribe to the NavigateToPose feedback topic.
    def _on_nav_state(self, state: str) -> None:
        self._nav_state = state
        if state not in ("active", "pending"):
            self._plan_xy = []
            if state == "succeeded":
                self.p_nav.set_feedback({"distance_remaining": 0.0,
                                         "estimated_time_remaining": {"sec": 0}})
            else:
                self.p_nav.clear_feedback()

    def _update_progress(self) -> None:
        if getattr(self, "_nav_state", "") not in ("active", "pending") or not getattr(self, "_plan_xy", None):
            return
        pts = self._plan_xy
        rx, ry = self._robot_x, self._robot_y
        i = min(range(len(pts)), key=lambda k: (pts[k][0] - rx) ** 2 + (pts[k][1] - ry) ** 2)
        d = math.hypot(pts[i][0] - rx, pts[i][1] - ry)
        d += sum(math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1])
                 for k in range(i, len(pts) - 1))
        v = abs(self._odom_v) if abs(self._odom_v) > 0.05 else (
            self.ros.profile.limits.max_linear if self.ros.profile else 0.4)
        self.p_nav.set_feedback({"distance_remaining": d,
                                 "estimated_time_remaining": {"sec": d / v}})

    # ---- E-stop (software, on the client) ----

    def _set_estop(self, on: bool, publish: bool = True) -> None:
        """Software E-stop. Nothing on the robot consumes /amr/estop, so the
        stop is enforced client-side: cancel the Nav2 goal, stop any mission,
        and stream zero velocity (teleop panel does that while engaged).
        This is not a substitute for the hardware E-stop."""
        if on == getattr(self, "_estop_on", False):
            return
        self._estop_on = on
        self.sidebar_estop.blockSignals(True)
        self.sidebar_estop.setChecked(on)
        self.sidebar_estop.blockSignals(False)
        self.sidebar_estop.setText("● E-STOP ON\nclick to release" if on else "● E-STOP")
        self.p_teleop.set_estop_state(on)
        self.p_status.set_estop(on)
        if on:
            self.sb_estop.setText("● estop: ENGAGED")
            self.sb_estop.setStyleSheet("color:#ff6a6a; font-weight:700;")
            self.p_mission.stop_mission()
            self.ros.cancel_nav_goal()
            self.ros.publish_cmd_vel(0.0, 0.0)
        else:
            self.sb_estop.setText("● estop: clear")
            self.sb_estop.setStyleSheet("color:#39d98a;")
        if publish:
            self.ros.publish_estop(on)

    def _on_estop_remote(self, on: bool) -> None:
        # /amr/estop from another client (e.g. amr_control_center) — mirror it
        self._set_estop(on, publish=False)
