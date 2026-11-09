#!/usr/bin/env python3
"""The robot's own E-STOP: the ROS side of estop_gate.py (read that first).

  /cmd_vel_safe (Twist)          in: Nav2 after the CBF filter, gamepad, RCS teleop
  /rcs/cmd_vel_gated (Twist)     out: what the driver follows (rcs_launcher.py starts
                                 bringup with cmd_vel_topic:=/rcs/cmd_vel_gated)
  /amr/estop (Bool)              true latches the E-STOP, false releases it
  /rcs/app/heartbeat (String)    {"id", "host", "driving", "bye"} from each RCS app
  /rcs/estop/state (String)      the gate's state, JSON, 2 Hz, latched

If this service is not running nothing reaches the drives, so a crash of it
stops the robot instead of letting it drive unguarded. start_rcs_robot.sh
starts it again within a second.

Environment: RCS_CMD_IN, RCS_CMD_OUT (the two Twist topics above),
RCS_NAV_ACTION (default /navigate_to_pose).
"""
from __future__ import annotations

import json
import os
import signal
import socket
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from estop_gate import Gate

CMD_IN = os.environ.get("RCS_CMD_IN", "/cmd_vel_safe")
CMD_OUT = os.environ.get("RCS_CMD_OUT", "/rcs/cmd_vel_gated")
NAV = os.environ.get("RCS_NAV_ACTION", "/navigate_to_pose")
ZERO_HZ = 20.0


class Safety(Node):
    def __init__(self) -> None:
        super().__init__("rcs_safety")
        self.gate = Gate(Path(os.environ.get("RCS_ESTOP_FILE", str(Path(__file__).with_name("estop.json")))))
        self.nav_active = False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.out = self.create_publisher(Twist, CMD_OUT, 10)
        self.state_pub = self.create_publisher(String, "/rcs/estop/state", latched)
        self.create_subscription(Twist, CMD_IN, self._on_cmd, 10)
        self.create_subscription(Bool, "/amr/estop", self._on_estop, 10)
        self.create_subscription(String, "/rcs/app/heartbeat", self._on_heartbeat, 20)
        self.create_subscription(GoalStatusArray, f"{NAV}/_action/status", self._on_nav, 10)
        self.cancel = self.create_client(CancelGoal, f"{NAV}/_action/cancel_goal")
        self.create_timer(1.0 / ZERO_HZ, self._hold)
        self.create_timer(0.25, self._tick)
        self.create_timer(0.5, self._publish_state)
        self.get_logger().info(f"E-STOP gate: {CMD_IN} -> {CMD_OUT}"
                               + (", ENGAGED (kept from before)" if self.gate.engaged else ""))
        if self.gate.engaged:
            self._cancel_nav()

    def _on_cmd(self, m: Twist) -> None:
        lin, ang = self.gate.filter(m.linear.x, m.angular.z)
        if (lin, ang) == (m.linear.x, m.angular.z):
            self.out.publish(m)
        else:
            self.out.publish(Twist())

    def _hold(self) -> None:
        """While stopped, zero goes out on its own: the driver follows the last
        message, and no one else may get through in between."""
        if self.gate.blocking():
            self.out.publish(Twist())

    def _on_estop(self, m: Bool) -> None:
        was = self.gate.engaged
        acts = self.gate.estop(bool(m.data), by="/amr/estop")
        if self.gate.engaged != was:
            self.get_logger().warn("E-STOP ENGAGED" if m.data else "E-STOP released")
        self._do(acts)
        self._publish_state()

    def _on_heartbeat(self, m: String) -> None:
        try:
            msg = json.loads(m.data)
        except ValueError:
            return
        if isinstance(msg, dict):
            self.gate.heartbeat(msg)

    def _on_nav(self, m: GoalStatusArray) -> None:
        self.nav_active = any(s.status in (1, 2) for s in m.status_list)    # accepted, executing

    def _tick(self) -> None:
        acts = self.gate.tick(self.nav_active)
        if acts and self.gate.last_stop and not self.gate.engaged:
            self.get_logger().warn(f"stopping: {self.gate.last_stop['reason']}")
        self._do(acts)

    def _do(self, acts: list) -> None:
        if "cancel_nav" in acts:
            self._cancel_nav()
            self.out.publish(Twist())

    def _cancel_nav(self) -> None:
        """A zero goal id and stamp cancels every goal of the action."""
        if self.cancel.service_is_ready():
            self.cancel.call_async(CancelGoal.Request())

    def _publish_state(self) -> None:
        st = self.gate.state()
        st["host"] = socket.gethostname()
        self.state_pub.publish(String(data=json.dumps(st)))


def _stop(*_args) -> None:
    raise KeyboardInterrupt


def main() -> None:
    # Our own Ctrl-C / kill handling: rclpy's would shut ROS down first, and
    # the last zero below could no longer go out.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:         # older rclpy
        rclpy.init()
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    node = Safety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.out.publish(Twist())       # stopped, not left on the last command
        except Exception:  # noqa: BLE001 - ROS already gone
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
