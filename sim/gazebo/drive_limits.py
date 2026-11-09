#!/usr/bin/env python3
"""What diff_drive_controller does on the robot, in front of Gazebo's plugin.

Reads the robot's E-STOP gate output (param cmd_topic, /rcs/cmd_vel_gated as on
the robot; /cmd_vel_safe without the gate) and drives /sim/cmd_vel with the limits of
bringup/config/controllers.yaml. Most important is cmd_vel_timeout: Gazebo's
plugin keeps the last command for ever, so without this a lost connection would
look far worse in simulation than on the robot.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

CMD_TIMEOUT = 0.5
MAX_V, MAX_ACC_V = 1.5, 3.0
MAX_W, MAX_ACC_W = 2.2, 4.0
RATE = 100.0


def _step(value: float, target: float, max_change: float) -> float:
    return value + max(-max_change, min(max_change, target - value))


class DriveLimits(Node):
    def __init__(self):
        super().__init__("sim_drive_limits")
        self.target = (0.0, 0.0)
        self.v = self.w = 0.0
        self.last_cmd = None
        self.pub = self.create_publisher(Twist, "/sim/cmd_vel", 10)
        topic = self.declare_parameter("cmd_topic", "/rcs/cmd_vel_gated").value
        self.create_subscription(Twist, topic, self._on_cmd, 10)
        self.get_logger().info(f"following {topic}")
        self.create_timer(1.0 / RATE, self._tick)

    def _on_cmd(self, msg: Twist) -> None:
        self.target = (max(-MAX_V, min(MAX_V, msg.linear.x)), max(-MAX_W, min(MAX_W, msg.angular.z)))
        self.last_cmd = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        stale = self.last_cmd is None or (now - self.last_cmd).nanoseconds * 1e-9 > CMD_TIMEOUT
        v, w = (0.0, 0.0) if stale else self.target
        self.v = _step(self.v, v, MAX_ACC_V / RATE)
        self.w = _step(self.w, w, MAX_ACC_W / RATE)
        msg = Twist()
        msg.linear.x, msg.angular.z = self.v, self.w
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DriveLimits()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
