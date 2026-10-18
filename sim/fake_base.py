#!/usr/bin/env python3
"""Kinematic stand-in for the Hinson base, for testing RCS + Nav2 without
hardware. Subscribes /cmd_vel_safe, integrates a unicycle model and publishes
what the real robot stack would: /odom, odom->base_link TF, static
base_link->{base_footprint,laser_frame} TFs, and /scan_filtered ray-cast
against an occupancy map (so AMCL and the costmaps see real-looking data).

Run with the system ROS 2 python (needs rclpy):
    python3 fake_base.py --map <map.yaml> --x 0 --y 0 --yaw 0
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

LASER_OFFSET_X = 0.40      # same as bringup.launch.py / URDF
CMD_TIMEOUT = 0.5          # s, matches the real driver chain's stale cutoff
N_BEAMS = 720
MAX_RANGE = 12.0


def _quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class FakeBase(Node):
    def __init__(self, map_yaml: str, x: float, y: float, yaw: float, cmd_topic: str):
        super().__init__("fake_base")
        meta = yaml.safe_load(Path(map_yaml).read_text())
        img = np.array(Image.open(Path(map_yaml).parent / meta["image"]))
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        # occupied cells, flipped so row 0 is the map's lowest y
        occ = (255 - img.astype(np.float32)) / 255.0 > float(meta["occupied_thresh"])
        self.occ = occ[::-1, :]
        self.h, self.w = self.occ.shape

        self.start = (x, y, yaw)
        self.pose = [x, y, yaw]          # ground truth in the map frame
        self.v = 0.0
        self.w_z = 0.0
        self.last_cmd = 0.0

        self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan_filtered", QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([
            self._tf("base_link", "base_footprint", 0.0, 0.0, 0.0),
            self._tf("base_link", "laser_frame", LASER_OFFSET_X, 0.0, 0.0),
        ])

        ang = np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False)
        self.beam_angles = ang
        self.steps = np.arange(0.0, MAX_RANGE, self.res * 0.5)

        self.t_prev = time.monotonic()
        self.create_timer(0.02, self._step)     # 50 Hz odom/TF
        self.create_timer(0.1, self._scan)      # 10 Hz scan
        self.get_logger().info(f"fake base at ({x:.2f}, {y:.2f}, {yaw:.2f}) on {map_yaml}, cmd={cmd_topic}")

    def _tf(self, parent, child, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        q = _quat(yaw)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        return t

    def _on_cmd(self, m: Twist) -> None:
        self.v = float(m.linear.x)
        self.w_z = float(m.angular.z)
        self.last_cmd = time.monotonic()

    def _odom_pose(self):
        """Perfect odometry: pose relative to where the robot started."""
        sx, sy, syaw = self.start
        x, y, yaw = self.pose
        dx, dy = x - sx, y - sy
        c, s = math.cos(-syaw), math.sin(-syaw)
        return c * dx - s * dy, s * dx + c * dy, yaw - syaw

    def _step(self) -> None:
        now = time.monotonic()
        dt = now - self.t_prev
        self.t_prev = now
        if now - self.last_cmd > CMD_TIMEOUT:
            self.v = 0.0
            self.w_z = 0.0
        x, y, yaw = self.pose
        yaw += self.w_z * dt
        nx = x + self.v * math.cos(yaw) * dt
        ny = y + self.v * math.sin(yaw) * dt
        if not self._hits(nx, ny):        # crude collision stop
            x, y = nx, ny
        self.pose = [x, y, yaw]

        ox, oy, oyaw = self._odom_pose()
        stamp = self.get_clock().now().to_msg()
        tf = self._tf("odom", "base_link", ox, oy, oyaw)
        tf.header.stamp = stamp
        self.tf.sendTransform(tf)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = ox
        od.pose.pose.position.y = oy
        q = _quat(oyaw)
        o = od.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w_z
        self.odom_pub.publish(od)

    def _hits(self, x: float, y: float) -> bool:
        c = int((x - self.ox) / self.res)
        r = int((y - self.oy) / self.res)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return True
        return bool(self.occ[r, c])

    def _scan(self) -> None:
        x, y, yaw = self.pose
        lx = x + LASER_OFFSET_X * math.cos(yaw)
        ly = y + LASER_OFFSET_X * math.sin(yaw)
        a = self.beam_angles + yaw
        px = lx + np.outer(np.cos(a), self.steps)
        py = ly + np.outer(np.sin(a), self.steps)
        cols = ((px - self.ox) / self.res).astype(np.int32)
        rows = ((py - self.oy) / self.res).astype(np.int32)
        inside = (rows >= 0) & (rows < self.h) & (cols >= 0) & (cols < self.w)
        hit = np.zeros_like(inside)
        hit[inside] = self.occ[rows[inside], cols[inside]]
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        ranges = np.where(first >= 0, self.steps[np.maximum(first, 0)], np.inf).astype(np.float32)

        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "laser_frame"
        m.angle_min = float(self.beam_angles[0])
        m.angle_increment = float(self.beam_angles[1] - self.beam_angles[0])
        m.angle_max = float(m.angle_min + m.angle_increment * (N_BEAMS - 1))
        m.scan_time = 0.1
        m.range_min = 0.05
        m.range_max = MAX_RANGE
        m.ranges = ranges.tolist()
        self.scan_pub.publish(m)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--cmd-topic", default="/cmd_vel_safe")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = FakeBase(args.map, args.x, args.y, args.yaw, args.cmd_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
