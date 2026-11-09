#!/usr/bin/env python3
"""Gazebo's camera output in the shape the Orbbec driver gives on the robot.

Gazebo gives depth as 32-bit float metres; the Astra gives 16-bit millimetres
with 0 for "no reading". image_transport's republish (started next to this node
in sim_bringup.launch.py) then makes the same /compressed and /compressedDepth
topics the robot has, with the same encoder.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraRelay(Node):
    def __init__(self):
        super().__init__("sim_camera_relay")
        self.depth_pub = self.create_publisher(Image, "/sim/depth16", 5)
        self.info_pubs = [self.create_publisher(CameraInfo, t, 5)
                          for t in ("/camera/depth/camera_info", "/camera/color/camera_info")]
        self.create_subscription(Image, "/sim_camera/depth/image_raw", self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/sim_camera/camera_info", self._on_info, qos_profile_sensor_data)

    def _on_depth(self, msg: Image) -> None:
        metres = np.frombuffer(bytes(msg.data), np.float32)
        mm = np.nan_to_num(metres, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0
        out = Image()
        out.header = msg.header
        out.height, out.width = msg.height, msg.width
        out.encoding = "16UC1"
        out.step = msg.width * 2
        out.data = np.clip(mm, 0, 65535).astype(np.uint16).tobytes()
        self.depth_pub.publish(out)

    def _on_info(self, msg: CameraInfo) -> None:
        for pub in self.info_pubs:
            pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
