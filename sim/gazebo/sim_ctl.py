#!/usr/bin/env python3
"""Presses the buttons of the Robot programs page from a script, through the
same /rcs/launcher/cmd topic: start bringup, start Nav2 on a map, set the
initial pose. run_gazebo.sh uses it so the app opens on a robot ready to drive.

    sim_ctl.py up <map name> <x> <y> <yaw>
    sim_ctl.py box <name> <x> <y>        put a 0.5 m box in the world
    sim_ctl.py remove <name>             take it away
    sim_ctl.py truth <file>              keep "x y yaw" of the real pose in a file
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Ctl(Node):
    def __init__(self):
        super().__init__("sim_ctl")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state = None
        self.create_subscription(String, "/rcs/launcher/state", self._on_state, latched)
        self.cmd = self.create_publisher(String, "/rcs/launcher/cmd", 10)
        self.pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def _on_state(self, msg: String) -> None:
        self.state = json.loads(msg.data)

    def wait(self, what: str, test, timeout: float) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.state and test(self.state):
                print(f"  {what}", flush=True)
                return
        raise SystemExit(f"timed out: {what}\nlast state: {self.state}")

    def send(self, **cmd) -> None:
        self.cmd.publish(String(data=json.dumps(cmd)))

    def start(self, stack: str, timeout: float, **args) -> None:
        self.wait("launcher ready", lambda s: not s["busy"], 30)
        self.send(op="start", stack=stack, **args)
        self.wait(f"{stack} running", lambda s: s["stacks"][stack]["state"] == "running", timeout)

    def initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05
        for _ in range(3):
            self.pose.publish(msg)
            time.sleep(1.0)
        print("  initial pose sent", flush=True)


    def call(self, kind, name: str, request) -> None:
        client = self.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=10):
            raise SystemExit(f"no service {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        result = future.result()
        if result is None or not result.success:
            raise SystemExit(f"{name} failed: {result}")

    def box(self, name: str, x: float, y: float) -> None:
        req = SpawnEntity.Request()
        req.name = name
        req.xml = BOX
        req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = x, y, 0.5
        self.call(SpawnEntity, "/spawn_entity", req)

    def remove(self, name: str) -> None:
        req = DeleteEntity.Request()
        req.name = name
        self.call(DeleteEntity, "/delete_entity", req)

    def truth(self, path: str) -> None:
        def write(msg: Odometry) -> None:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            with open(path + ".tmp", "w") as f:
                f.write(f"{p.x} {p.y} {yaw}")
            os.replace(path + ".tmp", path)
        self.create_subscription(Odometry, "/sim/ground_truth", write, 10)
        rclpy.spin(self)


BOX = """<sdf version="1.7"><model name="box"><static>true</static><link name="l">
  <collision name="c"><geometry><box><size>0.5 0.5 1</size></box></geometry></collision>
  <visual name="v"><geometry><box><size>0.5 0.5 1</size></box></geometry>
    <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Red</name></script></material>
  </visual></link></model></sdf>"""


def main() -> None:
    rclpy.init()
    ctl = Ctl()
    what, args = sys.argv[1], sys.argv[2:]
    if what == "up":
        ctl.start("bringup", 90)
        ctl.start("nav", 120, mode="map", map=args[0])
        time.sleep(8)           # AMCL activates after the lifecycle manager
        ctl.initial_pose(*map(float, args[1:4]))
    elif what == "box":
        ctl.box(args[0], float(args[1]), float(args[2]))
    elif what == "remove":
        ctl.remove(args[0])
    elif what == "truth":
        ctl.truth(args[0])


if __name__ == "__main__":
    main()
