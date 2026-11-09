#!/usr/bin/env python3
"""The robot's own URDF (src/bringup) made ready for Gazebo, without touching it.

Same links, meshes, masses, sensor positions and TF tree as the real robot.
Changed for the simulator only:
  - the ros2_control block goes (the Kinco hardware plugin needs the real CAN bus)
  - mesh collisions become simple shapes: a box for the body and four
    low-friction spheres for the casters, which are fixed joints in the URDF
    and would otherwise stop the robot from turning
  - the drive wheels touch the floor where the tyres do, at +-wheel_separation/2
    (controllers.yaml), whatever the wheel's frame in the URDF: a 4 cm cylinder
    of the real radius, placed from the joint. Off-centre contacts make the
    simulated robot turn about the wrong point and its odometry drift
  - a wheel whose axis points -y in base_link is flipped: Gazebo's plugin wants
    both to agree (positive = forward), as the drives do on the robot
  - added: diff drive, lidar, RGB-D camera, joint states, ground truth

    make_urdf.py <hinson_robot.urdf.xacro> <out.urdf>
"""
from __future__ import annotations

import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

WHEEL_RADIUS = 0.0825       # bringup/config/controllers.yaml
WHEEL_SEPARATION = 0.39
WHEEL_WIDTH = 0.04          # tyre: inner 35 cm, outer 43 cm apart
DRIVE_WHEELS = ("left_wheel_joint", "right_wheel_joint")

GAZEBO = f"""
<robot>
  <gazebo reference="base_link">
    <material>Gazebo/DarkGrey</material>
    <mu1>0.0</mu1><mu2>0.0</mu2>
  </gazebo>
  <gazebo reference="left_wheel_link">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1><mu2>1.0</mu2><kp>1000000</kp><kd>100</kd><minDepth>0.001</minDepth><maxVel>0.1</maxVel>
  </gazebo>
  <gazebo reference="right_wheel_link">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1><mu2>1.0</mu2><kp>1000000</kp><kd>100</kd><minDepth>0.001</minDepth><maxVel>0.1</maxVel>
  </gazebo>

  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros><remapping>cmd_vel:=/sim/cmd_vel</remapping><remapping>odom:=/odom</remapping></ros>
      <update_rate>100</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>{WHEEL_SEPARATION}</wheel_separation>
      <wheel_diameter>{2 * WHEEL_RADIUS}</wheel_diameter>
      <max_wheel_torque>80</max_wheel_torque>
      <max_wheel_acceleration>12</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odometry_source>0</odometry_source>
    </plugin>
    <plugin name="joint_states" filename="libgazebo_ros_joint_state_publisher.so">
      <update_rate>30</update_rate>
      <joint_name>left_wheel_joint</joint_name>
      <joint_name>right_wheel_joint</joint_name>
    </plugin>
    <plugin name="ground_truth" filename="libgazebo_ros_p3d.so">
      <ros><remapping>odom:=/sim/ground_truth</remapping></ros>
      <body_name>base_link</body_name>
      <frame_name>world</frame_name>
      <update_rate>20</update_rate>
    </plugin>
  </gazebo>

  <!-- HINS LE: 4995 points, 360 degrees, 20 Hz, as on the robot -->
  <gazebo reference="laser_frame">
    <sensor name="hins_lidar" type="gpu_ray">
      <always_on>true</always_on>
      <update_rate>20</update_rate>
      <visualize>false</visualize>
      <ray>
        <scan><horizontal>
          <samples>4995</samples><resolution>1</resolution>
          <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle>
        </horizontal></scan>
        <range><min>0.05</min><max>30.0</max><resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
      </ray>
      <plugin name="lidar" filename="libgazebo_ros_ray_sensor.so">
        <ros><remapping>~/out:=/scan</remapping></ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_frame</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Orbbec Astra: 640x480 RGB-D. sim/gazebo/camera_relay.py turns the raw
       output into the topics and depth format of the real driver. -->
  <gazebo reference="camera_link">
    <sensor name="astra" type="depth">
      <always_on>true</always_on>
      <update_rate>15</update_rate>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>1.02</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.15</near><far>10.0</far></clip>
      </camera>
      <plugin name="astra" filename="libgazebo_ros_camera.so">
        <camera_name>sim_camera</camera_name>
        <frame_name>camera_depth_optical_frame</frame_name>
        <hack_baseline>0</hack_baseline>
        <min_depth>0.15</min_depth>
        <max_depth>8.0</max_depth>
        <ros><remapping>sim_camera/points:=/camera/depth_registered/points</remapping></ros>
      </plugin>
    </sensor>
  </gazebo>
</robot>
"""


def _collision(xyz: str, rpy: str, shape: str, **attrs) -> ET.Element:
    c = ET.Element("collision")
    ET.SubElement(c, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(ET.SubElement(c, "geometry"), shape, attrs)
    return c


def _rpy(r: float, p: float, y: float):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _floats(text: str):
    return [float(v) for v in text.split()]


def wheel_contact(joint: ET.Element):
    """(collision xyz, collision rpy, axis xyz) in the wheel link's frame: the
    tyre at y = +-WHEEL_SEPARATION/2 of base_link, axis pointing +y there."""
    o = joint.find("origin")
    xyz = _floats(o.get("xyz", "0 0 0")) if o is not None else [0.0, 0.0, 0.0]
    rot = _rpy(*_floats(o.get("rpy", "0 0 0"))) if o is not None else _rpy(0, 0, 0)
    axis = _floats(joint.find("axis").get("xyz", "1 0 0")) if joint.find("axis") is not None else [1.0, 0, 0]
    side = 1.0 if xyz[1] > 0 else -1.0
    shift = [0.0, side * WHEEL_SEPARATION / 2 - xyz[1], 0.0]          # in base_link
    local = [sum(rot[r][c] * shift[r] for r in range(3)) for c in range(3)]   # rot^T @ shift
    axis_base_y = sum(rot[1][c] * axis[c] for c in range(3))
    if axis_base_y < 0:
        axis = [-a for a in axis]
    big = max(range(3), key=lambda i: abs(axis[i]))
    cyl_rpy = ("0 1.5708 0", "1.5708 0 0", "0 0 0")[big]    # cylinder z along the axle
    return " ".join(f"{v:.5f}" for v in local), cyl_rpy, " ".join(f"{a:g}" for a in axis)


def convert(xacro_file: str) -> ET.Element:
    urdf = subprocess.run(["xacro", xacro_file], check=True, capture_output=True, text=True).stdout
    robot = ET.fromstring(urdf)
    for tag in robot.findall("ros2_control"):
        robot.remove(tag)
    links = {link.get("name"): link for link in robot.findall("link")}
    for link in links.values():
        for c in link.findall("collision"):
            link.remove(c)

    base = links["base_link"]
    # body as in base_link.STL, and casters 2 mm clear of the floor so the weight
    # stays on the drive wheels
    base.append(_collision("0.005 0 0.0875", "0 0 0", "box", size="0.872 0.55 0.175"))
    for x in (0.315, -0.315):
        for y in (0.164, -0.164):
            base.append(_collision(f"{x} {y} -0.012", "0 0 0", "sphere", radius="0.03"))
    for joint in robot.findall("joint"):
        if joint.get("name") in DRIVE_WHEELS:
            xyz, rpy, axis = wheel_contact(joint)
            links[joint.find("child").get("link")].append(
                _collision(xyz, rpy, "cylinder", radius=str(WHEEL_RADIUS), length=str(WHEEL_WIDTH)))
            joint.find("axis").set("xyz", axis)
    for element in ET.fromstring(GAZEBO):
        robot.append(element)
    return robot


def main() -> None:
    robot = convert(sys.argv[1])
    ET.indent(robot)
    Path(sys.argv[2]).write_text(ET.tostring(robot, encoding="unicode"))
    print(f"{sys.argv[2]}: {len(robot.findall('link'))} links")


if __name__ == "__main__":
    main()
