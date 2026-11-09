"""Stands in for bringup.launch.py: same topics and TF, from Gazebo instead of
the hardware. rcs_launcher.py starts this when the app presses Start bringup
(RCS_BRINGUP_CMD, set by run_gazebo.sh). The scan filter is the robot's own node.
"""
import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_TIME = {"use_sim_time": True}


def _nodes(context):
    def arg(name):
        return context.launch_configurations[name]

    urdf = arg("urdf")
    with open(urdf) as f:
        description = f.read()
    server = ["gzserver", "--verbose", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so", arg("world"),
              "--ros-args", "--params-file", f"{HERE}/gazebo_params.yaml"]
    actions = [ExecuteProcess(cmd=server, output="screen")]
    if arg("gui") == "true":
        actions.append(ExecuteProcess(cmd=["gzclient"], output="screen"))

    actions += [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": description}, SIM_TIME]),
        TimerAction(period=3.0, actions=[Node(
            package="gazebo_ros", executable="spawn_entity.py", output="screen",
            arguments=["-entity", "hinson", "-file", urdf, "-x", arg("x"), "-y", arg("y"),
                       "-z", "0.05", "-Y", arg("yaw"), "-timeout", "60"])]),
        Node(package="bringup", executable="scan_median_filter_node", output="screen",
             parameters=[{"robot_length": 0.9, "robot_width": 0.6, "laser_offset_x": 0.4, "laser_offset_y": 0.0,
                          "footprint_margin": 0.05, "enable_median_filter": True, "window_size": 5,
                          "jump_threshold": 0.5}, SIM_TIME]),
        # the drives follow the robot's E-STOP gate, as bringup.launch.py
        # cmd_vel_topic:=/rcs/cmd_vel_gated does on the robot
        ExecuteProcess(cmd=["python3", f"{HERE}/drive_limits.py", "--ros-args", "-p", "use_sim_time:=true",
                            "-p", f"cmd_topic:={arg('cmd_vel_topic')}"], output="screen"),
        ExecuteProcess(cmd=["python3", f"{HERE}/camera_relay.py", "--ros-args", "-p", "use_sim_time:=true"],
                       output="screen"),
    ]
    # The transports the Orbbec driver offers. republish makes only one each:
    # without an output transport it gives raw alone.
    for name, source, topic, transport in (
            ("color_raw", "/sim_camera/image_raw", "/camera/color/image_raw", "raw"),
            ("color_jpeg", "/sim_camera/image_raw", "/camera/color/image_raw", "compressed"),
            ("depth_raw", "/sim/depth16", "/camera/depth/image_raw", "raw"),
            ("depth_png", "/sim/depth16", "/camera/depth/image_raw", "compressedDepth")):
        # Humble's republish sends a compressed transport on out/<transport>: a
        # remap of "out" alone does not reach it (it went to /out/compressed)
        remaps = [("in", source), ("out", topic)]
        if transport != "raw":
            remaps.append((f"out/{transport}", f"{topic}/{transport}"))
        actions.append(Node(package="image_transport", executable="republish", name=f"sim_{name}", output="screen",
                            arguments=["raw", transport], remappings=remaps))
    # the Orbbec driver publishes these on the robot
    for frame in ("camera_depth_optical_frame", "camera_color_optical_frame"):
        actions.append(Node(package="tf2_ros", executable="static_transform_publisher", output="screen",
                            arguments=["--frame-id", "camera_link", "--child-frame-id", frame,
                                       "--roll", str(-math.pi / 2), "--yaw", str(-math.pi / 2)]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world"),
        DeclareLaunchArgument("urdf"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/rcs/cmd_vel_gated"),
        OpaqueFunction(function=_nodes),
    ])
