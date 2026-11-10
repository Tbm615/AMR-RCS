"""Nav2 on a saved map, as rcs_launcher.py starts it: one launch, like the
workspace's robot_navigation.launch.py (map server + AMCL, Nav2, the CBF
safety filter), with the map's keepout and speed zones, and without its
initial pose: robot_navigation.launch.py sends (0, 0, 0) a few seconds after
the start, which could overwrite the pose the operator had just set.

    ros2 launch nav_map.launch.py map:=<map.yaml> params_file:=<nav2.yaml> cbf_params_file:=<cbf.yaml>
        [keepout_mask:=<yaml> speed_mask:=<yaml>] [use_sim_time:=true]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    here = os.path.dirname(os.path.abspath(__file__))
    sim = LaunchConfiguration("use_sim_time")
    params = LaunchConfiguration("params_file")
    keepout = LaunchConfiguration("keepout_mask")
    common = {"use_sim_time": sim, "autostart": "true", "params_file": params, "use_composition": "False",
              "use_respawn": "False", "log_level": "info"}
    return LaunchDescription([
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("cbf_params_file"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("keepout_mask", default_value=""),
        DeclareLaunchArgument("speed_mask", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, "costmap_filters.launch.py")),
            condition=IfCondition(PythonExpression(["'", keepout, "' != ''"])),
            launch_arguments={"keepout_mask": keepout, "speed_mask": LaunchConfiguration("speed_mask"),
                              "use_sim_time": sim}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "localization_launch.py")),
            launch_arguments={**common, "map": LaunchConfiguration("map")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "navigation_launch.py")),
            launch_arguments=common.items()),
        # between Nav2 and the drives: /cmd_vel (velocity smoother) -> CBF -> /cmd_vel_safe
        Node(package="navigation", executable="cbf_safety_filter.py", name="cbf_safety_filter", output="screen",
             parameters=[LaunchConfiguration("cbf_params_file"), {"use_sim_time": sim}],
             remappings=[("cmd_vel_raw", "/cmd_vel"), ("cmd_vel", "/cmd_vel_safe"),
                         ("scan", "/scan_filtered"), ("odom", "/odom")]),
    ])
