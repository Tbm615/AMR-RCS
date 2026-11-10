"""Keepout and speed zones for Nav2 (started by rcs_launcher.py next to Nav2).

    ros2 launch costmap_filters.launch.py keepout_mask:=<yaml> speed_mask:=<yaml> [use_sim_time:=true]

Two map servers publish the masks written by zone_masks.py, two filter info
servers tell the costmaps' KeepoutFilter and SpeedFilter where they are. The
masks can be reloaded live: <name>_mask_server/load_map.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = LaunchConfiguration("use_sim_time")
    nodes = ["keepout_mask_server", "keepout_filter_info_server", "speed_mask_server", "speed_filter_info_server"]
    common = {"use_sim_time": sim}
    return LaunchDescription([
        DeclareLaunchArgument("keepout_mask"),
        DeclareLaunchArgument("speed_mask"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="nav2_map_server", executable="map_server", name="keepout_mask_server", output="screen",
             parameters=[common, {"yaml_filename": LaunchConfiguration("keepout_mask"),
                                  "topic_name": "/keepout_filter_mask", "frame_id": "map"}]),
        Node(package="nav2_map_server", executable="costmap_filter_info_server", name="keepout_filter_info_server",
             output="screen",
             parameters=[common, {"type": 0, "filter_info_topic": "/costmap_filter_info",
                                  "mask_topic": "/keepout_filter_mask", "base": 0.0, "multiplier": 1.0}]),
        Node(package="nav2_map_server", executable="map_server", name="speed_mask_server", output="screen",
             parameters=[common, {"yaml_filename": LaunchConfiguration("speed_mask"),
                                  "topic_name": "/speed_filter_mask", "frame_id": "map"}]),
        Node(package="nav2_map_server", executable="costmap_filter_info_server", name="speed_filter_info_server",
             output="screen",
             parameters=[common, {"type": 1, "filter_info_topic": "/speed_filter_info",    # 1: percent
                                  "mask_topic": "/speed_filter_mask", "base": 0.0, "multiplier": 1.0}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_zones",
             output="screen", parameters=[common, {"autostart": True, "node_names": nodes}]),
    ])
