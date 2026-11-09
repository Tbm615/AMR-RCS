#!/bin/bash
# Stop everything run_gazebo.sh started (and stale copies of it).
pats='gzserver|gzclient|sim_bringup[.]launch|spawn_entity|rcs_launcher[.]py|rcs_safety[.]py|sim_ctl[.]py|drive_limits[.]py|camera_relay[.]py|image_transport/republish|robot_state_publisher|scan_median_filter_node|static_transform_publisher|rosbridge_websocket|rosapi_node|rviz2|robot_navigation[.]launch|slamtoolbox[.]launch|async_slam_toolbox|navigation_launch[.]py|component_container_isolated|cbf_safety_filter|lib/nav2_|map_server|lifecycle_manager|bt_navigator|controller_server|planner_server|behavior_server|smoother_server|velocity_smoother|waypoint_follower'
pids=$(ps -eo pid=,args= | grep -E "$pats" | grep -v -e grep -e stop_gazebo | awk '{print $1}')
[ -z "$pids" ] && { echo "sim not running"; exit 0; }
kill $pids 2>/dev/null
sleep 3
left=$(ps -eo pid=,args= | grep -E "$pats" | grep -v -e grep -e stop_gazebo | awk '{print $1}')
[ -n "$left" ] && kill -9 $left 2>/dev/null
echo "sim stopped"
