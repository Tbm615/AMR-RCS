#!/bin/bash
# Stop everything sim/run_sim.sh started (and stale copies of it).
pats='run_sim[.]sh|fake_base[.]py|rosbridge_websocket|rosapi_node|robot_navigation[.]launch|component_container_isolated|cbf_safety_filter|lib/nav2_|lib/amcl|map_server|lifecycle_manager|bt_navigator|controller_server|planner_server|behavior_server|smoother_server|velocity_smoother|waypoint_follower'
pids=$(ps -eo pid=,args= | grep -E "$pats" | grep -v -e grep -e stop_sim | awk '{print $1}')
[ -z "$pids" ] && { echo "sim not running"; exit 0; }
kill $pids 2>/dev/null
sleep 3
left=$(ps -eo pid=,args= | grep -E "$pats" | grep -v -e grep -e stop_sim | awk '{print $1}')
[ -n "$left" ] && kill -9 $left 2>/dev/null
echo "sim stopped"
