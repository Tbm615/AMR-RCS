#!/bin/bash
# Bring up a robot-less test stack on this PC so RCS can be driven end to end:
#   real rosbridge_server (port 9090) + this workspace's Nav2 launch (AMCL,
#   SmacPlanner2D + MPPI, velocity smoother, CBF safety filter) on a real map
#   + sim/fake_base.py standing in for the drive/lidar.
#
# Usage: sim/run_sim.sh [map.yaml] [x] [y] [yaw]
# Env:   AMR_WS     robot workspace (default: ../.. relative to RCS)
#        BRIDGE_WS  workspace with rosbridge_suite if not installed via apt
#        ROS_DOMAIN_ID (default 77, isolated from the real robot)
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
AMR_WS="${AMR_WS:-$(cd "$HERE/../.." && pwd)}"
MAP="${1:-$AMR_WS/src/navigation/maps/map23_04.yaml}"
X="${2:-3.55}"; Y="${3:--5.10}"; YAW="${4:--1.64}"

# Isolate from the real robot: own domain, loopback only. Use Fast DDS here:
# it works on loopback out of the box, while CycloneDDS needs multicast or a
# peer list (and the user's Cyclone config points at the real robot).
export ROS_DOMAIN_ID="${SIM_DOMAIN_ID:-77}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
source /opt/ros/humble/setup.bash
source "$AMR_WS/install/setup.bash"
[ -n "$BRIDGE_WS" ] && source "$BRIDGE_WS/install/setup.bash"

LOG="${SIM_LOG_DIR:-/tmp/rcs_sim}"
mkdir -p "$LOG"
pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT INT TERM

python3 "$HERE/fake_base.py" --map "$MAP" --x "$X" --y "$Y" --yaw "$YAW" > "$LOG/fake_base.log" 2>&1 &
pids+=($!)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 > "$LOG/rosbridge.log" 2>&1 &
pids+=($!)
ros2 launch navigation robot_navigation.launch.py map:="$MAP" rviz:=false use_cbf:=true \
    initial_pose_x:="$X" initial_pose_y:="$Y" initial_pose_yaw:="$YAW" > "$LOG/nav.log" 2>&1 &
pids+=($!)

echo "sim up (domain $ROS_DOMAIN_ID): rosbridge ws://127.0.0.1:9090, logs in $LOG"
wait
