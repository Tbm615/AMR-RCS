#!/bin/bash
# Robot side of RCS (Robot Control System). Started at boot by the user
# crontab:   @reboot sleep 10 && /home/iop/tbm_test/rcs_robot/start_rcs_robot.sh
# Runs, each restarted if it exits:
#   - rosbridge_server on port 9090 (the app connects here)
#   - rcs_launcher.py (lets the app start/stop bringup and Nav2)
# Logs: $RCS_WS/runlog/rosbridge.log, $RCS_WS/runlog/rcs_launcher.log
export RCS_WS="${RCS_WS:-/home/iop/tbm_test}"
source /opt/ros/humble/setup.bash
source "$RCS_WS/install/setup.bash"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$RCS_WS/install/bringup/share/bringup/config/cyclonedds.xml"
LOGS="$RCS_WS/runlog"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$LOGS"

keep_running() {   # keep_running <log> <command...>
    local log="$1"; shift
    while true; do
        echo "=== $(date) starting: $*" >> "$log"
        "$@" >> "$log" 2>&1
        echo "=== $(date) exited ($?), restarting in 5 s" >> "$log"
        tail -n 2000 "$log" > "$log.tmp" && mv "$log.tmp" "$log"
        sleep 5
    done
}

keep_running "$LOGS/rcs_launcher.log" python3 -u "$HERE/rcs_launcher.py" &
keep_running "$LOGS/rosbridge.log" \
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
wait
