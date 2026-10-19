#!/bin/bash
# rosbridge on port 9090 for RCS (Robot Control System). Started at boot by
# the user crontab (@reboot); restarts rosbridge if it exits.
# Log: ~/tbm_test/runlog/rosbridge.log
source /opt/ros/humble/setup.bash
source /home/iop/tbm_test/install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/iop/tbm_test/install/bringup/share/bringup/config/cyclonedds.xml
LOG=/home/iop/tbm_test/runlog/rosbridge.log
mkdir -p "$(dirname "$LOG")"
while true; do
    echo "=== $(date) starting rosbridge" >> "$LOG"
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 >> "$LOG" 2>&1
    echo "=== $(date) rosbridge exited ($?), restarting in 5 s" >> "$LOG"
    tail -n 2000 "$LOG" > "$LOG.tmp" && mv "$LOG.tmp" "$LOG"
    sleep 5
done
