#!/bin/bash
# Robot side of RCS (Robot Control System). Started at boot by the user
# crontab:   @reboot sleep 10 && /home/iop/tbm_test/rcs_robot/start_rcs_robot.sh
# Runs, each restarted if it exits:
#   - rosbridge_server on port 9090 (the app connects here)
#   - rcs_launcher.py (lets the app start/stop bringup and Nav2)
#   - rcs_battery.py (keeps the Bluetooth link to the JK-BMS and writes the
#     battery log, one file per boot in $RCS_WS/battery_log, see that file)
#   - rcs_safety.py (the robot's E-STOP: every velocity command passes it, see
#     estop_gate.py). The drives only move while it runs.
#   - rcs_gateway.py, once a user exists ($RCS_WS/rcs_users.json): sign-in and
#     roles on port 9090, rosbridge moved to 127.0.0.1:9091 behind it. Without a
#     user, rosbridge stays on 9090 open to the network, as before.
# Logs: $RCS_WS/runlog/<name>.log
export RCS_WS="${RCS_WS:-/home/iop/tbm_test}"
source /opt/ros/humble/setup.bash
source "$RCS_WS/install/setup.bash"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$RCS_WS/install/bringup/share/bringup/config/cyclonedds.xml"
LOGS="$RCS_WS/runlog"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$LOGS"

keep_running() {   # keep_running <log> <pause s> <command...>
    local log="$1" pause="$2"; shift 2
    while true; do
        echo "=== $(date) starting: $*" >> "$log"
        "$@" >> "$log" 2>&1
        echo "=== $(date) exited ($?), restarting in $pause s" >> "$log"
        tail -n 2000 "$log" > "$log.tmp" && mv "$log.tmp" "$log"
        sleep "$pause"
    done
}

USERS="${RCS_USERS:-$RCS_WS/rcs_users.json}"
if [ -s "$USERS" ] && grep -q '"role"' "$USERS"; then
    BRIDGE_ARGS="port:=9091 address:=127.0.0.1"     # only the gateway reaches it
    GATEWAY=1
else
    BRIDGE_ARGS="port:=9090"
    GATEWAY=0
fi

rosbridge() {
    # A rosbridge left behind (it ignores SIGTERM: pkill without -INT) keeps
    # its port with its ROS side shut down: the app would connect to it and
    # get nothing, and this one could not start. Clear it out first.
    if pkill -INT -f "rosbridge_server/rosbridge_websocket|rosapi/rosapi_node"; then
        sleep 3
        pkill -KILL -f "rosbridge_server/rosbridge_websocket|rosapi/rosapi_node"
    fi
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml $BRIDGE_ARGS &
    local launch=$!
    # The launch also runs rosapi and does not end when rosbridge does: after a
    # rosbridge crash it went on with rosapi alone, nothing listened on the port,
    # and no app could reach the robot until a reboot. Watch rosbridge itself.
    sleep 15
    while kill -0 "$launch" 2>/dev/null; do
        if ! pgrep -f "rosbridge_server/rosbridge_websocket" > /dev/null; then
            echo "=== $(date) rosbridge_websocket is gone: stopping the launch to start it again"
            kill -INT "$launch" 2>/dev/null
            sleep 8
            kill -KILL "$launch" 2>/dev/null
            pkill -KILL -f "rosapi/rosapi_node"
            break
        fi
        sleep 2
    done
    wait "$launch"
}

# the gate first: bringup is refused until it runs
keep_running "$LOGS/rcs_safety.log" 1 python3 -u "$HERE/rcs_safety.py" &
keep_running "$LOGS/rcs_launcher.log" 5 python3 -u "$HERE/rcs_launcher.py" &
keep_running "$LOGS/rosbridge.log" 5 rosbridge &
keep_running "$LOGS/rcs_battery.log" 1 python3 -u "$HERE/rcs_battery.py" &
if [ "$GATEWAY" = 1 ]; then
    keep_running "$LOGS/rcs_gateway.log" 2 python3 -u "$HERE/rcs_gateway.py" serve &
fi
wait
