#!/bin/bash
# Virtual robot for RCS: the Hinson model in Gazebo, on the walls of a saved map,
# with the real rosbridge, rcs_launcher.py, scan filter, Nav2 and CBF filter.
# RViz and the RCS app open by themselves, already connected.
#
#   sim/gazebo/run_gazebo.sh                    ready to drive
#   sim/gazebo/run_gazebo.sh --manual           app only; press Start bringup yourself
#   sim/gazebo/run_gazebo.sh --test             run tests/test_gazebo.py in front of you
#   sim/gazebo/run_gazebo.sh --map map23_04     a saved map instead of test_rooms
#   sim/gazebo/run_gazebo.sh --map map_3269 --pose 0 0 0
#   Options: --no-app  --no-rviz  --headless (no Gazebo window)
#
# Ctrl-C (or closing the app) stops everything.
# Env: AMR_WS robot workspace, BRIDGE_WS workspace with rosbridge_suite,
#      SIM_DOMAIN_ID (default 77, isolated from the real robot)
set -eo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
RCS="$(cd "$HERE/../.." && pwd)"
AMR_WS="${AMR_WS:-$(cd "$RCS/.." && pwd)}"
BRIDGE_WS="${BRIDGE_WS:-$(cd "$AMR_WS/.." && pwd)/rcs_sim_ws}"
SIM_DIR="${SIM_DIR:-/tmp/rcs_gazebo}"

MAP=test_rooms; X=""
MODE=up; APP=1; RVIZ=1; GUI=true
while [ $# -gt 0 ]; do
    case "$1" in
        --map) MAP="$2"; shift 2 ;;
        --pose) X="$2"; Y="$3"; YAW="$4"; shift 4 ;;
        --manual) MODE=manual; shift ;;
        --test) MODE=test; APP=0; MAP=test_rooms; shift ;;
        --no-app) APP=0; shift ;;
        --no-rviz) RVIZ=0; shift ;;
        --headless) GUI=false; shift ;;
        -h|--help) sed -n 2,14p "$0"; exit 0 ;;
        *) echo "unknown option $1 (see --help)"; exit 2 ;;
    esac
done

# Stay off the real robot: own domain, loopback only, Fast DDS (see run_sim.sh).
export ROS_DOMAIN_ID="${SIM_DOMAIN_ID:-77}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
source /opt/ros/humble/setup.bash
source "$AMR_WS/install/setup.bash"
[ -f "$BRIDGE_WS/install/setup.bash" ] && source "$BRIDGE_WS/install/setup.bash"
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH="$AMR_WS/install/bringup/share:$GAZEBO_MODEL_PATH"
export GAZEBO_MODEL_DATABASE_URI=""      # no online model lookup at start

PY="$RCS/.venv/bin/python"
[ -x "$PY" ] || { echo "no $PY: create the app's virtualenv first (README)"; exit 1; }
bash "$HERE/stop_gazebo.sh" > /dev/null
mkdir -p "$SIM_DIR/log" "$SIM_DIR/ws/maps" "$SIM_DIR/app/RCS"
ln -sfn "$AMR_WS/install" "$SIM_DIR/ws/install"
if [ "$MAP" = test_rooms ]; then
    python3 "$HERE/make_test_map.py" "$SIM_DIR/ws/maps" "$SIM_DIR"
    [ -n "$X" ] || { X=3.0; Y=4.0; YAW=0.0; }
else
    MAP_YAML="$AMR_WS/install/navigation/share/navigation/maps/$MAP.yaml"
    [ -f "$MAP_YAML" ] || MAP_YAML="$SIM_DIR/ws/maps/$MAP.yaml"
    [ -f "$MAP_YAML" ] || { echo "no map $MAP"; exit 1; }
    python3 "$HERE/make_world.py" "$MAP_YAML" "$SIM_DIR/$MAP.world"
    if [ -z "$X" ]; then
        if [ "$MAP" = map23_04 ]; then X=3.55; Y=-5.10; YAW=-1.64; else X=0.0; Y=0.0; YAW=0.0; fi
    fi
fi
python3 "$HERE/make_urdf.py" "$AMR_WS/install/bringup/share/bringup/urdf/hinson_robot.urdf.xacro" "$SIM_DIR/hinson_gazebo.urdf"

# what the launcher runs for Start bringup, and its own workspace: maps saved in
# the simulator stay out of the real workspace
export RCS_WS="$SIM_DIR/ws"
export RCS_USE_SIM_TIME=1
export RCS_BRINGUP_CMD="ros2 launch $HERE/sim_bringup.launch.py world:=$SIM_DIR/$MAP.world urdf:=$SIM_DIR/hinson_gazebo.urdf x:=$X y:=$Y yaw:=$YAW gui:=$GUI cmd_vel_topic:=/rcs/cmd_vel_gated"

pids=()
cleanup() {
    trap - EXIT INT TERM
    [ ${#pids[@]} -gt 0 ] && kill "${pids[@]}" 2>/dev/null || true
    bash "$HERE/stop_gazebo.sh" > /dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 > "$SIM_DIR/log/rosbridge.log" 2>&1 &
pids+=($!)
# the robot's E-STOP gate, as on the robot: the drives follow what it lets through
RCS_ESTOP_FILE="$SIM_DIR/estop.json" python3 -u "$RCS/robot/rcs_safety.py" > "$SIM_DIR/log/rcs_safety.log" 2>&1 &
pids+=($!)
python3 -u "$RCS/robot/rcs_launcher.py" > "$SIM_DIR/log/rcs_launcher.log" 2>&1 &
pids+=($!)
echo "rosbridge ws://127.0.0.1:9090, E-STOP gate and rcs_launcher up (domain $ROS_DOMAIN_ID), logs in $SIM_DIR"

if [ "$RVIZ" = 1 ]; then
    rviz2 -d "$AMR_WS/install/navigation/share/navigation/rviz/navigation.rviz" \
        --ros-args -p use_sim_time:=true > "$SIM_DIR/log/rviz.log" 2>&1 &
    pids+=($!)
fi

case "$MODE" in
    up)
        echo "starting the robot and Nav2 on $MAP..."
        python3 "$HERE/sim_ctl.py" up "$MAP" "$X" "$Y" "$YAW"
        ;;
    test)
        echo "running tests/test_gazebo.py..."
        rc=0
        QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" "$PY" "$RCS/tests/test_gazebo.py" || rc=$?
        exit $rc
        ;;
esac

if [ "$APP" = 1 ]; then
    echo "opening RCS (its settings are kept apart from your real ones, in $SIM_DIR/app)"
    cd "$RCS"
    XDG_CONFIG_HOME="$SIM_DIR/app" "$PY" -m rcs --autoconnect --host 127.0.0.1:9090 \
        > "$SIM_DIR/log/app.log" 2>&1 || true
else
    echo "sim is up; Ctrl-C to stop"
    wait
fi
