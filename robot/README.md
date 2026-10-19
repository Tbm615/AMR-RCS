# Robot side of RCS

RCS needs two programs on the robot, both started by
[`start_rcs_robot.sh`](start_rcs_robot.sh):

| Program | What it does |
|---|---|
| `rosbridge_server` (port 9090) | exposes the robot's ROS 2 topics to the app over WebSocket |
| [`rcs_launcher.py`](rcs_launcher.py) | starts and stops bringup and Nav2 when the app asks (Robot programs page), saves maps |

The script sources ROS and the workspace (`RCS_WS`, default
`/home/iop/tbm_test`), sets the robot's DDS settings (CycloneDDS, domain 0,
`bringup/config/cyclonedds.xml`) and restarts either program if it exits.

## Install

```bash
sudo apt install ros-humble-rosbridge-suite ros-humble-compressed-image-transport
mkdir -p ~/tbm_test/rcs_robot && cp robot/* ~/tbm_test/rcs_robot/
chmod +x ~/tbm_test/rcs_robot/*
(crontab -l 2>/dev/null; echo "@reboot sleep 10 && $HOME/tbm_test/rcs_robot/start_rcs_robot.sh") | crontab -
nohup setsid ~/tbm_test/rcs_robot/start_rcs_robot.sh >/dev/null 2>&1 &    # start now
```

The crontab entry needs no sudo. To use a systemd unit instead, see the end
of this file.

Check from the operator PC: `curl -i http://<robot-ip>:9090` answers with
any HTTP code (400 or 426), and the Robot programs page says **Robot
launcher online**.

## What the launcher runs

| App button | Command on the robot |
|---|---|
| Robot base → Start | `ros2 launch bringup bringup.launch.py` |
| Navigation, saved map | `ros2 launch navigation robot_navigation.launch.py map:=<map>.yaml use_cbf:=true rviz:=false` |
| Navigation, SLAM | `ros2 launch slam slamtoolbox.launch.xml`, `ros2 launch nav2_bringup navigation_launch.py params_file:=<navigation>/config/nav2_smac_mppi.yml`, `ros2 run navigation cbf_safety_filter.py` (remapped as in `robot_navigation.launch.py`) |
| Save map | `ros2 run nav2_map_server map_saver_cli -f $RCS_WS/maps/<name>` |

Maps are listed from `<workspace>/install/navigation/share/navigation/maps`
and `$RCS_WS/maps` (saved maps).

Rules: navigation needs the base; stopping the base stops navigation first;
starting navigation while it runs restarts it with the new map or mode. If
one process of a stack exits on its own, the rest of the stack is stopped
and the stack shows *failed* with its last log lines. Stop is Ctrl-C
(SIGINT) to the whole process group, then SIGTERM and SIGKILL if needed.
If the launcher itself restarts, it stops the programs it had started.

Protocol (JSON in `std_msgs/String`): commands on `/rcs/launcher/cmd`, state
on `/rcs/launcher/state` (1 Hz and on every change, transient local). See
the docstring at the top of `rcs_launcher.py`.

## Logs (on the robot)

| File | Content |
|---|---|
| `~/tbm_test/runlog/rosbridge.log` | rosbridge |
| `~/tbm_test/runlog/rcs_launcher.log` | launcher: commands received and their result |
| `~/tbm_test/runlog/rcs_bringup.log` | bringup output (capped at 20 MB) |
| `~/tbm_test/runlog/rcs_nav.log` | navigation output (capped at 20 MB) |

## Stop or remove

- Stop until the next boot: `pkill -f start_rcs_robot.sh; pkill -f rcs_launcher.py; pkill -f rosbridge_websocket`
- Remove: delete the line with `crontab -e`.

## systemd unit (alternative, needs sudo)

`/etc/systemd/system/rcs-robot.service`:

```ini
[Unit]
Description=RCS robot side (rosbridge + launcher)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=iop
ExecStart=/home/iop/tbm_test/rcs_robot/start_rcs_robot.sh
KillMode=control-group
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload && sudo systemctl enable --now rcs-robot.service
```
