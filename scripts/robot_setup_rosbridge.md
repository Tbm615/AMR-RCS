# Robot-side setup: expose ROS 2 to RCS via rosbridge

RCS does not need ROS 2 on the operator laptop. The robot runs
`rosbridge_server`, which exposes topics/services/actions over a JSON
WebSocket on TCP 9090. RCS speaks that protocol.

## Install

```bash
sudo apt install ros-humble-rosbridge-suite ros-humble-tf2-web-republisher
```

## Launch (quick test)

Source ROS and run:

```bash
source /opt/ros/humble/setup.bash
source ~/AMR_WS/first/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

The console should log `Rosbridge WebSocket server started on port 9090`.
Then on the operator PC: open RCS → Connection → set Host to the robot's
LAN IP (e.g. `192.168.10.10`) → Connect.

## Auto-start without sudo (what the Hinson robot uses)

[`start_rosbridge.sh`](start_rosbridge.sh) runs rosbridge in the robot's ROS
environment (CycloneDDS, domain 0, the workspace's `cyclonedds.xml`) and
restarts it if it exits. A user crontab entry starts it at boot:

```bash
cp start_rosbridge.sh ~/tbm_test/ && chmod +x ~/tbm_test/start_rosbridge.sh
(crontab -l 2>/dev/null; echo "@reboot sleep 10 && /home/iop/tbm_test/start_rosbridge.sh") | crontab -
```

Edit the workspace paths in the script if yours differ. The log is
`~/tbm_test/runlog/rosbridge.log`. To stop it until the next boot:
`pkill -f start_rosbridge.sh; pkill -f rosbridge_websocket`. To remove it,
delete the line with `crontab -e`.

## systemd unit (auto-start, needs sudo)

Create `/etc/systemd/system/rosbridge.service`:

```ini
[Unit]
Description=rosbridge_server for RCS
After=network-online.target

[Service]
Type=simple
User=iop
Environment=ROS_DOMAIN_ID=0
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
Environment=CYCLONEDDS_URI=file:///home/iop/AMR_WS/first/src/bringup/config/cyclonedds.xml
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /home/iop/AMR_WS/first/install/setup.bash && exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

Then:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now rosbridge.service
sudo systemctl status rosbridge.service
```

## Firewall

Allow 9090 inbound on the robot's LAN interface:

```bash
sudo ufw allow from 192.168.10.0/24 to any port 9090 proto tcp
```

## Verifying

From the operator PC:

```bash
curl -i http://192.168.10.10:9090        # expect 426 Upgrade Required — that's fine
```

If `curl` hangs or times out, firewall or the service isn't running.
