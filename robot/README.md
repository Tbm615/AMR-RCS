# Robot side of RCS

RCS runs these on the robot, all started by
[`start_rcs_robot.sh`](start_rcs_robot.sh):

| Program | What it does |
|---|---|
| `rosbridge_server` (port 9090) | exposes the robot's ROS 2 topics to the app over WebSocket |
| [`rcs_launcher.py`](rcs_launcher.py) | starts and stops bringup and Nav2 when the app asks (Robot programs page), saves maps |
| [`rcs_safety.py`](rcs_safety.py) | the robot's own E-STOP: every velocity command passes through it ([E-STOP](#e-stop)) |
| [`rcs_battery.py`](rcs_battery.py) | stays connected to the BMS, writes the battery log ([Battery](#battery)) |
| [`rcs_gateway.py`](rcs_gateway.py) | once there is a user: sign-in, a role per user and an audit log on port 9090, in front of rosbridge ([Sign-in and roles](#sign-in-and-roles)) |

The script sources ROS and the workspace (`RCS_WS`, default
`/home/iop/tbm_test`), sets the robot's DDS settings (CycloneDDS, domain 0,
`bringup/config/cyclonedds.xml`) and restarts any program that exits (the
E-STOP gate and the battery service after 1 s, the others after 5 s).

## Install

In one go, on the robot, from the release's `robot/` folder copied over:

```bash
bash install_robot.sh --dry-run     # what it would do
bash install_robot.sh               # copy to ~/tbm_test/rcs_robot, start at boot, start now
```

It checks ROS 2, the apt packages and the Python modules it needs and says
what is missing, keeps the E-STOP latch, users, audit and battery logs, and is
safe to run again to update. By hand, it is:

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

## Sign-in and roles

Without it, anyone on the robot's network can drive the robot with any
rosbridge client. With a user created on the robot, `start_rcs_robot.sh` puts
`rcs_gateway.py` on port 9090 and rosbridge on `127.0.0.1:9091`, where only the
gateway reaches it. Every app then signs in, and the robot passes on only what
the user's role may do:

| Role | May |
|---|---|
| viewer | see everything; stop the robot (E-STOP, cancel the goal) |
| operator | also drive, send goals, run missions, set the initial pose, start and stop bringup and Nav2, save and send maps, release the E-STOP |
| admin | also manage users, delete battery logs, any other topic or service |

```bash
pip3 install --user "websockets>=13"
cd ~/tbm_test/rcs_robot
python3 rcs_gateway.py users add admin --role admin      # asks the password (8 characters or more)
python3 rcs_gateway.py users add op1 --role operator
python3 rcs_gateway.py gencert                            # optional: wss://, prints the fingerprint
sudo reboot                                               # or restart start_rcs_robot.sh
```

- More users, roles, new passwords and the audit log are also in the app
  (Connection page, **Users and audit log…**, admins only).
  `rcs_gateway.py users list | passwd NAME | role NAME ROLE | remove NAME |
  signout NAME` does it on the robot.
- Tokens last 30 days. A new password, a removed user or **Sign out everywhere**
  ends that user's open connections within 5 s; a new role applies at once.
- Five wrong passwords from one address: it waits a minute.
- Audit log: `~/tbm_test/audit/audit-YYYY-MM.jsonl`, one line per thing done
  (sign-in, connect, teleop start and end, goals, E-STOP, programs, maps,
  refusals), kept until deleted.
- With `gencert` the gateway serves `wss://` (tick **Use secure WebSocket** in
  the app). The app shows the certificate's fingerprint the first time:
  compare it with what `gencert` printed. After that a different certificate is
  refused.
- To go back to an open robot: delete `~/tbm_test/rcs_users.json` and restart.
- **Known limit:** DDS (ROS 2 itself) is not covered. A PC on the robot's
  network with ROS 2 and the same domain can still publish to the robot's
  topics directly. Keep the robot on its own network, or set up SROS2.

## What the launcher runs

| App button | Command on the robot |
|---|---|
| Robot base → Start | `ros2 launch bringup bringup.launch.py cmd_vel_topic:=/rcs/cmd_vel_gated` (refused while `rcs_safety.py` is not running) |
| Navigation, saved map | `ros2 launch nav_map.launch.py map:=<map>.yaml ...`: one launch as `robot_navigation.launch.py` (map server + AMCL, Nav2, CBF filter) without its automatic initial pose of (0, 0), with the map's zones (`costmap_filters.launch.py` and a copy of `nav2_smac_mppi.yml` with the filters, see Keepout and speed zones) |
| Navigation, SLAM | `ros2 launch slam slamtoolbox.launch.xml`, `ros2 launch nav2_bringup navigation_launch.py params_file:=<navigation>/config/nav2_smac_mppi.yml`, `ros2 run navigation cbf_safety_filter.py` (remapped as in `robot_navigation.launch.py`) |
| Save map | `ros2 run nav2_map_server map_saver_cli -f $RCS_WS/maps/<name>` |

Maps are listed from `<workspace>/install/navigation/share/navigation/maps`
and `$RCS_WS/maps` (saved maps).

Rules: navigation needs the base; stopping the base stops navigation first;
starting navigation while it runs restarts it with the new map or mode. If
one process of a stack exits on its own, the rest of the stack is stopped
and the stack shows *failed* with its last log lines. On a saved map Nav2
is *running* once `/lifecycle_manager_localization/is_active` (and
`..._zones/is_active`) say so; `..._navigation` becomes active only after the
robot's pose is set (its costmaps wait for map -> base_link). A bring-up can
hang on a lost service answer (seen with Fast DDS in simulation; the robot
runs Cyclone DDS). When a lifecycle manager gives up, when Nav2 is not up
after 60 s (`RCS_NAV_UP_TIMEOUT`), or when navigation is still not active 45 s
after the pose (`RCS_NAV_ACTIVE_AFTER_POSE`), the launcher restarts it by
itself, up to twice, then shows *failed*. Stop is Ctrl-C
(SIGINT) to the whole process group, then SIGTERM and SIGKILL if needed.
If the launcher itself restarts, it stops the programs it had started.

Protocol (JSON in `std_msgs/String`): commands on `/rcs/launcher/cmd`, state
on `/rcs/launcher/state` (1 Hz and on every change, transient local), map
previews on `/rcs/launcher/preview`, map copies on `/rcs/launcher/map_data`,
zones on `/rcs/launcher/zones`. See the docstring at the top of
`rcs_launcher.py`. `map_preview.py`, `map_exchange.py`, `map_files.py`,
`zone_masks.py`, `nav_map.launch.py`, `costmap_filters.launch.py` and
`camera_check.py` must sit next to it.

Nav2 on a saved map is started **without an initial pose**: set it on the
2D Map page (or press *Use last position*); the Status page says until then.
`robot_navigation.launch.py` sends (0, 0, 0) a few seconds after its start,
which overwrote a pose set in those seconds.

Maps sent from the app land in `$RCS_WS/maps`. The app can replace a map there
only if the operator confirms it, and never the one Nav2 is running on.

## Keepout and speed zones

Drawn on the app's 2D Map page, kept on the robot next to their map as
`<map>.zones.json`. When Nav2 starts on that map the launcher turns them into
two masks the size of the map and starts Nav2's costmap filters
(`costmap_filters.launch.py`: two `map_server`s for the masks, two
`costmap_filter_info_server`s, a lifecycle manager):

- **keepout**: lethal in both costmaps: Nav2 plans no path into the zone and
  refuses goals inside it;
- **slow**: Nav2's `SpeedFilter` (in the local costmap, which updates several
  times a second) publishes `/speed_limit` (percent of the usual top speed)
  while the robot is inside; where two overlap, the lower one. Measured in
  simulation: 30 % of 0.4 m/s held at 0.12 m/s inside the zone.

The filters go into a copy of `nav2_smac_mppi.yml`
(`~/.ros/rcs_zones/nav2_params.yaml`, with the masks; `RCS_ZONES_DIR` moves it);
the workspace's file is not changed. They run even with no zones (empty
masks), so saving zones while Nav2 runs applies them at once: the launcher
reloads the masks (`<keepout|speed>_mask_server/load_map`). Zones stop Nav2
only: manual driving (Teleop, the gamepad) can still enter them, and the
app's Teleop page says when the robot is in one. Needs
`ros-humble-nav2-map-server` and `ros-humble-nav2-lifecycle-manager` (part of
Nav2), and `.pgm` maps.

Changing zones needs the operator role; the gateway's audit log has each change.

## E-STOP

`rcs_safety.py` stands between every velocity command and the drives:

```
Nav2 -> CBF -> /cmd_vel_safe  <- gamepad, RCS teleop
                    |
               rcs_safety.py   (estop_gate.py)
                    |
             /rcs/cmd_vel_gated -> diff_drive_controller
```

- **E-STOP** (`/amr/estop` true, from any RCS app, the Fleet page or the web
  console) latches here: commands become zero, zero is also sent on its own
  at 20 Hz, Nav2 goals are cancelled (again every second while one runs). It
  stays until an operator releases it (`/amr/estop` false). It is kept in
  `estop.json` next to the script, so it survives a restart of the service or
  of the robot.
- **Losing the app:** each RCS app sends a heartbeat (`/rcs/app/heartbeat`)
  twice a second saying whether it is driving (mission, goal, teleop). If a
  driving app is silent for 3 s (Wi-Fi gone, app crashed), the robot cancels
  its Nav2 goal and holds zero for 1.5 s. An app that only watches, or that
  is closed normally, stops nothing.
- **If `rcs_safety.py` is not running nothing reaches the drives**: a crash of
  it stops the robot. The launcher refuses to start bringup without it.
- State on `/rcs/estop/state` (JSON, 2 Hz, latched).

Bringup started by hand (`ros2 launch bringup bringup.launch.py` without
`cmd_vel_topic:=/rcs/cmd_vel_gated`) drives from `/cmd_vel_safe` directly and
is **not** guarded. `RCS_ESTOP_GATE=0` makes the launcher start it that way.
This is still a software stop: keep the hardware E-stop within reach.

## Battery

`rcs_battery.py` stays connected to the JK-BMS over Bluetooth and publishes
`/rcs/battery` for the app's Battery page. It only reads. It reconnects by
itself forever, and remembers the BMS it found first in `bms.json`: delete that
file to pair with another one. The BMS takes one connection at a time, so a
phone connected to it keeps the robot out, and the robot keeps a phone out.

It uses a USB Bluetooth dongle, placed as near the battery as the cable allows:
the card inside the Jetson has no antenna. The dongle is a CSR8510 clone
(`0a12:0001`) that BlueZ cannot bring up on kernel 5.15, so it is driven directly
over USB by [bumble](https://github.com/google/bumble), see `hci_quirks.py`.
With another dongle set `RCS_BMS_TRANSPORT`.

Needs, once:

```bash
pip3 install --user bumble           # no internet on the robot: copy the wheels over
sudo cp 99-rcs-bluetooth-dongle.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`jk_bms.py`, `bms_link.py`, `battery_log.py` and `hci_quirks.py` must sit next to `rcs_battery.py`.

**Staying connected.** Once the BMS is known (`bms.json` holds its address and
address type) a drop is followed by a connect straight to it, with no scan: a
scan cost up to 12 s on every drop. Only after 3 failed direct connects does it
scan again, and after 8 failures in a row it opens the dongle anew. Why each
drop happened (the Bluetooth reason, e.g. `CONNECTION_TIMEOUT_ERROR (8)`) is
counted in `/rcs/battery` and written to the log.

**Watchdog.** If the service stops going round for 20 s (a USB call that never
returns), a thread of its own writes a last row saying so and exits; the start
script starts it again after 1 s.

### Battery log

One CSV file per boot, one row every second, in `$RCS_WS/battery_log/`
(`RCS_BATTERY_LOG` to change it):

```
battery_<boot time YYYYmmdd-HHMMSS>_<boot id>.csv
```

- **Every row goes to the disk at once** (flush + fsync). After a power cut
  the file holds everything up to the last second, and its last row is how
  the battery was right then.
- **A row every second, BMS heard or not**: when it is not, the values are
  empty and `state` says why. A reading older than 15 s is not written as if
  it were current.
- **Everything the BMS reports**: charge, health, pack voltage, current,
  power, each cell (`cells`, `;`-separated) and its resistance
  (`resistances`), lowest, highest and average cell and which cell they are,
  both temperature sensors, MOS temperature, remaining and nominal Ah, cycles,
  total Ah discharged, charge and discharge switches, balancing and its
  current, alarm bits (raw), the BMS's running time; and the link: state,
  RSSI, age of the reading, reconnects, bad frames, error, and notes
  (connected, lost and why, what the BMS sent that is not a frame). Once a
  minute the BMS's raw frame (`frame`, hex) too, so the file can be decoded
  again if a field turns out to be read wrong. The alarm bits' meaning is not
  checked yet: the real frames seen so far have none set.
- **How each boot ended**: a clean stop (shut down, service stopped) writes a
  last row with the note `stopped`. A file that ends without it ended in a
  power cut or a crash; the next boot logs a warning with the battery's last
  values, and the app shows it.
- **Kept for good.** Nothing deletes a file by itself. Delete them from the
  app (Battery, History, one boot at a time, asked to confirm; the running
  boot's file cannot be deleted) or by hand. The app warns when the robot's
  disk has less than 2 GB free. A row is about 400 bytes: some 35 MB for a
  day of 24 h.
- Restarting the service during a boot carries on in the same file.

The app reads them with `log_list`, `log_get` and `log_delete` on
`/rcs/battery/cmd` (answers on `/rcs/battery/log`), see `battery_log.py`.
Only files of that folder can be read or deleted, by name.


## The drive wheels in RViz (optional)

The app does not need this. Up to now the robot's URDF put the left drive wheel
outside the body and the right one inside it (the wheel meshes have their
origin on one face). The workspace's `src/bringup/urdf/robot_description.urdf.xacro`
now centres each wheel in its wheel well (`y = ±0.17145`), shifts the meshes
onto their joints (`visual origin x = ±0.04388`), turns the left wheel's axis to
`-1 0 0` so both turn forward the same way, and gives each a 4 cm tyre
collision at `y = ±0.195` (wheel_separation 0.39). Only the drawing and the
simulator change: the drives and odometry use `wheel_track_m` and
`wheel_separation`. To see it in RViz on the robot, make the same edits to the
robot's copy of that file (its meshes are under `package://motor/...` there)
and rebuild that package.

## Logs (on the robot)

| File | Content |
|---|---|
| `~/tbm_test/runlog/rosbridge.log` | rosbridge |
| `~/tbm_test/runlog/rcs_launcher.log` | launcher: commands received and their result |
| `~/tbm_test/runlog/rcs_bringup.log` | bringup output (capped at 20 MB) |
| `~/tbm_test/runlog/rcs_nav.log` | navigation output (capped at 20 MB) |
| `~/tbm_test/runlog/rcs_safety.log` | E-STOP gate: engaged, released, stops for a lost app |
| `~/tbm_test/runlog/rcs_gateway.log` | gateway: started, rosbridge not reachable |
| `~/tbm_test/audit/` | who did what with the robot ([Sign-in and roles](#sign-in-and-roles)) |
| `~/tbm_test/runlog/rcs_battery.log` | battery service: link state changes |
| `~/tbm_test/battery_log/` | the battery log, a file per boot (above) |

## Stop or remove

- Stop until the next boot: `pkill -f start_rcs_robot.sh; pkill -f rcs_launcher.py; pkill -INT -f rosbridge_websocket; pkill -f rcs_battery.py; pkill -f rcs_safety.py; pkill -f rcs_gateway.py`.
  rosbridge needs `-INT`: a plain `pkill` leaves it holding port 9090 with its ROS
  side shut down, and the app then connects but receives nothing.
  `start_rcs_robot.sh` clears such a leftover before it starts rosbridge.
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
