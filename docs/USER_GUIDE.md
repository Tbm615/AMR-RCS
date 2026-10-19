# RCS User Guide

RCS (Robot Control System) is a desktop app for operating Hinson-class
differential-drive AMRs: drive them by hand, see the map, lidar and camera,
send navigation goals and run waypoint missions.

It runs on **Windows 10/11** and **Linux (x86-64)**. The operator PC does
**not** need ROS 2 or Python. The app talks to the robot over the network
through `rosbridge_server`, which runs on the robot.

- [1. How it fits together](#1-how-it-fits-together)
- [2. Prepare the robot (one time)](#2-prepare-the-robot-one-time)
- [3. Install the app](#3-install-the-app)
- [4. First connection](#4-first-connection)
- [5. The main window](#5-the-main-window)
- [6. Robot programs: start bringup and Nav2](#6-robot-programs-start-bringup-and-nav2)
- [7. Driving by hand (Teleop)](#7-driving-by-hand-teleop)
- [8. E-STOP](#8-e-stop)
- [9. Map, localization and goals (2D Map)](#9-map-localization-and-goals-2d-map)
- [10. Navigation page](#10-navigation-page)
- [11. Missions, checkpoints and loop routes](#11-missions-checkpoints-and-loop-routes)
- [12. Status, Camera, 3D View and Logs](#12-status-camera-3d-view-and-logs)
- [13. Several robots](#13-several-robots)
- [14. Settings files and advanced options](#14-settings-files-and-advanced-options)
- [15. Troubleshooting](#15-troubleshooting)

---

## 1. How it fits together

```
 Operator PC (Windows / Linux)                Robot PC (ROS 2 Humble)
 ┌──────────────────────────┐   WebSocket    ┌──────────────────────────────┐
 │ RCS app                  │ ─────────────▶ │ rosbridge_server :9090       │
 │ (no ROS 2 installed)     │ ◀───────────── │   ↕ ROS 2 topics / actions   │
 └──────────────────────────┘   port 9090    │ bringup, Nav2, AMCL, camera… │
                                             └──────────────────────────────┘
```

What the app uses on the robot (defaults, all changeable per robot):

| Purpose | Topic / frame |
|---|---|
| Teleop and E-STOP velocity | `/cmd_vel_safe` (geometry_msgs/Twist) |
| Robot speed | `/odom` |
| Lidar | `/scan_filtered` |
| Map | `/map` |
| Costmaps | `/global_costmap/costmap`, `/local_costmap/costmap` |
| Planned path | `/plan` |
| Goals | `/goal_pose` (PoseStamped), `/navigate_to_pose/_action/status` |
| Initial pose | `/initialpose` |
| Camera | `/camera/color/image_raw/compressed` |
| Point cloud | `/camera/depth/points` |
| Robot logs | `/rosout` |
| E-STOP flag | `/amr/estop` (std_msgs/Bool) |
| Frames | `map`, `odom`, `base_link`, `laser_frame` (from `/tf` and `/tf_static`) |

---

## 2. Prepare the robot (one time)

The robot needs two things for RCS: `rosbridge_server` (the app connects to
it) and the **RCS launcher** (lets the app start and stop bringup and Nav2,
see [section 6](#6-robot-programs-start-bringup-and-nav2)). Both are in the
[`robot/`](../robot/) folder of this repository and start by themselves every
time the robot boots.

On the robot PC, once:

```bash
sudo apt install ros-humble-rosbridge-suite ros-humble-compressed-image-transport
# copy the robot/ folder of this repository next to the workspace, e.g.:
mkdir -p ~/tbm_test/rcs_robot && cp robot/* ~/tbm_test/rcs_robot/
chmod +x ~/tbm_test/rcs_robot/*
# start both at every boot (user crontab, no sudo needed):
(crontab -l 2>/dev/null; echo "@reboot sleep 10 && $HOME/tbm_test/rcs_robot/start_rcs_robot.sh") | crontab -
# start them now without rebooting:
nohup setsid ~/tbm_test/rcs_robot/start_rcs_robot.sh >/dev/null 2>&1 &
```

`start_rcs_robot.sh` uses the robot's ROS settings (CycloneDDS, domain 0, the
workspace's `cyclonedds.xml`) and restarts rosbridge or the launcher if one of
them exits. If your workspace is not `~/tbm_test`, set `RCS_WS` at the top
of the script. Details, logs and how to remove it:
[`robot/README.md`](../robot/README.md).

Allow TCP port 9090 from the operator network if a firewall is active:

```bash
sudo ufw allow from 192.168.10.0/24 to any port 9090 proto tcp
```

`ros-humble-compressed-image-transport` makes the camera publish
`/camera/color/image_raw/compressed`, which the Camera page shows.

After that, everything is done from the app: power on the robot, connect,
and start bringup and Nav2 on the **Robot programs** page.

---

## 3. Install the app

Download the build for your OS from the
[Releases page](https://github.com/b4lanc3/AMR-RCS/releases/latest).

**Windows**

1. Download `rcs-windows-x86_64.zip` and unzip it anywhere (for example
   `C:\RCS`).
2. Run `rcs\rcs.exe`. Keep the `_internal` folder next to it.
3. The first time, Windows SmartScreen may warn that the app is from an
   unknown publisher (the build is not code-signed). Click **More info →
   Run anyway**.

`rcs-windows-x86_64-console.zip` is the same app but also opens a console
window with log output. Use it when troubleshooting.

**Linux**

```bash
tar xzf rcs-linux-x86_64.tar.gz
./rcs/rcs
```

On minimal desktop installs Qt may need `libxcb-cursor0`
(`sudo apt install libxcb-cursor0`).

---

## 4. First connection

![Connection page](screenshots/conn.png)

1. Open the **Connection** page (selected when the app starts).
2. A robot named **Hinson robot** already exists. Select it in the
   **Robots** list.
3. Set **Host** to the robot's IP address (for example `192.168.10.10`) and
   **Port** to `9090`. Leave **Use secure WebSocket** off unless you set up
   TLS on rosbridge.
4. Check the topic names under **Topics / Frames**. The defaults match this
   robot's bringup; change them only if your launch files differ.
5. Click **Save changes**, then **Connect**.

When it works, the text under the buttons turns green
(`connected — ws://…`) and the status bar at the bottom shows
**● connected**. If the robot reboots or Wi-Fi drops, the app keeps
reconnecting every 2 seconds until you click **Disconnect**.

---

## 5. The main window

- **Sidebar (left):** pages: Connection, Robot programs, Status, Teleop, 2D Map,
  Navigation, Missions, 3D View, Camera, Logs. The red **● E-STOP** button
  at the bottom is always visible.
- **Status bar (bottom):** active robot and address, connection state,
  robot pose in the map (`x`, `y`, `yaw` in rad), E-STOP state.

All map views share the same controls:

- **Mouse wheel:** zoom. **Left-drag:** pan.
- **Center on robot** / **Whole map** buttons (top-left of the map). A new
  map opens fitted; later map updates keep your zoom.
- Blue rectangle and line: the robot footprint and its heading.
- Red dots: lidar points. Green line: path planned by Nav2.
- Orange dots: waypoints of the open mission, labeled with their order and name.
- Costmaps (when enabled): magenta = obstacle, blue haze = inflation around it.

---

## 6. Robot programs: start bringup and Nav2

![Robot programs page](screenshots/robot.png)

This page starts and stops the ROS programs on the robot, so you do not need
a terminal on the robot. It needs the RCS launcher installed once
([section 2](#2-prepare-the-robot-one-time)); the banner at the top says
**Robot launcher online** when it answers.

**1 · Robot base (bringup).** Motor drives, lidar, scan filter and camera.
Click **Start**; it shows *running* once odometry and the lidar are up
(a few seconds). Every other page needs it. **Stop** disables the motors and
stops navigation too (you are asked to confirm).

**2 · Navigation (Nav2).** Planner, controller and the CBF safety filter.
Choose how the robot knows where it is, then click **Start**:

- **Use a saved map:** pick a map from the list (the workspace's maps and the
  ones you saved). After it starts, set the robot's position on the 2D Map
  page (**Set initial pose**): **Open 2D Map** takes you there.
- **Build a new map while driving (SLAM):** the map starts empty and grows on
  the 2D Map page as the robot moves (Teleop or goals). No initial pose
  needed; goals work right away inside the mapped area.

While navigation runs, the button reads **Restart**: change the map or the
mode and click it to switch.

**3 · Save the map.** Type a name (letters, digits, `-`, `_`) and click
**Save map**. The map is saved on the robot (`~/tbm_test/maps/`) and appears
under *Use a saved map*, for this and every later session.

Each stack shows its state (stopped, starting…, running, stopping…,
failed) and how long it has been in it. **Show log** shows the last lines of
its output; it opens by itself when a stack fails. Programs started by hand
on the robot (from a terminal) are shown as *running (started outside the
app)* and must be stopped where they were started.

The full logs are on the robot in `~/tbm_test/runlog/rcs_bringup.log` and
`rcs_nav.log`.

---

## 7. Driving by hand (Teleop)

![Teleop page](screenshots/teleop.png)

1. Open **Teleop**.
2. Drive with the **joystick**: press inside the ring and drag. Up is
   forward, sideways turns. Release to stop.
3. Or use the **keyboard**, after clicking once on the Teleop page so it
   has focus: **W / ↑** forward, **S / ↓** reverse, **A / ←** turn left,
   **D / →** turn right, **Space** stop.
4. **Speed limits** cap the commands (defaults 0.45 m/s and 0.8 rad/s). The
   live command is shown under the joystick.

Teleop commands go to `/cmd_vel_safe` at 20 Hz while you drive. When you let
go, the app sends zero velocity for 0.3 s and then stops publishing, so it
does not block Nav2. Like the gamepad, teleop bypasses the CBF safety
filter: watch the robot while driving.

---

## 8. E-STOP

Click **● E-STOP** (sidebar or Teleop page). While engaged:

- the current Nav2 goal is canceled and the running mission is stopped,
- the app publishes zero velocity on `/cmd_vel_safe` continuously,
- `/amr/estop` is set to `true` (other consoles listening to it, such as
  the web control center, show it too).

Click the button again to release it. If another console publishes
`/amr/estop`, RCS mirrors that state.

> **Important:** this is a *software* stop over Wi-Fi. Nothing on the robot
> enforces `/amr/estop` today, and another publisher (for example the
> gamepad) can still send commands. If the network drops, the app cannot
> stop the robot. Always keep the robot's hardware E-stop within reach.

---

## 9. Map, localization and goals (2D Map)

![2D Map page](screenshots/map.png)

The map appears as soon as the robot publishes `/map` (from SLAM, or from
Nav2's map server).

**Localize the robot (set initial pose).** After Nav2 starts, AMCL needs to
know where the robot is. If the robot rectangle is not where the robot
really is, or the Navigation page reports *no response from Nav2*:

1. In **Tools**, choose **Set initial pose**.
2. Click once on the map where the robot is.
3. Click a second time in the direction the robot is facing.

The lidar points should then line up with the walls on the map.

**Send a goal.**

1. In **Tools**, choose **Set goal** (the default).
2. Click the destination, then click again in the direction the robot
   should face when it arrives. A green circle marks the goal.
3. Watch progress on the **Navigation** page.

**Cancel current goal** stops the robot's navigation.

**Add waypoint** mode: each click adds a waypoint to the mission currently
open on the **Missions** page.

**Overlays** turn on or off the lidar points, the planned path, the
waypoints, and the global and local costmaps (shown in red/blue over the
map).

---

## 10. Navigation page

![Navigation page](screenshots/nav.png)

The same map with both costmaps shown, plus the **Nav2 status**:

| State | Meaning |
|---|---|
| goal sent, waiting for Nav2… | the goal was published, Nav2 has not picked it up yet |
| navigating | Nav2 accepted the goal and is driving |
| goal reached | done |
| canceled | canceled by you, the E-STOP, or a mission stop |
| aborted by Nav2 | Nav2 gave up (no path, robot stuck…) |
| no response from Nav2 | nothing happened within 4 s: Nav2 is not running, or the robot is not localized (set the initial pose) |

**Distance remaining** is measured along Nav2's planned path from the
robot's position. **Estimated time** divides it by the robot's current speed
(or the speed limit while the robot is standing still). You can send goals
and cancel from this page too.

---

## 11. Missions, checkpoints and loop routes

![Missions page](screenshots/mission.png)

A mission is an ordered list of waypoints (checkpoints). The robot drives to
each in turn.

**Create a mission**

1. Open **Missions**. Use **+** to create a mission, **Dup** to copy one,
   **Del** to delete one. Rename it in **Name**.
2. Add waypoints:
   - click on the map of the Missions page (or on the 2D Map page in
     **Add waypoint** mode), or
   - drive the robot somewhere and click **Set here** to add its current
     position.
3. Edit **Name**, **x**, **y** and **yaw°** directly in the table. Reorder
   with **↑ / ↓**, delete with **Remove**.
4. **Default per-waypoint settings** apply to waypoints you add next:
   - **Tolerance:** how close counts as "arrived" (default 0.25 m).
   - **Dwell:** how long to wait at the waypoint before moving on.
5. Tick **Loop route** to start again from the first waypoint after the last
   one, until you stop it.

Missions are saved automatically.

**Run a mission**

- **▶ Play:** start, or continue after a pause.
- **⏸ Pause:** cancel the current goal and keep the position in the list.
- **■ Stop:** cancel and reset to the first waypoint.

The status line shows the current target (`going to pick-A (2/4)`),
arrivals and dwell time.

The robot moves on to the next waypoint once it is within the tolerance and
Nav2 has finished the goal. If Nav2 drops a goal, aborts it, or reports
success while the robot is somewhere else, the app resends the same
waypoint up to 3 times, then stops the mission and says why.

---

## 12. Status, Camera, 3D View and Logs

![Status page](screenshots/status.png)

![Logs page](screenshots/logs.png)

- **Status:** connection, linear and angular speed (from `/odom`), E-STOP
  state, and the message rate of each topic. A rate of `—` means nothing
  is arriving on that topic: check the robot side.
- **Camera:** live image from `/camera/color/image_raw/compressed` (JPEG),
  limited to about 10 frames/s to spare Wi-Fi. The app only receives it
  while the Camera page is open.
- **3D View:** the camera point cloud (`/camera/depth/points`, about 2
  frames/s, at most 60 000 points) and the robot as a box. The cloud is
  about 1 MB per frame, so the app only receives it while the 3D View page is
  open; on the other pages it does not use the Wi-Fi at all. Drag to orbit,
  wheel to zoom. Needs OpenGL; on PCs without it the page shows a notice.
- **Logs:** messages from `/rosout`. Choose a minimum level, toggle
  auto-scroll, or **Clear**.

---

## 13. Several robots

Each robot is a profile on the **Connection** page.

1. Click **+ Add**, then fill in its name, host and topics. Click **Save
   changes**.
2. To switch robots: select it in the list and click **Connect**.

The app controls **one robot at a time**. Missions are stored per PC, not
per robot; give them clear names if robots share a site.

---

## 14. Settings files and advanced options

| | Linux | Windows |
|---|---|---|
| Robots | `~/.config/RCS/robots.json` | `%APPDATA%\RCS\robots.json` |
| Missions | `~/.config/RCS/missions.json` | `%APPDATA%\RCS\missions.json` |
| App log | `/tmp/rcs.log` | `%TEMP%\rcs.log` |

Some per-robot options are only in `robots.json` (edit while the app is
closed):

| Key | Default | Meaning |
|---|---|---|
| `limits.footprint_half_length` / `_half_width` | 0.45 / 0.30 m | robot rectangle on the maps |
| `limits.laser_offset_x` | 0.40 m | lidar position, used only if `/tf_static` has no `laser_frame` |
| `limits.image_throttle_ms` | 100 | minimum time between camera frames |
| `limits.cloud_throttle_ms` | 500 | minimum time between point clouds |
| `topics.*` | see section 1 | every topic and frame name |

**Command-line options**

```
rcs --host 192.168.10.10:9090 --autoconnect
```

- `--host HOST:PORT` sets the active robot's address.
- `--autoconnect` connects on start.

---

## 15. Troubleshooting

**Cannot connect / keeps reconnecting**
- Is rosbridge running on the robot? From the operator PC,
  `curl -i http://<robot-ip>:9090` should answer (any HTTP reply, e.g.
  `400` or `426`, means it is up). A timeout means it is not running or a
  firewall blocks it.
- Is the host/port right on the Connection page? Did you click **Save
  changes**?

**Robot programs page says "The robot does not answer"**
- The RCS launcher is not running on the robot. Install it (section 2). Its
  log is `~/tbm_test/runlog/rcs_launcher.log` on the robot.

**Bringup or navigation shows "failed"**
- Open **Show log** on that stack: the last lines usually name the problem
  (lidar not reachable, motor driver USB adapter missing, map file not found).
- Fix it and click **Start** again.

**Connected, but no map, scan or pose**
- Is bringup running? See the **Robot programs** page.
- Open **Status**: topics with `—` are not arriving.
- rosbridge must run with the same DDS settings as the robot stack
  (section 2). Restart it from a shell where `ros2 topic list` shows the
  robot topics.
- The topic names on the Connection page must match the robot.

**Lidar points do not match the walls**
- The robot is not localized: use **Set initial pose** (section 9).

**Goal shows "no response from Nav2"**
- Start navigation on the **Robot programs** page. With a saved map, set the
  initial pose: Nav2 only finishes starting once AMCL has a pose.

**The robot reaches the goal but faces the wrong way**
- In Nav2's planner settings (`nav2_smac_mppi.yml`, `GridBased`),
  `use_final_approach_orientation` must be `false`. With `true` the planner
  replaces the heading you clicked with the direction it arrives from. This
  workspace ships with `false`.

**Goal stays on "navigating" near the goal, robot shuffles or spins**
- Nav2 cannot close the last few centimetres to the side. The
  workspace uses `xy_goal_tolerance: 0.15` and starts turning to the goal
  heading 0.35 m before the goal (`GoalAngleCritic.threshold_to_consider`),
  which avoids it. Cancel the goal and send it again if it happens.

**Robot does not move with Teleop**
- The driver must listen on `/cmd_vel_safe` (this is the bringup default).
  If bringup was started with `cmd_vel_topic:=/cmd_vel`, change **cmd_vel**
  on the Connection page to `/cmd_vel`.
- Check the E-STOP is released (status bar).

**Camera page stays black**
- The camera is only received while the Camera page is open: give it a
  second after opening the page.
- Install `ros-humble-compressed-image-transport` on the robot and check
  that `/camera/color/image_raw/compressed` exists.

**The app closes unexpectedly**
- Look at the app log (section 14). The console build on Windows shows the
  same output live.
