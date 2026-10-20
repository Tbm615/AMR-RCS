Download the file for your computer. You don't need Python or ROS 2 on the operator PC.

| File | For |
|---|---|
| `rcs-windows-x86_64.zip` | Windows 10/11. Unzip, run `rcs\rcs.exe`. |
| `rcs-windows-x86_64-console.zip` | Same app; also opens a console window with logs, for troubleshooting. |
| `rcs-linux-x86_64.tar.gz` | Linux x86-64. `tar xzf rcs-linux-x86_64.tar.gz && ./rcs/rcs` |

The robot must run `rosbridge_server` on port 9090, plus the RCS launcher to use the Robot programs page. Setup and everything else are in the **[User guide](https://github.com/b4lanc3/AMR-RCS/blob/main/docs/USER_GUIDE.md)**.

### Changes in v0.4.0: 3D map, goal arrows, no more freezes

- **No more "Not Responding" on Windows.** Connecting (and every automatic
  reconnect, e.g. after a Wi-Fi drop) used to block the window for up to
  10 s, again and again while the robot did not answer. Connecting now
  happens in the background, and the app reconnects by itself within a few
  seconds. A manual Disconnect followed by Connect works again (it used to
  stop the network loop for good).
- Big maps, costmaps, lidar scans and camera frames are decoded off the GUI
  thread, and only the page you are looking at is redrawn (at most 20 times
  a second). The lidar points are drawn about 40× faster. The Logs page keeps
  the newest 3000 lines (it used to grow forever and slow down on long runs).
  Commands are no longer queued while offline, so nothing stale reaches the
  robot when the link comes back.
- **New 3D Map page** (replaces 3D View): a colored 3D map built while you
  drive, from the camera's depth + color images (about 1 MB/s over Wi-Fi).
  Each frame is placed with the full 3D TF at the moment it was taken, so the
  map no longer comes out rotated or smeared when the robot turns. Save and
  open it as PLY; it also opens the `cloud3d.ply` of a `save_run` result.
  Same look and controls as the report's 3D viewer. On the robot it needs
  `ros-humble-compressed-depth-image-transport`.
- **Goals with an arrow**, like RViz's *2D Goal Pose*: press on the
  destination, drag toward the heading, release. The goal stays on the map
  as a green arrow. The same arrow sets the initial pose and mission
  waypoints (which now get a heading too, shown on the map). Right-drag pans
  the map.
- Crashes, including hard ones inside Qt/OpenGL, now leave a stack trace in
  the app log (`%TEMP%\rcs.log` on Windows).

### Changes in v0.3.0: start bringup and Nav2 from the app

- New **Robot programs** page: start and stop the robot base (bringup) and navigation without a terminal on the robot. Navigation runs on a saved map, or builds a new map with SLAM while you drive, and you can save that map under a name. Each part shows its state, how long it has been running, and its last log lines when something fails.
- Needs the new robot-side launcher, installed once on the robot together with rosbridge. Both then start by themselves at every boot. See [robot/README.md](https://github.com/b4lanc3/AMR-RCS/blob/main/robot/README.md).
- Tested on the real robot: start bringup, start SLAM navigation, save a map, restart navigation on the saved map, stop everything. Nothing is left running after a stop.

### Changes in v0.2.2: tested on the real robot

- The camera image and the point cloud are only received while their page is open. The depth cloud is about 1 MB per frame, and over Wi-Fi it slowed everything else down.
- Odometry is received at 20 Hz instead of 100 Hz, which is plenty for display and costs less Wi-Fi.
- The Camera page shows its frame rate.
- Robot side (this workspace's `nav2_smac_mppi.yml`, not part of the app):
  - Goals now end with the heading you click. `use_final_approach_orientation` was `true`, so Nav2 ignored the heading.
  - The goal tolerance is now 0.15 m, and the robot starts turning to the goal heading 0.35 m before the goal (was 0.5 m), so it no longer gets stuck a few centimetres to the side of the goal.

### Changes in v0.2.1

- The local costmap was drawn in the wrong place. Nav2 keeps it in the `odom` frame, and it is now placed on the map through TF.
- Costmap colors are easier to read: obstacles are solid, and inflation fades out instead of covering the whole corridor.
- Redesigned look: no more dark patches behind labels, proper arrows on drop-downs and number fields, clearer Status and Teleop pages.
- **Center on robot** and **Whole map** buttons on every map. Large maps open fitted and keep your zoom when the map updates.
- Mission waypoints are labeled on the map (`1 Pick-up`, `2 Drop-off`…). The Missions map shows Nav2's planned path.
- The robot's yaw is shown in [-π, π].
- Screenshots in the guide are taken from the app driving real Nav2.

### Changes in v0.2.0 (v0.1.0 crashed once the robot moved, don't use it)

- Fixed the crash: PySide6 6.12 corrupts memory with the map widgets, so the build now uses PySide6 6.8.3.
- Teleop no longer floods `/cmd_vel_safe` with zero commands when idle. That used to block Nav2.
- E-STOP also cancels the Nav2 goal and the running mission, and publishes `/amr/estop`.
- The lidar position is read from `/tf_static`, so the scan lines up with the map.
- The Navigation page shows the real Nav2 state, distance and time remaining, and warns when Nav2 doesn't respond.
- Missions wait for Nav2 to finish each goal and retry dropped or aborted goals.

### How it was tested

- On real Windows and Linux machines (GitHub Actions), the built app connects, subscribes to every topic, and stays silent while idle.
- On a PC running this workspace's Nav2 launch on `map23_04` with a simulated base, through a real `rosbridge_server`, the app set the initial pose, sent goals from the map, canceled them, triggered E-STOP while driving, and ran a looping mission. All 28 checks passed.
- On the real robot over Wi-Fi (v0.2.2), with slam_toolbox, Nav2 and the CBF filter:
  - live map and costmaps, camera and point cloud;
  - 4 goals with a requested heading, each reached in 6–9 s;
  - cancel;
  - E-STOP while driving: the robot stopped in 0.55 s;
  - a looping mission, 4 legs, each finished within 0.16 m;
  - teleop.
