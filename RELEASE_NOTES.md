Download the file for your computer. You don't need Python or ROS 2 on the operator PC.

| File | For |
|---|---|
| `rcs-windows-x86_64.zip` | Windows 10/11. Unzip, run `rcs\rcs.exe`. |
| `rcs-windows-x86_64-console.zip` | Same app; also opens a console window with logs, for troubleshooting. |
| `rcs-windows-x86_64-setup.exe` | Windows installer: Start menu entry, uninstaller, no admin rights needed. |
| `rcs-linux-x86_64.tar.gz` | Linux x86-64. `tar xzf rcs-linux-x86_64.tar.gz && ./rcs/rcs` |
| `rcs_<version>_amd64.deb` | Ubuntu / Debian: `sudo apt install ./rcs_*.deb`, then `rcs` or the menu. |

The robot must run `rosbridge_server` on port 9090, plus the RCS launcher to use the Robot programs page. Setup and everything else are in the **[User guide](https://github.com/Tbm615/AMR-RCS/blob/main/docs/USER_GUIDE.md)**.

### v0.6.1

Fixes found on the real robot. **Update the robot side too** (copy `robot/` to
the robot, `bash install_robot.sh`), and install the camera rule once (below).

**Fixes**
- **The gamepad only drove forward and back, it did not turn.** The turn was
  read from axis 3, which is the right stick's *sideways* on Linux but its
  *up/down* on Windows (SDL with XInput/HIDAPI). Pads SDL knows (Xbox, Flydigi,
  PlayStation...) are now read through SDL's GameController API, the same
  sticks on every system; other pads get their layout from where the triggers
  rest. Either stick turns: the left stick alone, or the left for speed and the
  right for turning.
- **Teleop goes first.** With Nav2 or a mission driving, teleop and Nav2 both
  reached the drives and the robot jerked between the two. Teleop now goes on
  its own topic, `/cmd_vel_teleop`, and the robot's E-STOP gate holds Nav2 back
  while the operator drives and for 0.5 s after; then Nav2 drives on to its
  goal (Events: *teleop goes first*). Tested on the robot: a gamepad turn while
  "Nav2" sent 0.05 m/s forward turned the robot on the spot, and Nav2 went on
  once the stick was let go. An older robot side keeps the old topic.
- **The 3D map got no frames with a working connection.** The camera's color
  picture went on while its depth sensor had stopped, so nothing said why. The
  launcher now counts depth frames and reports the depth sensor's USB speed,
  and the 3D page, Status and Events say what is wrong: depth stopped, or the
  camera driver gone after the camera dropped off USB.
- **Astra+ depth stopped after seconds to minutes on USB 3.** Until now the
  camera always came up as USB 2.0 and depth was fine; once its USB 3 link
  trained, it failed (`-71`, "Cannot enable. Maybe the USB cable is bad?").
  `robot/99-rcs-camera-usb2.rules` + `robot/camera_usb2.py` keep the camera on
  USB 2.0 on any port, and `robot/99-rcs-camera-usb.rules` turns USB autosuspend
  off for it (that took the driver down). Measured: depth at 30 Hz for 10
  minutes, also after a reboot. Install once on the robot:
  ```bash
  cd ~/tbm_test/rcs_robot && chmod +x camera_usb2.py
  sudo cp 99-rcs-camera-usb.rules 99-rcs-camera-usb2.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  ```
  then re-plug the camera (see `robot/README.md`, "Camera").

### v0.6.0

The v0.6 betas, now tested on the real robot (Jetson, Cyclone DDS, JK-BMS over
the USB Bluetooth dongle) by `tests/test_real_robot.py`: the real app against
the real robot side, every page, standing still and driving. **Update the robot
side too** (copy `robot/` to the robot, `bash install_robot.sh`): this app
needs robot side v0.6.0 and says so until it has it.

What v0.6 brings over v0.5.0 (details in the beta notes below):
- **Keepout and speed zones** drawn on the 2D Map, kept on the robot, obeyed
  by Nav2 (beta.2).
- **Sign-in and roles** on the robot, with an audit log (beta.1).
- **Health** on the Status page and a diagnostic bundle (beta.1).
- **Robot models**: the Hinson robot as it is, drive wheels in place (beta.1).
- **The robot's own E-STOP gate**, stopping it when the driving app is lost
  (beta.1).
- **Battery log on the robot** and battery history, **Schedule**, **Fleet**,
  going to charge by itself, Vietnamese (beta.1).
- **Nav2 without the automatic (0, 0) pose**, a launcher that restarts a Nav2
  that hangs while starting, the 3D map recording on every page (beta.2).

New since v0.6.0-beta.2:

**Fixes**
- **The robot drawn at the old pose (often (0, 0)) on a newly started map.**
  The app kept the last `map -> odom` of a stopped SLAM or Nav2 for good, so
  after stopping SLAM and starting Nav2 on a saved map the robot showed at
  that pose (at (0, 0) right after a restart of the base) before anyone set
  the initial pose, and the Status page said *localized*. A transform that
  stops coming now goes after 5 s, and the app drops it at once when the
  robot says navigation stopped or restarts. `/tf_static` is kept as before.
- **Robot side:** after rosbridge crashed, nothing listened on port 9090
  again until a reboot: its launch went on running rosapi alone, so the start
  script never restarted it. The start script now watches rosbridge itself
  and restarts the pair (measured on the robot: back in 17 s).

**Status page**
- **Localization: on the map, but unsure (±0.40 m, ±18°)** when AMCL's spread
  is over 0.30 m or 12°, with what to do. Seen on the robot: on a map built
  from one spot AMCL was 0.15 m and 4.5° off while the page said *ok*.

**Robot programs**
- The SLAM hint says the map grows every 0.5 m driven and that turning on
  the spot adds nothing (measured: a full turn added no scan to the map).

**Measured on the robot** (see `tests/test_real_robot.py` for each check)
- Lidar 20 Hz (4995 points), odometry 100 Hz, camera 27 Hz on the robot;
  in the app over Wi-Fi: scan 8-12 Hz, camera 7-13 frames/s.
- Teleop exactly at the set speed (0.113 m/s at Slow), turning about the
  robot's centre (2.2 cm); E-STOP stops it in 0.32 s and holds with the key
  down; losing the app while driving stops it (the robot's gate, 3 s).
- Goals reached within 0.11 m, Esc cancels at once, a 2-waypoint mission and
  a saved place; keepout zone lethal in the costmap and refused by Nav2 and
  the app; in a 30 % slow zone Nav2 holds 0.121 m/s (limit 0.12), 0.21 s
  after the robot enters.
- Battery: 15 cells, pack voltage = sum of cells. The Bluetooth link drops
  about once or twice a minute (CONNECTION_TIMEOUT, RSSI about -82 dBm) and is
  back in 1-2 s, so the Battery page refreshes with a short gap now and then.
  A dongle nearer the battery (USB extension) would make it rarer. Asking the
  BMS for a longer supervision timeout did not change it (5 vs 6 drops in
  3 min), so that change was not kept.

### v0.6.0-beta.2

A beta, tested on this PC only (mock robots, real Nav2 with the real robot
launcher in simulation), not on the robot yet. **Update the robot side too**
(`robot/install_robot.sh`): this app asks for it.

**Keepout and speed zones**
- Draw them on the 2D Map page (**+ Keepout**, **+ Slow** with its %), on the
  map Nav2 runs on, and **Save to robot**. The robot keeps them next to the
  map (`<map>.zones.json`), so every PC sees the same ones.
- Nav2 obeys them through its costmap filters: a keepout zone is lethal in
  both costmaps (no path into it); in a slow zone Nav2 drives at most the set
  % of its top speed. Saved while Nav2 runs, they apply at once.
- The app refuses a goal or a saved place inside a keepout zone, and warns
  when a mission waypoint is in one. The Teleop page says when the robot is
  in or near a zone; manual driving is not limited by zones.
- Changing zones needs the operator role; each change is in the audit log.
- The robot's own Nav2 parameter file is not changed: the launcher starts
  Nav2 with a copy that has the filters.

**3D Map**
- Records on every page by default (**Record on every page**, remembered):
  the 3D map keeps growing while you use the other pages. Untick it to spare
  Wi-Fi (about 1 MB/s); it then records only while the page is open.

**Fixes**
- **Robot side:** Nav2 on a saved map no longer gets an initial pose of
  (0, 0) a few seconds after it starts. That pose came from
  `robot_navigation.launch.py` and could overwrite the pose just set in the
  app, leaving the robot lost on the map. Set the pose (or **Use last
  position**) after starting Nav2; the Status page says until then.
- **Robot side:** Nav2 on a saved map counts as *running* once its map
  server and AMCL (and the zone filters) say they are active; its navigation
  half then waits for the robot's pose. Now and then a bring-up hangs on a
  lost DDS answer (seen with Fast DDS in simulation: no map, AMCL never
  active, while the launcher said *running*; the robot runs Cyclone DDS). The
  launcher restarts such a Nav2 by itself (at once when a lifecycle manager
  gives up; after 60 s when it is not up; 45 s after the pose when navigation
  is still not active), up to twice, then shows it failed with the reason.
  Stopping a program now ends its whole process group, so nothing it started
  is left running.
- Before the robot is localized, the maps no longer draw it in the map's
  corner at (0, 0), often inside a wall. It is drawn faded, where it last was
  on this map or at a free place in the middle, with *Not on the map yet:
  set the initial pose*.
- The 2D Map side bar scrolls when the window is short.
- **Simulation (Gazebo):** on a map without a known start, the robot starts at
  a free place near the middle of the map instead of (0, 0).
- **Simulation (Gazebo):** the tyres touch the floor on a 1 cm strip at
  ±0.195 m. On their full 4 cm width the physics engine sometimes put the
  contact on the inner rims, and odometry turned 10 % short. Driving under
  Nav2 after a lost connection, which failed now and then, passes now that
  no (0, 0) pose overwrites the robot's.

### v0.6.0-beta.1

A beta: tested on this PC only (mock robots, the robot's own E-STOP and launcher
on real ROS 2, real Nav2 in simulation, the whole robot side in Gazebo), not on
the robot yet. v0.5.0 stays the stable download.

**Robot models**
- **The drive wheels are where they are on the robot**: centred in the body's
  wheel wells, both sides alike. Before, the robot's URDF put the left wheel
  outside the body and the right one inside it; the app's model, RViz and
  Gazebo now all agree. In Gazebo the tyres touch the floor at ±0.195 m
  (wheel_separation 0.39), so the simulated robot turns about its centre and
  its odometry no longer drifts on the spot.
- **The robot seen from above on the 2D maps** (2D Map, Navigation, Missions):
  its body, drive wheels, casters and sensors, with its footprint dashed around
  it, in place of a plain rectangle.
- **A model per robot** (Connection page, Robot model): the Hinson AMR, a box,
  or a model imported from the robot's URDF (meshes STL, OBJ or DAE; boxes,
  cylinders, spheres), from one mesh, or from a file another operator exported.
  It goes into the robot's backups and shares. **Footprint** can now be set on
  the Connection page, or taken from the model.
- **Fleet cards** show each robot from above, turned the way it faces and
  coloured by its state.

**Sign-in and roles** (new robot program `robot/rcs_gateway.py`, see
robot/README.md): once an admin creates a user on the robot, every app signs in
and the robot itself passes on only what the user's role may do: viewers watch
and may stop the robot, operators drive, admins also manage users. An audit
log on the robot says who did what. Optional `wss://` with the certificate
pinned on first use. A robot without users works as before.

**Health and support**
- **Health** on the Status page: each part of the robot green, amber or red,
  with what to do about it.
- **Save diagnostic bundle…** (Status page): the app's log, settings without
  passwords or tokens, events, crash reports and the robot's state, in one zip.
- Crash reports kept with the settings; the app says when it did not close
  properly last time.
- **Going to charge by itself** (Battery page, Charging): below a set charge the
  mission pauses, the robot drives to its charging place, and charged again the
  mission goes on from where it was.
- **Remote alerts** (Events page): Telegram and/or e-mail on errors and, if
  chosen, warnings, at most once per event every 10 minutes.
- **Statistics** (Schedule page): missions, success rate, distance, time at work,
  battery used, E-STOPs and incidents over a period, saved as a PDF report.
- **Installers**: a Windows installer and a .deb package, next to the zip and tar.gz.
- **? button** (next to Layout, or Shift+F1): what the page on screen does.
- **Tiếng Việt**: the app in Vietnamese or English (Connection page, applied at
  the next start).
- **New versions**: the app tells you when a newer release is out (asks GitHub
  once a day, can be turned off), and when the robot side is older than it needs.

**Fixes**
- The 2D Map could stay empty when Nav2 was started from the app after
  connecting (the saved map went out before rosbridge listened): the app now
  asks map_server for it.
- Selecting another robot in the Connection list while connected no longer
  gives the robot being driven that robot's speed limits and size.
- The 3D Map's trajectory no longer starts at the map's origin, with a straight
  line to where the robot really is (a pose was drawn before any was known,
  and odometry and map poses were joined into one line).
- The Fleet page keeps its cards a readable width, in as many columns as fit,
  instead of stretching one card over the page.
- The tests no longer write to your own settings (one of them replaced the
  first robot of the list with a test robot).
- **Robot side:** a rosbridge stopped with a plain `pkill` keeps port 9090 with
  its ROS side shut down, so the app connected and received nothing.
  `start_rcs_robot.sh` now clears such a leftover before starting rosbridge;
  stop it with `pkill -INT` (robot/README.md). The E-STOP gate sends a last
  zero to the drives when it is stopped.

**From the previous work, also in this beta**
- **E-STOP held by the robot** (`robot/rcs_safety.py`): every velocity command
  passes it on the way to the drives. Engaged, nothing moves the robot until an
  operator releases it, through app crashes, Wi-Fi drops and restarts; Nav2
  goals are cancelled. If the app driving the robot goes silent for 3 s, the
  robot stops by itself and the app pauses the mission. Bringup from the app
  now needs it running. **Update the robot side** (`robot/README.md`).
- **Battery log on the robot**: a file per boot, a row every second straight to
  the disk, with everything the BMS reports (each cell and its resistance,
  balancing, alarms, cycles, total Ah, the raw frame once a minute) and the
  link's state. Kept until you delete it. A boot that ended in a power cut is
  told apart from a shut down, with the battery's last values.
- **Battery history** in the app: a chart per boot or a trend over boots, a
  table of boots and how each ended, deleting a boot's log on purpose.
- **BMS link**: back within a second or two after a drop, straight to the known
  BMS with no 12 s scan; why each drop happened is logged; a watchdog restarts
  the battery service if it hangs; the dongle is opened anew after repeated
  failures. The drops themselves (every ~30 s, Bluetooth reason 8, link
  timeout) are not cured: the logs will now say more on the robot.
- **Schedule**: missions at set times and days, skipped with a reason when the
  robot is not ready; a report of every run, exported as CSV.
- **Fleet** page: several robots on one screen, E-STOP for one or all.
- **A virtual robot in Gazebo** for trying the app without the robot
  (`sim/gazebo/run_gazebo.sh`, see the README): the Hinson model on a saved
  map's walls with the robot's own launcher, E-STOP gate, Nav2 and camera.

### v0.5.0

Everything from the betas below, tested on the real robot with the app (except
the Battery page: see below), plus:

- **The real robot on the 3D Map**, from its CAD model, in place of a plain box.
  A robot of another size keeps a box of its own size.

**Update the robot:** copy the whole `robot/` folder over and follow
`robot/README.md`.

**Known limits**
- **Battery:** not yet tried on the real BMS: the robot's Bluetooth has not heard
  it so far. The page says what it can and cannot hear.
- **Camera:** the depth sensor can fail to come up when the camera tries USB 3
  over a bad link. Use a USB 2.0 cable, or a port where it works; if it happens,
  replug the camera (see Troubleshooting in the user guide).

### Changes in v0.5.0-beta.5: the battery in the app

- **Battery page** (last in the sidebar) and the charge in the status bar, from
  anywhere on the network: the robot itself reads the JK-BMS over Bluetooth, so
  no phone next to it is needed. Charge, voltage, current, temperatures, each
  cell, cycles. Warnings at 20% and 10%. Old numbers are marked as old.
- **The robot stays connected** by itself, however weak the signal: it keeps
  searching and reconnecting, and only ever reads from the BMS. The BMS takes
  one connection at a time, so a phone connected to it keeps the robot out.
- Beta: not yet tried on the real BMS, the robot's Bluetooth has not heard it so
  far. Needs the new files on the robot (`robot/README.md`, Battery). If the
  layout of your BMS differs, the page says the frames cannot be used instead
  of showing wrong numbers.
- **Camera:** when the depth sensor is missing, the app now says to replug the
  camera. Rebooting rarely helps: it fails when the camera tries USB 3 over a
  bad link. See Troubleshooting in the user guide.

### Changes in v0.5.0-beta.4: no more blank Camera and 3D Map without a reason

- **The app says why there is no picture.** The Astra+ depth sensor sometimes
  does not come up on USB when the robot boots. The driver then waits silently
  and Robot base still shows *running*, so the Camera and 3D Map pages stayed
  empty with no explanation. Camera, 3D Map and Robot programs now say so, and
  it is logged in Events. Needs the new `rcs_launcher.py` and `camera_check.py`
  on the robot.
- **Fixed: 3D Map empty on some starts.** Connecting before Robot base
  started could lose the robot's static transforms, cutting the camera off from
  the robot. The app now asks for them again.
- **3D Map works with Robot base alone**, in odometry. It starts again when Nav2
  or SLAM brings a map.

### Changes in v0.5.0-beta.2: each robot keeps its own data, share it with others

- **Each robot has its own missions, places and event log** on this PC, and
  the app remembers how it was left with each one: layout, open mission, saved
  map or SLAM, last position on each map. Saved as you go, so a crash loses at
  most a few seconds. Select a robot to open its data; while connected the app
  stays on the connected robot.
- **Nothing restarts by itself.** If the app closed during a mission, it shows
  the waypoint and lap it was at and selects it; you press *Start from selected
  waypoint* to go on.
- **Data page** (new, last in the sidebar): **Back up** a robot's data to one
  `.rcspack` file, **Share** the missions, places and maps with someone else,
  **Import** a file into a robot or as a new robot. Importing only adds: nothing
  you have is replaced, and importing the same file twice adds nothing.
- **Copy maps between the PC and the robot** on the Data page, so a map built on
  one robot can go to another. Needs the new `rcs_launcher.py`, `map_exchange.py`
  and `map_files.py` on the robot (`robot/README.md`); with the old launcher the
  page says so and everything else works.
- **A mission now remembers its map** and refuses to start while Nav2 runs on
  another one: the same coordinates would be somewhere else. Missions made
  before this version have no map, so they are not checked.
- A robot profile from a file cannot go past the limits of the Connection page.
- Fixed: rebuilding the robot list could silently select a different robot.

**On first start** your missions and places are copied to every robot, because
the old list does not say which robot each one was for: delete what does not
belong. The old files stay next to the settings as `*.migrated`.

### Changes in earlier betas, since v0.5.0-beta.1

- **Fusion view on the 3D Map:** the 3D map with the 2D map on the floor and
  the live lidar, planned path, costmaps, goal and mission waypoints in one
  scene, or 3D only, or 2D layers only. **Layers ▾** switches each one.
- **Goals, initial pose and waypoints from the 3D Map:** choose them in
  **Mouse**, press on the floor and drag toward the heading.
- **Clearance bars on Teleop:** free distance ahead, behind and in any
  direction, from the robot's body (green, amber under 0.80 m, red under
  0.25 m). Display only.
- **Events page:** connection, E-STOP, goals, missions, robot programs and
  lost sensors with the time they happened; filter, export to CSV, kept across
  restarts. The sidebar counts new warnings and errors.
- **Map preview on Robot programs:** picture, size, resolution and date of the
  selected map; click to enlarge. Needs the new `rcs_launcher.py` and
  `map_preview.py` on the robot (`robot/README.md`); with the old launcher the
  page says so and everything else works as before.
- **Gamepad on the PC:** hold LB and drive with the sticks; RB changes speed, B
  is E-STOP. **Speed steps** on Teleop: Slow, Medium, Full (keys 1, 2, 3).
- **Saved places:** name a spot, Go there in one click, or add it to a mission.
  **Use last position** sets the initial pose where the robot last was on this
  map.
- **Missions:** run a set number of laps, start from the selected waypoint, undo
  (Ctrl+Z), export and import.
- **Maps:** coordinates under the mouse, and a ruler (Measure distance).
- **Keyboard:** F1–F11 open pages, Ctrl+1–5 the layouts, Esc stops the current
  goal, F12 engages E-STOP from any window.
- **Save your own layouts**, connect to the last robot at start, and sound
  alerts on errors and warnings.
- Pages can shrink: a pane too small for its page gets scroll bars instead of
  stretching the window, and the 3D Map's controls wrap onto more lines.

### Changes in v0.5.0-beta.1: several pages at once, multi-select waypoints

Beta: the new layout was tested on Linux and in CI on Windows, not yet
with the real robot. Nothing changed on the robot side; the v0.4.0 robot
setup works as is.

- **See several pages at once.** A sidebar click still shows the page in
  place of the one you are using. **Ctrl+click** opens it to the right;
  **right-click** a sidebar entry or a tab to open it as a tab, beside,
  below or in **its own window** (e.g. on a second monitor), to move it
  back, or to close it. Drag the bar between pages to resize them.
- **▦ Layout** menu (above E-STOP): *Drive* (Teleop + 2D Map), *Mapping*
  (2D Map + 3D Map), *Monitor* (2D Map + Camera + Status), *Missions*
  (Missions + Camera), *Single page*.
- The layout, page windows included, comes back the next time you start
  the app.
- Pages you cannot see are not drawn, and the camera and 3D map only
  receive data while their page is on screen, as before.
- The 3D Map can be split and tabbed but not yet opened in its own window.
- **Missions: select several waypoints** with Ctrl/Shift-click or by
  dragging (they get a blue ring on the map), then move them with ↑/↓,
  delete them with **Remove** or the **Delete** key, or set their
  tolerance and dwell with **Apply to selected**. **Clear all** empties a
  mission. Reordering and removing are disabled while a mission runs.

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
- Needs the new robot-side launcher, installed once on the robot together with rosbridge. Both then start by themselves at every boot. See [robot/README.md](https://github.com/Tbm615/AMR-RCS/blob/main/robot/README.md).
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
