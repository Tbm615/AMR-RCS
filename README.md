# RCS — Robot Control System

Standalone desktop operator console for Hinson-class differential-drive AMRs,
designed to scale to a fleet of similar robots. Runs on Windows and Linux;
**does not require ROS 2 or Python on the operator PC** — the shipped
binaries are self-contained.

The robot runs `rosbridge_server` on port 9090; the app connects over
WebSocket and does everything else client-side in Python/Qt.

![2D map view with occupancy grid, scan, planned path and robot pose](docs/screenshots/map.png)

See `docs/screenshots/` for all panels: connection, status, teleop, map,
navigation, mission editor, camera, logs.

## Features

- Multi-robot profile manager (add/edit/select, persisted to disk).
- Live connection health with reconnect.
- Teleop: virtual joystick, keyboard (W/A/S/D or arrows), speed limits.
- Latched E-stop that beats any teleop stream.
- Status: battery (if available), velocities, topic rates, TF freshness.
- 2D map view: `OccupancyGrid` + live `/scan_filtered` overlay + robot pose
  from TF, click-to-send `NavigateToPose` goal.
- 3D view: camera point cloud + robot footprint.
- Nav view: Nav2 global/local costmaps, planned path, controller status.
- Missions: ordered waypoints with per-point tolerance and dwell;
  optional **loop route**; run/pause/stop; "set here" from current pose.
- Camera: compressed image stream (JPEG) from `/camera/color/image_raw/compressed`.
- Logs/diagnostics panel.

## Run from source (developer)

```bash
cd RCS
python -m venv .venv
# linux: source .venv/bin/activate   # windows: .venv\Scripts\activate
pip install -r requirements.txt
python -m rcs
```

## Testing without the robot

| Test | What it needs | What it checks |
|---|---|---|
| `tests/test_features.py` | nothing | map/scan/path rendering, PointCloud2 and JPEG decoding, TF chains, mission executor incl. retries (31 checks) |
| `tests/test_e2e.py` | nothing (built-in mock rosbridge) | the app connects, subscribes, streams teleop, publishes goals |
| `tests/test_built_binary_e2e.py` | a build in `dist/` | the packaged binary does the same |
| `tests/test_sim_nav2.py` | ROS 2 Humble on the PC, `sim/run_sim.sh` running | the app drives **real Nav2** through **real rosbridge** (26 checks) |

All tests run headless with `QT_QPA_PLATFORM=offscreen`.

### Simulated robot with real Nav2 (`sim/`)

`sim/run_sim.sh` starts, on an isolated ROS domain (77, loopback only):
`rosbridge_server` on port 9090, this workspace's
`navigation/robot_navigation.launch.py` (AMCL, SmacPlanner2D + MPPI, CBF
filter) on `maps/map23_04.yaml`, and `sim/fake_base.py`, a kinematic stand-in
for the drive and lidar (odom, TF, `/scan_filtered` ray-cast on the map).
`sim/stop_sim.sh` stops it all. Point RCS at `127.0.0.1:9090` to use it
interactively, or run `tests/test_sim_nav2.py`, which covers: localizing from
the app, scan-to-map alignment through `/tf_static`, goal from the map panel,
cancel, E-stop while driving, and a looping mission.

If `rosbridge_suite` is not installed via apt, build it into a separate
workspace and pass it as `BRIDGE_WS=/path/to/ws`.

## Build standalone executable

**Important:** build inside a clean virtualenv — PyInstaller otherwise bundles
every library it finds in your user site-packages (CUDA/Torch/etc.) and the
artifact balloons to multiple gigabytes. The project's `.venv` is fine:

```bash
python -m venv .venv
source .venv/bin/activate   # or: .venv\Scripts\activate on Windows
pip install -r requirements.txt pyinstaller
python build.py             # builds dist/rcs/ (linux) or dist/rcs\ (windows)
```

Default build is `--onedir` (a folder you ship). Set `RCS_ONEFILE=1` to try
`--onefile`; this can fail on some binutils versions with an objcopy error —
fall back to `--onedir` if so.

### Cross-platform packaging via GitHub Actions

`.github/workflows/build.yml` runs on every push, on a real Linux and a
real Windows machine: feature tests, the source E2E test, then builds the
app and runs the built binary against the mock rosbridge. On Windows it
builds and tests both the console and the windowed (no console) variants.
Download the archives from the run's Artifacts section on the Actions tab.

## Robot side

On the robot PC, install and launch `rosbridge_server`:

```bash
sudo apt install ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

See `scripts/robot_setup_rosbridge.md` for a systemd unit template.

## Safety notes

- The app's **E-STOP is software**: it cancels the Nav2 goal, stops the
  running mission, publishes `/amr/estop=true` and keeps sending zero
  velocity on `/cmd_vel_safe`. Nothing on the robot consumes `/amr/estop`
  today, and the driver follows the latest message on `/cmd_vel_safe`, so
  another publisher (e.g. the gamepad) can still move the robot. Keep the
  hardware E-stop within reach.
- Teleop only publishes while you are driving (plus 0.3 s of zero commands
  when you let go), so it never blocks Nav2.
- Teleop goes to `/cmd_vel_safe` and bypasses the CBF filter, like the
  gamepad and the web console.
