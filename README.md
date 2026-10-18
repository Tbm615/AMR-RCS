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

## End-to-end test (no robot needed)

`tests/test_e2e.py` spins up a minimal mock rosbridge WebSocket server
(`tests/mock_rosbridge.py`), connects the full application to it, drives
teleop + a nav goal, and asserts the server received the right messages
and the client subscribed to the right topics. Runs headless:

```bash
QT_QPA_PLATFORM=offscreen python tests/test_e2e.py
```

The current result on this machine is **ALL CHECKS PASSED** (11 checks).

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

`ci/github-actions-build.yml` builds **both** Linux (`.tar.gz`) and Windows
(`.zip`) artifacts, runs the smoke + E2E tests, and attaches them to the
workflow run. To enable it, move it to `.github/workflows/build.yml` and
push (pushing workflow files needs a token with the `workflow` scope), then
download the `rcs-windows-x86_64` artifact from the Actions tab.

## Robot side

On the robot PC, install and launch `rosbridge_server`:

```bash
sudo apt install ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

See `scripts/robot_setup_rosbridge.md` for a systemd unit template.
