"""Copy maps through the real robot/rcs_launcher.py and a real rosbridge_server.

Run (needs ROS 2 Humble and rosbridge_suite on this PC, no robot):
    BRIDGE_WS=../../rcs_sim_ws python tests/test_real_launcher.py

The launcher works on a temporary workspace, on its own ROS domain, on
loopback only: it cannot see or touch a real robot.
"""
from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_real_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]

from PySide6.QtCore import QCoreApplication

from rcs.core import map_files
from rcs.core.map_transfer import MapTransfer
from rcs.core.maps import MapLibrary
from rcs.core.profile import RobotProfile
from rcs.core.ros_client import RosClient

PORT = 9196
app = QCoreApplication(sys.argv)
PASS = FAIL = 0


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def wait_until(cond, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    return False


def main() -> int:
    ws = Path(tempfile.mkdtemp(prefix="rcs_real_ws_"))
    src = HERE.parent.parent / "src/navigation/maps"
    (ws / "maps").mkdir()
    for f in ("map_3269.yaml", "map_3269.pgm"):
        shutil.copy(src / f, ws / "maps" / f)
    bridge = os.environ.get("BRIDGE_WS", "")
    setup = "source /opt/ros/humble/setup.bash" + (f" && source {bridge}/install/setup.bash" if bridge else "")
    env = {**os.environ, "ROS_DOMAIN_ID": "78", "ROS_LOCALHOST_ONLY": "1",
           "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "RCS_WS": str(ws)}
    env.pop("CYCLONEDDS_URI", None)
    env.pop("VIRTUAL_ENV", None)
    procs = [subprocess.Popen(["bash", "-c", f"{setup} && exec {cmd}"], env=env, start_new_session=True,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
             for cmd in (f"ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:={PORT}",
                         f"/usr/bin/python3 -u {HERE.parent / 'robot/rcs_launcher.py'}")]
    try:
        return run(ws)
    finally:
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(1.5)
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def run(ws: Path) -> int:
    prof = RobotProfile.new("sim")
    prof.host, prof.port = "127.0.0.1", PORT
    ros = RosClient(profile=prof)
    lib = MapLibrary(ws.parent / (ws.name + "_lib"))
    xfer = MapTransfer(ros, lib)
    state: dict = {}
    ros.launcher_state.connect(state.update)
    ros.connect_robot()
    check("launcher answers through rosbridge", wait_until(lambda: "maps" in state, 30.0))
    check("says it can copy maps", "map_transfer" in state.get("caps", []), f"{state.get('caps')}")

    def go(start) -> tuple:
        out = []
        slot = lambda ok, t: out.append((ok, t))        # noqa: E731
        xfer.finished.connect(slot)
        start()
        wait_until(lambda: out, 40.0)
        xfer.finished.disconnect(slot)
        return out[0] if out else (None, "timed out")

    ok, text = go(lambda: xfer.download("map_3269"))
    check("map copied from the launcher", ok and lib.names() == ["map_3269"], text)
    check("same as on the robot", lib.read("map_3269") == map_files.read_map(ws / "maps/map_3269.yaml"))

    ok, text = go(lambda: xfer.upload("map_3269"))
    check("name taken: refused", ok is False and "already" in text, text)

    meta, _ = lib.read("map_3269")
    img = os.urandom(800 * 800)         # does not compress: several messages
    lib.add("from_pc", meta, f"P5\n800 800\n255\n".encode() + img)
    ok, text = go(lambda: xfer.upload("from_pc"))
    check("big map sent", ok, text)
    y = ws / "maps/from_pc.yaml"
    check("written to the workspace", y.exists() and map_files.read_map(y)[1].endswith(img))
    check("listed by the launcher", wait_until(lambda: "from_pc" in state.get("maps", []), 4.0))
    ok, text = go(lambda: xfer.upload("from_pc", overwrite=True))
    check("replaced when confirmed", ok, text)
    check("no leftovers", not list((ws / "maps").glob("*.tmp")))
    ros.disconnect_robot()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    os._exit(code)
