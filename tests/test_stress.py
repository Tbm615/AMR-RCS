"""Responsiveness, reconnect and 3D-map test against robot-sized traffic.

Run:
    python tests/test_stress.py

Starts the mock rosbridge in --heavy mode in its own process (a 1000 x 1000
map and costmap, 4995-point scans at 20 Hz, a synthetic RGB-D camera whose
frames arrive 0.3 s late) and drives the app like an operator: open the map
pages, record the 3D map while the robot turns, kill rosbridge and bring it
back, disconnect and reconnect. Throughout, a 10 ms timer measures how long
the GUI event loop stalls; Windows shows "Not Responding" after 5 s.
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_stress_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]

import numpy as np
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from rcs.core.profile import RobotProfile
from rcs.ui import theme
from rcs.ui.main_window import MainWindow
from tests import synthetic_rgbd as cam

PORT = 9193
MAX_STALL_S = float(os.environ.get("RCS_MAX_STALL_S", "0.5"))   # longest acceptable GUI freeze
app = QApplication(sys.argv)
PASS = FAIL = 0


def check(name: str, cond: bool, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


class StallMeter:
    def __init__(self) -> None:
        self.gaps: list = []
        self.last = time.perf_counter()
        self.t = QTimer()
        self.t.setInterval(10)
        self.t.timeout.connect(self._tick)
        self.t.start()

    def _tick(self) -> None:
        now = time.perf_counter()
        self.gaps.append(now - self.last)
        self.last = now

    def reset(self) -> None:
        self.gaps = []
        self.last = time.perf_counter()

    def report(self, label: str) -> float:
        g = np.array(self.gaps or [0.0])
        worst = float(g.max())
        check(f"GUI responsive: {label}", worst < MAX_STALL_S,
              f"(worst stall {worst * 1000:.0f} ms, p99 {np.percentile(g, 99) * 1000:.0f} ms, "
              f"{len(g)} ticks)")
        return worst


def run_for(seconds: float, every=None) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        if every:
            every()
        time.sleep(0.005)


def wait_until(cond, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    return False


def start_mock() -> subprocess.Popen:
    p = subprocess.Popen([sys.executable, str(HERE / "mock_rosbridge.py"), "--heavy",
                          "--port", str(PORT), "--duration", "120"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)
    return p


def wall_error(xyz: np.ndarray) -> np.ndarray:
    """Distance of each point to the nearest wall of the synthetic room."""
    r = cam.ROOM
    return np.min(np.stack([abs(xyz[:, 0] - r["x1"]), abs(xyz[:, 0] - r["x0"]),
                            abs(xyz[:, 1] - r["y1"]), abs(xyz[:, 1] - r["y0"])]), axis=0)


def main() -> int:
    mock = start_mock()
    theme.apply(app)
    w = MainWindow()
    w.resize(1400, 880)
    w.show()
    meter = StallMeter()

    prof = RobotProfile.new(name="heavy")
    prof.host, prof.port = "127.0.0.1", PORT
    w.profiles.upsert(prof)
    t0 = time.monotonic()
    w._on_connect(prof)
    check("connect returns at once", time.monotonic() - t0 < 0.3, f"({time.monotonic() - t0:.2f} s)")
    check("connected", wait_until(w.ros.is_connected, 10))

    print("\n[2D pages under load]")
    meter.reset()
    for page in ("map", "nav", "mission", "status", "map"):
        w.btns[page].click()
        run_for(2.0)
    w.p_map.ck_gc.setChecked(True)
    w.p_map.ck_lc.setChecked(True)
    run_for(3.0)
    meter.report("2D map / navigation pages with big map + costmaps")
    check("big map drawn", w.p_map.view.map_img.image is not None
          and w.p_map.view.map_img.image.shape == (1000, 1000))
    check("scan drawn (4995-point scans)", len(w.p_map.view.scan_scatter.getData()[0]) > 4000)

    print("\n[3D map while the robot turns]")
    w.btns["3d"].click()
    meter.reset()
    turn = lambda: w.ros.publish_cmd_vel(0.0, 0.6)        # noqa: E731
    run_for(12.0, every=turn)
    w.ros.publish_cmd_vel(0.0, 0.0)
    meter.report("3D map recording (RGB-D frames 0.3 s late)")
    xyz, rgb, _ = w.ros.cloud_map.snapshot()
    check("3D map has points", len(xyz) > 5000, f"({len(xyz)} points, {w.ros.map3d_frames} frames)")
    walls = (xyz[:, 2] > 0.4) & (xyz[:, 2] < 2.0)
    err = wall_error(xyz[walls]) if walls.any() else np.array([9.0])
    check("walls land on the room walls (TF + yaw right)", np.percentile(err, 90) < 0.08,
          f"(90% within {np.percentile(err, 90) * 100:.1f} cm)")
    red = walls & (abs(xyz[:, 0] - cam.ROOM["x1"]) < 0.08) & (abs(xyz[:, 1]) < 2.5)
    if red.any():
        c = rgb[red].astype(int)
        frac = float(np.mean((c[:, 0] > c[:, 1] + 40) & (c[:, 0] > c[:, 2] + 40)))
        check("wall colors match (red wall is red)", frac > 0.8, f"({frac * 100:.0f}% red)")
    else:
        check("red wall seen", False)
    floor = (abs(xyz[:, 2]) < 0.05)
    check("floor is flat at z=0", floor.sum() > 100, f"({int(floor.sum())} floor points)")

    print("\n[PLY save / load]")
    tmp = Path(tempfile.mkdtemp(prefix="rcs_ply_")) / "map3d.ply"
    w.p_3d.rec_btn.setChecked(False)
    run_for(1.0)                     # let the last in-flight frame land
    n = w.ros.cloud_map.save_ply(tmp)
    w.p_3d._clear()
    left = len(w.ros.cloud_map)
    check("clear empties the map", left == 0, f"({left} left, streams={w.ros._streams_on})")
    m = w.p_3d.load_ply(str(tmp))
    check("PLY round-trip keeps every point", m == n, f"({n} -> {m})")
    check("loading a map pauses recording", not w.p_3d.rec_btn.isChecked())
    report_ply = HERE.parent.parent / "result" / "run_20261016_154638" / "map" / "cloud3d.ply"
    if report_ply.exists():
        t = time.monotonic()
        k = w.p_3d.load_ply(str(report_ply))
        check("opens save_run's cloud3d.ply", k > 100000, f"({k} points in {time.monotonic() - t:.2f} s)")

    print("\n[rosbridge dies and comes back]")
    w.btns["map"].click()
    meter.reset()
    mock.kill()
    mock.wait()
    check("app notices the drop", wait_until(lambda: not w.ros.is_connected(), 5))
    run_for(4.0)
    meter.report("while rosbridge is down")
    mock = start_mock()
    check("reconnects by itself", wait_until(w.ros.is_connected, 12))
    n_before = w.ros.topic_stats.get("scan").last_t if w.ros.topic_stats.get("scan") else 0
    check("data flows again after reconnect",
          wait_until(lambda: w.ros.topic_stats.get("scan") and w.ros.topic_stats["scan"].last_t > n_before
                     and w.ros.topic_stats["scan"].fresh(), 6))

    print("\n[manual disconnect / connect]")
    meter.reset()
    w.ros.disconnect_robot()
    check("disconnected", wait_until(lambda: not w.ros.is_connected(), 5))
    run_for(1.0)
    w._on_connect(prof)
    check("connects again after a manual disconnect", wait_until(w.ros.is_connected, 10))
    check("streams again", wait_until(lambda: w.ros.topic_stats.get("odom") is not None
                                      and w.ros.topic_stats["odom"].fresh(0.5), 6))
    meter.report("disconnect / reconnect")

    print("\n[unreachable robot]")
    meter.reset()
    bad = RobotProfile.new(name="nowhere")
    bad.host, bad.port = "10.255.255.1", 9090        # blackholed address: connect hangs
    t0 = time.monotonic()
    w._on_connect(bad)
    check("connecting to an unreachable host does not block", time.monotonic() - t0 < 0.3)
    run_for(4.0)
    meter.report("connecting to an unreachable host")
    w.ros.disconnect_robot()

    mock.kill()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    rc = main()
    print(f"exit code: {rc}")
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(rc)
