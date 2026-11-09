"""The two documentation screenshots tests/screenshot_live.py cannot take.

  robot  docs/screenshots/robot.png: the Robot programs page, against the mock
         robot (tests/mock_rosbridge.py) listing the maps of a folder.
             QT_QPA_PLATFORM=offscreen python tests/screenshot_extra.py robot <maps folder>
  3d     docs/screenshots/map3d.png: the 3D Map with a saved 3D map (PLY), the
         2D map under it and the robot's model in it. Needs a real display
         (OpenGL); the picture is scaled to 1440 x 900 on a HiDPI screen.
             python tests/screenshot_extra.py 3d <folder with cloud3d.ply and map2d.yaml>

The maps folder is e.g. src/navigation/maps of the workspace; the 3D folder a
result/run_*/map folder written by bringup's save_run.
"""
from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

os.environ["XDG_CONFIG_HOME"] = os.environ["APPDATA"] = tempfile.mkdtemp(prefix="rcs_shot_cfg_")
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
OUT = ROOT / "docs" / "screenshots"


def mock_with_maps(maps: str, argv: list) -> None:
    """Child process: the mock robot, its launcher listing the maps of a folder."""
    sys.path.insert(0, str(ROOT / "tests"))
    sys.argv = [sys.argv[0]] + argv
    import mock_rosbridge as mb
    lau = mb.LAUNCHER
    lau.maps = sorted(n[:-5] for n in os.listdir(maps) if n.endswith(".yaml"))
    for n in os.listdir(maps):
        shutil.copy2(os.path.join(maps, n), lau.map_dir)

    def preview(name):
        out = {"name": name, "saved": False,
               **mb.map_preview.map_preview(os.path.join(lau.map_dir, name + ".yaml"))}
        return {"op": "publish", "topic": "/rcs/launcher/preview", "msg": {"data": json.dumps(out)}}

    plain = lau.state_msg

    def state_msg():
        m = plain()
        st = json.loads(m["msg"]["data"])
        st["map_mtimes"] = {n: os.path.getmtime(os.path.join(lau.map_dir, n + ".yaml")) for n in lau.maps}
        m["msg"]["data"] = json.dumps(st)
        return m

    lau._preview, lau.state_msg = preview, state_msg
    mb.main()


if len(sys.argv) > 1 and sys.argv[1] == "--mock":
    mock_with_maps(sys.argv[2], sys.argv[3:])
    sys.exit(0)

import numpy as np  # noqa: E402
from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

app = QApplication(sys.argv)
from rcs.core import map_files  # noqa: E402
from rcs.ui import theme  # noqa: E402
from rcs.ui.main_window import MainWindow  # noqa: E402


def pump(sec: float) -> None:
    end = time.monotonic() + sec
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.01)


def wait_until(cond, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    return False


def save(w, name: str) -> None:
    pm = w.grab()
    if pm.width() != 1440:
        pm = pm.scaled(1440, 900, Qt.KeepAspectRatio, Qt.SmoothTransformation)
    path = OUT / name
    pm.save(str(path))
    print(f"  wrote {path.relative_to(ROOT)}", flush=True)


def robot_page(w, maps: str) -> None:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    bridge = subprocess.Popen([sys.executable, __file__, "--mock", maps, "--robot", "--heavy",
                               "--port", str(port), "--name", "Hinson robot"], cwd=str(ROOT))
    try:
        prof = w.profiles.active()
        prof.host, prof.port = "127.0.0.1", port
        w.profiles.upsert(prof)
        w.p_conn._reload()
        time.sleep(1.5)
        w._on_connect(prof)
        assert wait_until(w.ros.is_connected, 15), "the mock robot did not answer"
        w._show("robot")
        page = w.p_robot
        assert wait_until(page.base.start_btn.isEnabled, 15), "launcher not online"
        page.base.start_btn.click()
        wait_until(lambda: "running" in page.base.status.text(), 10)
        pump(1.5)
        page.rb_map.setChecked(True)
        if page.map_combo.count() > 1:
            sizes = {n: os.path.getsize(os.path.join(maps, n + ".pgm")) if os.path.exists(
                os.path.join(maps, n + ".pgm")) else 0 for n in (page.map_combo.itemText(i)
                                                                 for i in range(page.map_combo.count()))}
            page.map_combo.setCurrentText(max(sizes, key=sizes.get))     # the biggest map shows best
        pump(2)
        page.nav.start_btn.click()
        wait_until(lambda: "running" in page.nav.status.text(), 10)
        pump(6)
        save(w, "robot.png")
        w.ros.disconnect_robot()
        pump(0.5)
    finally:
        bridge.terminate()
        bridge.wait(5)


def map3d(w, folder: Path) -> None:
    from scipy.ndimage import distance_transform_edt
    meta, pgm = map_files.read_map(folder / "map2d.yaml")
    wd, ht = map_files.pgm_size(pgm)
    img = np.frombuffer(pgm[-wd * ht:], np.uint8).reshape(ht, wd)[::-1]
    occ = np.where(img < 100, 100, np.where(img > 230, 0, -1)).astype(np.int8)
    res, (ox, oy, _) = float(meta["resolution"]), meta["origin"]
    w._show("3d")
    pump(1.0)
    p = w.p_3d
    print(f"  {p.load_ply(str(folder / 'cloud3d.ply')):,} points", flush=True)
    p.set_occupancy({"header": {"frame_id": "map"},
                     "info": {"width": wd, "height": ht, "resolution": res,
                              "origin": {"position": {"x": ox, "y": oy, "z": 0.0},
                                         "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
                     "data": occ.ravel().tolist()})
    # the robot on a free spot 0.7 m from any wall, nearest the middle of the 3D map
    xyz = p.cloud_map.snapshot()[0]
    lo, hi = np.percentile(xyz, 2, axis=0), np.percentile(xyz, 98, axis=0)
    cx, cy = ((lo[0] + hi[0]) / 2 - ox) / res, ((lo[1] + hi[1]) / 2 - oy) / res
    clear = distance_transform_edt(occ == 0)
    rows, cols = np.indices(occ.shape)
    score = np.where(clear > 0.7 / res, -np.hypot(rows - cy, cols - cx), -1e9)
    r, c = np.unravel_index(np.argmax(score), occ.shape)
    rx, ry = ox + (c + 0.5) * res, oy + (r + 0.5) * res
    p.ck_robot.setChecked(True)
    for i in range(30):                     # a short drive, so the trajectory shows
        p.update_robot_pose(rx - 0.6 + i * 0.02, ry, 0.35, frame="map")
    pump(0.5)
    p.perspective_view()
    p.view.setCameraPosition(distance=12.0)
    pump(2.0)
    save(w, "map3d.png")


def main() -> int:
    if len(sys.argv) < 3 or sys.argv[1] not in ("robot", "3d"):
        print(__doc__)
        return 2
    theme.apply(app)
    w = MainWindow()
    w.resize(1440, 900)
    w.show()
    if sys.argv[1] == "robot":
        robot_page(w, sys.argv[2])
    else:
        map3d(w, Path(sys.argv[2]))
    return 0


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    os._exit(rc)
