"""No camera picture: saying why, and the 3D map with no map frame yet.

Run:
    python tests/test_camera_status.py

Needs no robot. The real fault behind this: the Astra+ depth sensor can miss
the USB bus at boot, the driver then waits silently and bringup looks fine.
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "robot"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_cam_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]

import numpy as np
from PySide6.QtWidgets import QApplication

from camera_check import camera_usb
from rcs.core.camera_status import NO_DEPTH_USB, NO_USB, camera_problem, usb_problem
from rcs.core.profile import RobotProfile
from rcs.core.ros_client import RosClient
from rcs.ui.main_window import MainWindow

app = QApplication(sys.argv)
PASS = FAIL = 0


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def usb_tree(*devices) -> Path:
    root = Path(tempfile.mkdtemp(prefix="rcs_sysfs_"))
    (root / "usb1-interface").mkdir()           # as an interface: no idVendor
    for i, (vendor, product) in enumerate(devices):
        d = root / f"1-2.{i}"
        d.mkdir()
        (d / "idVendor").write_text(vendor + "\n")
        (d / "idProduct").write_text(product + "\n")
    return root


def state(base="running", cam=None, external=False) -> dict:
    st = {"stacks": {"bringup": {"state": base, "external": external}}}
    if cam is not None:
        st["camera"] = cam
    return st


def tf(parent: str, child: str) -> dict:
    return {"transforms": [{"header": {"frame_id": parent, "stamp": {"sec": 1, "nanosec": 0}},
                            "child_frame_id": child,
                            "transform": {"translation": {"x": 1.0, "y": 0.0, "z": 0.0},
                                          "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}]}


def main() -> int:
    print("\n[USB on the robot]")
    both = usb_tree(("2bc5", "0536"), ("2bc5", "0636"), ("05e3", "0610"))
    check("both found", camera_usb(both) == {"color": True, "depth": True})
    check("depth missing, as seen at boot", camera_usb(usb_tree(("2bc5", "0536"))) == {"color": True, "depth": False})
    check("none", camera_usb(usb_tree(("05e3", "0610"))) == {"color": False, "depth": False})
    check("no such folder is not an error", camera_usb("/nonexistent") == {"color": False, "depth": False})

    print("\n[what the operator is told]")
    up = {"color": True, "depth": True, "publishers": 1}
    t = "/cam"
    check("frames: nothing to say", camera_problem(True, state(cam=up), True, t) is None)
    check("offline", "Not connected" in camera_problem(False, {}, False, t))
    check("depth sensor missing", camera_problem(True, state(cam={"color": True, "depth": False, "publishers": 0}),
                                                 False, t) == NO_DEPTH_USB)
    check("said even with the base off", camera_problem(
        True, state("stopped", {"color": True, "depth": False}), False, t) == NO_DEPTH_USB)
    check("camera missing", camera_problem(True, state(cam={"color": False, "depth": False}), False, t) == NO_USB)
    check("base off", "start Robot base" in camera_problem(True, state("stopped", up), False, t))
    check("started by hand counts as up", "start Robot base" not in camera_problem(
        True, state("stopped", up, external=True), False, t))
    check("driver starting", "starting" in camera_problem(True, state(cam={**up, "publishers": 0}), False, t))
    check("running but nothing arrives: names the topic", t in camera_problem(True, state(cam=up), False, t))
    check("no launcher", "does not answer" in camera_problem(True, {}, False, t))
    check("old launcher", "Update rcs_launcher" in camera_problem(True, state(), False, t))
    check("no usb info, no usb claim", usb_problem(state()) is None)

    print("\n[pages]")
    w = MainWindow()
    w.show()
    w.ros.is_connected = lambda: True
    w._on_launcher_events(state(cam={"color": True, "depth": False, "publishers": 0}))
    w._update_camera_status()
    check("Camera page says it", w.p_cam.problem_lbl.text() == NO_DEPTH_USB and not w.p_cam.problem_lbl.isHidden())
    check("3D page says it", w.p_3d.camera_problem == NO_DEPTH_USB)
    check("Robot programs says it", w.p_robot.cam_lbl.text() == NO_DEPTH_USB and not w.p_robot.cam_lbl.isHidden())
    w._update_camera_status()
    n = sum(e.text == NO_DEPTH_USB for e in w.events.events)
    check("logged once, not every second", n == 1, f"{n}")
    w._on_launcher_events(state(cam=up))
    w.ros._mark("image")
    w._update_camera_status()
    check("gone when frames arrive", w.p_cam.problem_lbl.isHidden() and not w.p_3d.camera_problem
          and w.p_robot.cam_lbl.isHidden())
    del w.ros.is_connected
    w.ros.topic_stats.clear()
    w._on_ros_disconnected()
    w._update_camera_status()
    check("offline says so", "Not connected" in w.p_cam.problem_lbl.text())
    w.close()

    print("\n[3D map before there is a map frame]")
    r = RosClient(profile=RobotProfile.new("t"))
    r.tf.ingest(tf("odom", "base_link"))
    r.tf.ingest(tf("base_link", "cam"))
    check("bringup alone: built in odom", r._world_frame() == "odom")
    check("finds the camera", r.tf.matrix("odom", "cam") is not None)
    r.cloud_map.integrate(np.array([[1.0, 2.0, 0.5]], np.float32), None)
    check("has a point", len(r.cloud_map) == 1)
    check("same frame keeps it", r._world_frame() == "odom" and len(r.cloud_map) == 1)
    r.tf.ingest(tf("map", "odom"))
    check("map arrives: built in map", r._world_frame() == "map")
    check("starts again, the frames do not line up", len(r.cloud_map) == 0)
    r.cloud_map.integrate(np.array([[1.0, 2.0, 0.5]], np.float32), None)
    check("stays on map", r._world_frame() == "map" and len(r.cloud_map) == 1)

    print("\n[static transforms missed by connecting before bringup]")
    s = RosClient(profile=RobotProfile.new("t"))
    calls = []

    class Sub:
        def unsubscribe(self):
            calls.append("off")

    s._ros = object()
    s._subs["/tf_static"] = Sub()
    s._subscribe = lambda name, *a, **k: (calls.append(name), s._subs.__setitem__(name, Sub()))
    s.refresh_static_tf()
    check("subscribes again", calls == ["off", "/tf_static"], f"{calls}")
    s.refresh_static_tf()
    check("not on every frame", len(calls) == 2)
    s._static_tf_t -= 10.0
    s.refresh_static_tf()
    check("tries again a little later", len(calls) == 4)
    s._static_tf_t = -1e9
    calls.clear()
    base = lambda state: {"data": json.dumps({"stacks": {"bringup": {"state": state}}})}     # noqa: E731
    s._on_launcher_state(base("starting"))
    check("not while starting", not calls)
    s._on_launcher_state(base("running"))
    check("when the base comes up", calls == ["off", "/tf_static"], f"{calls}")
    s._static_tf_t = -1e9
    calls.clear()
    s._on_launcher_state(base("running"))
    check("only on the change", not calls)


    empty = RosClient(profile=RobotProfile.new("t"))
    check("no TF at all: map, reported as an error by the caller", empty._world_frame() == "map")

    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    os._exit(code)
