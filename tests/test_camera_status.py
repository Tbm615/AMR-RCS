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
from rcs.core import robot_model
from rcs.core.camera_status import (DEPTH_STOPPED, DEPTH_STOPPED_USB3, DRIVER_GONE, NO_DEPTH_USB, NO_USB,
                                    camera_problem, depth_problem, usb_problem)
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
    check("both found", camera_usb(both) == {"color": True, "depth": True, "depth_usb": 0})
    check("depth missing, as seen at boot", camera_usb(usb_tree(("2bc5", "0536")))
          == {"color": True, "depth": False, "depth_usb": 0})
    check("none", camera_usb(usb_tree(("05e3", "0610"))) == {"color": False, "depth": False, "depth_usb": 0})
    check("no such folder is not an error", camera_usb("/nonexistent") == {"color": False, "depth": False,
                                                                           "depth_usb": 0})
    (both / "1-2.1" / "speed").write_text("5000\n")
    check("depth sensor's link speed", camera_usb(both)["depth_usb"] == 5000, str(camera_usb(both)))

    import camera_usb2
    check("camera hub on a hub port: that hub, that port",
          camera_usb2.parent_and_port(Path("/sys/devices/x/usb2/2-1/2-1.4")) == (Path("/sys/devices/x/usb2/2-1"), 4))
    check("on a root hub port: the root hub", camera_usb2.parent_and_port(Path("/sys/devices/x/usb2/2-1"))
          == (Path("/sys/devices/x/usb2"), 1))
    hub = usb_tree(("05e3", "0620"))
    (hub / "1-2.0" / "speed").write_text("480\n")
    import subprocess
    r = subprocess.run([sys.executable, str(HERE.parent / "robot" / "camera_usb2.py"), str(hub / "1-2.0")],
                       capture_output=True, text=True)
    check("already USB 2: nothing to do, no error", r.returncode == 0 and not r.stdout, r.stdout + r.stderr)

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

    print("\n[depth stopped while the color picture goes on]")

    def running(cam, up_s=60.0, base="running", external=False):
        st = state(base, cam, external)
        st["time"] = 1000.0 + up_s
        st["stacks"]["bringup"]["since"] = 1000.0
        return st
    dead = {**up, "depth_hz": 0.0, "depth_usb": 5000}
    check("as on the robot: USB 3, depth stopped", depth_problem(running(dead)) == DEPTH_STOPPED_USB3)
    check("USB 2: replug", depth_problem(running({**dead, "depth_usb": 480})) == DEPTH_STOPPED)
    check("depth arriving: nothing", depth_problem(running({**dead, "depth_hz": 29.5})) is None)
    check("just started: give it time", depth_problem(running(dead, up_s=5)) is None)
    check("base off: not this", depth_problem(running(dead, base="stopped")) is None)
    check("base started by hand counts", depth_problem(running(dead, base="stopped", external=True))
          == DEPTH_STOPPED_USB3)
    check("driver not up yet: not this", depth_problem(running({**dead, "publishers": 0})) is None)
    check("old robot side (no count): nothing claimed", depth_problem(running(up)) is None)
    check("depth not on USB: the USB message, not this", depth_problem(running({**dead, "depth": False})) is None)
    check("driver gone long after bringup: says so", camera_problem(True, running({**up, "publishers": 0}), False, t)
          == DRIVER_GONE)
    check("right after bringup: still starting", "starting" in camera_problem(
        True, running({**up, "publishers": 0}, up_s=10), False, t))

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
    w.ros.is_connected = lambda: True
    w._on_launcher_events(running(dead))
    w.ros._mark("image")                # the color picture still comes
    w._update_camera_status()
    check("depth stopped: the Camera page shows the picture", w.p_cam.problem_lbl.isHidden())
    check("the 3D page says why it gets nothing", w.p_3d.camera_problem == DEPTH_STOPPED_USB3)
    check("and the events, once", sum(e.text == DEPTH_STOPPED_USB3 for e in w.events.events) == 1)
    w._on_launcher_events(running({**dead, "depth_hz": 30.0}))
    w._update_camera_status()
    check("depth back: the 3D page says nothing", not w.p_3d.camera_problem)
    del w.ros.is_connected
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

    print("\n[a saved map sent once, before rosbridge heard it]")
    import rcs.core.ros_client as rc
    asked, got = [], []

    class FakeService:
        def __init__(self, ros, name, typ):
            self.name = name

        def call(self, req, ok, err):
            asked.append(self.name)
            ok({"map": {"info": {"width": 0, "height": 0}, "data": []}} if answer[0] else {})
    answer = [False]
    real_service = rc.roslibpy.Service
    rc.roslibpy.Service = FakeService
    s._on_map = lambda m, _o=s._on_map: (got.append(m), _o(m))
    nav = lambda state: {"data": json.dumps({"stacks": {"bringup": {"state": "running"},       # noqa: E731
                                                        "nav": {"state": state}}})}
    s._on_launcher_state(nav("starting"))
    s._on_launcher_state(nav("running"))
    check("Nav2 just up: give the map a moment", not asked, f"{asked}")
    s._nav_up_t -= 4.0
    s._on_launcher_state(nav("running"))
    check("no map 3 s after Nav2 came up: asked map_server for it", asked == ["/map_server/map"], f"{asked}")
    s._on_launcher_state(nav("running"))
    check("not every second", len(asked) == 1)
    s._map_retry_t -= 11.0
    answer[0] = True
    s._on_launcher_state(nav("running"))
    check("asked again later, and the answer is the map", len(asked) == 2 and got, f"{asked} {len(got)}")
    s._map_retry_t = -1e9
    asked.clear()
    s._on_launcher_state(nav("running"))
    check("map here: no more asking", not asked, f"{asked}")
    s._map_t -= 60.0                    # that map came a minute ago
    s._on_launcher_state(nav("stopped"))
    s._on_launcher_state(nav("running"))
    s._nav_up_t -= 4.0
    s._on_launcher_state(nav("running"))
    check("Nav2 started again: the new map is asked for", asked == ["/map_server/map"], f"{asked}")
    rc.roslibpy.Service = real_service

    empty = RosClient(profile=RobotProfile.new("t"))
    check("no TF at all: map, reported as an error by the caller", empty._world_frame() == "map")

    print("\n[robot model on the 3D Map]")
    model = robot_model.load()
    tri, col = model.triangles, model.colors
    pts = tri.reshape(-1, 3)
    check("loads", len(tri) > 1000 and len(col) == len(tri))
    check("metres, the size of the robot", 0.8 < np.ptp(pts[:, 0]) < 1.0 and 0.5 < np.ptp(pts[:, 1]) < 0.65)
    check("stands on the floor", abs(pts[:, 2].min()) < 1e-3)
    check("light enough for a laptop", len(tri) < 60000, f"{len(tri)}")
    check("left and right match", abs(pts[:, 1].max() + pts[:, 1].min()) < 0.02)
    check("fits the default profile", robot_model.fits(tri, 0.45, 0.30))
    check("not another robot's", not robot_model.fits(tri, 0.25, 0.20))
    check("missing file is no problem", robot_model.load(Path("/nonexistent.npz")) is None)
    w2 = MainWindow()
    w2.show()
    v = w2.p_3d
    check("the 3D page has it", v.mesh is not None and v.uses_model())
    prof = w2.profiles.active()
    prof.limits.footprint_half_length, prof.limits.footprint_half_width = 0.25, 0.20
    w2._apply_profile_to_panels(prof)
    check("a smaller robot gets its box", not v.uses_model())
    prof.model = "hinson"
    w2._apply_profile_to_panels(prof)
    check("unless the Hinson is chosen for it", v.uses_model())
    prof.model, prof.limits.footprint_half_length, prof.limits.footprint_half_width = "auto", 0.45, 0.30
    w2._apply_profile_to_panels(prof)
    check("the default profile gets the Hinson", v.uses_model())
    v.set_robot_model(robot_model.builtin(), "auto")
    v.set_robot_footprint(0.25, 0.20)
    check("the page alone: auto follows the footprint", not v.uses_model())
    v.set_robot_footprint(0.45, 0.30)
    w2._show("3d")
    w2.ros.is_connected = lambda: False
    v.update_robot_pose(1.0, 2.0, 0.5)
    check("follows the robot", v.mesh.visible() and not v.box.visible())
    m = np.array(v.mesh.transform().data()).reshape(4, 4).T
    check("placed at its pose", abs(m[0, 3] - 1.0) < 1e-5 and abs(m[1, 3] - 2.0) < 1e-5
          and abs(m[1, 0] - np.sin(0.5)) < 1e-5, f"{m[:2, 3]}")
    v.ck_robot.setChecked(False)
    check("Robot layer hides it", not v.mesh.visible())
    w2.close()

    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    os._exit(code)
