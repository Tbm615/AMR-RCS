"""Page layout: several pages at once (split, tabs, separate windows), presets,
and that only the pages on screen are drawn and stream data.

Run:
    python tests/test_layout.py

Needs no robot: the camera / 3D streams are checked through the client's
subscription flags, which are set whether or not a robot is connected.
"""
from __future__ import annotations

import os
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_layout_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]

import numpy as np
from PySide6.QtCore import QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from rcs.core.events import EventLog, write_csv
from rcs.core.places import PlaceStore
from rcs.core.profile import load_settings
from rcs.ui import theme
from rcs.ui.main_window import MainWindow

app = QApplication(sys.argv)
PASS = FAIL = 0


def check(name: str, cond: bool, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def pump(t: float = 0.15) -> None:
    end = time.monotonic() + t
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.01)


def main() -> int:
    theme.apply(app)
    w = MainWindow()
    w.resize(1400, 880)
    w.show()
    pump()
    ws = w.workspace
    vis = lambda: ws.visible_keys()                       # noqa: E731
    img_on = lambda: bool(w.ros._streams_on.get("image"))  # noqa: E731

    print("\n[sidebar click = one page, like before]")
    check("starts on Connection", vis() == {"conn"} and w.active_page() == "conn", f"{vis()}")
    w.btns["map"].click(); pump()
    check("click shows 2D Map in its place", vis() == {"map"} and ws.open_keys() == {"map"}, f"{vis()}")
    w.btns["cam"].click(); pump()
    check("camera page subscribes the camera", vis() == {"cam"} and img_on())
    pane = ws._pane_of("cam")
    check("single page has no tab bar", pane is not None and not pane.tabBar().isVisible())

    print("\n[split / tab / window]")
    QTest.mouseClick(w.btns["map"], Qt.LeftButton, Qt.ControlModifier)
    pump()
    check("Ctrl+click opens 2D Map beside", vis() == {"cam", "map"}, f"{vis()}")
    check("both panes get a tab bar", all(p.tabBar().isVisible() for p in ws.root.findChildren(type(pane))))
    gm, gc = w.p_map.mapToGlobal(QPoint(0, 0)), w.p_cam.mapToGlobal(QPoint(0, 0))
    check("2D Map is right of Camera", gm.x() > gc.x() + 200, f"{gc.x()} {gm.x()}")
    w._show("teleop", "below"); pump()
    gt = w.p_teleop.mapToGlobal(QPoint(0, 0))
    check("Teleop opens below the page in use", vis() == {"cam", "map", "teleop"}
          and gt.y() > gm.y() + 100 and abs(gt.x() - gm.x()) < 20, f"{gm} {gt}")
    w._show("logs", "tab"); pump()
    check("Logs as a tab hides Teleop behind it", "logs" in vis() and "teleop" not in vis()
          and ws.open_keys() == {"cam", "map", "teleop", "logs"}, f"{vis()}")
    w.btns["teleop"].click(); pump()
    check("clicking an open page brings its tab forward", "teleop" in vis() and "logs" not in vis()
          and w.active_page() == "teleop")
    w.workspace.close_page("cam"); pump()
    check("closing Camera stops its stream", "cam" not in ws.open_keys() and not img_on())
    tree = ws.save_layout()["tree"]
    check("empty split collapses", "split" in tree and tree["split"] == "v", f"{tree}")

    w._show("cam", "float"); pump()
    check("Camera in its own window", len(ws.floats) == 1 and ws.floats[0].isVisible()
          and "cam" in vis() and img_on())
    check("window titled after the page", "Camera" in ws.floats[0].windowTitle())
    ws._dock_back("cam"); pump()
    check("move back to the main window", not ws.floats and "cam" in ws.open_keys())
    w._show("cam", "float"); pump()
    ws.floats[0].close(); pump()
    check("closing the window closes its page", not ws.floats and "cam" not in ws.open_keys()
          and not img_on())

    # the 3D page keeps its GL context only inside the main window
    w._show("3d", "float"); pump()
    check("3D Map is never put in its own window", not ws.floats and "3d" in vis())
    check("3D Map stays a child of the main window", w.p_3d.window() is w)
    ws.close_page("3d"); pump()
    check("closed pages stay inside the main window", w.p_3d.window() is w and not w.p_3d.isVisible())

    print("\n[presets]")
    w.apply_layout("drive"); pump()
    check("Drive = Teleop + 2D Map", vis() == {"teleop", "map"} and w.active_page() == "map", f"{vis()}")
    w.apply_layout("mapping"); pump()
    check("Mapping = 2D Map + 3D Map", vis() == {"map", "3d"}, f"{vis()}")
    w.apply_layout("monitor"); pump()
    check("Monitor = 2D Map + Camera + Status", vis() == {"map", "cam", "status"} and img_on(), f"{vis()}")
    w.apply_layout("missions"); pump()
    check("Missions = Missions + Camera", vis() == {"mission", "cam"}, f"{vis()}")
    w.apply_layout("single"); pump()
    check("Single keeps the page in use", vis() == {"mission"}, f"{vis()}")

    print("\n[only pages on screen are drawn]")
    w.apply_layout("drive"); pump()
    views = w._map_views()
    check("2D Map redrawn, Navigation/Missions not", views == (w.p_map.view,), f"{views}")
    w._show("nav", "right"); pump()
    check("Navigation joins once on screen", set(w._map_views()) == {w.p_map.view, w.p_nav.view})

    print("\n[fusion view gets the same data as the 2D map]")
    w.apply_layout("mapping"); pump()
    p3 = w.p_3d
    w._send_goal(1.0, 2.0, 0.5)
    check("goal shown in 3D too", p3._goal == (1.0, 2.0, 0.5))
    w._on_nav_state("succeeded")
    check("reached goal cleared in 3D", p3._goal is None)
    w.p_mission.add_waypoint_at(2.0, 3.0, 0.0); pump()
    check("mission waypoints reach the 3D page", len(p3.wp_pts.pos) == len(w.p_mission.cur.waypoints))
    n = len(w.p_mission.cur.waypoints)
    p3.waypoint_requested.emit(3.0, 3.0, 1.0); pump()
    check("waypoint set on the 3D floor joins the mission", len(w.p_mission.cur.waypoints) == n + 1)
    sent = []
    w.ros.send_nav_goal = lambda *a: sent.append(a)
    p3.goal_requested.emit(4.0, 5.0, 0.0)
    check("goal set on the 3D floor is sent", sent == [(4.0, 5.0, 0.0)])
    w._on_scan({"_xy": (np.array([1.0], "float32"), np.array([0.0], "float32"))})
    pump()
    check("lidar reaches the 3D page", len(p3.scan_pts.pos) == 1)
    w._on_plan({"poses": [{"pose": {"position": {"x": 0.0, "y": 0.0}}}] * 3})
    check("path reaches the 3D page", len(p3.path_line.pos) == 3)
    w._on_local_costmap({"info": {"width": 2, "height": 2, "resolution": 0.5, "origin": {}},
                         "data": [0, 0, 0, 100], "header": {"frame_id": "map"}})
    pump()
    check("local costmap reaches the 3D page", p3.lcost is not None)

    print("\n[events]")
    log, pe = w.events, w.p_events
    log.clear()
    texts = lambda: [e.text for e in log.events]            # noqa: E731
    w.btns["map"].click(); pump()
    w._set_estop(True)
    check("E-STOP is an error event", log.events[-1].level == "error" and "engaged" in texts()[-1])
    check("sidebar counts unseen problems", w.btns["events"].text() == "Events  (1)", w.btns["events"].text())
    check("shown in the status bar", "E-STOP" in w.statusBar().currentMessage())
    w._set_estop(False)
    w._on_estop_remote(True)
    check("remote E-STOP says so", "another console" in texts()[-1])
    w._on_estop_remote(False)
    w._nav_state = ""
    w._on_nav_state("active")
    w._on_nav_state("aborted"); w._on_nav_state("aborted")
    check("Nav2 abort logged once", texts().count("Nav2 aborted the goal") == 1)
    w._on_launcher_events({"stacks": {"bringup": {"state": "stopped"}}})
    check("first launcher state is not an event", not any("Robot base" in t for t in texts()))
    w._on_launcher_events({"stacks": {"bringup": {"state": "running"}}})
    w._on_launcher_events({"stacks": {"bringup": {"state": "failed", "detail": "lidar not reachable"}}})
    check("stack failure with its reason", log.events[-1].level == "error"
          and "lidar not reachable" in texts()[-1], texts()[-1])
    w._on_launcher_events({"stacks": {}, "result": {"id": "a", "ok": True, "msg": "old"}})
    w._on_launcher_events({"stacks": {}, "result": {"id": "b", "ok": True, "msg": "map saved"}})
    check("launcher results, not the one from before", texts()[-1] == "map saved" and "old" not in texts())

    class St:
        ok = True
        def fresh(self, _t): return self.ok          # noqa: E704
    st = St()
    w.ros.is_connected = lambda: True
    w.ros.topic_stats["scan"] = st
    w._sensor_ok.clear()
    w._check_sensors(); st.ok = False; w._check_sensors()
    check("lidar lost", texts()[-1] == "no lidar for 3 s", texts()[-1])
    st.ok = True; w._check_sensors()
    check("lidar back", texts()[-1] == "lidar is back", texts()[-1])
    del w.ros.is_connected, w.ros.topic_stats["scan"]

    w.btns["events"].click(); pump()
    check("opening Events clears the count", w.btns["events"].text() == "Events")
    check("newest first", pe.table.item(0, 2).text() == "lidar is back", pe.table.item(0, 2).text())
    pe.level.setCurrentIndex(2)
    check("filter: errors only", pe.table.rowCount() == 3 and all(e.level == "error" for e in pe.shown()),
          f"{pe.table.rowCount()}")
    pe.level.setCurrentIndex(0)
    pe.source.setCurrentIndex(pe.source.findData("Sensors"))
    check("filter: by source", pe.table.rowCount() == 2)
    pe.source.setCurrentIndex(0)
    n = pe.table.rowCount()
    w._log("info", "Mission", "a new event")
    check("a live event goes on top", pe.table.rowCount() == n + 1 and pe.table.item(0, 2).text() == "a new event")
    csv_path = Path(os.environ["XDG_CONFIG_HOME"]) / "export.csv"
    check("CSV export", write_csv(csv_path, pe.shown()) == n + 1 and "a new event" in csv_path.read_text())
    log2 = EventLog()
    check("events survive a restart", [e.text for e in log2.events] == texts())

    print("\n[keyboard]")
    w.apply_layout("single"); pump()
    w.activateWindow()
    QTest.qWaitForWindowActive(w, 2000)
    QTest.keyClick(w, Qt.Key_F5); pump()
    check("F5 opens the 5th page", w.active_page() == "map", w.active_page())
    QTest.keyClick(w, Qt.Key_F11); pump()
    check("F11 opens Events", w.active_page() == "events", w.active_page())
    QTest.keyClick(w, Qt.Key_2, Qt.ControlModifier); pump()
    check("Ctrl+2 = Drive", vis() == {"teleop", "map"}, f"{vis()}")
    QTest.keyClick(w, Qt.Key_F12); pump()
    check("F12 engages E-STOP", w._estop_on)
    QTest.keyClick(w, Qt.Key_F12); pump()
    check("a second F12 does not release it", w._estop_on)
    w._set_estop(False)
    w.p_map.view.set_click_mode("measure")
    w.p_map.view.ruler_m = 1.0
    calls = []
    real_cancel = w.ros.cancel_nav_goal
    w.ros.cancel_nav_goal = lambda: calls.append(1)
    w._nav_state = "active"
    w.on_escape()
    check("Esc clears the ruler first", w.p_map.view.ruler_m is None and not calls)
    w.on_escape()
    check("then Esc cancels the goal", calls == [1])
    w._nav_state = "idle"
    w.on_escape()
    check("no goal: Esc does nothing", calls == [1])
    w.ros.cancel_nav_goal = real_cancel
    w.p_map.view.set_click_mode("goal")

    print("\n[saved layouts]")
    w.apply_layout("mapping"); pump()
    w._show("logs", "below"); pump()
    w.save_layout_as("my setup")
    w.apply_layout("single"); pump()
    w._fill_layout_menu()
    check("listed in the Layout menu", any(a.text() == "my setup" for a in w.layout_menu.actions()))
    w.load_layout("my setup"); pump()
    check("brings the pages back", vis() == {"map", "3d", "logs"}, f"{vis()}")
    check("kept in the settings", "my setup" in load_settings().get("layouts", {}))
    w.delete_layout("my setup")
    w._fill_layout_menu()
    check("can be deleted", not any(a.text() == "my setup" for a in w.layout_menu.actions())
          and "my setup" not in load_settings().get("layouts", {}))

    print("\n[start-up options]")
    w.p_conn.auto_ck.setChecked(True)
    check("auto-connect saved", load_settings().get("auto_connect") is True)
    w.p_events.sound_ck.setChecked(False)
    check("sound off saved", load_settings().get("sound") is False and not w.sounds.enabled)
    w.p_events.sound_ck.setChecked(True)
    done = []
    w._on_connect, keep = (lambda p: done.append(p)), w._on_connect
    w.settings["auto_connect"] = True
    w.connect_on_start()
    w._on_connect = keep
    check("connects the active robot on start", len(done) == 1 and done[0].id == w.profiles.active().id)
    w.p_conn.auto_ck.setChecked(False)
    w.settings["auto_connect"] = False

    print("\n[saved places]")
    pl = w.p_nav.places
    w.btns["nav"].click(); pump()
    check("needs a robot position first", not pl.here_btn.isEnabled() or pl.pose is not None)
    pl.set_pose(1.0, 2.0, 0.5)
    pl.set_map("office")
    check("saved at the robot", pl.add_here("Charger") is not None
          and (pl.store.places[-1].x, pl.store.places[-1].map) == (1.0, "office"))
    check("listed", pl.list.count() == 1 and pl.list.item(0).text() == "Charger")
    check("drawn on the maps", len(w.p_map.view.place_dots.data) == 1 and len(w.p_nav.view._place_labels) == 1)
    pl.list.setCurrentRow(0)
    sent = []
    w.ros.send_nav_goal = lambda *a: sent.append(a)
    pl.go_btn.click()
    check("Go sends a goal with the saved heading", sent == [(1.0, 2.0, 0.5)], f"{sent}")
    pl.set_map("warehouse")
    check("hidden on another map", pl.list.count() == 0 and len(w.p_map.view._place_labels) == 0)
    pl.set_map("")
    check("all shown when the map is unknown", pl.list.count() == 1)
    pl.list.setCurrentRow(0)
    n = len(w.p_mission.cur.waypoints)
    pl.mis_btn.click()
    check("added to the mission with its name", len(w.p_mission.cur.waypoints) == n + 1
          and w.p_mission.cur.waypoints[-1].name == "Charger")
    pl.rename_selected("Dock")
    check("renamed", PlaceStore().places[-1].name == "Dock", PlaceStore().places[-1].name)
    pl.list.setCurrentRow(0)
    pl.delete_selected()
    check("deleted", pl.list.count() == 0 and not PlaceStore().places)

    print("\n[last position]")
    prof = w.profiles.active()
    w.ros.set_profile(prof)
    nav = {"state": "running", "args": {"mode": "map", "map": "office"}}
    w._on_launcher_events({"stacks": {"nav": nav}})
    check("nothing remembered yet", w.last_pose() is None and w.p_map.last_pose_btn.isHidden())
    t = prof.topics
    w.tf.pose_xyyaw = lambda a, b: (0.0, 0.0, 0.0)    # localized, launch default
    w._odom_v = 0.0
    w._track_pose((0.0, 0.0, 0.0))
    check("the launch default is not stored", w.last_pose() is None)
    w._odom_v = 0.3
    w._pose_saved_t = 0.0
    w._track_pose((3.0, 4.0, 1.0))
    check("stored while driving", w.last_pose() == (3.0, 4.0, 1.0), f"{w.last_pose()}")
    w._track_pose((3.1, 4.0, 1.0))
    check("not on every tick", w.last_pose() == (3.0, 4.0, 1.0))
    w._odom_v = 0.0
    w._track_pose((3.2, 4.0, 1.0))
    check("stored once stopped", w.last_pose() == (3.2, 4.0, 1.0), f"{w.last_pose()}")
    del w.tf.pose_xyyaw
    w._on_launcher_events({"stacks": {"nav": nav}})
    check("button offers it", not w.p_map.last_pose_btn.isHidden() and "3.2" in w.p_map.last_pose_btn.text())
    poses = []
    w.ros.publish_initial_pose = lambda *a: poses.append(a)
    w.p_map.last_pose_btn.click()
    check("one click sets it", poses == [(3.2, 4.0, 1.0)], f"{poses}")
    w._on_launcher_events({"stacks": {"nav": {**nav, "args": {"mode": "map", "map": "other"}}}})
    check("per map", w.last_pose() is None and w.p_map.last_pose_btn.isHidden())
    w._on_launcher_events({"stacks": {"nav": {"state": "running", "args": {"mode": "slam"}}}})
    check("not in SLAM mode", w.last_pose() is None)
    w._on_launcher_events({"stacks": {"nav": nav}})
    check("kept in the settings", load_settings()["last_pose"][f"{prof.id}/office"][:2] == [3.2, 4.0])
    w._on_launcher_events({"stacks": {"nav": {"state": "stopped"}}})

    print("\n[layout is kept]")
    w.apply_layout("monitor"); pump()
    w._show("logs", "float"); pump()
    w.close(); pump()
    saved = load_settings().get("layout")
    check("layout saved on close", bool(saved) and saved["floats"][0]["tabs"] == ["logs"], f"{saved}")
    w2 = MainWindow()
    w2.resize(1400, 880)
    w2.show()
    pump(0.3)
    check("restored on start", w2.workspace.visible_keys() == {"map", "cam", "status", "logs"}
          and len(w2.workspace.floats) == 1, f"{w2.workspace.visible_keys()}")
    check("restored camera stream", bool(w2.ros._streams_on.get("image")))
    check("bad saved layout falls back", not w2.workspace.restore_layout({"tree": {"tabs": ["nope"]}})
          and w2.workspace.open_keys())

    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    rc = main()
    print(f"exit code: {rc}")
    sys.stdout.flush(); sys.stderr.flush()
    if sys.platform == "win32":
        import ctypes
        from ctypes import wintypes
        k32 = ctypes.windll.kernel32
        k32.GetCurrentProcess.restype = wintypes.HANDLE
        k32.TerminateProcess.argtypes = [wintypes.HANDLE, wintypes.UINT]
        k32.TerminateProcess(k32.GetCurrentProcess(), rc)
    os._exit(rc)
