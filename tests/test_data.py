"""Per-robot data, .rcspack files and copying maps to and from the robot.

Run:
    python tests/test_data.py

Needs no robot: maps go through the mock rosbridge, which runs the robot's real
robot/map_exchange.py. Files that claim to be packs are built by hand here to
check that one from someone else cannot do harm.
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
import tempfile
import threading
import time
import zipfile
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
CFG = tempfile.mkdtemp(prefix="rcs_data_cfg_")
os.environ["XDG_CONFIG_HOME"] = CFG
os.environ["APPDATA"] = CFG

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from rcs.core import map_files, pack as packs
from rcs.core.maps import MapLibrary
from rcs.core.mission import Mission, MissionStore, Waypoint
from rcs.core.places import PlaceStore
from rcs.core.profile import CONFIG_DIR, RobotProfile, checked_limits, load_settings
from rcs.core.robot_data import RobotState, robot_dir
from rcs.ui import theme
from rcs.ui.main_window import MainWindow, PAGES
from tests import mock_rosbridge

PORT = 9194
app = QApplication(sys.argv)
PASS = FAIL = 0
TMP = Path(tempfile.mkdtemp(prefix="rcs_data_tmp_"))
META = {"resolution": 0.05, "origin": [-1.0, -2.0, 0.0], "negate": 0, "occupied_thresh": 0.65,
        "free_thresh": 0.25, "mode": "trinary"}


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def pump(t: float = 0.1) -> None:
    end = time.monotonic() + t
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


def pgm(w: int = 40, h: int = 30, fill: int = 254) -> bytes:
    return f"P5\n{w} {h}\n255\n".encode() + bytes([fill]) * (w * h)


def raises(fn) -> bool:
    try:
        fn()
    except ValueError:
        return True
    return False


def seed_old_install() -> tuple:
    """Files as v0.5 left them, before the app starts for the first time."""
    a, b = RobotProfile.new("alpha"), RobotProfile.new("beta")
    (CONFIG_DIR / "robots.json").write_text(json.dumps(
        {"profiles": [a.to_dict(), b.to_dict()], "active_id": a.id}))
    m = Mission.new("Old route")
    m.waypoints = [Waypoint.new("w1", 1.0, 2.0)]
    (CONFIG_DIR / "missions.json").write_text(json.dumps({"missions": [m.to_dict()]}))
    (CONFIG_DIR / "places.json").write_text(json.dumps(
        {"places": [{"id": "p1", "name": "Dock", "x": 1, "y": 2, "yaw": 0, "map": ""}]}))
    (CONFIG_DIR / "events.csv").write_text("1000.0,info,Connection,hello\n")
    (CONFIG_DIR / "settings.json").write_text(json.dumps(
        {"sound": False, "last_pose": {f"{a.id}/office": [1.0, 2.0, 0.5]}}))
    return a, b


def test_map_files() -> None:
    print("\n[map files]")
    check("robot and app have the same map_files.py",
          (HERE.parent / "robot/map_files.py").read_bytes() == (HERE.parent / "rcs/core/map_files.py").read_bytes())
    # as map_saver writes it
    (TMP / "saver.yaml").write_text("image: saver.pgm\nmode: trinary\nresolution: 0.05\n"
                                    "origin: [-4.6, -3.3, 0]\nnegate: 0\noccupied_thresh: 0.65\n"
                                    "free_thresh: 0.25")
    (TMP / "saver.pgm").write_bytes(b"P5\n# CREATOR: map_saver.cpp 0.050 m/pix\n40 30\n255\n" + bytes(1200))
    check("a map_saver map reads", map_files.read_map(TMP / "saver.yaml")[0]["origin"] == [-4.6, -3.3, 0.0])
    check("cut short PGM refused", raises(lambda: map_files.pgm_size(pgm()[:100])))
    check("header without end refused, no hang", raises(lambda: map_files.pgm_size(b"P5 12")))
    check("comment without end refused, no hang", raises(lambda: map_files.pgm_size(b"P5\n# x")))
    check("16-bit PGM refused", raises(lambda: map_files.pgm_size(b"P5\n1 1\n65535\n\x00\x00\x00")))
    check("bad name refused", raises(lambda: map_files.check_name("../evil")))
    check("NaN resolution refused", raises(lambda: map_files.parse_yaml("resolution: nan\norigin: [0, 0, 0]\n")))
    check("no origin refused", raises(lambda: map_files.parse_yaml("resolution: 0.05\n")))
    y = map_files.canonical_yaml("x", map_files.parse_yaml(
        "image: /etc/passwd\nresolution: 0.05\norigin: [1, 2, 0] # here\n"))
    check("an image path in a map does not survive", "/etc/passwd" not in y and "image: x.pgm" in y, y)
    blob = map_files.to_blob("x", META, pgm())
    check("packs and unpacks", map_files.from_blob(blob)[1] == pgm())
    check("not a map refused", raises(lambda: map_files.from_blob(b"junk")))

    lib = MapLibrary(TMP / "lib")
    check("stored", lib.add("site", META, pgm()) == ("site", True))
    check("same map again adds nothing", lib.add("site", META, pgm()) == ("site", False))
    check("other map, same name: nothing replaced", lib.add("site", META, pgm(fill=0)) == ("site_2", True)
          and lib.read("site")[1] == pgm())
    lib.remove("site_2")
    check("removed", lib.names() == ["site"])


def test_limits() -> None:
    print("\n[limits from a file]")
    lim = checked_limits({"max_linear": 99.0, "max_angular": float("inf"), "teleop_timeout": 1e9,
                          "cloud_throttle_ms": "x", "footprint_half_length": True})
    check("speed clamped to what the form allows", lim.max_linear == 2.0, f"{lim.max_linear}")
    check("infinity falls back to the default", lim.max_angular == 0.8)
    check("stale-command cutoff cannot be switched off", lim.teleop_timeout == 2.0)
    check("wrong types fall back", lim.cloud_throttle_ms == 500 and lim.footprint_half_length == 0.45)


def hand_made(name: str, members: dict) -> Path:
    path = TMP / name
    with zipfile.ZipFile(path, "w") as z:
        for k, v in members.items():
            z.writestr(k, v if isinstance(v, bytes) else json.dumps(v))
    return path


def test_hostile_packs() -> None:
    print("\n[packs from someone else]")
    head = {"format": "rcs-pack", "version": 1, "app": "x", "created": "y", "robot": "z"}
    junk = TMP / "junk.rcspack"
    junk.write_bytes(b"this is no zip")
    check("not a zip", raises(lambda: packs.read_pack(junk)))
    check("a zip that is not a pack", raises(lambda: packs.read_pack(hand_made("a.rcspack", {"x": {}}))))
    check("from a newer app", raises(lambda: packs.read_pack(
        hand_made("b.rcspack", {"manifest.json": {**head, "version": 99}}))))
    wp = {"id": "a", "name": "n", "x": float("nan"), "y": 0, "yaw": 0}
    check("NaN waypoint", raises(lambda: packs.read_pack(hand_made("c.rcspack", {
        "manifest.json": head, "missions.json": {"missions": [{"waypoints": [wp]}]}}))))
    check("waypoint of the wrong shape", raises(lambda: packs.read_pack(hand_made("d.rcspack", {
        "manifest.json": head, "missions.json": {"missions": [{"waypoints": ["x"]}]}}))))
    check("broken JSON", raises(lambda: packs.read_pack(hand_made("e.rcspack", {
        "manifest.json": head, "places.json": b"{"}))))
    check("damaged map", raises(lambda: packs.read_pack(hand_made("f.rcspack", {
        "manifest.json": head, "maps/m.yaml": b"resolution: 0.05\norigin: [0, 0, 0]\n",
        "maps/m.pgm": b"P5\n9 9\n255\n"}))))

    victim = TMP / "evil_target.yaml"
    p = packs.read_pack(hand_made("g.rcspack", {
        "manifest.json": head,
        f"maps/../../{victim.name}": b"resolution: 0.05\norigin: [0, 0, 0]\n",
        f"maps/../../{victim.stem}.pgm": pgm()}))
    check("zip-slip names are not maps", not p.maps and not victim.exists())

    p = packs.read_pack(hand_made("h.rcspack", {
        "manifest.json": head,
        "profile.json": {"id": "x/../../y", "name": "fast", "host": "h", "port": 999999,
                         "limits": {"max_linear": 50}, "topics": {"cmd_vel": 7}}}))
    check("profile gets its own id", p.profile.id != "x/../../y" and "/" not in p.profile.id)
    check("profile limit clamped", p.profile.limits.max_linear == 2.0)
    check("port made valid", p.profile.port == 65535)
    check("non-text topic ignored", p.profile.topics.cmd_vel == "/cmd_vel_safe")

    lib = MapLibrary(TMP / "lib2")
    p = packs.read_pack(hand_made("i.rcspack", {
        "manifest.json": head,
        "maps/m.yaml": b"image: /etc/passwd\nresolution: 0.05\norigin: [0, 0, 0]\n", "maps/m.pgm": pgm()}))
    packs.import_into(p, {"maps"}, lib, MissionStore(TMP / "m.json"), PlaceStore(TMP / "p.json"))
    check("image path from a pack is dropped", "passwd" not in (lib.root / "m.yaml").read_text())


def test_migration(w, a, b) -> None:
    print("\n[from an older version]")
    check("missions moved to every robot", all(
        MissionStore(robot_dir(p.id) / "missions.json").missions[0].name == "Old route" for p in (a, b)))
    check("places too", all(PlaceStore(robot_dir(p.id) / "places.json").places[0].name == "Dock" for p in (a, b)))
    check("old files kept aside", (CONFIG_DIR / "missions.json.migrated").exists()
          and not (CONFIG_DIR / "missions.json").exists())
    check("events went to the active robot", len(w.events.events) == 1)
    check("last position went to its robot", w.robot_state.get("last_pose")["office"] == [1.0, 2.0, 0.5])
    check("other settings untouched", load_settings().get("sound") is False
          and "last_pose" not in load_settings())
    check("a new robot starts empty", not (robot_dir("new") / "missions.json").exists())


def test_keys(w) -> None:
    print("\n[keyboard]")
    check("Data page exists", "data" in dict(PAGES))
    QTest.keyClick(w, Qt.Key_F12); pump()
    check("F12 is still E-STOP with a 12th page", w._estop_on and w.active_page() != "data", w.active_page())
    w._set_estop(False)
    check("Data tooltip names no F key", not w.btns["data"].toolTip().startswith("F"))


def test_per_robot(w, a, b) -> None:
    print("\n[each robot has its own]")
    pm = w.p_mission
    w.p_conn._select_id(a.id); pump()
    pm._new_mission(); pm.cur.name = "A only"; pm._save()
    pm.add_waypoint_at(1.0, 1.0)
    a_mission = pm.cur.id
    w.p_conn._select_id(b.id); pump()
    check("selecting offline opens that robot", w._data_robot == b.id)
    check("its missions, not the other's", w.missions.get(a_mission) is None)
    check("path follows", w.missions.path.parent == robot_dir(b.id))
    w._log("info", "Test", "only on beta")
    w.p_conn._select_id(a.id); pump()
    check("first robot's data comes back", pm.cur is not None and w.missions.get(a_mission) is not None)
    check("the mission that was open", pm.cur.id == a_mission, pm.cur.name)
    check("events are per robot", not any(e.text == "only on beta" for e in w.events.events))
    check("page shows them", all("only on beta" not in (w.p_events.table.item(r, 2).text())
                                 for r in range(w.p_events.table.rowCount())))

    print("\n[switching with a mission running]")
    sent = []
    w.ros.send_nav_goal = lambda *x: sent.append(x)
    cancels = []
    w.ros.cancel_nav_goal = lambda: cancels.append(1)
    pm._play()
    check("running", pm._exec_running)
    w.ros._want_connected = True            # as while connected
    w.p_conn._select_id(b.id); pump()
    check("selecting another robot while connected changes nothing",
          w._data_robot == a.id and pm._exec_running)
    w.p_conn._select_id(a.id); pump()
    w.ros._want_connected = False
    w._switch_robot(b)
    check("a real switch stops the mission", not pm._exec_running and pm._exec_idx == -1 and cancels)
    n = len(sent)
    pump(0.5)
    check("no goal goes to the other robot", len(sent) == n)
    w._switch_robot(a)
    check("back on the first robot", w._data_robot == a.id)
    del w.ros.send_nav_goal, w.ros.cancel_nav_goal


def test_wrong_map(w) -> None:
    print("\n[a mission belongs to its map]")
    pm = w.p_mission
    sent = []
    w.ros.send_nav_goal = lambda *x: sent.append(x)
    nav = {"stacks": {"nav": {"state": "running", "args": {"mode": "map", "map": "office"}}}}
    w._on_launcher_events(nav)
    pm._new_mission()
    pm.add_waypoint_at(2.0, 2.0)
    check("new mission takes the running map", pm.cur.map == "office")
    pm.add_waypoint_at(3.0, 2.0)
    check("kept in the file", MissionStore(w.missions.path).get(pm.cur.id).map == "office")
    w._on_launcher_events({"stacks": {"nav": {"state": "running", "args": {"mode": "map", "map": "plant"}}}})
    pm._play()
    check("refused on another map", not pm._exec_running and not sent)
    check("says why", "office" in pm.status_lbl.text() and "plant" in pm.status_lbl.text(), pm.status_lbl.text())
    w._on_launcher_events(nav)
    pm._play()
    check("runs on its map", pm._exec_running and len(sent) == 1)
    pm._stop()
    w._on_launcher_events({"stacks": {"nav": {"state": "running", "args": {"mode": "slam"}}}})
    sent.clear()
    pm._play()
    check("SLAM cannot be compared, so it is not blocked", pm._exec_running)
    pm._stop()
    w._on_launcher_events({"stacks": {"nav": {"state": "stopped"}}})
    del w.ros.send_nav_goal


def test_pack_round_trip(w, a) -> None:
    print("\n[back up, share, import]")
    w.library.add("office", META, pgm())
    w.p_data.refresh_library()
    w.place_store.add("Charger", 1.0, 2.0, 0.5, "office")
    check("backing up the robot with the old last position",
          w._data_robot == a.id and w.robot_state.get("last_pose", {}).get("office"))
    w.settings.setdefault("layouts", {})["wide"] = w.workspace.save_layout()
    n_missions = len(w.missions.missions)
    every = {"profile": True, "missions": True, "places": True, "layouts": True, "state": True,
             "events": True, "maps": ["office"]}
    path = TMP / "backup"
    check("export", w.export_pack_to(str(path), every))
    path = TMP / "backup.rcspack"
    check("suffix added", path.exists() and not list(TMP.glob("*.tmp")))
    p = packs.read_pack(path)
    check("everything in it", p.profile and p.state and p.events and "office" in p.maps
          and len(p.missions) == n_missions and p.layouts)
    check("export to a bad place says so", not w.export_pack_to("/nonexistent/x/y", every)
          and "cannot write" in w.p_data.result.text())

    share = TMP / "share.rcspack"
    w.export_pack_to(str(share), {"missions": True, "places": True, "maps": ["office"]})
    with zipfile.ZipFile(share) as z:
        names = set(z.namelist())
    check("a share leaves out history and address",
          not names & {"profile.json", "state.json", "events.csv", "layouts.json"}, f"{names}")

    print("\n[import]")
    before = [m.id for m in w.missions.missions]
    res = w.import_pack_from(str(share), False, {"missions", "places", "maps"})
    check("again into the same robot adds nothing", res.missions_added == 0 and res.places_added == 0
          and res.maps_same == 1 and [m.id for m in w.missions.missions] == before, res.text())

    res = w.import_pack_from(str(path), True, {"missions", "places", "maps", "layouts", "state", "events"})
    new = res.robot
    check("backup as a new robot", new is not None and new.id != a.id
          and any(x.id == new.id for x in w.profiles.profiles))
    check("named apart", new.name != a.name, new.name)
    check("listed on the Connection page", any(new.name in w.p_conn.list.item(i).text()
                                               for i in range(w.p_conn.list.count())))
    check("the open robot does not change", w._data_robot == a.id)
    st = RobotState(robot_dir(new.id) / "state.json")
    check("last positions came along", st.get("last_pose", {}).get("office") == [1.0, 2.0, 0.5])
    check("a run in progress did not", st.get("run") is None)
    check("events came along", (robot_dir(new.id) / "events.csv").exists())

    print("\n[a colleague's map has the same name as ours]")
    other = packs.Pack(manifest={})
    other.maps["office"] = (META, pgm(fill=0))
    m = Mission.new("Their route")
    m.map = "office"
    m.waypoints = [Waypoint.new("a", 5.0, 5.0)]
    other.missions = [m]
    res = packs.import_into(other, {"maps", "missions"}, w.library, w.missions, w.place_store)
    final = res.renamed.get("office")
    check("kept next to ours", final and w.library.read("office")[1] == pgm())
    check("their mission follows their map", w.missions.missions[-1].map == final, final)

    print("\n[import while a mission runs]")
    pm = w.p_mission
    pm.reload()
    pm._set_combo_to(w.missions.missions[-1].id)
    w.ros.send_nav_goal = lambda *x: None
    pm._play()
    check("refused for this robot", w.import_pack_from(str(share), False, {"missions"}) is None
          and "stop the mission" in w.p_data.result.text())
    pm._stop()
    del w.ros.send_nav_goal
    bad = TMP / "junk.rcspack"
    check("a bad file says so", w.import_pack_from(str(bad), False, {"missions"}) is None
          and "cannot import" in w.p_data.result.text())


def transfer(w, start, timeout=20.0) -> tuple:
    out = []
    w.transfer.finished.connect(lambda ok, t: out.append((ok, t)))
    start()
    wait_until(lambda: out, timeout)
    w.transfer.finished.disconnect()
    w.transfer.finished.connect(w.p_data._on_finished)
    w.transfer.finished.connect(lambda ok, text: w._log("info" if ok else "warn", "Data", text))
    return out[0] if out else (None, "timed out")


def test_transfer(w) -> None:
    print("\n[maps to and from the robot]")
    L = mock_rosbridge.LAUNCHER
    d = w.p_data
    d.confirm_dialogs = False
    w.transfer.answer_ms = 1000
    prof = RobotProfile.new("mock")
    prof.host, prof.port = "127.0.0.1", PORT
    w.profiles.upsert(prof)
    w._on_connect(prof)
    check("connects", wait_until(lambda: w.ros.is_connected() and d.robot_maps(), 8.0))
    check("robot maps listed", d.robot_list.count() == 2)

    ok, text = transfer(w, lambda: w.transfer.download("map_a"))
    check("copied from the robot", ok and "map_a" in w.library.names(), text)
    check("identical", w.library.read("map_a") == map_files.read_map(Path(L.map_dir) / "map_a.yaml"))
    ok, text = transfer(w, lambda: w.transfer.download("map_a"))
    check("again: nothing new", ok and "already has" in text, text)
    ok, text = transfer(w, lambda: w.transfer.download("map_b"))
    check("a map it cannot read gives the reason", ok is False, text)
    ok, text = transfer(w, lambda: w.transfer.download("nope"))
    check("unknown map", ok is False and "not found" in text, text)

    w.library.add("sent", META, pgm())
    ok, text = transfer(w, lambda: w.transfer.upload("sent"))
    check("sent to the robot", ok and "sent" in L.maps, text)
    meta, data = map_files.read_map(Path(L.map_dir) / "sent.yaml")
    check("identical on the robot", data == pgm() and meta == META)
    check("listed after the next state", wait_until(lambda: "sent" in d.robot_maps(), 3.0))
    ok, text = transfer(w, lambda: w.transfer.upload("sent"))
    check("not replaced without asking", ok is False and "already has" in text, text)
    ok, text = transfer(w, lambda: w.transfer.upload("sent", overwrite=True))
    check("replaced when confirmed", ok is True, text)
    L.nav_map = "sent"
    ok, text = transfer(w, lambda: w.transfer.upload("sent", overwrite=True))
    check("never the map Nav2 runs on", ok is False and "Nav2" in text, text)
    L.nav_map = ""

    print("\n[big map, lost messages]")
    rng = np.random.default_rng(1)
    big = f"P5\n900 900\n255\n".encode() + rng.integers(0, 255, 900 * 900, dtype=np.uint8).tobytes()
    w.library.add("big", META, big)
    seen = []
    w.transfer.progress.connect(lambda a_, b_: seen.append((a_, b_)))
    ok, text = transfer(w, lambda: w.transfer.upload("big"))
    check("several chunks", ok and seen and seen[-1][1] >= 4, f"{seen[-1:]}")
    check("arrives whole", map_files.read_map(Path(L.map_dir) / "big.yaml")[1] == big)
    w.library.remove("big")
    L.drop_next = 2
    ok, text = transfer(w, lambda: w.transfer.download("big"))
    check("survives lost answers", ok and w.library.read("big")[1] == big, text)
    L.drop_next = 2
    w.library.add("big2", META, big)
    ok, text = transfer(w, lambda: w.transfer.upload("big2"))
    check("lost answers on the way up too", ok and map_files.read_map(Path(L.map_dir) / "big2.yaml")[1] == big, text)
    L.drop_next = 99
    ok, text = transfer(w, lambda: w.transfer.download("map_a"))
    check("gives up with a reason", ok is False and "no answer" in text, text)
    L.drop_next = 0

    print("\n[page]")
    d.refresh_library()
    d._select(d.pc_list, "sent")
    d._select(d.robot_list, "map_a")
    pump()
    check("buttons on", d.send_btn.isEnabled() and d.get_btn.isEnabled())
    check("map details shown", "m" in d.pc_info.text() and d.thumb.pixmap() is not None
          and not d.thumb.pixmap().isNull())
    d.send_btn.click()
    check("buttons off while copying", w.transfer.busy and not d.get_btn.isEnabled()
          and not d.send_btn.isEnabled() and not d.bar.isHidden())
    check("result shown", wait_until(lambda: not w.transfer.busy, 5.0) and "saved on the robot" in d.result.text(),
          d.result.text())
    check("buttons back", d.send_btn.isEnabled() and d.bar.isHidden())
    L.old = True
    check("old launcher noticed", wait_until(lambda: not d._can_transfer(), 4.0))
    check("old launcher: buttons off, says what to update",
          not d.send_btn.isEnabled() and "update" in d.robot_hint.text(), d.robot_hint.text())
    L.old = False
    check("updated launcher noticed", wait_until(d._can_transfer, 4.0))
    w.transfer.upload("sent", overwrite=True)
    w.ros.disconnect_robot()
    pump(0.2)
    check("disconnecting ends a transfer", not w.transfer.busy)
    check("robot list cleared", d.robot_list.count() == 0 and not d.send_btn.isEnabled())


def test_restart(a) -> None:
    print("\n[how the app was left]")
    w = MainWindow()
    w.resize(1400, 880)
    w.show()
    pump(0.2)
    check("opens on the last robot", w._data_robot == a.id)
    pm = w.p_mission
    w.ros.send_nav_goal = lambda *x: None
    pm._new_mission()
    pm.cur.name = "Rounds"
    pm.cur.loop, pm.cur.laps = True, 5
    for i in range(3):
        pm.add_waypoint_at(float(i), 0.0)
    mid = pm.cur.id
    pm._exec_idx, pm._lap, pm._exec_running = 1, 2, True
    w.p_robot.rb_slam.setChecked(True)
    w._nav_map = "office"
    w._store_pose(4.0, 5.0, 0.25)
    w.apply_layout("monitor"); pump()
    w._sync_state()
    # as after a crash: state as of the last tick, nothing said goodbye
    pm._exec_running, pm._exec_idx = False, -1
    w._state_timer.stop()
    w.deleteLater()
    pump()

    w2 = MainWindow()
    w2.resize(1400, 880)
    w2.show()
    pump(0.3)
    p2 = w2.p_mission
    sent = []
    w2.ros.send_nav_goal = lambda *x: sent.append(x)
    check("mission open again", p2.cur.id == mid, p2.cur.name)
    check("waypoint where it was selected", p2.selected_rows() == [1], f"{p2.selected_rows()}")
    check("says where it was", "waypoint 2" in p2.status_lbl.text(), p2.status_lbl.text())
    check("logged", any("Not restarted" in e.text for e in w2.events.events))
    check("SLAM choice back", w2.p_robot.rb_slam.isChecked())
    check("layout back", w2.workspace.visible_keys() == {"map", "cam", "status"}, f"{w2.workspace.visible_keys()}")
    pump(0.6)
    check("NOTHING moves by itself", not p2._exec_running and not sent)
    w2._sync_state()
    check("still remembered if closed again", w2.robot_state.get("run", {}).get("index") == 1)
    p2._play_from_selected()
    check("goes on from there with its lap", p2._exec_running and p2._exec_idx == 1 and p2._lap == 2
          and len(sent) == 1, f"{p2._exec_idx} {p2._lap}")
    p2._stop()
    w2._sync_state()
    check("forgotten once stopped", w2.robot_state.get("run") is None)
    del w2.ros.send_nav_goal
    w2.close()


def main() -> int:
    a, b = seed_old_install()
    test_map_files()
    test_limits()
    test_hostile_packs()
    threading.Thread(target=lambda: asyncio.run(
        mock_rosbridge.main_async("127.0.0.1", PORT, duration=300)), daemon=True).start()
    time.sleep(0.6)
    theme.apply(app)
    w = MainWindow()
    w.resize(1400, 880)
    w.show()
    pump(0.2)
    test_migration(w, a, b)
    test_keys(w)
    test_per_robot(w, a, b)
    test_wrong_map(w)
    test_pack_round_trip(w, a)
    test_transfer(w)
    w.profiles.set_active(a.id)
    w._switch_robot(a)
    w.close()
    test_restart(a)
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    os._exit(code)
