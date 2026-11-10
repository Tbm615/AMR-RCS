"""Keepout and speed zones: the rules (app and robot side), the masks Nav2 gets,
and the 2D Map page against the mock robot.

Run:
    python tests/test_zones.py

tests/test_zones_ros.py runs them in real Nav2 (needs ROS 2 and the workspace).
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "robot"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_zones_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]

import numpy as np
from PySide6.QtCore import QPoint, QPointF, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

import zone_masks as zm
from rcs.core import zones as zn
from rcs.core.profile import RobotProfile
from rcs.ui import theme
from rcs.ui.main_window import MainWindow
from tests import mock_rosbridge

PORT = 9197
TMP = Path(tempfile.mkdtemp(prefix="rcs_zones_"))
app = QApplication(sys.argv)
PASS = FAIL = 0


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def raises(fn, *words) -> str:
    try:
        fn()
    except ValueError as e:
        return str(e) if all(w in str(e) for w in words) else f"other error: {e}"
    return ""


def wait_until(cond, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.02)
    return False


def box(cx, cy, r):
    return [[cx - r, cy - r], [cx + r, cy - r], [cx + r, cy + r], [cx - r, cy + r]]


def test_rules() -> None:
    print("\n[the rules, app and robot side alike]")
    good = [{"id": "k1", "name": "Stairs", "kind": "keepout", "points": box(1, 1, 0.5)},
            {"id": "s1", "name": "Hall", "kind": "slow", "speed": 40, "points": box(3, 1, 1)}]
    for mod, fn in (("app", zn.check), ("robot", zm.check_zones)):
        out = fn(good)
        check(f"{mod}: good zones kept", [z["id"] for z in out] == ["k1", "s1"] and out[1]["speed"] == 40)
        check(f"{mod}: two corners refused", raises(lambda: fn([{"kind": "keepout", "points": [[0, 0], [1, 1]]}]),
                                                    "corners") != "")
        check(f"{mod}: a line has no area", raises(lambda: fn([{"kind": "keepout",
                                                                "points": [[0, 0], [1, 0], [2, 0]]}]), "area") != "")
        check(f"{mod}: speed 5 % refused", raises(lambda: fn([{"kind": "slow", "speed": 5, "points": box(0, 0, 1)}]),
                                                  "speed") != "")
        check(f"{mod}: unknown kind refused", raises(lambda: fn([{"kind": "fast", "points": box(0, 0, 1)}]),
                                                     "kind") != "")
        check(f"{mod}: NaN refused", raises(lambda: fn([{"kind": "keepout",
                                                         "points": [[0, 0], [1, float("nan")], [0, 1]]}])) != "")
        check(f"{mod}: repeated id refused", raises(lambda: fn([good[0], dict(good[0])]), "id") != "")
        check(f"{mod}: too many refused", raises(lambda: fn([dict(good[0], id=f"k{i}") for i in range(201)]),
                                                 "at most") != "")
    z = good
    check("inside a square", zn.contains(z[0]["points"], 1.2, 0.9) and not zn.contains(z[0]["points"], 1.6, 1.0))
    concave = [[0, 0], [4, 0], [4, 4], [2, 1], [0, 4]]          # a notch from the top
    check("concave: the notch is outside", not zn.contains(concave, 2, 3) and zn.contains(concave, 0.5, 3))
    check("keepout found at a point", zn.keepout_at(z, 1, 1)["name"] == "Stairs" and zn.keepout_at(z, 3, 1) is None)
    two = z + [{"id": "s2", "name": "Door", "kind": "slow", "speed": 20, "points": box(3, 1, 0.3)}]
    check("overlapping slow zones: the lowest speed", zn.speed_at(two, 3, 1) == 20 and zn.speed_at(two, 3.8, 1) == 40)
    check("centroid of a square", np.allclose(zn.centroid(box(2, -1, 1)), (2, -1)))
    check("new ids and names do not repeat", zn.new_id(z, "keepout") == "k2" and zn.new_name(z, "slow") == "Slow 1")


def test_masks() -> None:
    print("\n[masks for Nav2]")
    maps = TMP / "maps"
    maps.mkdir()
    img = np.full((60, 80), 254, np.uint8)
    (maps / "m.pgm").write_bytes(b"P5\n# a comment\n80 60\n255\n" + img.tobytes())
    (maps / "m.yaml").write_text("image: m.pgm\nresolution: 0.05\norigin: [-1.0, -2.0, 0.0]\nnegate: 0\n"
                                 "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    check("geometry read", zm.map_geometry(maps / "m.yaml") == (80, 60, 0.05, -1.0, -2.0))
    zones = [{"id": "k1", "name": "K", "kind": "keepout", "points": [[0, 0], [1, 0], [1, 0.5], [0, 0.5]]},
             {"id": "s1", "name": "S", "kind": "slow", "speed": 50, "points": [[1.5, -1.5], [2.5, -1.5], [2.5, -0.5]]},
             {"id": "s2", "name": "S2", "kind": "slow", "speed": 30, "points": [[2.0, -1.4], [2.4, -1.4], [2.4, -1.0], [2.0, -1.0]]},
             {"id": "o1", "name": "Off", "kind": "keepout", "points": box(50, 50, 1)}]
    keep, speed = zm.rasterize(zones, 80, 60, 0.05, -1.0, -2.0)
    check("keepout cells: 1 m x 0.5 m = 200", int((keep == 100).sum()) == 200, str(int((keep == 100).sum())))
    # the cell of map point (0.5, 0.25): column 30, row from the bottom 45, so PGM row 60-1-45 = 14
    check("keepout where the zone is (PGM rows from the top)", keep[14, 30] == 100 and keep[60 - 1 - 45 + 20, 30] == 0)
    check("slow: the lowest where two overlap", speed[60 - 1 - int((-1.2 + 2) / 0.05), int((2.2 + 1) / 0.05)] == 30)
    check("slow zone value elsewhere in it", 50 in set(speed.ravel().tolist()))
    check("a zone off the map is skipped", keep.shape == (60, 80))
    out = zm.write_masks(maps / "m.yaml", zones, TMP / "masks")
    check("masks written, raw mode", "mode: raw" in Path(out["keepout"]).read_text()
          and Path(out["speed"]).with_suffix(".pgm").exists(), str(out))
    data = Path(out["keepout"]).with_suffix(".pgm").read_bytes()
    check("mask the size of the map", data.startswith(b"P5\n80 60\n255\n") and len(data) == len(b"P5\n80 60\n255\n") + 4800)
    empty = zm.write_masks(maps / "m.yaml", [], TMP / "masks2")
    check("no zones: empty masks (the filters still run)", empty["keepout_cells"] == 0 and empty["slow_cells"] == 0)
    saved = zm.write_zones(maps / "m.yaml", zones[:2])
    check("zones kept next to the map", (maps / "m.zones.json").exists() and zm.read_zones(maps / "m.yaml") == saved)
    (maps / "m.zones.json").write_text("{broken")
    check("a broken zones file reads as none", zm.read_zones(maps / "m.yaml") == [])
    zm.write_zones(maps / "m.yaml", [])
    check("no zones: no file", not (maps / "m.zones.json").exists())
    (maps / "p.yaml").write_text("image: p.png\nresolution: 0.05\norigin: [0, 0, 0]\n")
    check("a .png map cannot have zones", raises(lambda: zm.write_masks(maps / "p.yaml", [], TMP / "m3"), ".pgm") != "")
    params = TMP / "nav2.yml"
    params.write_text("global_costmap:\n  global_costmap:\n    ros__parameters:\n      plugins: [static_layer]\n"
                      "local_costmap:\n  local_costmap:\n    ros__parameters:\n      filters: [other]\n"
                      "controller_server:\n  ros__parameters:\n    controller_frequency: 20.0\n")
    try:
        import yaml     # on the robot it comes with ROS; the app does not need it
    except ImportError:
        print("  (no PyYAML here: the Nav2 parameters part is skipped)")
        return
    d = yaml.safe_load(Path(zm.params_with_filters(params, TMP / "with.yml")).read_text())
    g = d["global_costmap"]["global_costmap"]["ros__parameters"]
    lc = d["local_costmap"]["local_costmap"]["ros__parameters"]
    check("global costmap: keepout only", g["filters"] == ["keepout_filter"] and "speed_filter" not in g)
    check("local costmap (fast updates): keepout and speed, its own filters kept",
          lc["filters"] == ["other", "keepout_filter", "speed_filter"]
          and lc["speed_filter"]["plugin"] == "nav2_costmap_2d::SpeedFilter")
    check("controller listens to the speed limit", d["controller_server"]["ros__parameters"]["speed_limit_topic"]
          == "/speed_limit" and d["controller_server"]["ros__parameters"]["controller_frequency"] == 20.0)
    check("the robot's own file untouched", "filters" not in params.read_text().split("local_costmap")[0])


def click_map(view, x, y, double=False) -> None:
    vb = view.getPlotItem().vb
    pt = view.mapFromScene(vb.mapViewToScene(QPointF(x, y)))
    if double:
        QTest.mouseDClick(view.viewport(), Qt.LeftButton, Qt.NoModifier, QPoint(pt.x(), pt.y()))
    else:
        QTest.mouseClick(view.viewport(), Qt.LeftButton, Qt.NoModifier, QPoint(pt.x(), pt.y()))
    app.processEvents()


def test_app() -> None:
    print("\n[2D Map page against the mock robot]")
    threading.Thread(target=lambda: asyncio.run(
        mock_rosbridge.main_async("127.0.0.1", PORT, duration=120)), daemon=True).start()
    time.sleep(0.6)
    theme.apply(app)
    w = MainWindow()
    w.resize(1400, 900)
    w.show()
    zb = w.p_map.zones
    check("offline: says so, nothing to edit", "Connect" in zb.where.text() and not zb.add_keep.isEnabled())
    prof = RobotProfile.new(name="mock")
    prof.host, prof.port = "127.0.0.1", PORT
    w.profiles.upsert(prof)
    w._on_connect(prof)
    check("connected", wait_until(w.ros.is_connected, 10))
    check("no Nav2: zones belong to a map", wait_until(lambda: "saved map" in zb.where.text(), 5), zb.where.text())
    L = mock_rosbridge.LAUNCHER
    w.ros.send_launcher_cmd("start", stack="bringup")
    wait_until(lambda: L.stacks["bringup"]["state"] == "running", 5)
    w.ros.send_launcher_cmd("start", stack="nav", mode="map", map="map_a")
    check("Nav2 on map_a: its zones asked for", wait_until(lambda: zb.loaded and zb.map_name == "map_a", 8),
          zb.where.text())
    check("none yet", zb.zones == [] and "0 on the robot" in zb.where.text(), zb.where.text())

    w._show("map")
    app.processEvents()
    view = w.p_map.view
    view.fit_map()
    app.processEvents()
    zb.add_keep.click()
    check("drawing tool chosen", view._click_mode == "zone" and view.zone_kind == "keepout"
          and w.p_map.mode.currentIndex() == 4)
    for x, y in ((0.2, -1.2), (1.2, -1.2), (1.2, -0.4)):
        click_map(view, x, y)
    check("corners added with the mouse", len(view.zone_corners()) == 3, str(view.zone_corners()))
    click_map(view, 0.2, -0.4, double=True)
    check("double-click closes it: a keepout zone", len(zb.zones) == 1 and zb.zones[0]["kind"] == "keepout"
          and len(zb.zones[0]["points"]) == 4, json.dumps(zb.zones)[:200])
    z = zb.zones[0]
    check("corners where clicked", abs(min(p[0] for p in z["points"]) - 0.2) < 0.06
          and abs(max(p[1] for p in z["points"]) + 0.4) < 0.06, str(z["points"]))
    check("drawn on the 2D Map, the Navigation and Missions maps", view.zone_count() == 1
          and w.p_nav.view.zone_count() == 1 and w.p_mission.view.zone_count() == 1)
    check("not saved yet: said, Save on", zb.dirty() and zb.save_btn.isEnabled() and "not saved" in zb.status.text())
    # a slow zone with keys: Enter closes, Backspace takes a corner back
    zb.speed.setValue(30)
    zb.add_slow.click()
    for x, y in ((1.5, 0.2), (2.5, 0.2), (9.0, 9.0), (2.5, 0.8)):
        view.add_zone_corner(x, y)
    QTest.keyClick(view, Qt.Key_Backspace)
    check("Backspace: last corner back", len(view.zone_corners()) == 3)
    view.add_zone_corner(1.5, 0.8)
    QTest.keyClick(view, Qt.Key_Return)
    check("Enter closes it: a slow zone at 30 %", len(zb.zones) == 2 and zb.zones[1]["kind"] == "slow"
          and zb.zones[1]["speed"] == 30)
    zb.speed.setValue(50)
    check("the selected slow zone's speed edited", zb.zones[1]["speed"] == 50 and "50 %" in zb.list.item(1).text())
    view.add_zone_corner(5, 5)
    QTest.keyClick(view, Qt.Key_Escape)
    check("Esc drops a zone being drawn", view.zone_corners() == [] and len(zb.zones) == 2)
    view.add_zone_corner(5, 5)
    view.add_zone_corner(6, 5)
    check("two corners do not make a zone", not view.finish_zone() and len(zb.zones) == 2)
    view.cancel_zone()
    zb.list.setCurrentRow(1)
    zb.rename("Loading dock")
    check("renamed", zb.zones[1]["name"] == "Loading dock")

    zb.save_btn.click()
    check("saved on the robot", wait_until(lambda: not zb.dirty() and "saved on the robot" in zb.status.text(), 5),
          zb.status.text())
    check("the robot has them", [x["name"] for x in L.zones.get("map_a", [])] == ["Keepout 1", "Loading dock"])
    check("Nav2 uses them now", zb.applied is True and "in use" in zb.where.text(), zb.where.text())

    print("\n[goals and keepout zones]")
    w._goal = None
    w.p_map.goal_requested.emit(0.7, -0.8, 0.0)
    app.processEvents()
    check("a goal inside a keepout zone is refused", w._goal is None
          and "keepout zone Keepout 1" in w.statusBar().currentMessage(), w.statusBar().currentMessage())
    w.p_map.goal_requested.emit(2.0, 0.5, 0.0)
    app.processEvents()
    check("a goal in a slow zone goes", w._goal == (2.0, 0.5, 0.0))
    w.p_3d.goal_requested.emit(0.7, -0.8, 0.0)
    app.processEvents()
    check("the 3D Map's goals too", w._goal == (2.0, 0.5, 0.0))

    print("\n[undo, delete, refusals]")
    zb.list.setCurrentRow(0)
    zb.delete()
    check("deleted here, not on the robot yet", len(zb.zones) == 1 and len(L.zones["map_a"]) == 2 and zb.dirty())
    zb.undo_btn.click()
    check("undo brings it back", len(zb.zones) == 2 and not zb.dirty())
    n = len(L.zone_cmds)
    w.ros.send_launcher_cmd("zones_put", name="map_a", zones=[{"kind": "keepout", "points": [[0, 0], [1, 1]]}])
    check("the robot checks too and says why", wait_until(lambda: "Robot:" in zb.status.text()
                                                          and "corners" in zb.status.text(), 5), zb.status.text())
    check("its zones unchanged", len(L.zones["map_a"]) == 2 and len(L.zone_cmds) == n + 1)
    w._apply_role("viewer")
    check("viewer: sees them, cannot change them", not zb.add_keep.isEnabled() and not zb.delete_btn.isEnabled()
          and "viewer" in zb.status.text())
    w._apply_role("")

    print("\n[another map, reconnecting]")
    w.ros.send_launcher_cmd("start", stack="nav", mode="map", map="map_b")
    check("Nav2 on map_b: its own (no) zones", wait_until(lambda: zb.map_name == "map_b" and zb.loaded, 6)
          and zb.zones == [] and view.zone_count() == 0)
    w.ros.send_launcher_cmd("start", stack="nav", mode="map", map="map_a")
    check("back on map_a: the robot's zones again", wait_until(lambda: zb.map_name == "map_a" and len(zb.zones) == 2, 6))
    L.zones["map_a"] = L.zones["map_a"][:1]         # another PC changed them meanwhile
    w.ros.disconnect_robot()
    check("offline: no editing", wait_until(lambda: not zb.add_keep.isEnabled(), 5))
    w._on_connect(prof)
    check("reconnected: asked again", wait_until(lambda: w.ros.is_connected() and len(zb.zones) == 1, 10),
          str(len(zb.zones)))
    L.old = True
    check("a robot side without zones: told to update",
          wait_until(lambda: "too old" in zb.where.text(), 5) and not zb.add_keep.isEnabled(), zb.where.text())
    L.old = False
    w.ros.disconnect_robot()
    app.processEvents()


def main() -> int:
    test_rules()
    test_masks()
    test_app()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
