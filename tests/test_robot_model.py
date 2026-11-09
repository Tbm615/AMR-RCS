"""Robot models: the Hinson AMR the app comes with (wheels in their wells), models
made from a URDF and its meshes, seen from above on the 2D maps and the Fleet
cards, chosen per robot, carried in packs.

Run:
    python tests/test_robot_model.py

Needs nothing: the URDFs and meshes are written here. With the robot's
workspace next to RCS (../src/bringup) its URDF is checked too.
"""
from __future__ import annotations

import json
import math
import os
import struct
import sys
import tempfile
import zipfile
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
CFG = tempfile.mkdtemp(prefix="rcs_model_cfg_")
os.environ["XDG_CONFIG_HOME"] = CFG
os.environ["APPDATA"] = CFG

import numpy as np
from PySide6.QtWidgets import QApplication

from rcs.core import pack as packs
from rcs.core import robot_model as rm
from rcs.core.fleet import describe, heading, overall
from rcs.core.maps import MapLibrary
from rcs.core.profile import ProfileStore, RobotProfile
from rcs.core.robot_data import robot_dir

app = QApplication(sys.argv)
from rcs.ui.main_window import MainWindow  # noqa: E402
from rcs.ui.widgets.map_view import MapView  # noqa: E402
from rcs.ui.widgets.robot_shape import RobotIcon, paths_for  # noqa: E402

PASS = FAIL = 0
TMP = Path(tempfile.mkdtemp(prefix="rcs_model_tmp_"))
WELL = (0.124, 0.219)           # |y| of the drive wheel wells in base_link.STL


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def raises(fn, *words) -> str:
    try:
        fn()
    except (rm.ModelError, ValueError) as e:
        text = str(e)
        return text if all(w in text for w in words) else f"wrong reason: {text}"
    return ""


# ---------------------------------------------------------------- fixtures

def write_stl(path: Path, tris: np.ndarray, binary: bool = True) -> None:
    if binary:
        with open(path, "wb") as f:
            f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
            for t in tris:
                f.write(struct.pack("<3f", 0, 0, 0) + struct.pack("<9f", *t.reshape(-1)) + b"\0\0")
    else:
        lines = ["solid x"]
        for t in tris:
            lines += ["facet normal 0 0 0", " outer loop"] + [f"  vertex {a} {b} {c}" for a, b, c in t]
            lines += [" endloop", "endfacet"]
        path.write_text("\n".join(lines + ["endsolid x"]))


def make_package() -> Path:
    """A ROS-style package: urdf/robot.urdf, meshes/ (STL body in mm-free metres,
    OBJ bumper, COLLADA lidar in millimetres with y up)."""
    pkg = TMP / "ws" / "src" / "toy_robot"
    if (pkg / "urdf" / "robot.urdf").exists():
        return pkg / "urdf" / "robot.urdf"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    (pkg / "package.xml").write_text("<package><name>toy_robot</name></package>")
    write_stl(pkg / "meshes" / "body.stl", rm.box(0.6, 0.4, 0.2))
    write_stl(pkg / "meshes" / "plate.STL", rm.box(0.1, 0.1, 0.01), binary=False)
    (pkg / "meshes" / "bumper.obj").write_text(
        "v 0 -0.2 0\nv 0.05 -0.2 0\nv 0.05 0.2 0\nv 0 0.2 0\nv 0 -0.2 0.05\nv 0 0.2 0.05\n"
        "f 1 2 3 4\nf 1 4 6 5\n")
    # 100 x 100 x 40 mm, y up, in millimetres: lands 0.04 m tall, z up
    cube = rm.box(100, 40, 100)        # 40 mm along y = up
    pos = " ".join(f"{v:g}" for v in cube.reshape(-1))
    idx = " ".join(str(i) for i in range(len(cube) * 3))
    (pkg / "meshes" / "lidar.dae").write_text(f"""<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="0.001"/><up_axis>Y_UP</up_axis></asset>
  <library_geometries><geometry id="g"><mesh>
    <source id="p"><float_array id="pa" count="{cube.size}">{pos}</float_array></source>
    <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
    <triangles count="{len(cube)}"><input semantic="VERTEX" source="#v" offset="0"/><p>{idx}</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="s"><node><instance_geometry url="#g"/></node></visual_scene>
  </library_visual_scenes>
</COLLADA>""")
    (pkg / "urdf" / "robot.urdf").write_text("""<?xml version="1.0"?>
<robot name="toy">
  <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>
  <link name="base_footprint"/>
  <link name="base_link">
    <visual><origin xyz="0 0 0.15"/><geometry><mesh filename="package://toy_robot/meshes/body.stl"/></geometry>
      <material name="grey"/></visual>
    <visual><origin xyz="0.3 0 0.1"/><geometry><mesh filename="package://toy_robot/meshes/bumper.obj"/></geometry>
    </visual>
    <visual><origin xyz="0 0 0.26"/><geometry><mesh filename="../meshes/plate.STL"/></geometry></visual>
  </link>
  <joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 0.05"/></joint>
  <link name="left_wheel_link"><visual><origin rpy="1.5708 0 0"/>
    <geometry><cylinder radius="0.1" length="0.04"/></geometry>
    <material name="black"><color rgba="0.1 0.1 0.1 1"/></material></visual></link>
  <joint name="left_wheel_joint" type="continuous"><parent link="base_link"/><child link="left_wheel_link"/>
    <origin xyz="0 0.22 0.05"/><axis xyz="0 1 0"/></joint>
  <link name="right_wheel_link"><visual><origin rpy="1.5708 0 0"/>
    <geometry><cylinder radius="0.1" length="0.04"/></geometry></visual></link>
  <joint name="right_wheel_joint" type="continuous"><parent link="base_link"/><child link="right_wheel_link"/>
    <origin xyz="0 -0.22 0.05"/><axis xyz="0 1 0"/></joint>
  <link name="caster_link"><visual><geometry><sphere radius="0.05"/></geometry></visual></link>
  <joint name="caster_joint" type="fixed"><parent link="base_link"/><child link="caster_link"/>
    <origin xyz="-0.25 0 0"/></joint>
  <link name="laser"><visual><geometry><mesh filename="package://toy_robot/meshes/lidar.dae"/></geometry>
  </visual></link>
  <joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser"/>
    <origin xyz="0.2 0 0.3"/></joint>
  <link name="ghost"><visual><geometry><mesh filename="package://toy_robot/meshes/missing.stl"/></geometry>
  </visual></link>
  <joint name="ghost_joint" type="fixed"><parent link="base_link"/><child link="ghost"/></joint>
</robot>""")
    return pkg / "urdf" / "robot.urdf"


# ---------------------------------------------------------------- tests

def test_builtin() -> None:
    print("\n[the Hinson AMR that comes with the app]")
    m = rm.builtin()
    check("loads", m is not None and len(m) > 1000)
    lo, hi = m.bounds()
    check("its size", 0.85 < hi[0] - lo[0] < 0.9 and 0.53 < hi[1] - lo[1] < 0.57, f"{hi - lo}")
    check("stands on the floor", abs(lo[2]) < 1e-4)
    check("every part found", all(m.count(p) > 0 for p in (rm.BODY, rm.WHEEL, rm.CASTER, rm.SENSOR)),
          str({rm.PART_NAMES[p]: m.count(p) for p in rm.PART_NAMES}))
    w = m.triangles[m.parts == rm.WHEEL].reshape(-1, 3)
    left, right = w[w[:, 1] > 0], w[w[:, 1] < 0]
    for name, pts in (("left", left), ("right", right)):
        ylo, yhi = np.abs(pts[:, 1]).min(), np.abs(pts[:, 1]).max()
        check(f"{name} drive wheel inside its wheel well", WELL[0] - 0.005 <= ylo and yhi <= WELL[1] + 0.005,
              f"|y| {ylo:.4f}..{yhi:.4f}, well {WELL}")
        centre = (ylo + yhi) / 2
        check(f"{name} drive wheel centred in it (0.1715)", abs(centre - 0.1715) < 0.003, f"{centre:.4f}")
        check(f"{name} drive wheel on the floor", pts[:, 2].min() < 0.002, f"{pts[:, 2].min():.4f}")
    check("left and right the same", abs(left[:, 1].mean() + right[:, 1].mean()) < 0.003)
    check("wheels under the body, x about 0", abs(w[:, 0].mean()) < 0.01)
    c = m.triangles[m.parts == rm.CASTER].reshape(-1, 3)
    check("casters on the floor", c[:, 2].min() < 0.002)
    check("camera at the front, on top", m.triangles[m.parts == rm.SENSOR].reshape(-1, 3)[:, 0].min() > 0.3)
    fl, fw = m.footprint()
    check("footprint from the model", abs(fl - 0.441) < 0.005 and abs(fw - 0.276) < 0.005, f"{fl:.3f} {fw:.3f}")
    check("auto: the default profile is its size", rm.fits(m, 0.45, 0.30))
    check("auto: a small robot is not", not rm.fits(m, 0.25, 0.20))


def test_workspace_urdf() -> None:
    print("\n[the robot's own URDF, when the workspace is here]")
    urdf = ROOT.parent / "src" / "bringup" / "urdf" / "robot_description.urdf.xacro"
    if not urdf.exists():
        print("  (skipped: no ../src/bringup)")
        return
    import xml.etree.ElementTree as ET
    text = urdf.read_text()
    root = ET.fromstring(text.replace("${prefix}", ""))
    joints = {j.get("name"): j for j in root.iter("joint")}
    for name, sign in (("left_wheel_joint", 1), ("right_wheel_joint", -1)):
        j = joints[name]
        xyz = [float(v) for v in j.find("origin").get("xyz").split()]
        check(f"{name} at the centre of its wheel well", abs(xyz[1] - sign * 0.17145) < 1e-4, str(xyz))
        rot = rm.rpy_matrix(*[float(v) for v in j.find("origin").get("rpy").split()])
        axis = rot @ np.array([float(v) for v in j.find("axis").get("xyz").split()])
        check(f"{name} turns forward about +y", axis[1] > 0.99, str(axis.round(3)))
        link = root.find(f".//link[@name='{name.replace('_joint', '_link')}']")
        vis = link.find("visual/origin")
        mesh = rm.read_stl(ROOT.parent / "src" / "bringup" / "meshes" /
                           Path(link.find("visual/geometry/mesh").get("filename")).name)
        local = mesh.reshape(-1, 3) + [float(v) for v in vis.get("xyz").split()]
        mid = (local.min(0) + local.max(0)) / 2
        check(f"{name.split('_')[0]} wheel mesh centred on its joint", np.abs(mid).max() < 1e-3, str(mid.round(4)))
        col = link.find("collision")
        world = rot @ np.array([float(v) for v in col.find("origin").get("xyz").split()]) + xyz
        check(f"{name.split('_')[0]} tyre contact at y = {sign * 0.195}", abs(world[1] - sign * 0.195) < 1e-3,
              str(world.round(4)))


def test_urdf_import() -> None:
    print("\n[a model from a URDF and its meshes]")
    urdf = make_package()
    m, warnings = rm.import_file(urdf)
    check("made", len(m) > 0, m.describe())
    check("named after the robot", m.name == "toy" and m.source == "robot.urdf")
    lo, hi = m.bounds()
    check("stands on the floor", abs(lo[2]) < 1e-5)
    check("placed from base_link, not the root link",
          abs(lo[0] + 0.30) < 0.002 and abs(hi[0] - 0.35) < 0.002, f"x {lo[0]:.3f}..{hi[0]:.3f}")
    body = m.triangles[m.parts == rm.BODY].reshape(-1, 3)
    # wheel bottom = 0.05 - 0.1 = -0.05 in base_link; floor = -0.05; body (z 0.05..0.25) +0.05
    check("body above its wheels", abs(body[:, 2].min() - 0.10) < 0.002, f"{body[:, 2].min():.3f}")
    wheels = m.triangles[m.parts == rm.WHEEL].reshape(-1, 3)
    check("cylinder wheels, both sides", wheels[:, 1].max() > 0.23 and wheels[:, 1].min() < -0.23)
    check("cylinder axis along y", np.ptp(wheels[wheels[:, 1] > 0][:, 1]) < 0.041)
    check("caster found", m.count(rm.CASTER) > 0)
    s = m.triangles[m.parts == rm.SENSOR].reshape(-1, 3)
    check("COLLADA in millimetres read as metres", 0.09 < np.ptp(s[:, 0]) < 0.11, f"{np.ptp(s, axis=0)}")
    check("COLLADA y up turned to z up", 0.035 < np.ptp(s[:, 2]) < 0.045 and 0.09 < np.ptp(s[:, 1]) < 0.11,
          f"{np.ptp(s, axis=0)}")
    grey = (m.colors == np.array([0.5, 0.5, 0.5, 1], np.float32)).all(1)
    check("material by name", grey.any())
    check("material in place", ((m.colors[:, 0] - 0.1) ** 2 < 1e-6).any())
    check("a missing mesh is said, the rest still drawn", any("missing.stl" in w for w in warnings), str(warnings))
    check("OBJ and text STL read", m.count(rm.BODY) >= 12 + 4 + 12)


def test_files() -> None:
    print("\n[model files from anyone]")
    m = rm.builtin()
    path = rm.save(m, TMP / "m.npz")
    back = rm.read_file(path)
    check("saved and read back", np.allclose(back.triangles, m.triangles) and (back.parts == m.parts).all()
          and back.name == m.name)
    data = rm.to_bytes(m)
    check("as bytes too", np.allclose(rm.from_bytes(data).triangles, m.triangles))
    bad = TMP / "pickle.npz"
    np.savez(bad, triangles=np.array([object()], dtype=object), colors=np.zeros((1, 4)))
    check("no pickles", raises(lambda: rm.read_file(bad)) != "" and rm.load(bad) is None)
    t = np.zeros((2, 3, 3), np.float32)
    t[0, 0, 0] = np.nan
    check("NaN refused", raises(lambda: rm.checked(t, np.ones((2, 4))), "not numbers") != "")
    check("wrong shape refused", raises(lambda: rm.checked(np.zeros((2, 3)), np.ones((2, 4)))) != "")
    check("kilometres refused", raises(lambda: rm.checked(np.full((1, 3, 3), 900.0), np.ones((1, 4))),
                                       "metres") != "")
    check("colours that do not match refused", raises(lambda: rm.checked(np.zeros((2, 3, 3)), np.ones((3, 4)))) != "")
    c = rm.checked(np.zeros((1, 3, 3)) + [[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.array([[2.0, -1, 0.5, 1]]),
                   np.array([9]))
    check("colours and parts clamped", c.colors.max() <= 1 and c.colors.min() >= 0 and c.parts[0] == rm.SENSOR)
    check("not a model", raises(lambda: rm.read_file(TMP / "nothing.npz")) != "")
    check("unknown file type", raises(lambda: rm.import_file(TMP / "robot.step"), "pick") != "")
    one, w = rm.import_file(make_stl_mm())
    check("one STL in millimetres", 0.4 < np.ptp(one.triangles.reshape(-1, 3)[:, 0]) < 0.6
          and any("millimetres" in x for x in w), str(w))
    xa = TMP / "r.xacro"
    xa.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="r"><link name="a"/></robot>')
    old = os.environ["PATH"]
    os.environ["PATH"] = str(TMP)          # no xacro command here
    try:
        check("a xacro without ROS says how", raises(lambda: rm.import_file(xa), "xacro") != "")
    finally:
        os.environ["PATH"] = old
    bad_dae = TMP / "bad.dae"
    bad_dae.write_text('<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema"><library_geometries>'
                       '<geometry id="g"><mesh><source id="p"><float_array>0 0 0 1 0 0</float_array></source>'
                       '<vertices id="v"><input semantic="POSITION" source="#p"/></vertices><triangles>'
                       '<input semantic="VERTEX" source="#v" offset="0"/><p>0 1 7</p></triangles></mesh>'
                       '</geometry></library_geometries></COLLADA>')
    two = TMP / "two.urdf"
    two.write_text('<robot name="r"><link name="base_link"><visual><geometry><box size="1 1 1"/></geometry>'
                   '</visual><visual><geometry><mesh filename="bad.dae"/></geometry></visual></link></robot>')
    m2, w2 = rm.import_file(two)
    check("a damaged mesh is skipped, the rest drawn", len(m2) == 12 and any("damaged" in x for x in w2), str(w2))
    check("alone, it says why", raises(lambda: rm.import_file(bad_dae), "damaged") != "")
    nan = TMP / "nan.obj"
    nan.write_text("v 0 0 0\nv nan 0 0\nv 0 1 0\nv 1 1 0\nf 1 2 3\nf 1 3 4\n")
    check("triangles that are not numbers are dropped", len(rm.import_file(nan)[0]) == 1)
    loop = TMP / "loop.urdf"
    loop.write_text('<robot name="l"><link name="a"><visual><geometry><box size="1 1 1"/></geometry></visual>'
                    '</link><link name="b"/><joint name="j1" type="fixed"><parent link="a"/><child link="b"/>'
                    '</joint><joint name="j2" type="fixed"><parent link="b"/><child link="a"/></joint></robot>')
    check("joints in a loop are refused", raises(lambda: rm.import_file(loop), "loop") != "")
    big = rm.checked(np.random.default_rng(1).random((rm.MAX_TRIANGLES + 5000, 3, 3)).astype(np.float32),
                     np.ones((rm.MAX_TRIANGLES + 5000, 4)))
    warn: list = []
    t2, _c2, _p2 = rm._budget(big.triangles.astype(np.float64), big.colors, big.parts, warn)
    check("too many triangles: simplified", len(t2) <= rm.MAX_TRIANGLES and warn, f"{len(t2)} {warn}")


def make_stl_mm() -> Path:
    p = TMP / "chassis_mm.stl"
    write_stl(p, rm.box(500, 300, 200))
    return p


def test_top_view() -> None:
    print("\n[seen from above]")
    m = rm.checked(rm.box(0.8, 0.5, 0.3) + [0.1, 0, 0.15], np.ones((12, 4)))
    tv = m.top_view(0.01)
    area = tv.masks[rm.BODY].sum() * 0.01 ** 2
    check("a box covers its area", abs(area - 0.40) < 0.03, f"{area:.3f} m²")
    hin = rm.builtin().top_view()
    wheel = hin.masks[rm.WHEEL]
    rows, cols = np.nonzero(wheel)
    ys = hin.y0 + (rows + 0.5) * hin.cell
    check("Hinson wheels seen in their wells", (np.abs(ys) >= WELL[0] - 0.01).all() and (np.abs(ys) <= WELL[1] + 0.01).all())
    paths = paths_for(rm.builtin(), 0.45, 0.30)
    r = paths[rm.BODY].boundingRect()
    check("body outline in metres", abs(r.width() - 0.875) < 0.02 and abs(r.height() - 0.55) < 0.02,
          f"{r.width():.3f} × {r.height():.3f}")
    check("one outline, no seams between cells", paths[rm.BODY].elementCount() < 600, str(paths[rm.BODY].elementCount()))
    box = paths_for(None, 0.3, 0.2)
    check("no model: a box of the footprint", set(box) == {rm.BODY}
          and abs(box[rm.BODY].boundingRect().width() - 0.6) < 1e-6)


def test_views() -> None:
    print("\n[2D map and Fleet icon]")
    v = MapView()
    v.resize(600, 500)
    v.show()
    v.set_robot_footprint(0.45, 0.30)
    v.set_robot_model(rm.builtin())
    v.set_robot_pose(2.0, 1.0, math.pi / 2)
    parts = v.robot_shape.parts()
    check("body, wheels, casters, sensors drawn", parts == [rm.BODY, rm.WHEEL, rm.CASTER, rm.SENSOR], str(parts))
    t = v.robot_shape.items[rm.BODY].transform()
    check("placed at the pose", abs(t.dx() - 2.0) < 1e-9 and abs(t.dy() - 1.0) < 1e-9 and abs(t.m11()) < 1e-9
          and abs(t.m12() - 1.0) < 1e-9, f"{t.dx()} {t.dy()} {t.m11():.3f} {t.m12():.3f}")
    bx, by = v.robot_body.getData()
    check("footprint still drawn (dashed)", len(bx) == 5)
    v.set_robot_model(None)
    check("a box when there is no model", v.robot_shape.parts() == [rm.BODY])
    v.set_robot_footprint(0.3, 0.2)
    r = v.robot_shape.items[rm.BODY].path().boundingRect()
    check("the box follows the footprint", abs(r.width() - 0.6) < 1e-6 and abs(r.height() - 0.4) < 1e-6)
    icon = RobotIcon(64)
    icon.set_model(rm.builtin())
    icon.set_state(0.3, "error", 0.4, 0.0)
    icon.show()
    app.processEvents()
    img = icon.grab().toImage()
    reds = sum(1 for x in range(0, 64, 2) for y in range(0, 64, 2)
               if img.pixelColor(x, y).red() > 200 and img.pixelColor(x, y).green() < 140)
    check("icon painted in the level's colour", reds > 5, f"{reds} red samples")
    d = {"_connected": True, "_ages": {"odom": 1, "map_pose": 1, "gate": 1},
         "odom": {"v": 0, "w": 0, "x": 0, "y": 0, "yaw": 0.5}, "map_pose": {"x": 1, "y": 2, "yaw": 1.2},
         "gate": {"engaged": False}}
    check("heading on the map when AMCL has it", heading(d) == 1.2)
    d["_ages"]["map_pose"] = 60
    check("else by odometry", heading(d) == 0.5)
    check("no heading without data", heading({"_ages": {}}) is None)
    check("overall: offline is error", overall(describe({"_connected": False})) == "error")
    d["gate"] = {"engaged": True}
    check("overall: E-STOP is error", overall(describe(d)) == "error")


def test_connection_page() -> None:
    print("\n[Connection page: a model per robot]")
    w = MainWindow()
    w.resize(1400, 900)
    w.show()
    w._show("conn")
    page = w.p_conn
    page.confirm_dialogs = False
    prof = w.profiles.active()
    check("Hinson shown for the default robot", "Hinson AMR" in page.model_info.text(), page.model_info.text())
    check("nothing imported yet", not page.model_remove.isEnabled()
          and not page.model_combo.model().item(page.model_combo.findData("imported")).isEnabled())
    ok = page.import_model_from(str(make_package()))
    check("URDF imported", ok and prof.model == "imported", page.model_error)
    check("kept in the robot's own folder", (robot_dir(prof.id) / rm.MODEL_FILE).exists())
    check("the page says so, with what was missing", "toy" in page.model_info.text()
          and "missing.stl" in page.model_info.text(), page.model_info.text()[:200])
    check("profile saved", ProfileStore().active().model == "imported")
    check("the 3D Map draws it", w.p_3d.model is not None and w.p_3d.model.name == "toy" and w.p_3d.uses_model())
    check("the 2D maps draw it", w.p_map.view.robot_shape.model is not None
          and w.p_map.view.robot_shape.model.name == "toy")
    check("its Fleet card too", w.p_fleet.cards[prof.id].icon.model is not None
          and w.p_fleet.cards[prof.id].icon.model.name == "toy")
    page._footprint_from_model()
    check("footprint from the model", abs(page.l_len.value() - 0.70) < 0.011 and abs(page.l_wid.value() - 0.48) < 0.011,
          f"{page.l_len.value()} × {page.l_wid.value()}")
    page._save()
    check("footprint saved", abs(prof.limits.footprint_half_length - 0.35) < 0.006)
    bad = TMP / "broken.urdf"
    bad.write_text("<robot><link")
    check("a broken file is not imported", not page.import_model_from(str(bad)) and "not valid XML" in page.model_error
          and prof.model == "imported", page.model_error)
    page.model_combo.setCurrentIndex(page.model_combo.findData("box"))
    page._model_chosen(0)
    check("box chosen", prof.model == "box" and w.p_3d.model is None and w.p_map.view.robot_shape.model is None)
    page.model_combo.setCurrentIndex(page.model_combo.findData("hinson"))
    page._model_chosen(0)
    check("Hinson chosen whatever the footprint", w.p_3d.uses_model() and w.p_3d.model.name == "Hinson AMR")
    page._remove_model()
    check("imported model removed", not (robot_dir(prof.id) / rm.MODEL_FILE).exists() and prof.model == "hinson")

    print("\n[selecting another robot while connected]")
    a = prof
    b = RobotProfile.new("other", "10.0.0.9")
    b.limits.max_linear, b.limits.footprint_half_length, b.model = 1.5, 1.0, "box"
    w.profiles.upsert(b)
    page.refresh()
    w.ros.profile, w.ros._want_connected = a, True
    w._apply_profile_to_panels(a)
    lin = w.p_teleop.lin.value()
    page._select_id(b.id)
    app.processEvents()
    check("the robot being driven keeps its speed limit", abs(w.p_teleop.lin.value() - lin) < 1e-9,
          f"{w.p_teleop.lin.value()} vs {lin}")
    check("and its model", w.p_3d.model is not None)
    w.ros._want_connected = False
    page._select_id(a.id)
    page._select_id(b.id)
    app.processEvents()
    check("not connected: the selected robot's limits apply", abs(w.p_teleop.lin.value() - 1.5) < 1e-9)
    w.close()


def test_packs() -> None:
    print("\n[the model in backups and shared files]")
    model, _w = rm.import_file(TMP / "ws" / "src" / "toy_robot" / "urdf" / "robot.urdf")
    prof = RobotProfile.new("toy one")
    prof.model = "imported"
    out = packs.export_pack(TMP / "toy", robot_name="toy one", profile=prof, model=model)
    p = packs.read_pack(out)
    check("read back", p.model is not None and p.model.name == "toy" and len(p.model) == len(model))
    store = ProfileStore(TMP / "robots.json")
    lib = MapLibrary(TMP / "maps")
    res = packs.import_as_new_robot(p, {"model"}, lib, store)
    f = robot_dir(res.robot.id) / rm.MODEL_FILE
    check("new robot gets it", f.exists() and res.robot.model == "imported" and res.model == "added", res.text())
    res2 = packs.ImportResult()
    packs.install_model(p, res.robot, store, res2)
    check("never replaces one it has", res2.model == "kept" and "not taken" in res2.text())
    res3 = packs.import_as_new_robot(p, set(), lib, store)
    check("without the model: not drawn as one it lacks", res3.robot.model == "auto")
    with zipfile.ZipFile(out) as z:
        members = {n: z.read(n) for n in z.namelist()}
    members["model.npz"] = b"PK\x03\x04 not a model"
    bad = TMP / "bad.rcspack"
    with zipfile.ZipFile(bad, "w") as z:
        for n, data in members.items():
            z.writestr(n, data)
    check("a damaged model is said", raises(lambda: packs.read_pack(bad), "robot model") != "")
    d = json.loads(members["profile.json"])
    d["model"] = "../../etc"
    check("an unknown model kind is auto", RobotProfile.from_dict(d).model == "auto")


def main() -> int:
    test_builtin()
    test_workspace_urdf()
    test_urdf_import()
    test_files()
    test_top_view()
    test_views()
    test_connection_page()
    test_packs()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
