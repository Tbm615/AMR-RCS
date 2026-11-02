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

from PySide6.QtCore import QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

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
