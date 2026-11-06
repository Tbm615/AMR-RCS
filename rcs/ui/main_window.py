"""Main window — sidebar + page workspace (docks), central controller for signals."""
from __future__ import annotations

import math
import sys
import time

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QAction, QIcon, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication, QButtonGroup, QDialog, QFileDialog, QFrame, QHBoxLayout, QInputDialog, QLabel,
    QMainWindow, QMenu, QMessageBox, QPushButton, QStatusBar, QVBoxLayout, QWidget,
)

from .. import APP_NAME, __version__
from ..core.camera_status import camera_problem, usb_problem
from ..core.clearance import compute_clearance
from ..core.events import EventLog
from ..core.map_transfer import MapTransfer
from ..core.maps import MapLibrary
from ..core.mission import MissionStore
from ..core.pack import (
    SUFFIX, ImportResult, export_pack, import_as_new_robot, import_into, merge_layouts, read_pack,
)
from ..core.places import PlaceStore
from ..core.profile import ProfileStore, RobotProfile, load_settings, save_settings
from ..core.robot_data import RobotState, migrate_legacy, robot_dir
from ..core.ros_client import RosClient, quat_to_yaw
from ..core.sound import Sounds
from .panels.camera import CameraPanel
from .panels.connection import ConnectionPanel
from .panels.data import DataPanel, ExportDialog, ImportDialog
from .panels.events import EventsPanel
from .panels.logs import LogsPanel
from .panels.map2d import Map2DPanel
from .panels.mission import MissionPanel
from .panels.navigation import NavigationPanel
from .panels.status import StatusPanel
from .panels.robot import RobotPanel
from .panels.teleop import TeleopPanel
from .panels.view3d import View3DPanel
from .workspace import PRESETS, Workspace

PAGES = [
    ("conn", "Connection"),
    ("robot", "Robot programs"),
    ("status", "Status"),
    ("teleop", "Teleop"),
    ("map", "2D Map"),
    ("nav", "Navigation"),
    ("mission", "Missions"),
    ("3d", "3D Map"),
    ("cam", "Camera"),
    ("logs", "Logs"),
    ("events", "Events"),
    ("data", "Data"),
]
F_KEY_PAGES = 11        # F1..F11: F12 is E-STOP and must stay E-STOP


class _NavButton(QPushButton):
    def __init__(self, text: str, parent=None):
        super().__init__(text, parent)
        self.setObjectName("NavButton")
        self.setCheckable(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(42)
        self.mods = Qt.NoModifier       # keyboard modifiers of the last mouse click

    def mouseReleaseEvent(self, ev) -> None:
        self.mods = ev.modifiers()
        super().mouseReleaseEvent(ev)

    def take_mods(self):
        m, self.mods = self.mods, Qt.NoModifier
        return m


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME} — Robot Control System")
        self.resize(1400, 880)

        self.profiles = ProfileStore()
        migrate_legacy(self.profiles)
        # The robot whose data is open. While connected that is the connected
        # robot, whatever is selected on the Connection page.
        active = self.profiles.active()
        self._data_robot = active.id if active else ""
        self._layout_robot = self._data_robot    # whose layout is on screen
        folder = robot_dir(self._data_robot)
        self.missions = MissionStore(folder / "missions.json")
        self.events = EventLog(folder / "events.csv")
        self.place_store = PlaceStore(folder / "places.json")
        self.robot_state = RobotState(folder / "state.json")
        self.library = MapLibrary()
        self.settings = load_settings()
        self.sounds = Sounds()
        self.sounds.enabled = bool(self.settings.get("sound", True))
        self.ros = RosClient(profile=self.profiles.active())
        self.tf = self.ros.tf            # filled on the network thread

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._laser_x = 0.0
        self._laser_y = 0.0
        self._laser_yaw = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_v = 0.0
        self._plan_xy: list = []
        self._nav_state = ""
        self._visible: set = set()       # pages shown on screen right now
        # Incoming data only marks things dirty; a 20 Hz timer draws what the
        # open page shows. Drawing every TF/odom/scan on every page kept the
        # GUI thread busy enough for Windows to report "Not Responding".
        self._pose_dirty = False
        self._scan_msg = None
        self._scan_dirty = False
        self._goal = None                # (x, y, yaw) of the active Nav2 goal
        # newest map / costmaps, applied lazily to the views of the open page
        self._layers: dict = {}          # kind -> (version, msg, frame_pose)
        self._layer_ver = 0
        self._applied: dict = {}         # (id(view), kind) -> version

        self._build()
        self._wire()
        self._render_timer = QTimer(self)
        self._render_timer.setInterval(50)
        self._render_timer.timeout.connect(self._render)
        self._render_timer.start()
        self._sensor_ok: dict = {}       # topic stat key -> was it arriving
        self._sensor_timer = QTimer(self)
        self._sensor_timer.setInterval(1000)
        self._sensor_timer.timeout.connect(self._check_sensors)
        self._sensor_timer.start()
        self._restore_robot_state()
        self._state_timer = QTimer(self)        # so that a crash loses little
        self._state_timer.setInterval(5000)
        self._state_timer.timeout.connect(self._sync_state)
        self._state_timer.start()

    # ---- construct UI ----

    def _build(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        side = QFrame(); side.setObjectName("Sidebar")
        side.setFixedWidth(200)
        sv = QVBoxLayout(side)
        sv.setContentsMargins(0, 12, 0, 12)
        sv.setSpacing(0)

        brand = QLabel("RCS")
        brand.setStyleSheet("color:#3b82f6; font-size:22px; font-weight:700; padding:10px 19px 0 19px;")
        sv.addWidget(brand)
        sub = QLabel(f"Robot Control System · v{__version__}")
        sub.setStyleSheet("padding: 0 19px 16px 19px; color:#5b6378; font-size:11px;")
        sv.addWidget(sub)

        self.nav_group = QButtonGroup(self)
        self.nav_group.setExclusive(True)

        self.btns = {}
        for i, (key, label) in enumerate(PAGES):
            b = _NavButton(label)
            f_key = f"F{i + 1} · " if i < F_KEY_PAGES else ""
            b.setToolTip(f"{f_key}Click: show here · Ctrl+click: open beside · right-click: more")
            b.setContextMenuPolicy(Qt.CustomContextMenu)
            b.customContextMenuRequested.connect(lambda _p, k=key: self._page_menu(k))
            self.nav_group.addButton(b)
            sv.addWidget(b)
            self.btns[key] = b
        sv.addStretch(1)

        self.layout_btn = QPushButton("▦  Layout")
        self.layout_btn.setObjectName("NavButton")
        self.layout_btn.setToolTip("Show several pages at once")
        self.layout_menu = QMenu(self.layout_btn)
        self.layout_menu.aboutToShow.connect(self._fill_layout_menu)
        self._fill_layout_menu()
        self.layout_btn.setMenu(self.layout_menu)
        sv.addWidget(self.layout_btn)

        est_frame = QFrame()
        est_frame.setObjectName("EstopBar")
        est_frame.setStyleSheet("QFrame#EstopBar { border-top: 1px solid #1a1d26; }")
        esv = QVBoxLayout(est_frame)
        esv.setContentsMargins(10, 10, 10, 10)
        self.sidebar_estop = QPushButton("● E-STOP")
        self.sidebar_estop.setProperty("kind", "danger")
        self.sidebar_estop.setCheckable(True)
        self.sidebar_estop.setMinimumHeight(44)
        self.sidebar_estop.setStyleSheet("font-weight:700; font-size:14px;")
        esv.addWidget(self.sidebar_estop)
        sv.addWidget(est_frame)

        root.addWidget(side)

        # page area: every page is a dock (split, tab or float it)
        self.workspace = Workspace()
        root.addWidget(self.workspace, 1)

        # instantiate panels
        self.p_conn = ConnectionPanel(self.profiles)
        self.p_robot = RobotPanel(self.ros)
        self.p_status = StatusPanel(self.ros)
        self.p_teleop = TeleopPanel()
        self.p_map = Map2DPanel()
        self.p_nav = NavigationPanel(self.place_store)
        self.p_mission = MissionPanel(self.missions)
        self.p_3d = View3DPanel(self.ros.cloud_map)
        self.p_cam = CameraPanel()
        self.p_logs = LogsPanel()
        self.p_events = EventsPanel(self.events)
        self.transfer = MapTransfer(self.ros, self.library, self)
        self.p_data = DataPanel(self.library, self.transfer)

        self._pages = {
            "conn": self.p_conn,
            "robot": self.p_robot,
            "status": self.p_status,
            "teleop": self.p_teleop,
            "map": self.p_map,
            "nav": self.p_nav,
            "mission": self.p_mission,
            "3d": self.p_3d,
            "cam": self.p_cam,
            "logs": self.p_logs,
            "events": self.p_events,
            "data": self.p_data,
        }
        for key, label in PAGES:
            self.workspace.add_page(key, label, self._pages[key])
        self.workspace.no_float = {"3d"}     # see workspace.py: GL context per window
        self.workspace.visible_changed.connect(self._on_visible_changed)
        self.workspace.active_changed.connect(self._on_active_changed)

        for key, btn in self.btns.items():
            btn.clicked.connect(lambda _c=False, k=key, b=btn: self._show(
                k, "right" if b.take_mods() & Qt.ControlModifier else "replace"))
        saved = self.robot_state.get("layout") or self.settings.get("layout")
        if not (saved and self.workspace.restore_layout(saved)):
            self.workspace.apply_preset("single", "conn")
        self.p_events.sound_ck.setChecked(self.sounds.enabled)
        self.p_events.sound_ck.toggled.connect(self._set_sound)
        self.p_conn.auto_ck.setChecked(bool(self.settings.get("auto_connect", False)))
        self.p_conn.auto_ck.toggled.connect(lambda on: save_settings({"auto_connect": on}))
        self._shortcuts()

        # status bar
        sb = QStatusBar()
        self.setStatusBar(sb)
        self.sb_conn = QLabel("● offline"); self.sb_conn.setStyleSheet("color:#ff6a6a;")
        self.sb_robot = QLabel("—")
        self.sb_pose = QLabel("pose: —")
        self.sb_estop = QLabel("● estop: clear"); self.sb_estop.setStyleSheet("color:#39d98a;")
        sb.addPermanentWidget(self.sb_robot)
        sb.addPermanentWidget(QLabel("   |   "))
        sb.addPermanentWidget(self.sb_conn)
        sb.addPermanentWidget(QLabel("   |   "))
        sb.addPermanentWidget(self.sb_pose)
        sb.addPermanentWidget(QLabel("   |   "))
        sb.addPermanentWidget(self.sb_estop)

    def _show(self, key: str, where: str = "replace") -> None:
        """Show a page: 'replace' (in place of the page in use, like a plain
        sidebar click), 'tab', 'right', 'below' or 'float'."""
        try:
            self.workspace.show_page(key, where)
        except RuntimeError as e:      # e.g. OpenGL refused to start for the 3D page
            print(f"showing page {key}: {e}", file=sys.stderr)
        self._on_visible_changed()

    def active_page(self) -> str:
        return self.workspace.active

    def apply_layout(self, name: str) -> None:
        self.workspace.apply_preset(name, self.workspace.active or "conn")
        self._on_visible_changed()

    # ---- layouts ----

    def _fill_layout_menu(self) -> None:
        m = self.layout_menu
        m.clear()
        for i, (name, label) in enumerate(PRESETS):
            m.addAction(f"{label}\tCtrl+{i + 1}", lambda n=name: self.apply_layout(n))
        mine = self.settings.get("layouts", {})
        if mine:
            m.addSeparator()
            for name in sorted(mine):
                m.addAction(name, lambda n=name: self.load_layout(n))
        m.addSeparator()
        m.addAction("Save this layout…", self._ask_save_layout)
        if mine:
            d = m.addMenu("Delete a layout")
            for name in sorted(mine):
                d.addAction(name, lambda n=name: self.delete_layout(n))

    def _ask_save_layout(self) -> None:
        name, ok = QInputDialog.getText(self, "Save layout", "Name:")
        if ok and name.strip():
            self.save_layout_as(name.strip())

    def save_layout_as(self, name: str) -> None:
        self.settings.setdefault("layouts", {})[name] = self.workspace.save_layout()
        save_settings({"layouts": self.settings["layouts"]})

    def load_layout(self, name: str) -> None:
        data = self.settings.get("layouts", {}).get(name)
        if data and self.workspace.restore_layout(data):
            self._on_visible_changed()

    def delete_layout(self, name: str) -> None:
        self.settings.get("layouts", {}).pop(name, None)
        save_settings({"layouts": self.settings.get("layouts", {})})

    # ---- keyboard ----

    def _shortcuts(self) -> None:
        def key(seq, fn, context=Qt.WindowShortcut):
            QShortcut(QKeySequence(seq), self, fn, context=context)

        for i, (page, _label) in enumerate(PAGES[:F_KEY_PAGES]):
            key(f"F{i + 1}", lambda k=page: self._show(k))
        for i, (name, _label) in enumerate(PRESETS):
            key(f"Ctrl+{i + 1}", lambda n=name: self.apply_layout(n))
        # F12 only engages, from any window of the app: a stuck key or a second
        # press must never release the E-STOP
        key("F12", lambda: self._set_estop(True), Qt.ApplicationShortcut)
        key("Esc", self.on_escape)

    def on_escape(self) -> None:
        """Esc: drop what the mouse is doing on a map, else stop the current goal."""
        for v in (self.p_map.view, self.p_nav.view, self.p_mission.view):
            if v._anchor is not None or v.ruler_m is not None:
                v.cancel_aim()
                v.clear_ruler()
                return
        if self.p_mission._exec_running:
            self.p_mission._pause()
        elif self._nav_state in ("active", "pending"):
            self.ros.cancel_nav_goal()

    def _set_sound(self, on: bool) -> None:
        self.sounds.enabled = on
        save_settings({"sound": on})

    def connect_on_start(self) -> None:
        if self.settings.get("auto_connect") and self.profiles.active():
            self._on_connect(self.profiles.active())

    def _page_menu(self, key: str) -> None:
        self.workspace.page_menu(key).exec(self.btns[key].mapToGlobal(self.btns[key].rect().topRight()))

    def _on_active_changed(self, key: str) -> None:
        if key in self.btns:
            self.btns[key].setChecked(True)

    def _on_visible_changed(self) -> None:
        vis = self.workspace.visible_keys()
        newly = vis - self._visible
        self._visible = vis
        if "events" in vis:
            self._set_unseen(0)
        for k, b in self.btns.items():
            open_ = k in self.workspace.open_keys()
            if b.property("open") != open_:
                b.setProperty("open", open_)
                b.style().unpolish(b); b.style().polish(b)
        self._sync_streams()
        if newly:
            # those pages were not drawn while hidden: bring them up to date now
            self._pose_dirty = True
            self._scan_dirty = self._scan_msg is not None
            self._render()

    def closeEvent(self, ev) -> None:
        self._sync_state()
        save_settings({"layout": self.workspace.save_layout()})   # for robots with none of their own
        super().closeEvent(ev)

    # ---- per-robot data ----

    def _data_profile(self):
        return next((p for p in self.profiles.profiles if p.id == self._data_robot), None)

    def _sync_state(self) -> None:
        """Note how the app is, in the open robot's state."""
        cur = self.p_mission.cur
        now = {"mission": cur.id if cur else "", "run": self.p_mission.run_state(),
               "nav": self.p_robot.choice()}
        if self._layout_robot == self._data_robot:
            now["layout"] = self.workspace.save_layout()
        self.robot_state.update(**now)

    def _restore_robot_state(self) -> None:
        """Bring the screens back as they were left. Only that: no mission,
        goal or program is started from here."""
        st = self.robot_state
        nav = st.get("nav")
        if isinstance(nav, dict):
            self.p_robot.set_choice(str(nav.get("mode", "map")), str(nav.get("map", "")))
        if st.get("mission"):
            self.p_mission.open_mission(str(st.get("mission")))
        run = st.get("run")
        text = self.p_mission.show_last_run(run) if isinstance(run, dict) else None
        if text:
            self._log("info", "Mission", f"{text} when the app closed. Not restarted")
        self._update_last_pose_buttons()
        self._show_data_robot()

    def _show_data_robot(self) -> None:
        prof = self._data_profile()
        name = prof.name if prof else "no robot"
        seen = self.robot_state.get("last_connected")
        when = time.strftime(" · last connected %Y-%m-%d %H:%M", time.localtime(seen)) if seen else ""
        self.p_data.set_robot(
            f"<b>{name}</b> · missions: {len(self.missions.missions)} · "
            f"places: {len(self.place_store.places)}{when}", str(robot_dir(self._data_robot)))

    def _switch_robot(self, profile: RobotProfile) -> None:
        """Open another robot's data. A running mission stops first: it would
        carry on sending that robot's waypoints to the next one."""
        if profile.id == self._data_robot:
            return
        self.p_mission.stop_mission()
        self._sync_state()
        self._data_robot = profile.id
        folder = robot_dir(profile.id)
        self.missions.use(folder / "missions.json")
        self.place_store.use(folder / "places.json")
        self.events.use(folder / "events.csv")
        self.robot_state = RobotState(folder / "state.json")
        self._set_unseen(0)
        self._nav_map = ""
        self._moved = False
        self.p_mission.set_map("")
        self.p_mission.reload()
        self.p_nav.places.map_name = ""
        self.p_nav.places.refresh()
        self._restore_robot_state()

    def _bring_layout(self, profile: RobotProfile) -> None:
        """A robot's layout comes with connecting to it, not with selecting it
        in the list: that would pull the Connection page away mid-click."""
        if profile.id == self._layout_robot:
            return
        self._layout_robot = profile.id
        saved = self.robot_state.get("layout")
        if saved and self.workspace.restore_layout(saved):
            self._on_visible_changed()

    # ---- back up, share, import ----

    def _map_refs(self) -> set:
        refs = {m.map for m in self.missions.missions} | {p.map for p in self.place_store.places}
        return refs - {""}

    def _ask_export(self, kind: str) -> None:
        prof = self._data_profile()
        if prof is None:
            return
        names = self.library.names()
        refs = self._map_refs()
        dlg = ExportDialog(kind, prof.name, {
            "missions": len(self.missions.missions), "places": len(self.place_store.places),
            "layouts": len(self.settings.get("layouts", {})), "events": len(self.events.events),
        }, names, refs, refs - set(names), self)
        if dlg.exec() != QDialog.Accepted:
            return
        stem = "".join(c if c.isalnum() or c in "-_" else "_" for c in prof.name)
        path, _ = QFileDialog.getSaveFileName(
            self, "Save as", f"{stem}_{time.strftime('%Y%m%d')}{SUFFIX}", f"RCS pack (*{SUFFIX})")
        if path:
            self.export_pack_to(path, dlg.items())

    def export_pack_to(self, path: str, items: dict) -> bool:
        self._sync_state()
        prof = self._data_profile()
        try:
            out = export_pack(
                path, robot_name=prof.name if prof else "",
                profile=prof if items.get("profile") else None,
                missions=self.missions.missions if items.get("missions") else None,
                places=self.place_store.places if items.get("places") else None,
                library=self.library, maps=tuple(items.get("maps", ())),
                layouts=self.settings.get("layouts") if items.get("layouts") else None,
                state=self.robot_state.data if items.get("state") else None,
                events_path=self.events.path if items.get("events") else None)
        except (OSError, ValueError) as e:
            self.p_data.show_result(False, f"cannot write {path}: {e}")
            return False
        self.p_data.show_result(True, f"saved to {out}")
        self._log("info", "Data", f"exported to {out}")
        return True

    def _ask_import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Import", "", f"RCS pack (*{SUFFIX})")
        if not path:
            return
        try:
            pack = read_pack(path)
        except (OSError, ValueError) as e:
            self.p_data.show_result(False, f"cannot read {path}: {e}")
            return
        prof = self._data_profile()
        dlg = ImportDialog(pack, prof.name if prof else "this robot", self)
        if dlg.exec() == QDialog.Accepted:
            self.import_pack_from(path, dlg.to_new.isChecked(), dlg.want())

    def import_pack_from(self, path: str, new_robot: bool, want: set):
        """Returns the ImportResult, or None if nothing was imported."""
        if not new_robot and self.p_mission._exec_idx >= 0:
            self.p_data.show_result(False, "stop the mission before importing into this robot")
            return None
        try:
            pack = read_pack(path)
            if new_robot:
                if pack.profile is None:
                    raise ValueError("this file has no robot profile")
                res = import_as_new_robot(pack, want, self.library, self.profiles)
            else:
                res = import_into(pack, want, self.library, self.missions, self.place_store)
        except (OSError, ValueError) as e:
            self.p_data.show_result(False, f"cannot import {path}: {e}")
            return None
        if "layouts" in want:
            mine = self.settings.setdefault("layouts", {})
            merge_layouts(mine, pack, res)
            save_settings({"layouts": mine})
        self._after_import(res)
        return res

    def _after_import(self, res: ImportResult) -> None:
        if res.robot is not None:
            self.p_conn.refresh()
        else:
            self.p_mission.reload()
            self.p_nav.places.refresh()
        self.p_data.refresh_library()
        self._show_data_robot()
        missing = sorted(res.map_refs - set(self.library.names()) - set(self.p_data.robot_maps()))
        text = res.text()
        if missing:
            text += f". Still needed: map {', '.join(missing)}, on neither this PC nor the robot"
        self.p_data.show_result(not missing, text)
        self._log("warn" if missing else "info", "Data", f"imported: {text}")

    def _sync_streams(self) -> None:
        self.ros.set_stream_active("image", "cam" in self._visible)
        self.ros.map3d_range = self.p_3d.range.value() if hasattr(self.p_3d, "range") else 4.0
        on = self.p_3d.wants_stream("3d" in self._visible)
        if on and not self.ros._streams_on.get("map3d"):
            self.p_3d._rec_since = time.monotonic()
        if self.ros.profile:
            t = self.ros.profile.topics
            self.p_3d.source_hint = (t.camera_depth if self.ros.map3d_source == "rgbd" else t.camera_cloud)
        self.ros.set_stream_active("map3d", on)

    def _map_views(self):
        """MapViews on the pages that are on screen (only those get redrawn)."""
        views = {"map": self.p_map.view, "nav": self.p_nav.view, "mission": self.p_mission.view}
        return tuple(v for k, v in views.items() if k in self._visible)

    # ---- wire signals ----

    def _wire(self) -> None:
        # connection panel ↔ ros client
        self.p_conn.connect_requested.connect(self._on_connect)
        self.p_conn.disconnect_requested.connect(self.ros.disconnect_robot)
        self.p_conn.profile_changed.connect(self._on_profile_changed)

        self.ros.connected.connect(self._on_ros_connected)
        self.ros.disconnected.connect(self._on_ros_disconnected)
        self.ros.connection_error.connect(self._on_ros_error)

        # ros → panels
        self.ros.odom_received.connect(self._on_odom)
        self.ros.scan_received.connect(self._on_scan)
        self.ros.map_received.connect(self._on_map)
        self.ros.global_costmap_received.connect(self._on_global_costmap)
        self.ros.local_costmap_received.connect(self._on_local_costmap)
        self.ros.plan_received.connect(self._on_plan)
        self.ros.image_received.connect(self.p_cam.set_frame)
        self.ros.image_received.connect(
            lambda _: self.p_cam.set_rate(self.ros.topic_stats["image"].hz
                                          if "image" in self.ros.topic_stats else 0.0))
        self.ros.cloud_updated.connect(self.p_3d.on_cloud_stats)
        if hasattr(self.p_3d, "rec_btn"):
            self.p_3d.recording_changed.connect(lambda _on: self._sync_streams())
            self.p_3d.keep_bg.toggled.connect(lambda _on: self._sync_streams())
            self.p_3d.range.valueChanged.connect(lambda v: setattr(self.ros, "map3d_range", v))
            self.p_3d.source_changed.connect(
                lambda src: (self.ros.set_map3d_source(src), self._sync_streams()))
        self.ros.rosout_received.connect(self.p_logs.append_log)
        self.ros.tf_received.connect(self._on_tf)
        self.ros.nav_status.connect(self._on_nav_state)
        self.ros.nav_status.connect(self.p_nav.set_status)
        self.ros.estop_received.connect(self._on_estop_remote)
        self.ros.launcher_state.connect(self._on_launcher_events)
        self.p_mission.event.connect(lambda level, text: self._log(level, "Mission", text))
        self.p_teleop.event.connect(lambda level, text: self._log(level, "Teleop", text))

        self.p_robot.open_page.connect(lambda k: self._show(k))
        self.ros.launcher_state.connect(self.p_data.on_launcher_state)
        self.p_data.export_requested.connect(self._ask_export)
        self.p_data.import_requested.connect(self._ask_import)
        self.transfer.finished.connect(
            lambda ok, text: self._log("info" if ok else "warn", "Data", text))

        # teleop → ros
        self.p_teleop.cmd_vel.connect(self.ros.publish_cmd_vel)
        self.p_teleop.estop_changed.connect(self._set_estop)
        self.sidebar_estop.toggled.connect(self._set_estop)

        # map / nav / mission → ros
        self.p_map.goal_requested.connect(self._send_goal)
        self.p_map.cancel_requested.connect(self.ros.cancel_nav_goal)
        self.p_map.set_initial_pose.connect(self._set_initial_pose)
        self.p_nav.goal_requested.connect(self._send_goal)
        self.p_nav.cancel_requested.connect(self.ros.cancel_nav_goal)
        self.p_mission.goal_requested.connect(self._send_goal)
        self.p_mission.cancel_requested.connect(self.ros.cancel_nav_goal)
        self.p_mission.waypoints_changed.connect(self.p_map.view.set_waypoints)
        self.ros.nav_status.connect(self.p_mission.on_nav_status)
        # "Add waypoint" mode on the 2D Map page appends to the open mission
        self.p_map.view.waypoint_requested.connect(self.p_mission.add_waypoint_at)
        self.p_mission.waypoints_changed.connect(self.p_nav.view.set_waypoints)
        self.p_mission.waypoints_changed.connect(self.p_3d.set_waypoints)
        pl = self.p_nav.places
        pl.go_requested.connect(self._go_to_place)
        pl.add_to_mission.connect(lambda p: self.p_mission.add_waypoint_at(p.x, p.y, p.yaw, p.name))
        pl.changed.connect(self._draw_places)
        self._draw_places()
        self.p_map.last_pose_btn.clicked.connect(self._use_last_pose)
        self.p_robot.last_pose_btn.clicked.connect(self._use_last_pose)
        # the 3D Map can set the same things as the 2D Map
        self.p_3d.goal_requested.connect(self._send_goal)
        self.p_3d.pose_requested.connect(self._set_initial_pose)
        self.p_3d.waypoint_requested.connect(self.p_mission.add_waypoint_at)
        self.p_mission._emit_wps()      # the open mission was loaded before these connections

    def _on_connect(self, profile: RobotProfile) -> None:
        self.profiles.set_active(profile.id)
        self._switch_robot(profile)
        self._bring_layout(profile)
        self.ros.set_profile(profile)
        self.ros.connect_robot()
        self.sb_robot.setText(f"{profile.name} ({profile.ws_url})")
        self._apply_profile_to_panels(profile)

    def _on_profile_changed(self, profile: RobotProfile) -> None:
        self._apply_profile_to_panels(profile)
        if not self.ros._want_connected:    # connected: the pages stay on that robot
            self._switch_robot(profile)
        self._show_data_robot()

    def _apply_profile_to_panels(self, profile: RobotProfile) -> None:
        self.p_teleop.apply_limits(profile.limits.max_linear, profile.limits.max_angular)
        self.p_map.view.set_robot_footprint(
            profile.limits.footprint_half_length, profile.limits.footprint_half_width)
        self.p_nav.view.set_robot_footprint(
            profile.limits.footprint_half_length, profile.limits.footprint_half_width)
        self.p_mission.view.set_robot_footprint(
            profile.limits.footprint_half_length, profile.limits.footprint_half_width)
        self.p_3d.set_robot_footprint(
            profile.limits.footprint_half_length, profile.limits.footprint_half_width)

    def _on_ros_connected(self) -> None:
        self.sb_conn.setText("● connected"); self.sb_conn.setStyleSheet("color:#39d98a;")
        self._linked = True
        self._last_conn_error = None
        self._log("info", "Connection", f"connected to {self.ros.profile.ws_url if self.ros.profile else 'robot'}")
        self.p_conn.set_connected(True, self.ros.profile.ws_url if self.ros.profile else "")
        self.p_status.set_connected(True)
        self.p_robot.set_connected(True)
        self.p_data.set_connected(True)
        self.robot_state.update(last_connected=time.time())
        self._show_data_robot()

    def _on_ros_disconnected(self) -> None:
        self.sb_conn.setText("● offline"); self.sb_conn.setStyleSheet("color:#ff6a6a;")
        if getattr(self, "_linked", False):
            self._linked = False
            self._sensor_ok.clear()
            if self.ros._want_connected:
                self._log("warn", "Connection", "connection lost, reconnecting")
            else:
                self._log("info", "Connection", "disconnected")
        self.p_conn.set_connected(False)
        self.p_status.set_connected(False)
        self.p_robot.set_connected(False)
        self.p_data.set_connected(False)
        self._launcher = {}
        active = self.profiles.active()
        if active and not self.ros._want_connected:     # another robot was selected meanwhile
            self._switch_robot(active)

    def _on_ros_error(self, msg: str) -> None:
        self.p_conn.set_connected(False, msg)
        if msg != getattr(self, "_last_conn_error", None):      # retried every few seconds
            self._last_conn_error = msg
            self._log("warn", "Connection", msg)

    # ---- events ----

    def _log(self, level: str, source: str, text: str) -> None:
        self.events.add(level, source, text)
        self.statusBar().showMessage(f"{source}: {text}", 6000)
        if level != "info":
            if QApplication.platformName() != "offscreen":
                self.sounds.play("alarm" if level == "error" else "warn")
            if "events" not in self._visible:
                self._set_unseen(self._unseen + 1)

    _unseen = 0

    def _set_unseen(self, n: int) -> None:
        """Warnings and errors since the Events page was last on screen."""
        self._unseen = n
        self.btns["events"].setText(f"Events  ({n})" if n else "Events")

    def _check_sensors(self) -> None:
        """A sensor that was arriving and stopped is worth an event; one that
        never arrived is not (bringup may simply not be running)."""
        self._update_camera_status()
        if not self.ros.is_connected():
            return
        for key, label in (("odom", "odometry"), ("scan", "lidar")):
            st = self.ros.topic_stats.get(key)
            if st is None:
                continue
            ok = st.fresh(3.0)
            was = self._sensor_ok.get(key)
            if was is not None and ok != was:
                self._log("info" if ok else "warn", "Sensors",
                          f"{label} is back" if ok else f"no {label} for 3 s")
            self._sensor_ok[key] = ok

    _launcher: dict = {}    # latest state from the robot's launcher
    _usb_problem = None

    def _update_camera_status(self) -> None:
        """Say why there is no picture, on the pages that would show one."""
        connected = self.ros.is_connected()
        st = self._launcher if connected else {}
        usb = usb_problem(st)
        self.p_robot.set_camera_problem(usb)
        if usb != self._usb_problem:
            self._usb_problem = usb
            if usb:
                self._log("warn", "Sensors", usb)
        stats = self.ros.topic_stats
        frames = any(k in stats and stats[k].fresh(3.0) for k in ("image", "depth", "cloud"))
        prof = self.ros.profile
        topic = prof.topics.camera_image if prof else "the camera topic"
        text = camera_problem(connected, st, frames, topic)
        self.p_cam.set_problem(text)
        self.p_3d.camera_problem = text or ""

    def _on_launcher_events(self, st: dict) -> None:
        self._launcher = st
        nav = st.get("stacks", {}).get("nav", {})
        args = nav.get("args") or {}
        up = nav.get("state") in ("starting", "running") and args.get("mode") == "map"
        self._nav_map = str(args.get("map", "")) if up else ""
        self.p_nav.places.set_map(self._nav_map)
        self.p_mission.set_map(self._nav_map)
        self._update_last_pose_buttons()
        names = {"bringup": "Robot base", "nav": "Navigation"}
        seen = self.__dict__.setdefault("_stack_state", {})
        for key, label in names.items():
            s = st.get("stacks", {}).get(key, {})
            state = s.get("state")
            prev = seen.get(key)
            seen[key] = state
            if prev is None or state == prev or state is None:
                continue
            if state == "failed":
                self._log("error", "Robot", f"{label} failed: {s.get('detail') or 'see its log'}")
            elif state in ("running", "stopped"):
                self._log("info", "Robot", f"{label} {state}")
        r = st.get("result") or {}
        if r.get("id") and r["id"] != getattr(self, "_last_result", None):
            first = getattr(self, "_last_result", None) is None
            self._last_result = r["id"]
            if not first and r.get("msg"):
                self._log("info" if r.get("ok") else "warn", "Robot", str(r["msg"]))

    def _set_initial_pose(self, x: float, y: float, yaw: float) -> None:
        self._log("info", "Navigation", f"initial pose set at ({x:.2f}, {y:.2f})")
        self.ros.publish_initial_pose(x, y, yaw)
        self._store_pose(x, y, yaw)

    # ---- places and the last position ----

    def _draw_places(self) -> None:
        pts = [(p.x, p.y, p.name) for p in self.place_store.on_map(self.p_nav.places.map_name)]
        self.p_map.view.set_places(pts)
        self.p_nav.view.set_places(pts)

    def _go_to_place(self, p) -> None:
        self._log("info", "Navigation", f"going to {p.name}")
        self._send_goal(p.x, p.y, p.yaw)

    _nav_map = ""           # saved map Nav2 runs on; "" = none, or SLAM

    def last_pose(self):
        """(x, y, yaw) where the robot last was on the current saved map."""
        p = self.robot_state.get("last_pose", {}).get(self._nav_map)
        return tuple(p[:3]) if p else None

    def _store_pose(self, x: float, y: float, yaw: float) -> None:
        if self._nav_map:
            poses = dict(self.robot_state.get("last_pose", {}))
            poses[self._nav_map] = [round(x, 3), round(y, 3), round(yaw, 3)]
            self.robot_state.update(last_pose=poses)
            self._pose_saved_t = time.monotonic()
            self._moved = False

    _moved = False
    _pose_saved_t = 0.0

    def _track_pose(self, pose) -> None:
        """Keep the last position while driving and once stopped. Not before
        the robot has moved: right after a start the pose is just the launch
        default, and storing it would lose the real one."""
        if not self._nav_map or not self.ros.profile:
            return
        t = self.ros.profile.topics
        if self.tf.pose_xyyaw(t.map_frame, t.base_frame) is None:
            return
        driving = abs(self._odom_v) > 0.02
        self._moved = self._moved or driving
        if self._moved and (not driving or time.monotonic() - self._pose_saved_t > 5.0):
            self._store_pose(*pose)

    def _use_last_pose(self) -> None:
        pose = self.last_pose()
        if pose:
            self._set_initial_pose(*pose)

    def _update_last_pose_buttons(self) -> None:
        pose = self.last_pose()
        for b in (self.p_map.last_pose_btn, self.p_robot.last_pose_btn):
            b.setVisible(pose is not None)
            if pose:
                b.setText(f"Use last position ({pose[0]:.1f}, {pose[1]:.1f})")

    # ---- ros message handlers (on GUI thread via queued Qt signals) ----

    def _on_odom(self, m: dict) -> None:
        p = m.get("pose", {}).get("pose", {})
        pos = p.get("position", {})
        self._odom_x = float(pos.get("x", 0.0))
        self._odom_y = float(pos.get("y", 0.0))
        self._odom_yaw = quat_to_yaw(p.get("orientation", {}))
        self._odom_v = float(m.get("twist", {}).get("twist", {}).get("linear", {}).get("x", 0.0))
        self._pose_dirty = True

    def _on_tf(self, _m=None) -> None:
        self._pose_dirty = True        # RosClient already put it in self.tf

    def _update_robot_pose(self) -> None:
        """Recompute the robot and laser pose from TF (or odom)."""
        if not self.ros.profile:
            return
        p = self.ros.profile.topics
        pose = self.tf.pose_xyyaw(p.map_frame, p.base_frame)
        if pose is None:
            pose = self.tf.pose_xyyaw(p.odom_frame, p.base_frame)
        if pose is None:
            pose = (self._odom_x, self._odom_y, self._odom_yaw)
        self._robot_x, self._robot_y, self._robot_yaw = pose

        laser_pose = self.tf.pose_xyyaw(p.map_frame, p.laser_frame)
        if laser_pose is None:
            laser_pose = self.tf.pose_xyyaw(p.odom_frame, p.laser_frame)
        if laser_pose is None:
            # robot URDF usually mounts the lidar forward of base_link
            off = self.ros.profile.limits.laser_offset_x
            laser_pose = (
                pose[0] + off * math.cos(pose[2]),
                pose[1] + off * math.sin(pose[2]),
                pose[2],
            )
        self._laser_x, self._laser_y, self._laser_yaw = laser_pose

    def _render(self) -> None:
        """Draw what changed since the last tick, on the pages on screen only."""
        views = self._map_views()
        if self._pose_dirty:
            self._pose_dirty = False
            self._update_robot_pose()
            pose = (self._robot_x, self._robot_y, self._robot_yaw)
            for v in views:
                v.set_robot_pose(*pose)
            self.p_mission.update_robot_pose(*pose, draw="mission" in self._visible)
            self.p_3d.update_robot_pose(*pose)
            self.p_nav.places.set_pose(*pose)
            self._track_pose(pose)
            self.sb_pose.setText(f"pose: x={pose[0]:+.2f} y={pose[1]:+.2f} yaw={pose[2]:+.2f}")
            now = time.monotonic()
            if now - getattr(self, "_progress_t", 0.0) > 0.2:      # 5 Hz is plenty
                self._progress_t = now
                self._update_progress()
        for kind, (ver, msg, pose) in list(self._layers.items()):
            targets = list(views)
            if "3d" in self._visible:
                targets.append(self.p_3d)
            for v in targets:
                key = (id(v), kind)
                if self._applied.get(key) == ver:
                    continue
                if kind != "map" and not v.layer_on(kind):
                    continue            # overlay off: applied when it is switched on
                self._applied[key] = ver
                if kind == "map":
                    v.set_occupancy(msg)
                elif kind == "gcost":
                    v.set_global_costmap(msg, pose)
                else:
                    v.set_local_costmap(msg, pose)
        if self._scan_dirty and self._scan_msg is not None:
            self._scan_dirty = False
            for v in views:
                if v is not self.p_mission.view:
                    v.set_scan(self._scan_msg, self._laser_x, self._laser_y, self._laser_yaw)
            if "3d" in self._visible:
                self.p_3d.set_scan(self._scan_msg, self._laser_x, self._laser_y, self._laser_yaw)
            if "teleop" in self._visible:
                self._update_clearance()

    def _update_clearance(self) -> None:
        prof = self.ros.profile
        xy = self._scan_msg.get("_xy")
        if prof is None or xy is None:
            return
        t, lim = prof.topics, prof.limits
        laser = self.tf.pose_xyyaw(t.base_frame, t.laser_frame) or (lim.laser_offset_x, 0.0, 0.0)
        self.p_teleop.set_clearance(compute_clearance(
            xy[0], xy[1], laser, lim.footprint_half_length, lim.footprint_half_width))

    def _on_scan(self, m: dict) -> None:
        self._scan_msg = m
        self._scan_dirty = True

    def _set_layer(self, kind: str, msg: dict, pose=(0.0, 0.0, 0.0)) -> None:
        self._layer_ver += 1
        self._layers[kind] = (self._layer_ver, msg, pose)

    def _on_map(self, m: dict) -> None:
        self._set_layer("map", m)

    def _send_goal(self, x: float, y: float, yaw: float) -> None:
        self._goal = (x, y, yaw)
        self._log("info", "Navigation", f"goal sent to ({x:.2f}, {y:.2f})")
        for v in (self.p_map.view, self.p_nav.view, self.p_mission.view, self.p_3d):
            v.set_goal(x, y, yaw)
        self.ros.send_nav_goal(x, y, yaw)

    def _frame_in_map(self, m: dict):
        """Pose of a message's header frame in the map frame, or None if TF
        doesn't know it yet (the local costmap usually lives in odom)."""
        frame = (m.get("header", {}).get("frame_id") or "").lstrip("/")
        map_frame = self.ros.profile.topics.map_frame if self.ros.profile else "map"
        if not frame or frame == map_frame:
            return (0.0, 0.0, 0.0)
        return self.tf.pose_xyyaw(map_frame, frame)

    def _on_global_costmap(self, m: dict) -> None:
        pose = self._frame_in_map(m)
        if pose is None:
            return
        self._set_layer("gcost", m, pose)

    def _on_local_costmap(self, m: dict) -> None:
        pose = self._frame_in_map(m)
        if pose is None:
            return
        self._set_layer("lcost", m, pose)

    def _on_plan(self, m: dict) -> None:
        self.p_map.view.set_path(m)
        self.p_nav.view.set_path(m)
        self.p_mission.view.set_path(m)
        self.p_3d.set_path(m)
        poses = m.get("poses", [])
        self._plan_xy = [(p["pose"]["position"]["x"], p["pose"]["position"]["y"]) for p in poses]
        self._update_progress()

    # Distance/time remaining are computed here from /plan and the robot pose:
    # rosbridge (Humble) cannot subscribe to the NavigateToPose feedback topic.
    def _on_nav_state(self, state: str) -> None:
        if state != self._nav_state:
            ev = {"succeeded": ("info", "goal reached"), "canceled": ("info", "goal canceled"),
                  "aborted": ("warn", "Nav2 aborted the goal"),
                  "no_response": ("warn", "no response from Nav2")}.get(state)
            # cancel_nav_goal reports "canceled" even with no goal running
            if ev and (state != "canceled" or self._nav_state in ("active", "pending")):
                self._log(ev[0], "Navigation", ev[1])
        self._nav_state = state
        if state in ("succeeded", "canceled", "aborted"):
            self._goal = None
            for v in (self.p_map.view, self.p_nav.view, self.p_mission.view, self.p_3d):
                v.set_goal(None, None)
        if state not in ("active", "pending"):
            self._plan_xy = []
            if state == "succeeded":
                self.p_nav.set_feedback({"distance_remaining": 0.0,
                                         "estimated_time_remaining": {"sec": 0}})
            else:
                self.p_nav.clear_feedback()

    def _update_progress(self) -> None:
        if getattr(self, "_nav_state", "") not in ("active", "pending") or not getattr(self, "_plan_xy", None):
            return
        pts = self._plan_xy
        rx, ry = self._robot_x, self._robot_y
        i = min(range(len(pts)), key=lambda k: (pts[k][0] - rx) ** 2 + (pts[k][1] - ry) ** 2)
        d = math.hypot(pts[i][0] - rx, pts[i][1] - ry)
        d += sum(math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1])
                 for k in range(i, len(pts) - 1))
        v = abs(self._odom_v) if abs(self._odom_v) > 0.05 else (
            self.ros.profile.limits.max_linear if self.ros.profile else 0.4)
        self.p_nav.set_feedback({"distance_remaining": d,
                                 "estimated_time_remaining": {"sec": d / v}})

    # ---- E-stop (software, on the client) ----

    def _set_estop(self, on: bool, publish: bool = True) -> None:
        """Software E-stop. Nothing on the robot consumes /amr/estop, so the
        stop is enforced client-side: cancel the Nav2 goal, stop any mission,
        and stream zero velocity (teleop panel does that while engaged).
        This is not a substitute for the hardware E-stop."""
        if on == getattr(self, "_estop_on", False):
            return
        self._estop_on = on
        self.sidebar_estop.blockSignals(True)
        self.sidebar_estop.setChecked(on)
        self.sidebar_estop.blockSignals(False)
        self.sidebar_estop.setText("● E-STOP ON\nclick to release" if on else "● E-STOP")
        self.p_teleop.set_estop_state(on)
        self.p_status.set_estop(on)
        self._log("error" if on else "info", "E-STOP",
                  ("engaged" if on else "released") + ("" if publish else " by another console"))
        if on:
            self.sb_estop.setText("● estop: ENGAGED")
            self.sb_estop.setStyleSheet("color:#ff6a6a; font-weight:700;")
            self.p_mission.stop_mission()
            self.ros.cancel_nav_goal()
            self.ros.publish_cmd_vel(0.0, 0.0)
        else:
            self.sb_estop.setText("● estop: clear")
            self.sb_estop.setStyleSheet("color:#39d98a;")
        if publish:
            self.ros.publish_estop(on)

    def _on_estop_remote(self, on: bool) -> None:
        # /amr/estop from another client (e.g. amr_control_center) — mirror it
        self._set_estop(on, publish=False)
