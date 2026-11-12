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
from ..core.battery import alarm, live
from ..core.battery_history import BatteryLogs
from ..core.camera_status import camera_problem, depth_problem, usb_problem
from ..core.clearance import compute_clearance
from ..core.events import EventLog
from ..core.fleet import FleetMonitor
from ..core.map_transfer import MapTransfer
from ..core.maps import MapLibrary
from ..core.mission import MissionStore
from ..core.pack import (
    SUFFIX, ImportResult, export_pack, import_as_new_robot, import_into, install_model, merge_layouts,
    read_pack,
)
from ..core.places import PlaceStore
from ..core.profile import ProfileStore, RobotProfile, load_settings, save_settings
from ..core import diagnostics
from ..core.alerts import Alerts
from ..core.charging import ChargeConfig, ChargeKeeper
from ..core import gateway as gwc
from ..core import versions
from .. import i18n
from ..core import permissions
from ..core import robot_model
from ..core import zones as zn
from ..core.robot_data import RobotState, migrate_legacy, robot_dir
from ..core.ros_client import RosClient, quat_to_yaw
from ..core.runs import RunLog
from ..core.schedule import Scheduler, ScheduleStore
from ..core.sound import Sounds
from .panels.battery import BatteryPanel
from .panels.camera import CameraPanel
from .panels.connection import ConnectionPanel
from .panels.data import DataPanel, ExportDialog, ImportDialog
from .panels.events import EventsPanel
from .panels.fleet import FleetPanel
from .panels.logs import LogsPanel
from .panels.map2d import Map2DPanel
from .panels.mission import MissionPanel
from .panels.navigation import NavigationPanel
from .panels.status import StatusPanel
from .panels.robot import RobotPanel
from .panels.schedule import SchedulePanel
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
    ("battery", "Battery"),
    ("schedule", "Schedule"),
    ("fleet", "Fleet"),
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
        self.runs = RunLog(folder / "runs.json")
        self.schedules = ScheduleStore(folder / "schedules.json")
        self.fleet = FleetMonitor(self)
        self.library = MapLibrary()
        self.settings = load_settings()
        self.sounds = Sounds()
        self.sounds.enabled = bool(self.settings.get("sound", True))
        self._alert_results: list = []      # (ok, text) from the sending thread, logged on the GUI thread
        self.alerts = Alerts(self.settings.get("alerts") or {}, report=lambda ok, t: self._alert_results.append((ok, t)))
        self.ros = RosClient(profile=self.profiles.active())
        self.tf = self.ros.tf            # filled on the network thread
        self.battery_logs = BatteryLogs(self.ros, folder / "battery_log", self)

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
        self._odom_seen = False         # an odom message has come (else _odom_* are just zeros)
        self._pose_frame = ""           # frame of the pose: map, odom, or "" (none known yet)
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
        self._unclean = diagnostics.start_session()     # the last run never closed: crash, kill, power cut
        if self._unclean:
            started = time.strftime("%Y-%m-%d %H:%M", time.localtime(self._unclean.get("started", 0)))
            self._log("warn", "App", f"RCS did not close properly last time (started {started}). If something "
                      "went wrong, save a diagnostic bundle on the Status page.")
        self.charger = ChargeKeeper(ChargeConfig.from_dict(self.robot_state.get("charging")))
        self.p_battery.charge_changed.connect(self._set_charge)
        self.p_nav.places.changed.connect(self._show_charge)    # places added or removed
        self._show_charge()
        self.p_status.health_source = self.health_context
        self.p_status.bundle_requested.connect(self._ask_bundle)
        if self.profiles.active() is not None:
            self._apply_profile_to_panels(self.profiles.active())
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
        self.scheduler = Scheduler(self.schedules, self._start_scheduled, parent=self)
        self.scheduler.fired.connect(self._on_schedule_fired)
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
        lrow = QHBoxLayout()
        lrow.setContentsMargins(0, 0, 8, 0)
        lrow.setSpacing(4)
        lrow.addWidget(self.layout_btn, 1)
        self.help_btn = QPushButton("?")
        self.help_btn.setObjectName("HelpButton")
        self.help_btn.setFixedSize(30, 30)
        self.help_btn.setCursor(Qt.PointingHandCursor)
        self.help_btn.setToolTip("What this page does (Shift+F1)")
        self.help_btn.setStyleSheet("QPushButton#HelpButton { border-radius: 15px; font-weight: 700; padding: 0; "
                                    "background: #1f2533; border: 1px solid #3b82f6; color: #8ab4ff; }"
                                    "QPushButton#HelpButton:hover { background: #24304a; color: #ffffff; }")
        self.help_btn.clicked.connect(self.show_help)
        lrow.addWidget(self.help_btn)
        sv.addLayout(lrow)

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
        self.p_battery = BatteryPanel()
        self.p_battery.set_logs(self.battery_logs)
        self.p_mission.runs = self.runs
        self.p_mission.soc = self._soc_now
        self.p_schedule = SchedulePanel(self.schedules, self.runs, self.missions)
        self.p_schedule.events = self.events
        self.p_fleet = FleetPanel(self.profiles, self.fleet, set(self.settings.get("fleet_watch", [])),
                                  model_of=self.robot_model_of)

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
            "battery": self.p_battery,
            "schedule": self.p_schedule,
            "fleet": self.p_fleet,
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
        self.p_events.alerts_btn.clicked.connect(self._ask_alerts)
        self.p_conn.auto_ck.setChecked(bool(self.settings.get("auto_connect", False)))
        self.p_conn.auto_ck.toggled.connect(lambda on: save_settings({"auto_connect": on}))
        self.p_conn.update_ck.setChecked(bool(self.settings.get("check_updates", True)))
        self.p_conn.update_ck.toggled.connect(self._set_check_updates)
        lang = self.settings.get("lang", "en")
        self.p_conn.lang.setCurrentIndex(max(0, self.p_conn.lang.findData(lang)))
        self.p_conn.lang.currentIndexChanged.connect(self._set_language)
        self._shortcuts()

        # status bar
        sb = QStatusBar()
        self.setStatusBar(sb)
        self.sb_conn = QLabel("● offline"); self.sb_conn.setStyleSheet("color:#ff6a6a;")
        self.sb_robot = QLabel("—")
        self.sb_pose = QLabel("pose: —")
        self.sb_estop = QLabel("● estop: clear"); self.sb_estop.setStyleSheet("color:#39d98a;")
        self.sb_battery = QLabel("🔋 —")
        sb.addPermanentWidget(self.sb_battery)
        sb.addPermanentWidget(QLabel("   |   "))
        self.sb_update = QLabel("")
        self.sb_update.setOpenExternalLinks(True)
        self.sb_update.setTextFormat(Qt.RichText)
        sb.addPermanentWidget(self.sb_update)
        sb.addPermanentWidget(self.sb_robot)
        self.sb_user = QLabel("")
        self.sb_user.setToolTip("Signed in to the robot's gateway")
        sb.addPermanentWidget(self.sb_user)
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

    def show_help(self, key: str = "") -> "QDialog":
        """The guide of the page on screen (or key), in the app's language."""
        from PySide6.QtWidgets import QDialogButtonBox, QTextBrowser
        from .. import help as helptext
        key = key or self.active_page() or "conn"
        title, body = helptext.page_help(key, i18n.current)
        dlg = QDialog(self)
        dlg.setWindowTitle(f"? {title}")
        v = QVBoxLayout(dlg)
        view = QTextBrowser()
        view.setOpenExternalLinks(True)
        view.setHtml(body)
        v.addWidget(view, 1)
        box = QDialogButtonBox(QDialogButtonBox.Close)
        box.rejected.connect(dlg.reject)
        v.addWidget(box)
        # as tall as the text (a short page gets a short window), within the screen
        dlg.resize(620, 500)
        dlg.layout().activate()
        doc = view.document()
        doc.setTextWidth(view.viewport().width())
        need = int(doc.size().height() + 0.999) + 2 * view.frameWidth() + 2
        room = (self.screen().availableGeometry().height() if self.screen() else 900) - 120
        dlg.resize(620, max(260, min(dlg.height() - view.height() + need, room)))
        self._help_dialog = dlg            # kept: tests read it
        dlg.setModal(False)
        dlg.show()
        return dlg

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
        key("Shift+F1", lambda: self.show_help())
        # F12 only engages, from any window of the app: a stuck key or a second
        # press must never release the E-STOP
        key("F12", lambda: self._set_estop(True), Qt.ApplicationShortcut)
        key("Esc", self.on_escape)

    def on_escape(self) -> None:
        """Esc: drop what the mouse is doing on a map, else stop the current goal."""
        for v in (self.p_map.view, self.p_nav.view, self.p_mission.view):
            if v.zone_corners():                # a zone being drawn
                v.cancel_zone()
                return
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

    UPDATE_EVERY_S = 86400

    def check_updates(self, force: bool = False) -> None:
        """Once a day, off the GUI thread: is there a newer release on GitHub?"""
        offer = self.settings.get("update_offer")
        if offer and versions.parse(offer.get("tag", "")) > versions.parse(__version__):
            self._show_update(offer)
        if not force and (not self.settings.get("check_updates", True)
                          or time.time() - float(self.settings.get("update_checked", 0)) < self.UPDATE_EVERY_S):
            return

        def done(r, e):
            if e is not None:
                return                  # offline: try again tomorrow
            stamp = {"update_checked": time.time()}
            if r:
                stamp["update_offer"] = {"tag": r.get("tag_name", ""), "url": r.get("html_url", versions.PAGE)}
                self._show_update(stamp["update_offer"])
            self.settings.update(stamp)
            save_settings(stamp)
        from .panels.account import run_bg
        run_bg(versions.check, done)

    def _set_language(self, _i: int) -> None:
        code = self.p_conn.lang.currentData()
        save_settings({"lang": code})
        self.settings["lang"] = code
        self.p_conn.lang_note.setText("Restart RCS to change the language." if code != i18n.current else "")

    def _set_check_updates(self, on: bool) -> None:
        self.settings["check_updates"] = on
        save_settings({"check_updates": on})
        if on:
            self.check_updates()

    def _show_update(self, offer: dict) -> None:
        self.sb_update.setText(f'<a href="{offer.get("url", versions.PAGE)}" style="color:#4fd1ff;">'
                               f'⬆ RCS {offer.get("tag", "")} is out</a>   |   ')
        self.sb_update.setToolTip("Open the download page")

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
        if "fleet" in newly:
            self.p_fleet.rebuild()
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
        diagnostics.end_session()
        self._sync_state()
        self.fleet.stop()
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
        self.charger = ChargeKeeper(ChargeConfig.from_dict(self.robot_state.get("charging")))
        self.runs.use(folder / "runs.json")
        self.schedules.use(folder / "schedules.json")
        self.scheduler.reset()
        self.p_schedule.refresh()
        self.battery_logs.use(folder / "battery_log")
        self.p_battery._rows.clear()
        self.p_battery._fill_boots([])
        self._robot_stops = None
        self._set_unseen(0)
        self._nav_map = ""
        self._moved = False
        self.p_mission.set_map("")
        self.p_mission.reload()
        self.p_nav.places.map_name = ""
        self.p_nav.places.refresh()
        self._show_charge()
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
        imported = self._imported_model(prof)
        dlg = ExportDialog(kind, prof.name, {
            "missions": len(self.missions.missions), "places": len(self.place_store.places),
            "layouts": len(self.settings.get("layouts", {})), "events": len(self.events.events),
            "model": (imported.name or "model") if imported is not None else "",
        }, names, refs, refs - set(names), self)
        if dlg.exec() != QDialog.Accepted:
            return
        stem = "".join(c if c.isalnum() or c in "-_" else "_" for c in prof.name)
        path, _ = QFileDialog.getSaveFileName(
            self, "Save as", f"{stem}_{time.strftime('%Y%m%d')}{SUFFIX}", f"RCS pack (*{SUFFIX})")
        if path:
            self.export_pack_to(path, dlg.items())

    def _imported_model(self, prof):
        return robot_model.load(robot_dir(prof.id) / robot_model.MODEL_FILE) if prof else None

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
                events_path=self.events.path if items.get("events") else None,
                model=self._imported_model(prof) if items.get("model") else None)
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
                prof = self._data_profile()
                if "model" in want and prof is not None:
                    install_model(pack, prof, self.profiles, res)
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
        if res.model == "added" and res.robot is None:
            prof = self._data_profile()
            if prof is not None:
                self._on_profile_changed(prof)
                self.p_conn.refresh()
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
        self.p_conn.signed_in.connect(self._on_signed_in)
        self.ros.session.connect(self._on_session)
        self.ros.denied.connect(self._on_denied)
        self.ros.auth_problem.connect(self._on_auth_problem)

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
            self.p_3d.keep_bg.setChecked(bool(self.settings.get("map3d_every_page", True)))
            self.p_3d.keep_bg.toggled.connect(lambda on: (save_settings({"map3d_every_page": on}),
                                                          self._sync_streams()))
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
        self.ros.battery.connect(self._on_battery)
        self.ros.estop_state.connect(self._on_estop_state)
        self.p_teleop.cmd_vel.connect(self._on_teleop_cmd)
        self.p_fleet.open_requested.connect(self._open_from_fleet)
        self.p_fleet.watched_changed.connect(lambda ids: save_settings({"fleet_watch": sorted(ids)}))
        self.p_fleet.estop_one.connect(self._on_fleet_estop)
        self.p_fleet.estop_all_done.connect(self._on_fleet_estop_all)
        self.p_schedule.changed.connect(lambda: self._log("info", "Schedule", "schedules changed"))
        self.p_data.export_requested.connect(self._ask_export)
        self.p_data.import_requested.connect(self._ask_import)
        self.transfer.finished.connect(
            lambda ok, text: self._log("info" if ok else "warn", "Data", text))

        # teleop → ros
        self.p_teleop.cmd_vel.connect(self.ros.publish_cmd_vel)
        self.p_teleop.estop_changed.connect(self._set_estop)
        self.sidebar_estop.toggled.connect(self._set_estop)

        # map / nav / mission → ros
        self.p_map.goal_requested.connect(self._send_goal_checked)
        self.p_map.cancel_requested.connect(self.ros.cancel_nav_goal)
        self.p_map.set_initial_pose.connect(self._set_initial_pose)
        self.p_nav.goal_requested.connect(self._send_goal_checked)
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
        self.p_3d.goal_requested.connect(self._send_goal_checked)
        self.p_3d.pose_requested.connect(self._set_initial_pose)
        # keepout and speed zones of the map Nav2 runs on
        zb = self.p_map.zones
        zb.reload_requested.connect(lambda name: self.ros.send_launcher_cmd("zones_get", name=name))
        zb.save_requested.connect(self._save_zones)
        zb.changed.connect(self._draw_zones)
        self.ros.zones.connect(zb.on_answer)
        self.p_map.view.waypoint_requested.connect(self._warn_keepout_waypoint)
        self.p_3d.waypoint_requested.connect(self.p_mission.add_waypoint_at)
        self.p_mission._emit_wps()      # the open mission was loaded before these connections

    def _on_connect(self, profile: RobotProfile) -> None:
        """Connect. Without a token, the robot's gateway is asked at the same
        time; if it wants a sign-in, auth_problem comes and asks for it."""
        self._connect_t = time.monotonic()
        self._connect_now(profile)
        if not profile.token:
            self.ros.probe_gateway()

    _connect_t = -1e9

    def _connect_now(self, profile: RobotProfile) -> None:
        self._odom_seen = False
        if self.ros.profile is None or self.ros.profile.id != profile.id:
            self.p_map.zones.set_map("")        # another robot: its maps have their own zones
        self.profiles.set_active(profile.id)
        self._switch_robot(profile)
        self._bring_layout(profile)
        self.ros.set_profile(profile)
        self.ros.connect_robot()
        self._apply_role(profile.role if profile.token else "")
        self._show_user(profile)
        self.sb_robot.setText(f"{profile.name} ({profile.ws_url})")
        self._apply_profile_to_panels(profile)

    # ---- health and diagnostics ----

    _camera_problem = ""

    def health_context(self) -> dict:
        """What the Status page's health checks look at."""
        now = time.monotonic()
        launcher = self.p_robot._state or {}
        online = self.p_robot._launcher_online()
        t = self.ros.profile.topics if self.ros.profile else None
        return {
            "connected": self.ros.is_connected(), "launcher_online": online, "launcher": launcher,
            "gate": self.gate_state or None, "estop": self._estop_on, "battery": getattr(self, "_battery", None),
            "ages": {k: now - st.last_t for k, st in self.ros.topic_stats.items() if st.last_t > 0},
            "localized": bool(t and self.tf.pose_xyyaw(t.map_frame, t.base_frame) is not None),
            "amcl": self.ros.amcl_spread,       # AMCL says it on each update only: valid while it runs
            "camera_problem": self._camera_problem,
            "depth_problem": self._depth_problem or "",
            "robot_note": versions.robot_side_note(launcher) if online else "",
            "role": self.ros.role, "user": self.ros.user,
            "gateway": bool(self.ros.role) or bool(self.p_conn.account.gateway),
        }

    def _ask_bundle(self) -> None:
        name = f"rcs_diagnostics_{time.strftime('%Y%m%d_%H%M')}.zip"
        path, _ = QFileDialog.getSaveFileName(self, "Save a diagnostic bundle", name, "Zip (*.zip)")
        if path:
            self.save_bundle(path)

    def save_bundle(self, path: str):
        ctx = self.health_context()
        from ..core import health
        live = {"health": [c._asdict() for c in health.checks(ctx)],
                "rates": {k: round(st.hz, 2) for k, st in self.ros.topic_stats.items()},
                "launcher": ctx["launcher"], "gate": ctx["gate"], "battery": ctx["battery"],
                "profile": self.ros.profile.shared() if self.ros.profile else None,
                "role": self.ros.role, "nav_state": self._nav_state, "app_previous_run_unclean": self._unclean}
        folder = robot_dir(self._data_robot)
        files = [(f"robot/{n}", folder / n) for n in ("events.csv", "state.json", "missions.json", "places.json",
                                                      "runs.json", "schedules.json")]
        try:
            out = diagnostics.make_bundle(path, live, files)
        except OSError as e:
            self.statusBar().showMessage(f"cannot write {path}: {e}", 8000)
            return None
        self.statusBar().showMessage(f"diagnostic bundle saved: {out}", 8000)
        self._log("info", "App", f"diagnostic bundle saved: {out}")
        return out

    # ---- signing in (robot/rcs_gateway.py) ----

    def _on_signed_in(self, profile: RobotProfile) -> None:
        """A new token (or none): connect again with it if this is the robot in use."""
        if self.ros.profile is not None and self.ros.profile.id == profile.id:
            if profile.token:
                self._connect_now(profile)
            else:
                self.ros.disconnect_robot()
                self._apply_role("")
        self._show_user(profile)

    def _on_session(self, st: dict) -> None:
        p = self.ros.profile
        if p is None:
            return
        role, user = st.get("role", ""), st.get("user", "")
        if (p.role, p.user) != (role, user):
            p.role, p.user = role, user
            self.profiles.upsert(p)
        self._apply_role(role)
        self.p_conn.account.set_session(user, role)
        self._show_user(p)

    def _show_user(self, p) -> None:
        if p is not None:
            self.p_schedule.robot_name = p.name
        self.sb_user.setText(f"   👤 {p.user} · {p.role}" if p and p.token and p.role else "")

    def _apply_role(self, role: str) -> None:
        def why(action):
            return permissions.refusal(role, action)
        self.p_teleop.set_can_drive(permissions.can(role, "drive"), why("drive"))
        self.p_robot.set_can_operate(permissions.can(role, "launch"), why("launch"))
        self.p_mission.set_can_run(permissions.can(role, "goal"), why("goal"))
        self.p_data.set_can_send(permissions.can(role, "send_map"), why("send_map"))
        self.p_map.zones.set_can_edit(permissions.can(role, "zones"))
        self.p_battery.set_can_delete(permissions.can(role, "delete_log"))

    def _on_denied(self, d: dict) -> None:
        text = d.get("what", "")
        if not d.get("local"):
            text = f"The robot refused: {text} needs {d.get('need')} (you are {d.get('role')})"
        self.statusBar().showMessage(text, 8000)
        self._log("warn", "Access", text)

    def _on_auth_problem(self, why: str) -> None:
        p = self.ros.profile
        self.ros.disconnect_robot()
        if p is not None and "ended" in why and p.token:
            p.token, p.role = "", ""
            self.profiles.upsert(p)
        self._apply_role("")
        self._show_user(p)
        self.p_conn.account.show_problem(why)
        self._log("warn", "Access", f"{p.name if p else 'robot'}: {why}")
        self.statusBar().showMessage(why, 10000)
        # just pressed Connect: ask now; otherwise the Connection page says so
        if p is not None and time.monotonic() - self._connect_t < 30 and self.p_conn.account.profile is p:
            self.p_conn.account.ask_sign_in()      # signed in: _on_signed_in connects again

    def _on_profile_changed(self, profile: RobotProfile) -> None:
        self.p_fleet.rebuild()
        # Connected, the pages stay on that robot: selecting another one in the
        # list must not give the robot being driven its speed limits or size.
        driven = self.ros.profile.id if self.ros._want_connected and self.ros.profile else None
        if driven is None or driven == profile.id:
            self._apply_profile_to_panels(profile)
        if not self.ros._want_connected:
            self._switch_robot(profile)
        self._show_data_robot()

    def robot_model_of(self, profile: RobotProfile):
        """The model this robot is drawn with (None: a box of its footprint)."""
        lim = profile.limits
        return robot_model.for_profile(profile.model, robot_dir(profile.id),
                                       lim.footprint_half_length, lim.footprint_half_width)

    def _apply_profile_to_panels(self, profile: RobotProfile) -> None:
        self.p_teleop.apply_limits(profile.limits.max_linear, profile.limits.max_angular)
        hl, hw = profile.limits.footprint_half_length, profile.limits.footprint_half_width
        model = self.robot_model_of(profile)
        for view in (self.p_map.view, self.p_nav.view, self.p_mission.view):
            view.set_robot_footprint(hl, hw)
            view.set_robot_model(model)
        self.p_3d.set_robot_footprint(hl, hw)
        self.p_3d.set_robot_model(model, "fixed")

    def _on_ros_connected(self) -> None:
        self.sb_conn.setText("● connected"); self.sb_conn.setStyleSheet("color:#39d98a;")
        self._linked = True
        self._last_conn_error = None
        self._log("info", "Connection", f"connected to {self.ros.profile.ws_url if self.ros.profile else 'robot'}")
        self.p_conn.set_connected(True, self.ros.profile.ws_url if self.ros.profile else "")
        self.p_status.set_connected(True)
        self.p_robot.set_connected(True)
        self.p_data.set_connected(True)
        self._gate_seen = False         # the next E-STOP state is the first of this connection
        self.p_fleet.driven = self.ros.profile.id if self.ros.profile else ""
        self.robot_state.update(last_connected=time.time())
        self._show_data_robot()

    def _on_ros_disconnected(self) -> None:
        self.sb_conn.setText("● offline"); self.sb_conn.setStyleSheet("color:#ff6a6a;")
        self.p_map.zones.set_link(False, self.p_map.zones.available)
        if getattr(self, "_linked", False):
            self._linked = False
            self._sensor_ok.clear()
            if self.ros._want_connected:
                self._log("warn", "Connection", "connection lost, reconnecting")
            else:
                self._log("info", "Connection", "disconnected")
        if self.p_mission._exec_running:
            # the robot stops by itself (its E-STOP gate misses this app's
            # heartbeat): the mission waits for an operator, it does not go on
            self.p_mission._pause()
            self.runs.event("connection to the robot lost: paused")
            self._log("warn", "Mission", "connection lost: mission paused, the robot stops by itself")
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
        if source != "Alerts":
            p = self.ros.profile or self._data_profile()
            self.alerts.event(level, source, text, robot=p.name if p else "")
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

    def _drain_alert_results(self) -> None:
        while self._alert_results:
            ok, text = self._alert_results.pop(0)
            self._log("info" if ok else "warn", "Alerts", text)

    def _ask_alerts(self) -> None:
        from .panels.alerts_dialog import AlertsDialog

        def test(cfg):
            Alerts(cfg, report=lambda ok, t: self._alert_results.append((ok, t))).test()
        dlg = AlertsDialog(self.settings.get("alerts") or {}, test, self)
        if dlg.exec() == QDialog.Accepted:
            self.set_alerts(dlg.config())

    def set_alerts(self, cfg: dict) -> None:
        self.settings["alerts"] = cfg
        save_settings({"alerts": cfg})
        self.alerts.configure(cfg)

    # ---- going to charge by itself ----

    _charger_goal = False

    def _show_charge(self) -> None:
        places = [(p.id, p.name) for p in self.place_store.places if not self._nav_map or p.map in ("", self._nav_map)]
        self.p_battery.set_charge(self.charger.cfg.to_dict(), places)

    def _set_charge(self, cfg: dict) -> None:
        self.charger.cfg = ChargeConfig.from_dict(cfg)
        self.robot_state.update(charging=self.charger.cfg.to_dict())

    def _charge_step(self) -> None:
        cfg = self.charger.cfg
        place = self.place_store.get(cfg.place) if cfg.place else None
        b = self._battery or {}
        cur = (b.get("reading") or {}).get("current") if b and live(b) else None
        acts = self.charger.step(self._soc_now(), cur, self.p_mission._exec_running, self._nav_state,
                                 self._estop_on, place is not None and self.ros.is_connected())
        for a in acts:
            if a == "pause_mission":
                self.p_mission._pause()
            elif a == "go_charger" and place is not None:
                self._log("warn", "Charging", f"battery {self._soc_now():.0f}%: going to charge at {place.name}")
                self._charger_goal = True
                try:
                    self._send_goal(place.x, place.y, place.yaw)
                finally:
                    self._charger_goal = False
            elif a == "arrived":
                self._log("info", "Charging", f"at {place.name if place else 'the charging place'}, waiting to charge")
            elif a == "charged":
                self._log("info", "Charging", f"charged ({self._soc_now():.0f}%)")
            elif a == "resume_mission":
                self._log("info", "Charging", "the mission goes on")
                self.p_mission._play()
            elif a == "failed":
                self._log("error", "Charging", "could not reach the charging place: the robot needs someone")
        self.p_battery.show_charge_status(self.charger.why)

    def _check_sensors(self) -> None:
        self._drain_alert_results()
        self._charge_step()
        """A sensor that was arriving and stopped is worth an event; one that
        never arrived is not (bringup may simply not be running)."""
        self._update_driving()
        self._update_camera_status()
        self._update_battery_silence()
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

    _battery = None         # latest report of the robot's battery service
    _battery_alarm = ""

    def _soc_now(self):
        """Charge now, or None when the battery is not heard live."""
        b = self._battery
        return float(b["reading"]["soc"]) if b and b.get("reading") and live(b) else None

    def _on_battery(self, b: dict) -> None:
        self._battery = b
        self._show_battery()
        self._report_last_boot(b.get("log") or {})

    def _report_last_boot(self, log: dict) -> None:
        """Once per boot file: how the robot's last boot ended, when it was not
        a clean shut down (power cut or crash), and a disk getting full."""
        prev = log.get("previous") or {}
        name = prev.get("name")
        if name and not prev.get("clean_end") and self.robot_state.get("power_loss_seen") != name:
            from ..core.battery_history import how_it_ended
            self.robot_state.update(power_loss_seen=name)
            self._log("warn", "Battery", f"the robot's last boot ended with {how_it_ended(prev)}")
        disk = log.get("disk") or {}
        if disk.get("low") and not getattr(self, "_disk_warned", False):
            self._disk_warned = True
            self._log("warn", "Battery", "the robot's disk is nearly full: the battery log is kept until "
                                         "you delete old boots (Battery, History)")

    def _show_battery(self) -> None:
        b = self._battery
        self.p_battery.show_status(b)
        r = (b or {}).get("reading")
        if not r:
            self.sb_battery.setText("🔋 —")
            self.sb_battery.setStyleSheet("color:#8b93a7;")
            return
        fresh = live(b)
        level = alarm(r["soc"]) if fresh else ""
        self.sb_battery.setText(f"🔋 {r['soc']}%" + (f" · {r['voltage']:.1f} V" if fresh else " (old)"))
        self.sb_battery.setStyleSheet("color:" + ("#8b93a7" if not fresh else {
            "error": "#ff6a6a", "warn": "#ffb547"}.get(level, "#39d98a")) + ";")
        if fresh and level != self._battery_alarm:
            # once per step down; a reading back above the limit re-arms it
            if level and (level == "error" or self._battery_alarm != "error"):
                self._log(level, "Battery", f"battery at {r['soc']}%")
            self._battery_alarm = level

    def _update_battery_silence(self) -> None:
        """The service reports every second: when it stops, say so, and let the
        numbers on the page age instead of looking live."""
        st = self.ros.topic_stats.get("battery")
        if self.ros.is_connected() and (st is None or not st.fresh(5.0)):
            self._battery = None
            self.p_battery.show_status(None)
            self.sb_battery.setText("🔋 —")
            self.sb_battery.setStyleSheet("color:#8b93a7;")

    _launcher: dict = {}    # latest state from the robot's launcher
    _usb_problem = None
    _depth_problem = None

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
        self._camera_problem = text or ""
        self.p_cam.set_problem(text)
        # The 3D map needs depth; the color picture alone does not tell it has stopped.
        depth = depth_problem(st) if connected else None
        if depth != self._depth_problem:
            self._depth_problem = depth
            if depth:
                self._log("warn", "Sensors", depth)
        self.p_3d.camera_problem = text or depth or ""

    def _on_launcher_events(self, st: dict) -> None:
        self._launcher = st
        nav = st.get("stacks", {}).get("nav", {})
        args = nav.get("args") or {}
        up = nav.get("state") in ("starting", "running") and args.get("mode") == "map"
        self._nav_map = str(args.get("map", "")) if up else ""
        self.p_map.zones.set_link(self.ros.is_connected(), zn.CAP in (st.get("caps") or []))
        self.p_map.zones.set_map(self._nav_map)
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
        if self._keepout_refusal(p.x, p.y):
            return
        self._log("info", "Navigation", f"going to {p.name}")
        self._send_goal(p.x, p.y, p.yaw)

    # ---- keepout and speed zones ----

    def zones(self) -> list:
        """The zones on the maps (as edited on the 2D Map page)."""
        return self.p_map.zones.zones

    def _keepout_refusal(self, x: float, y: float) -> bool:
        """True (and the operator told) if (x, y) is inside a keepout zone."""
        z = zn.keepout_at(self.p_map.zones.robot_zones, x, y)
        if z is None:
            return False
        text = f"({x:.2f}, {y:.2f}) is inside the keepout zone {z['name']}: Nav2 will not go there"
        self.statusBar().showMessage(text, 8000)
        self._log("warn", "Navigation", text)
        return True

    def _send_goal_checked(self, x: float, y: float, yaw: float) -> None:
        if not self._keepout_refusal(x, y):
            self._send_goal(x, y, yaw)

    def _warn_keepout_waypoint(self, x: float, y: float, _yaw: float) -> None:
        z = zn.keepout_at(self.p_map.zones.robot_zones, x, y)
        if z is not None:
            self.statusBar().showMessage(f"That waypoint is inside the keepout zone {z['name']}: "
                                         "the mission will stop there", 8000)

    def _save_zones(self, name: str, zones: list) -> None:
        if not permissions.can(self.ros.role, "zones"):
            self._on_denied({"local": True, "what": permissions.refusal(self.ros.role, "zones")})
            return
        self.ros.send_launcher_cmd("zones_put", name=name, zones=zones)
        self._log("info", "Robot", f"zones of {name} sent to the robot ({len(zones)})")

    def _show_zone_note(self, pose) -> None:
        """Teleop page: the zone the robot is in or near (on the map Nav2 runs on)."""
        zb = self.p_map.zones
        if self._pose_frame != "map" or not zb.loaded or zb.map_name != self._nav_map:
            self.p_teleop.set_zone_note("", [])
            return
        self.p_teleop.set_zone_note(*zn.note_at(zb.robot_zones, pose[0], pose[1]))

    def _draw_zones(self) -> None:
        zb = self.p_map.zones
        self.p_map.view.set_zones(zb.zones, zb.selected_id())
        for v in (self.p_nav.view, self.p_mission.view):
            v.set_zones(zb.zones)

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
        self._odom_seen = True
        self._pose_dirty = True

    def _on_tf(self, _m=None) -> None:
        self._pose_dirty = True        # RosClient already put it in self.tf

    def _update_robot_pose(self) -> None:
        """Recompute the robot and laser pose from TF (or odom)."""
        if not self.ros.profile:
            return
        p = self.ros.profile.topics
        pose, self._pose_frame = self.tf.pose_xyyaw(p.map_frame, p.base_frame), "map"
        if pose is None:
            pose, self._pose_frame = self.tf.pose_xyyaw(p.odom_frame, p.base_frame), "odom"
        if pose is None:
            pose = (self._odom_x, self._odom_y, self._odom_yaw)
            self._pose_frame = "odom" if self._odom_seen else ""
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
            guess = None if self._pose_frame == "map" else self._unlocalized_spot()
            for v in views:
                if guess is None:
                    v.set_robot_pose(*pose)
                else:                       # a map but no place on it: not the corner at (0, 0)
                    v.set_robot_pose(*guess, localized=False)
            self.p_mission.update_robot_pose(*pose, draw="mission" in self._visible)
            if self._pose_frame:        # never a made-up (0, 0) in the trajectory
                self.p_3d.update_robot_pose(*pose, frame=self._pose_frame)
            self.p_nav.places.set_pose(*pose)
            self._show_zone_note(pose)
            self._track_pose(pose)
            self._track_run_distance(pose)
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
        self._map_msg = m
        self._map_spot = None               # worked out again for this map when needed
        self._pose_dirty = True

    _map_msg = None
    _map_spot = None

    def _unlocalized_spot(self):
        """Where to draw the robot while it is not on the map: where it last was
        on this map, else a free place near the middle; None when there is no map."""
        if self._map_msg is None:
            return None
        last = self.last_pose()
        if last:
            return tuple(last)
        if self._map_spot is None:
            from ..core.free_spot import nearest_free
            from ..core.ros_client import grid_from_msg
            m = self._map_msg
            info = m.get("info", {})
            g = m["_grid"] if "_grid" in m else grid_from_msg(m)
            o = info.get("origin", {}).get("position", {})
            spot = None if g is None else nearest_free(g, float(info.get("resolution", 0.05)),
                                                       float(o.get("x", 0.0)), float(o.get("y", 0.0)))
            self._map_spot = (spot[0], spot[1], 0.0) if spot else ()
        return self._map_spot or None

    def _send_goal(self, x: float, y: float, yaw: float) -> None:
        if not self._charger_goal:
            self.charger.take_over()        # the operator's goal (or a mission they started)
        self._goal = (x, y, yaw)
        self._log("info", "Navigation", f"goal sent to ({x:.2f}, {y:.2f})")
        for v in (self.p_map.view, self.p_nav.view, self.p_mission.view, self.p_3d):
            v.set_goal(x, y, yaw)
        self.ros.send_nav_goal(x, y, yaw)
        self._update_driving()

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
            self.p_mission.stop_mission(reason="E-STOP")
            self.ros.cancel_nav_goal()
            self.ros.publish_cmd_vel(0.0, 0.0)
        else:
            self._released_t = time.monotonic()
            self.sb_estop.setText("● estop: clear")
            self.sb_estop.setStyleSheet("color:#39d98a;")
        if publish:
            self.ros.publish_estop(on)

    def _on_estop_remote(self, on: bool) -> None:
        # /amr/estop from another client (e.g. amr_control_center) — mirror it
        self._set_estop(on, publish=False)

    # ---- the robot's own E-STOP (robot/rcs_safety.py) ----

    _estop_on = False       # the app's E-STOP, engaged here or mirrored from the robot
    _gate_seen = False
    _released_t = -1e9      # a state sent before the robot heard our release still says engaged
    _robot_stops = None     # how often the robot stopped itself, as last seen
    gate_state: dict = {}

    def _on_estop_state(self, st: dict) -> None:
        self.gate_state = st
        first = not self._gate_seen
        self._gate_seen = True
        engaged = bool(st.get("engaged"))
        mine = getattr(self, "_estop_on", False)
        # This only ever engages the app's E-STOP, never releases it: a state
        # sent just before the robot heard our E-STOP still says "clear".
        # A release comes as /amr/estop false (_on_estop_remote).
        if engaged and not mine and time.monotonic() - self._released_t > 2.0:
            self._set_estop(True, publish=False)
        elif first and mine and not engaged:
            # engaged here while the robot could not hear it: tell it now
            self.ros.publish_estop(True)
        stops, last = st.get("stops"), st.get("last_stop") or {}
        if self._robot_stops is not None and stops is not None and stops > self._robot_stops and last:
            self._log("warn", "E-STOP", f"the robot stopped itself: {last.get('reason', '')}")
            self.runs.event(f"the robot stopped itself: {last.get('reason', '')}")
        if stops is not None:
            self._robot_stops = stops
        over = bool(st.get("nav_overridden"))
        if over and not self._nav_overridden:
            self._log("info", "Teleop", "teleop goes first: Nav2 and the mission wait until you let go")
        self._nav_overridden = over

    _teleop_t = 0.0
    _nav_overridden = False

    def _on_teleop_cmd(self, lin: float, ang: float) -> None:
        if abs(lin) > 1e-4 or abs(ang) > 1e-4:
            self.charger.take_over()
            self._teleop_t = time.monotonic()
            self._update_driving()

    def _update_driving(self) -> None:
        """Told to the robot in the heartbeat: if this app goes silent while
        driving, the robot stops."""
        self.ros.driving = bool(
            self.p_mission._exec_running
            or (self._goal is not None and self._nav_state in ("active", "pending", ""))
            or time.monotonic() - self._teleop_t < 1.0)

    _run_pose = None

    def _track_run_distance(self, pose) -> None:
        if self.runs.active is None:
            self._run_pose = None
            return
        if self._run_pose is not None:
            d = math.hypot(pose[0] - self._run_pose[0], pose[1] - self._run_pose[1])
            if d < 1.0:             # a jump is relocalization, not driving
                self.runs.add_distance(d)
        self._run_pose = pose

    # ---- schedule ----

    def _start_scheduled(self, s) -> str:
        """Start a scheduled run now, or say why not (that is reported too)."""
        m = self.missions.get(s.mission_id)
        name = m.name if m else "(deleted mission)"
        why = self._not_ready(s)
        if not why:
            why = self.p_mission.run_mission(s.mission_id, "schedule", s.id)
        if why:
            self.runs.skipped(s.mission_id, name, why, s.id, self._soc_now())
        return why

    def _not_ready(self, s) -> str:
        prof = self._data_profile()
        if not self.ros.is_connected() or not self.ros.profile or not prof or self.ros.profile.id != prof.id:
            return "the app was not connected to this robot"
        if getattr(self, "_estop_on", False):
            return "E-STOP was engaged"
        nav = (self._launcher.get("stacks") or {}).get("nav", {}) if self._launcher else {}
        if nav.get("state") != "running":
            return "navigation was not running on the robot"
        if self.p_mission._exec_idx >= 0:
            return "another mission was running"
        if s.min_soc:
            soc = self._soc_now()
            if soc is None:
                return f"the battery was not known (needs at least {s.min_soc}%)"
            if soc < s.min_soc:
                return f"the battery was at {soc:.0f}%, below {s.min_soc}%"
        return ""

    def _on_schedule_fired(self, s, why: str) -> None:
        m = self.missions.get(s.mission_id)
        name = m.name if m else "(deleted mission)"
        if why:
            self._log("warn", "Schedule", f"'{name}' at {s.at} skipped: {why}")
        else:
            self._log("info", "Schedule", f"'{name}' started by the schedule ({s.at})")
        self._update_driving()
        self.p_schedule.refresh()

    # ---- fleet ----

    def _open_from_fleet(self, rid: str) -> None:
        prof = next((p for p in self.profiles.profiles if p.id == rid), None)
        if prof is None:
            return
        if self.ros._want_connected and self.ros.profile and self.ros.profile.id == rid:
            self._show("status")
            return
        if self.p_mission._exec_idx >= 0:
            self.statusBar().showMessage("Stop the mission before opening another robot", 6000)
            return
        self._on_connect(prof)
        self.p_conn.refresh()

    def _on_fleet_estop(self, rid: str) -> None:
        prof = next((p for p in self.profiles.profiles if p.id == rid), None)
        self._log("error", "E-STOP", f"engaged on {prof.name if prof else rid} from the Fleet page")
        if self.ros.profile and self.ros.profile.id == rid and self.ros.is_connected():
            self._set_estop(True)

    def _on_fleet_estop_all(self, names) -> None:
        self._log("error", "E-STOP", f"engaged on all watched robots from the Fleet page: "
                                     f"{', '.join(names) or 'none reached'}")
        if self.ros.is_connected():
            self._set_estop(True)
