"""Robot programs panel: start/stop bringup and Nav2 on the robot, save maps.

Talks to robot/rcs_launcher.py through RosClient (launcher_state signal,
send_launcher_cmd). Nothing here runs ROS on the operator PC.
"""
from __future__ import annotations

import re

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QButtonGroup, QComboBox, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QMessageBox, QPlainTextEdit, QPushButton, QRadioButton, QVBoxLayout, QWidget,
)

from ...core.ros_client import RosClient

GREEN, RED, AMBER, MUTED = "#39d98a", "#ff6a6a", "#ffb547", "#8b93a7"

_STATE_TEXT = {
    "stopped": ("stopped", MUTED),
    "starting": ("starting…", AMBER),
    "running": ("running", GREEN),
    "stopping": ("stopping…", AMBER),
    "failed": ("failed", RED),
}


def _duration(sec: float) -> str:
    sec = max(0, int(sec))
    if sec < 60:
        return f"{sec} s"
    if sec < 3600:
        return f"{sec // 60} min"
    return f"{sec // 3600} h {sec % 3600 // 60} min"


class _StackBox(QGroupBox):
    """Status line, start/stop buttons and a log for one stack."""

    def __init__(self, title: str, blurb: str, parent=None):
        super().__init__(title, parent)
        self.v = QVBoxLayout(self)
        self.v.setSpacing(8)
        b = QLabel(blurb); b.setProperty("muted", True); b.setWordWrap(True)
        self.v.addWidget(b)
        self.status = QLabel("—")
        self.status.setTextFormat(Qt.RichText)
        self.v.addWidget(self.status)
        self.extra = QVBoxLayout(); self.v.addLayout(self.extra)
        self.row = row = QHBoxLayout()
        self.start_btn = QPushButton("Start"); self.start_btn.setProperty("kind", "success")
        self.stop_btn = QPushButton("Stop")
        self.log_btn = QPushButton("Show log"); self.log_btn.setCheckable(True)
        for w in (self.start_btn, self.stop_btn, self.log_btn):
            w.setMinimumWidth(96)
            row.addWidget(w)
        row.addStretch(1)
        self.v.addLayout(row)
        self.log = QPlainTextEdit(); self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200); self.log.setFixedHeight(170)
        self.log.setStyleSheet("font-family: monospace; font-size: 11px;")
        self.log.hide()
        self.v.addWidget(self.log)
        self.log_btn.toggled.connect(self.log.setVisible)

    def show_state(self, st: dict, now: float) -> str:
        """Render one stack's state; returns the effective state key."""
        key = st.get("state", "stopped")
        if st.get("external") and key in ("stopped", "failed"):
            self.status.setText(f'<span style="color:{GREEN}">●</span> running '
                                f'<span style="color:{MUTED}">(started outside the app, '
                                f'stop it where it was started)</span>')
            key = "external"
        else:
            text, color = _STATE_TEXT.get(key, (key, MUTED))
            since = st.get("since")
            ago = f' <span style="color:{MUTED}">· {_duration(now - since)}</span>' \
                if since and key in ("running", "failed") else ""
            detail = f' <span style="color:{MUTED}">— {st["detail"]}</span>' if st.get("detail") else ""
            self.status.setText(f'<span style="color:{color}">●</span> {text}{ago}{detail}')
        lines = st.get("log") or []
        text = "\n".join(lines)
        if self.log.toPlainText() != text:
            self.log.setPlainText(text)
            self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())
        if key == "failed" and lines and not self.log_btn.isChecked():
            self.log_btn.setChecked(True)
        return key


class RobotPanel(QWidget):
    open_page = Signal(str)          # e.g. "map" after starting navigation

    # Set False in tests (QMessageBox would block offscreen).
    confirm_dialogs = True

    def __init__(self, ros: RosClient, parent=None):
        super().__init__(parent)
        self.ros = ros
        self._state: dict = {}
        self._pending = ""           # id of the command we wait for
        self._connected = False
        self._build()
        ros.launcher_state.connect(self._on_state)
        t = QTimer(self); t.timeout.connect(self._refresh); t.start(1000)

    # ---- UI ----

    def _build(self) -> None:
        outer = QHBoxLayout(self)
        outer.setContentsMargins(20, 20, 20, 20)
        col = QVBoxLayout(); col.setSpacing(14)
        wrap = QWidget(); wrap.setLayout(col); wrap.setMinimumWidth(620); wrap.setMaximumWidth(760)
        outer.addWidget(wrap, 1, Qt.AlignTop | Qt.AlignLeft)

        self.banner = QLabel(); self.banner.setWordWrap(True)
        self.banner.setTextFormat(Qt.RichText)
        col.addWidget(self.banner)

        self.base = _StackBox(
            "1 · Robot base (bringup)",
            "Motor drives, lidar and camera. Start this first: every other page "
            "needs it. Stopping it disables the motors and also stops navigation.")
        self.base.start_btn.clicked.connect(lambda: self._send("start", stack="bringup"))
        self.base.stop_btn.clicked.connect(self._stop_base)
        col.addWidget(self.base)

        self.nav = _StackBox(
            "2 · Navigation (Nav2)",
            "Path planning and obstacle avoidance, with the CBF safety filter.")
        self.rb_map = QRadioButton("Use a saved map")
        self.rb_slam = QRadioButton("Build a new map while driving (SLAM)")
        self.rb_map.setChecked(True)
        grp = QButtonGroup(self); grp.addButton(self.rb_map); grp.addButton(self.rb_slam)
        self.map_combo = QComboBox(); self.map_combo.setMinimumWidth(220)
        r1 = QHBoxLayout(); r1.addWidget(self.rb_map); r1.addWidget(self.map_combo); r1.addStretch(1)
        self.nav.extra.addLayout(r1)
        self.nav.extra.addWidget(self.rb_slam)
        self.nav_hint = QLabel(); self.nav_hint.setProperty("muted", True); self.nav_hint.setWordWrap(True)
        self.nav.extra.addWidget(self.nav_hint)
        self.rb_map.toggled.connect(self._update_hint)
        self.nav.start_btn.clicked.connect(self._start_nav)
        self.nav.stop_btn.clicked.connect(lambda: self._send("stop", stack="nav"))
        self.goto_map = QPushButton("Open 2D Map")
        self.goto_map.clicked.connect(lambda: self.open_page.emit("map"))
        self.nav.row.insertWidget(3, self.goto_map)
        col.addWidget(self.nav)

        save = QGroupBox("3 · Save the map")
        sv = QVBoxLayout(save)
        b = QLabel("Saves the map Nav2 is using (usually the one built with SLAM) on "
                   "the robot. It then appears under \"Use a saved map\".")
        b.setProperty("muted", True); b.setWordWrap(True)
        sv.addWidget(b)
        r = QHBoxLayout()
        self.map_name = QLineEdit(); self.map_name.setPlaceholderText("map name, e.g. warehouse_a")
        self.save_btn = QPushButton("Save map"); self.save_btn.setProperty("kind", "primary")
        self.save_btn.clicked.connect(self._save_map)
        self.map_name.returnPressed.connect(self._save_map)
        r.addWidget(self.map_name, 1); r.addWidget(self.save_btn)
        sv.addLayout(r)
        col.addWidget(save)

        self.result = QLabel(); self.result.setWordWrap(True)
        col.addWidget(self.result)
        col.addStretch(1)
        self._update_hint()
        self._refresh()

    def _update_hint(self) -> None:
        self.map_combo.setEnabled(self.rb_map.isChecked())
        self.nav_hint.setText(
            "After it starts, set the robot's position on the 2D Map page "
            "(Set initial pose)." if self.rb_map.isChecked() else
            "The map starts empty and grows as the robot drives (Teleop or goals). "
            "No initial pose needed. Save it below when done.")

    # ---- actions ----

    def _send(self, op: str, **kw) -> None:
        self._pending = self.ros.send_launcher_cmd(op, **kw)
        self.result.setText(f'<span style="color:{AMBER}">sent: {op} '
                            f'{kw.get("stack", kw.get("name", ""))}…</span>')
        self._refresh()

    def _ask(self, title: str, text: str) -> bool:
        if not self.confirm_dialogs:
            return True
        return QMessageBox.question(self, title, text) == QMessageBox.Yes

    def _stop_base(self) -> None:
        if self._ask("Stop the robot base?",
                     "The motors are disabled and navigation stops too. Continue?"):
            self._send("stop", stack="bringup")

    def _start_nav(self) -> None:
        if self.rb_map.isChecked():
            m = self.map_combo.currentText()
            if not m:
                self.result.setText(f'<span style="color:{RED}">No saved map on the robot. '
                                    'Use SLAM to build one.</span>')
                return
            self._send("start", stack="nav", mode="map", map=m)
        else:
            self._send("start", stack="nav", mode="slam")

    def _save_map(self) -> None:
        name = self.map_name.text().strip()
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", name):
            self.result.setText(f'<span style="color:{RED}">Map name: letters, digits, '
                                '- and _ only.</span>')
            return
        if name in self._state.get("maps", []) and not self._ask(
                "Replace map?", f"A map called {name} already exists on the robot. Replace it?"):
            return
        self._send("save_map", name=name)

    # ---- state ----

    def set_connected(self, ok: bool) -> None:
        self._connected = ok
        if not ok:
            self._state = {}
            self._pending = ""
        self._refresh()

    def _on_state(self, st: dict) -> None:
        self._state = st
        maps = st.get("maps") or []
        if maps != [self.map_combo.itemText(i) for i in range(self.map_combo.count())]:
            cur = self.map_combo.currentText()
            self.map_combo.blockSignals(True)
            self.map_combo.clear(); self.map_combo.addItems(maps)
            if cur in maps:
                self.map_combo.setCurrentText(cur)
            self.map_combo.blockSignals(False)
        res = st.get("result") or {}
        if self._pending and res.get("id") == self._pending:
            self._pending = ""
            color = GREEN if res.get("ok") else RED
            self.result.setText(f'<span style="color:{color}">{res.get("msg", "")}</span>')
        self._refresh()

    def _launcher_online(self) -> bool:
        stats = self.ros.topic_stats.get("launcher")
        return bool(self._connected and stats and stats.fresh(3.5))

    def _refresh(self) -> None:
        online = self._launcher_online()
        if not self._connected:
            self.banner.setText(f'<span style="color:{MUTED}">Connect to a robot first '
                                '(Connection page).</span>')
        elif not online:
            self.banner.setText(
                f'<span style="color:{AMBER}">● The robot does not answer.</span> '
                'The RCS launcher is not running on the robot. Install it once: see '
                '"Robot setup" in the user guide (robot/start_rcs_robot.sh).')
        else:
            busy = self._state.get("busy")
            self.banner.setText(f'<span style="color:{GREEN}">●</span> Robot launcher online'
                                + (f' <span style="color:{AMBER}">· working: {busy}</span>' if busy else ""))
        stacks = self._state.get("stacks", {}) if online else {}
        now = self._state.get("time", 0.0)
        waiting = bool(self._pending) or bool(self._state.get("busy"))
        base = self.base.show_state(stacks.get("bringup", {}), now) if online else "stopped"
        nav = self.nav.show_state(stacks.get("nav", {}), now) if online else "stopped"
        if not online:
            for box in (self.base, self.nav):
                box.status.setText(f'<span style="color:{MUTED}">● unknown</span>')
        base_up = base in ("running", "external")
        nav_up = nav in ("starting", "running", "external")
        ok = online and not waiting
        self.base.start_btn.setEnabled(ok and base in ("stopped", "failed"))
        self.base.stop_btn.setEnabled(ok and base in ("starting", "running", "failed"))
        self.nav.start_btn.setEnabled(ok and base_up and nav != "external")
        self.nav.start_btn.setText("Restart" if nav in ("starting", "running") else "Start")
        self.nav.stop_btn.setEnabled(ok and nav in ("starting", "running", "failed"))
        self.goto_map.setVisible(nav_up)
        self.save_btn.setEnabled(ok and nav in ("running", "external"))
