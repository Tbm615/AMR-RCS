"""Connection panel — manage robot profiles, connect/disconnect."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem, QMessageBox,
    QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

from ...core.profile import ProfileStore, RobotProfile


class ConnectionPanel(QWidget):
    connect_requested = Signal(RobotProfile)
    disconnect_requested = Signal()
    profile_changed = Signal(RobotProfile)

    def __init__(self, store: ProfileStore, parent=None):
        super().__init__(parent)
        self.store = store
        self._current: RobotProfile | None = None
        self._build()
        self._reload()

    def _build(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(16)

        # --- left: list of robots ---
        left = QVBoxLayout()
        left.addWidget(QLabel("<b>Robots</b>"))
        self.list = QListWidget()
        self.list.currentItemChanged.connect(self._on_select)
        left.addWidget(self.list, 1)

        btns = QHBoxLayout()
        add = QPushButton("+ Add")
        add.clicked.connect(self._add)
        rm = QPushButton("Remove")
        rm.clicked.connect(self._remove)
        btns.addWidget(add)
        btns.addWidget(rm)
        left.addLayout(btns)
        root.addLayout(left, 1)

        # --- right: edit form + connect buttons ---
        right = QVBoxLayout()
        right.setSpacing(12)

        conn_box = QGroupBox("Connection")
        f = QFormLayout(conn_box)
        self.name = QLineEdit()
        self.host = QLineEdit()
        self.port = QSpinBox(); self.port.setRange(1, 65535); self.port.setValue(9090)
        self.ssl = QCheckBox("Use secure WebSocket (wss://)")
        self.note = QLineEdit()
        f.addRow("Name", self.name)
        f.addRow("Host", self.host)
        f.addRow("Port", self.port)
        f.addRow("", self.ssl)
        f.addRow("Note", self.note)
        right.addWidget(conn_box)

        topics_box = QGroupBox("Topics / Frames (defaults match this workspace's bringup)")
        tf = QFormLayout(topics_box)
        self.t_cmd = QLineEdit()
        self.t_odom = QLineEdit()
        self.t_scan = QLineEdit()
        self.t_map = QLineEdit()
        self.t_img = QLineEdit()
        self.t_cloud = QLineEdit()
        self.f_base = QLineEdit()
        self.f_map = QLineEdit()
        tf.addRow("cmd_vel", self.t_cmd)
        tf.addRow("odom", self.t_odom)
        tf.addRow("scan", self.t_scan)
        tf.addRow("map", self.t_map)
        tf.addRow("camera image", self.t_img)
        tf.addRow("camera cloud", self.t_cloud)
        tf.addRow("base frame", self.f_base)
        tf.addRow("map frame", self.f_map)
        right.addWidget(topics_box)

        lim_box = QGroupBox("Limits")
        lf = QFormLayout(lim_box)
        self.l_lin = QDoubleSpinBox(); self.l_lin.setRange(0.05, 2.0); self.l_lin.setSingleStep(0.05); self.l_lin.setSuffix(" m/s")
        self.l_ang = QDoubleSpinBox(); self.l_ang.setRange(0.1, 3.0); self.l_ang.setSingleStep(0.1); self.l_ang.setSuffix(" rad/s")
        lf.addRow("Max linear", self.l_lin)
        lf.addRow("Max angular", self.l_ang)
        right.addWidget(lim_box)

        save = QPushButton("Save changes")
        save.setProperty("kind", "primary")
        save.clicked.connect(self._save)
        right.addWidget(save)

        row = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setProperty("kind", "success")
        self.connect_btn.clicked.connect(self._do_connect)
        self.disc_btn = QPushButton("Disconnect")
        self.disc_btn.clicked.connect(self.disconnect_requested.emit)
        row.addWidget(self.connect_btn)
        row.addWidget(self.disc_btn)
        right.addLayout(row)

        self.status_lbl = QLabel("not connected")
        self.status_lbl.setProperty("muted", True)
        right.addWidget(self.status_lbl)
        right.addStretch(1)

        root.addLayout(right, 2)

    # ---- data flow ----

    def _reload(self) -> None:
        self.list.clear()
        for p in self.store.profiles:
            item = QListWidgetItem(f"{p.name}   —   {p.ws_url}")
            item.setData(Qt.UserRole, p.id)
            self.list.addItem(item)
        active = self.store.active()
        if active:
            self._select_id(active.id)

    def _select_id(self, pid: str) -> None:
        for i in range(self.list.count()):
            if self.list.item(i).data(Qt.UserRole) == pid:
                self.list.setCurrentRow(i)
                return

    def _on_select(self, cur, _prev) -> None:
        if not cur:
            return
        pid = cur.data(Qt.UserRole)
        p = next((x for x in self.store.profiles if x.id == pid), None)
        if not p:
            return
        self._current = p
        self.store.set_active(p.id)
        self.name.setText(p.name)
        self.host.setText(p.host)
        self.port.setValue(p.port)
        self.ssl.setChecked(p.use_ssl)
        self.note.setText(p.note)
        self.t_cmd.setText(p.topics.cmd_vel)
        self.t_odom.setText(p.topics.odom)
        self.t_scan.setText(p.topics.scan)
        self.t_map.setText(p.topics.map)
        self.t_img.setText(p.topics.camera_image)
        self.t_cloud.setText(p.topics.camera_cloud)
        self.f_base.setText(p.topics.base_frame)
        self.f_map.setText(p.topics.map_frame)
        self.l_lin.setValue(p.limits.max_linear)
        self.l_ang.setValue(p.limits.max_angular)
        self.profile_changed.emit(p)

    def _save(self) -> None:
        if not self._current:
            return
        p = self._current
        p.name = self.name.text().strip() or "robot"
        p.host = self.host.text().strip() or "127.0.0.1"
        p.port = self.port.value()
        p.use_ssl = self.ssl.isChecked()
        p.note = self.note.text().strip()
        p.topics.cmd_vel = self.t_cmd.text().strip()
        p.topics.odom = self.t_odom.text().strip()
        p.topics.scan = self.t_scan.text().strip()
        p.topics.map = self.t_map.text().strip()
        p.topics.camera_image = self.t_img.text().strip()
        p.topics.camera_cloud = self.t_cloud.text().strip()
        p.topics.base_frame = self.f_base.text().strip() or "base_link"
        p.topics.map_frame = self.f_map.text().strip() or "map"
        p.limits.max_linear = self.l_lin.value()
        p.limits.max_angular = self.l_ang.value()
        self.store.upsert(p)
        self._reload()
        self.profile_changed.emit(p)

    def _add(self) -> None:
        p = RobotProfile.new(name=f"robot {len(self.store.profiles)+1}")
        self.store.upsert(p)
        self.store.set_active(p.id)
        self._reload()

    def _remove(self) -> None:
        if not self._current:
            return
        if QMessageBox.question(self, "Remove robot",
                                f"Remove '{self._current.name}'?") != QMessageBox.Yes:
            return
        self.store.remove(self._current.id)
        self._current = None
        self._reload()

    def _do_connect(self) -> None:
        if self._current:
            self._save()
            self.connect_requested.emit(self._current)

    # ---- external status hooks ----

    def set_connected(self, ok: bool, info: str = "") -> None:
        if ok:
            self.status_lbl.setProperty("success", True)
            self.status_lbl.setProperty("muted", False)
            self.status_lbl.setProperty("danger", False)
            self.status_lbl.setText(f"connected — {info}" if info else "connected")
        else:
            self.status_lbl.setProperty("success", False)
            self.status_lbl.setProperty("danger", True if info else False)
            self.status_lbl.setProperty("muted", not info)
            self.status_lbl.setText(info or "not connected")
        self.status_lbl.style().unpolish(self.status_lbl)
        self.status_lbl.style().polish(self.status_lbl)
