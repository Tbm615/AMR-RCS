"""Connection panel — manage robot profiles, connect/disconnect, and how each
robot is drawn (its model and footprint)."""
from __future__ import annotations

import threading

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem, QMessageBox,
    QPushButton, QScrollArea, QSpinBox, QVBoxLayout, QWidget,
)

from ...core import robot_model as rm
from ...core.profile import ProfileStore, RobotProfile
from ...core.robot_data import robot_dir
from ..widgets.robot_shape import RobotIcon
from .account import AccountBox

MODEL_CHOICES = (("auto", "Automatic: the Hinson AMR if the footprint is its size, else a box"),
                 ("hinson", "Hinson AMR (comes with the app)"),
                 ("box", "A box of the footprint"),
                 ("imported", "Imported model"))
IMPORT_FILTER = ("Robot models (*.urdf *.xacro *.xml *.stl *.obj *.dae *.npz);;URDF (*.urdf *.xacro *.xml);;"
                 "Meshes (*.stl *.obj *.dae);;RCS model (*.npz)")


class _Relay(QObject):
    done = Signal(str, object, object, str)     # robot id, model, warnings, error


class ConnectionPanel(QWidget):
    connect_requested = Signal(RobotProfile)
    disconnect_requested = Signal()
    profile_changed = Signal(RobotProfile)
    signed_in = Signal(RobotProfile)        # its sign-in changed: connect again with it

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

        self.account = AccountBox()
        self.account.changed.connect(self._account_changed)
        right.addWidget(self.account)

        topics_box = QGroupBox("Topics / Frames (defaults match this workspace's bringup)")
        tf = QFormLayout(topics_box)
        self.t_cmd = QLineEdit()
        self.t_odom = QLineEdit()
        self.t_scan = QLineEdit()
        self.t_map = QLineEdit()
        self.t_img = QLineEdit()
        self.t_depth = QLineEdit()
        self.t_cloud = QLineEdit()
        self.f_base = QLineEdit()
        self.f_map = QLineEdit()
        tf.addRow("cmd_vel", self.t_cmd)
        tf.addRow("odom", self.t_odom)
        tf.addRow("scan", self.t_scan)
        tf.addRow("map", self.t_map)
        tf.addRow("camera image", self.t_img)
        tf.addRow("camera depth", self.t_depth)
        tf.addRow("point cloud", self.t_cloud)
        tf.addRow("base frame", self.f_base)
        tf.addRow("map frame", self.f_map)
        right.addWidget(topics_box)

        lim_box = QGroupBox("Limits")
        lf = QFormLayout(lim_box)
        self.l_lin = QDoubleSpinBox(); self.l_lin.setRange(0.05, 2.0); self.l_lin.setSingleStep(0.05); self.l_lin.setSuffix(" m/s")
        self.l_ang = QDoubleSpinBox(); self.l_ang.setRange(0.1, 3.0); self.l_ang.setSingleStep(0.1); self.l_ang.setSuffix(" rad/s")
        lf.addRow("Max linear", self.l_lin)
        lf.addRow("Max angular", self.l_ang)
        fp = QHBoxLayout()
        self.l_len = QDoubleSpinBox(); self.l_len.setRange(0.10, 10.0); self.l_len.setSingleStep(0.01)
        self.l_len.setDecimals(2); self.l_len.setSuffix(" m long")
        self.l_wid = QDoubleSpinBox(); self.l_wid.setRange(0.10, 10.0); self.l_wid.setSingleStep(0.01)
        self.l_wid.setDecimals(2); self.l_wid.setSuffix(" m wide")
        self.fp_from_model = QPushButton("From model")
        self.fp_from_model.setToolTip("The smallest rectangle centred on base_link that holds the whole model")
        self.fp_from_model.clicked.connect(self._footprint_from_model)
        fp.addWidget(self.l_len)
        fp.addWidget(self.l_wid)
        fp.addWidget(self.fp_from_model)
        lf.addRow("Footprint", fp)
        fp_hint = QLabel("Used for the lidar clearance bars and drawn dashed on the maps. "
                         "Applies with Save changes.")
        fp_hint.setProperty("muted", True)
        fp_hint.setWordWrap(True)
        lf.addRow("", fp_hint)
        right.addWidget(lim_box)

        model_box = QGroupBox("Robot model (3D Map, 2D maps, Fleet)")
        mh = QHBoxLayout(model_box)
        self.model_icon = RobotIcon(132)
        mh.addWidget(self.model_icon)
        mv = QVBoxLayout()
        self.model_combo = QComboBox()
        for key, text in MODEL_CHOICES:
            self.model_combo.addItem(text, key)
        self.model_combo.activated.connect(self._model_chosen)
        mv.addWidget(self.model_combo)
        self.model_info = QLabel("")
        self.model_info.setWordWrap(True)
        self.model_info.setProperty("muted", True)
        self.model_info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        mv.addWidget(self.model_info)
        mb = QHBoxLayout()
        self.model_import = QPushButton("Import URDF / mesh…")
        self.model_import.setToolTip("A URDF (meshes STL, OBJ or DAE next to it), one STL/OBJ/DAE, "
                                     "or a model another operator exported (.npz)")
        self.model_import.clicked.connect(self._ask_import_model)
        self.model_export = QPushButton("Export…")
        self.model_export.setToolTip("Save this robot's model to give to someone else")
        self.model_export.clicked.connect(self._ask_export_model)
        self.model_remove = QPushButton("Remove imported")
        self.model_remove.clicked.connect(self._remove_model)
        for b in (self.model_import, self.model_export, self.model_remove):
            mb.addWidget(b)
        mb.addStretch(1)
        mv.addLayout(mb)
        mv.addStretch(1)
        mh.addLayout(mv, 1)
        right.addWidget(model_box)
        self._relay = _Relay()
        self._relay.done.connect(self._model_import_done)
        self._importing = ""            # robot id while an import runs

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
        self.auto_ck = QCheckBox("Connect to the last robot when the app starts")
        right.addWidget(self.auto_ck)
        self.update_ck = QCheckBox("Tell me when a new version of RCS is out (asks GitHub once a day)")
        right.addWidget(self.update_ck)
        lang_row = QHBoxLayout()
        lang_row.addWidget(QLabel("Language / Ngôn ngữ"))
        self.lang = QComboBox()
        from ... import i18n
        for code, name in i18n.LANGS.items():
            self.lang.addItem(name, code)
        lang_row.addWidget(self.lang)
        self.lang_note = QLabel("")
        self.lang_note.setProperty("muted", True)
        lang_row.addWidget(self.lang_note, 1)
        right.addLayout(lang_row)

        self.status_lbl = QLabel("not connected")
        self.status_lbl.setProperty("muted", True)
        right.addWidget(self.status_lbl)
        right.addStretch(1)

        holder = QWidget()
        holder.setLayout(right)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setWidget(holder)
        root.addWidget(scroll, 2)

    # ---- data flow ----

    def _reload(self) -> None:
        # Qt may move the current row while the list fills: that must not count
        # as the operator picking a robot, each robot has its own data.
        self.list.blockSignals(True)
        self.list.clear()
        for p in self.store.profiles:
            item = QListWidgetItem(f"{p.name}   —   {p.ws_url}")
            item.setData(Qt.UserRole, p.id)
            self.list.addItem(item)
        self.list.setCurrentRow(-1)
        self.list.blockSignals(False)
        active = self.store.active()
        if active:
            self._select_id(active.id)

    def refresh(self) -> None:
        """The list of robots changed from outside this page."""
        self._reload()

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
        self.t_depth.setText(p.topics.camera_depth)
        self.t_cloud.setText(p.topics.camera_cloud)
        self.f_base.setText(p.topics.base_frame)
        self.f_map.setText(p.topics.map_frame)
        self.l_lin.setValue(p.limits.max_linear)
        self.l_ang.setValue(p.limits.max_angular)
        self.l_len.setValue(2 * p.limits.footprint_half_length)
        self.l_wid.setValue(2 * p.limits.footprint_half_width)
        self._show_model()
        self.account.set_profile(p)
        if not p.token:
            self.account.probe()
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
        p.topics.camera_depth = self.t_depth.text().strip()
        p.topics.camera_cloud = self.t_cloud.text().strip()
        p.topics.base_frame = self.f_base.text().strip() or "base_link"
        p.topics.map_frame = self.f_map.text().strip() or "map"
        p.limits.max_linear = self.l_lin.value()
        p.limits.max_angular = self.l_ang.value()
        p.limits.footprint_half_length = round(self.l_len.value() / 2, 4)
        p.limits.footprint_half_width = round(self.l_wid.value() / 2, 4)
        self.store.upsert(p)
        self._reload()
        self.profile_changed.emit(p)

    def _account_changed(self, p: RobotProfile) -> None:
        self.store.upsert(p)
        self.signed_in.emit(p)

    # ---- robot model ----

    def model_for(self, p: RobotProfile):
        lim = p.limits
        return rm.for_profile(p.model, robot_dir(p.id), lim.footprint_half_length, lim.footprint_half_width)

    def _show_model(self) -> None:
        p = self._current
        if p is None:
            return
        imported = rm.load_cached(robot_dir(p.id) / rm.MODEL_FILE)
        i = self.model_combo.findData(p.model)
        self.model_combo.setCurrentIndex(max(0, i))
        item = self.model_combo.model().item(self.model_combo.findData("imported"))
        item.setEnabled(imported is not None)
        item.setText(f"Imported: {imported.name}" if imported is not None else "Imported model (none yet)")
        model = self.model_for(p)
        lim = p.limits
        self.model_icon.set_model(model, lim.footprint_half_length, lim.footprint_half_width)
        self.model_icon.set_state(None, "ok")
        if self._importing == p.id:
            text = "importing…"
        elif model is None:
            why = " (the Hinson AMR is not this size)" if p.model == "auto" else ""
            text = f"A box {2 * lim.footprint_half_length:.2f} × {2 * lim.footprint_half_width:.2f} m{why}."
        else:
            text = f"{model.name or 'model'}: {model.describe()}" + (
                f", from {model.source}" if model.source else "")
            fl, fw = model.footprint()
            if abs(fl - lim.footprint_half_length) > 0.05 or abs(fw - lim.footprint_half_width) > 0.05:
                text += (f". Its footprint would be {2 * fl:.2f} × {2 * fw:.2f} m "
                         f"(set: {2 * lim.footprint_half_length:.2f} × {2 * lim.footprint_half_width:.2f})")
        self.model_info.setText(text)
        self.model_export.setEnabled(model is not None)
        self.model_remove.setEnabled(imported is not None)
        self.fp_from_model.setEnabled(model is not None)
        self.model_import.setEnabled(not self._importing)

    def _set_model_kind(self, kind: str) -> None:
        p = self._current
        if p is None or p.model == kind:
            return
        p.model = kind
        self.store.upsert(p)
        self._show_model()
        self.profile_changed.emit(p)

    def _model_chosen(self, _index: int) -> None:
        self._set_model_kind(self.model_combo.currentData())

    def _footprint_from_model(self) -> None:
        p = self._current
        model = self.model_for(p) if p else None
        if model is None:
            return
        fl, fw = model.footprint()
        # up to the next centimetre: the rectangle holds the whole robot
        self.l_len.setValue(int(2 * fl * 100 + 0.999) / 100)
        self.l_wid.setValue(int(2 * fw * 100 + 0.999) / 100)

    def _ask_import_model(self) -> None:
        if self._current is None:
            return
        path, _ = QFileDialog.getOpenFileName(self, "Import a robot model", "", IMPORT_FILTER)
        if path:
            self.import_model_from(path, wait=False)

    def import_model_from(self, path: str, wait: bool = True) -> bool:
        """Make the current robot's model from a file. In a thread (a URDF with
        large meshes takes seconds); wait=True for tests and scripts."""
        p = self._current
        if p is None or self._importing:
            return False
        self._importing = p.id
        base = p.topics.base_frame or "base_link"
        self._show_model()

        def work() -> None:
            try:
                model, warnings = rm.import_file(path, base_frame=base)
                self._relay.done.emit(p.id, model, warnings, "")
            except (rm.ModelError, OSError, ValueError) as e:
                self._relay.done.emit(p.id, None, [], str(e))
            except Exception as e:  # noqa: BLE001 - a broken file must not kill the app
                self._relay.done.emit(p.id, None, [], f"{type(e).__name__}: {e}")

        if wait:
            work()
            from PySide6.QtCore import QCoreApplication
            QCoreApplication.processEvents()
            return self.model_error == ""
        threading.Thread(target=work, daemon=True).start()
        return True

    model_error = ""

    def _model_import_done(self, rid: str, model, warnings, error: str) -> None:
        self._importing = ""
        self.model_error = error
        p = next((x for x in self.store.profiles if x.id == rid), None)
        if p is None:
            return
        if model is None:
            if p is self._current:
                self._show_model()
                self.model_info.setText(f'<span style="color:#ff6a6a">Not imported: {error}</span>')
            return
        rm.save(model, robot_dir(rid) / rm.MODEL_FILE)
        p.model = "imported"
        self.store.upsert(p)
        if p is self._current:
            self._show_model()
            if warnings:
                self.model_info.setText(self.model_info.text() + "<br><span style='color:#ffb547'>"
                                        + "<br>".join(str(w) for w in warnings[:8]) + "</span>")
        self.profile_changed.emit(p)

    def _ask_export_model(self) -> None:
        p = self._current
        model = self.model_for(p) if p else None
        if model is None:
            return
        stem = "".join(c if c.isalnum() or c in "-_" else "_" for c in (model.name or p.name))
        path, _ = QFileDialog.getSaveFileName(self, "Export the robot model", f"{stem}.npz", "RCS model (*.npz)")
        if path:
            if not path.lower().endswith(".npz"):
                path += ".npz"
            try:
                rm.save(model, path)
                self.model_info.setText(f"{self.model_info.text()}<br>saved to {path}")
            except OSError as e:
                self.model_info.setText(f'<span style="color:#ff6a6a">cannot write {path}: {e}</span>')

    def _remove_model(self) -> None:
        p = self._current
        if p is None:
            return
        f = robot_dir(p.id) / rm.MODEL_FILE
        if self.confirm_dialogs and QMessageBox.question(
                self, "Remove the imported model", f"Remove the model imported for '{p.name}'?") != QMessageBox.Yes:
            return
        f.unlink(missing_ok=True)
        if p.model == "imported":
            p.model = "auto"
            self.store.upsert(p)
        self._show_model()
        self.profile_changed.emit(p)

    confirm_dialogs = True

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
