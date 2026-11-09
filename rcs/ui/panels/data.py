"""Data page: back up or share a robot's data, and move maps around.

Maps on this PC are shared by all robots; copying one from a robot and sending
it to another is how a map gets across. The dialogs only collect choices: the
work is done by MainWindow (export_pack_to, import_pack_from) and rcs.core.pack.
"""
from __future__ import annotations

from typing import Iterable, List, Optional

from PySide6.QtCore import Qt, QUrl, Signal
from PySide6.QtGui import QDesktopServices, QPixmap
from PySide6.QtWidgets import (
    QCheckBox, QDialog, QDialogButtonBox, QGroupBox, QHBoxLayout, QLabel, QListWidget,
    QListWidgetItem, QMessageBox, QProgressBar, QPushButton, QRadioButton, QVBoxLayout, QWidget,
)

from ...core.map_transfer import MapTransfer
from ...core.maps import MapLibrary
from ...core.pack import Pack

GREEN, RED, AMBER = "#39d98a", "#ff6a6a", "#ffb547"


class ExportDialog(QDialog):
    """What goes in the file. kind "backup": everything; "share": what is
    useful to someone else (not this PC's history, not its window layout)."""

    def __init__(self, kind: str, robot: str, counts: dict, maps: List[str], used_maps: Iterable[str],
                 missing_maps: Iterable[str], parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Back up this robot" if kind == "backup" else "Share with others")
        backup = kind == "backup"
        v = QVBoxLayout(self)
        v.addWidget(QLabel(f"<b>{robot}</b>"))
        self.checks = {}
        for key, label, on in (
                ("missions", f"Missions ({counts.get('missions', 0)})", True),
                ("places", f"Saved places ({counts.get('places', 0)})", True),
                ("profile", "Robot profile (address, topics, limits)", backup),
                ("layouts", f"My saved layouts ({counts.get('layouts', 0)})", backup),
                ("state", "How the app was left (layout, open mission, last positions)", backup),
                ("events", f"Event log ({counts.get('events', 0)})", backup),
                ("model", f"Its imported robot model ({counts.get('model', '')})", True)):
            if key == "model" and not counts.get("model"):
                continue
            ck = QCheckBox(label)
            ck.setChecked(on)
            v.addWidget(ck)
            self.checks[key] = ck
        v.addWidget(QLabel("Maps on this PC to include:"))
        self.map_list = QListWidget()
        used = set(used_maps)
        for name in maps:
            it = QListWidgetItem(name)
            it.setFlags(it.flags() | Qt.ItemIsUserCheckable)
            it.setCheckState(Qt.Checked if backup or name in used else Qt.Unchecked)
            self.map_list.addItem(it)
        self.map_list.setMaximumHeight(130)
        v.addWidget(self.map_list)
        missing = sorted(missing_maps)
        if missing:
            w = QLabel(f"These missions and places are on maps that are not on this PC: "
                       f"{', '.join(missing)}. Copy them from the robot first, or the others "
                       "cannot use them.")
            w.setWordWrap(True)
            w.setStyleSheet(f"color:{AMBER};")
            v.addWidget(w)
        box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        box.accepted.connect(self.accept)
        box.rejected.connect(self.reject)
        v.addWidget(box)

    def items(self) -> dict:
        out = {k: ck.isChecked() for k, ck in self.checks.items()}
        out["maps"] = [self.map_list.item(i).text() for i in range(self.map_list.count())
                       if self.map_list.item(i).checkState() == Qt.Checked]
        return out


class ImportDialog(QDialog):
    """What is in the file, what to take from it, and for which robot."""

    def __init__(self, pack: Pack, current_robot: str, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Import")
        m = pack.manifest
        v = QVBoxLayout(self)
        v.addWidget(QLabel(f"From <b>{m.get('robot', '?')}</b>, {m.get('created', '?')}, RCS {m.get('app', '?')}"))
        self.checks = {}
        for key, label, n in (("missions", "Missions", len(pack.missions)),
                              ("places", "Saved places", len(pack.places)),
                              ("maps", "Maps", len(pack.maps)),
                              ("layouts", "Saved layouts", len(pack.layouts)),
                              ("model", f"Robot model ({pack.model.name if pack.model else ''})",
                               int(pack.model is not None))):
            if n:
                ck = QCheckBox(f"{label} ({n})")
                ck.setChecked(True)
                v.addWidget(ck)
                self.checks[key] = ck
        note = QLabel("Nothing here is replaced: what is new is added next to what you have.")
        note.setProperty("muted", True)
        note.setWordWrap(True)
        v.addWidget(note)
        self.to_current = QRadioButton(f"Add to {current_robot}")
        self.to_new = QRadioButton("As a new robot")
        self.to_current.setChecked(True)
        v.addWidget(self.to_current)
        v.addWidget(self.to_new)
        if pack.profile is None:
            self.to_new.setEnabled(False)
            self.to_new.setToolTip("This file has no robot profile")
        else:
            p = pack.profile
            lim = QLabel(f"{p.name} at {p.ws_url}: up to {p.limits.max_linear:.2f} m/s, "
                         f"{p.limits.max_angular:.2f} rad/s. Check the limits before driving.")
            lim.setProperty("muted", True)
            lim.setWordWrap(True)
            v.addWidget(lim)
            if pack.state is not None:      # a backup: most likely a new PC
                self.to_new.setChecked(True)
        box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        box.accepted.connect(self.accept)
        box.rejected.connect(self.reject)
        v.addWidget(box)

    def want(self) -> set:
        out = {k for k, ck in self.checks.items() if ck.isChecked()}
        if self.to_new.isChecked():
            out |= {"state", "events"}      # belong to a robot of their own
        return out


class DataPanel(QWidget):
    export_requested = Signal(str)      # "backup" | "share"
    import_requested = Signal()
    library_changed = Signal()

    # Set False in tests (QMessageBox would block offscreen).
    confirm_dialogs = True

    def __init__(self, library: MapLibrary, transfer: MapTransfer, parent=None) -> None:
        super().__init__(parent)
        self.library = library
        self.transfer = transfer
        self._connected = False
        self._state: dict = {}
        self._folder = ""
        self._build()
        transfer.progress.connect(self._on_progress)
        transfer.finished.connect(self._on_finished)
        self.refresh_library()
        self._sync()

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(14)

        top = QGroupBox("This robot's data")
        tv = QVBoxLayout(top)
        self.robot_lbl = QLabel("—")
        self.robot_lbl.setWordWrap(True)
        tv.addWidget(self.robot_lbl)
        b = QLabel("Missions, places, the event log and how the app was left are kept per robot on "
                   "this PC, and saved as you go. Back up keeps all of it in one file. Share leaves "
                   "out what only matters here.")
        b.setProperty("muted", True)
        b.setWordWrap(True)
        tv.addWidget(b)
        r = QHBoxLayout()
        self.backup_btn = QPushButton("Back up…")
        self.share_btn = QPushButton("Share…")
        self.share_btn.setProperty("kind", "primary")
        self.import_btn = QPushButton("Import…")
        self.folder_btn = QPushButton("Open folder")
        self.backup_btn.clicked.connect(lambda: self.export_requested.emit("backup"))
        self.share_btn.clicked.connect(lambda: self.export_requested.emit("share"))
        self.import_btn.clicked.connect(self.import_requested.emit)
        self.folder_btn.clicked.connect(self._open_folder)
        for w in (self.backup_btn, self.share_btn, self.import_btn):
            r.addWidget(w)
        r.addStretch(1)
        r.addWidget(self.folder_btn)
        tv.addLayout(r)
        root.addWidget(top)

        cols = QHBoxLayout()
        cols.setSpacing(14)
        pc = QGroupBox("Maps on this PC")
        pv = QVBoxLayout(pc)
        self.pc_list = QListWidget()
        self.pc_list.itemSelectionChanged.connect(self._on_pc_select)
        pv.addWidget(self.pc_list, 1)
        self.thumb = QLabel()
        self.thumb.setFixedHeight(120)
        self.thumb.setAlignment(Qt.AlignCenter)
        pv.addWidget(self.thumb)
        self.pc_info = QLabel("")
        self.pc_info.setProperty("muted", True)
        pv.addWidget(self.pc_info)
        pr = QHBoxLayout()
        self.send_btn = QPushButton("Send to robot →")
        self.send_btn.setToolTip("Copy the selected map to the connected robot")
        self.send_btn.clicked.connect(self._send)
        self.del_btn = QPushButton("Delete")
        self.del_btn.setToolTip("Delete it from this PC. The robot's copy is not touched")
        self.del_btn.clicked.connect(self._delete)
        pr.addWidget(self.send_btn)
        pr.addWidget(self.del_btn)
        pv.addLayout(pr)
        cols.addWidget(pc, 1)

        bot = QGroupBox("Maps on the robot")
        bv = QVBoxLayout(bot)
        self.robot_list = QListWidget()
        self.robot_list.itemSelectionChanged.connect(self._sync)
        bv.addWidget(self.robot_list, 1)
        self.robot_hint = QLabel("")
        self.robot_hint.setProperty("muted", True)
        self.robot_hint.setWordWrap(True)
        bv.addWidget(self.robot_hint)
        self.get_btn = QPushButton("← Copy to this PC")
        self.get_btn.clicked.connect(self._get)
        bv.addWidget(self.get_btn)
        cols.addWidget(bot, 1)
        root.addLayout(cols, 1)

        self.bar = QProgressBar()
        self.bar.setVisible(False)
        root.addWidget(self.bar)
        self.result = QLabel("")
        self.result.setWordWrap(True)
        root.addWidget(self.result)

    # ---- called by MainWindow ----

    def set_robot(self, text: str, folder: str) -> None:
        self.robot_lbl.setText(text)
        self._folder = folder

    def set_connected(self, ok: bool) -> None:
        self._connected = ok
        if not ok:
            self._state = {}
            self.robot_list.clear()
        self._sync()

    def on_launcher_state(self, st: dict) -> None:
        self._state = st
        maps = st.get("maps") or []
        if maps != [self.robot_list.item(i).text() for i in range(self.robot_list.count())]:
            keep = self._selected(self.robot_list)
            self.robot_list.clear()
            self.robot_list.addItems(maps)
            self._select(self.robot_list, keep)
        self._sync()

    def show_result(self, ok: bool, text: str) -> None:
        self.result.setText(f'<span style="color:{GREEN if ok else RED}">{text}</span>')

    def robot_maps(self) -> List[str]:
        return list(self._state.get("maps") or [])

    def refresh_library(self) -> None:
        keep = self._selected(self.pc_list)
        self.pc_list.clear()
        self.pc_list.addItems(self.library.names())
        self._select(self.pc_list, keep)
        self._on_pc_select()

    # ---- helpers ----

    @staticmethod
    def _selected(lst: QListWidget) -> Optional[str]:
        it = lst.currentItem()
        return it.text() if it is not None and it.isSelected() else None

    @staticmethod
    def _select(lst: QListWidget, name: Optional[str]) -> None:
        for i in range(lst.count()):
            if lst.item(i).text() == name:
                lst.setCurrentRow(i)
                return

    def _can_transfer(self) -> bool:
        return "map_transfer" in (self._state.get("caps") or [])

    can_send = True         # False for a signed-in viewer

    def set_can_send(self, ok: bool, why: str = "") -> None:
        self.can_send = ok
        self.send_btn.setToolTip("Copy the selected map to the connected robot" if ok else why)
        self._sync()

    def _sync(self) -> None:
        ok = self._connected and self._can_transfer() and not self.transfer.busy
        self.send_btn.setEnabled(ok and self.can_send and self._selected(self.pc_list) is not None)
        self.get_btn.setEnabled(ok and self._selected(self.robot_list) is not None)
        self.del_btn.setEnabled(self._selected(self.pc_list) is not None)
        if not self._connected:
            self.robot_hint.setText("Connect to a robot to see its maps.")
        elif not self._state:
            self.robot_hint.setText("The RCS launcher is not running on the robot.")
        elif not self._can_transfer():
            self.robot_hint.setText("To copy maps, update rcs_launcher.py, map_exchange.py and "
                                    "map_files.py on the robot.")
        else:
            self.robot_hint.setText("")

    def _on_pc_select(self) -> None:
        self._sync()
        name = self._selected(self.pc_list)
        self.thumb.clear()
        self.pc_info.setText("")
        if name is None:
            return
        try:
            i = self.library.info(name)
        except (OSError, ValueError) as e:
            self.pc_info.setText(f"cannot read it: {e}")
            return
        pix = QPixmap()
        if pix.loadFromData(i["pgm"], "PGM"):
            self.thumb.setPixmap(pix.scaled(self.thumb.width(), self.thumb.height(), Qt.KeepAspectRatio,
                                            Qt.SmoothTransformation))
        self.pc_info.setText(f"{i['width_m']:.1f} × {i['height_m']:.1f} m · "
                             f"{i['resolution'] * 100:.0f} cm per cell · {i['date']}")

    def _ask(self, title: str, text: str) -> bool:
        if not self.confirm_dialogs:
            return True
        return QMessageBox.question(self, title, text) == QMessageBox.Yes

    # ---- actions ----

    def _open_folder(self) -> None:
        if self._folder:
            QDesktopServices.openUrl(QUrl.fromLocalFile(self._folder))

    def _start_bar(self) -> None:
        self.bar.setRange(0, 0)
        self.bar.setVisible(True)
        self.result.setText("")
        self._sync()

    def _get(self) -> None:
        name = self._selected(self.robot_list)
        if name is not None and not self.transfer.busy:
            self.transfer.download(name)
            self._start_bar()

    def _send(self) -> None:
        name = self._selected(self.pc_list)
        if name is None or self.transfer.busy:
            return
        replace = name in self.robot_maps()
        if replace and not self._ask("Replace map?", f"The robot already has a map called {name}. "
                                                     "Replace it with the one from this PC?"):
            return
        self.transfer.upload(name, overwrite=replace)
        if self.transfer.busy:
            self._start_bar()

    def _delete(self) -> None:
        name = self._selected(self.pc_list)
        if name is not None and self._ask("Delete map?", f"Delete {name} from this PC?"):
            self.library.remove(name)
            self.refresh_library()
            self.library_changed.emit()

    def _on_progress(self, done: int, total: int) -> None:
        self.bar.setRange(0, max(1, total))
        self.bar.setValue(done)

    def _on_finished(self, ok: bool, text: str) -> None:
        self.bar.setVisible(False)
        self.show_result(ok, text)
        self.refresh_library()
        self.library_changed.emit()
