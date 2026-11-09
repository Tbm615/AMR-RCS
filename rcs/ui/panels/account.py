"""Signing in to a robot that has the gateway (robot/rcs_gateway.py), and, for
admins, its users and audit log. The calls go off the GUI thread
(rcs.core.gateway blocks); answers come back through a signal.
"""
from __future__ import annotations

import csv
import threading
import time
from typing import Callable, Optional

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QComboBox, QDialog, QDialogButtonBox, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout,
    QHeaderView, QInputDialog, QLabel, QLineEdit, QMessageBox, QPushButton, QTableWidget, QTableWidgetItem,
    QTabWidget, QVBoxLayout, QWidget,
)

from ...core import gateway as gw

GREEN, RED, AMBER, MUTED = "#39d98a", "#ff6a6a", "#ffb547", "#8b93a7"
ROLES = ("viewer", "operator", "admin")
ROLE_TEXT = {"viewer": "sees everything, may stop the robot",
             "operator": "drives, sends goals, runs missions, starts programs",
             "admin": "everything, and the users"}


class _Relay(QObject):
    done = Signal(object, object, object)     # callback, result, error


_relay: Optional[_Relay] = None


def run_bg(fn: Callable, done: Callable) -> None:
    """fn() on a thread; done(result, error) on the GUI thread."""
    global _relay
    if _relay is None:
        _relay = _Relay()
        _relay.done.connect(lambda cb, r, e: cb(r, e))

    def work() -> None:
        try:
            r, e = fn(), None
        except Exception as ex:  # noqa: BLE001 - shown to the operator
            r, e = None, ex
        _relay.done.emit(done, r, e)
    threading.Thread(target=work, daemon=True).start()


def confirm_certificate(parent, host: str, fp: str) -> bool:
    if getattr(parent, "confirm_dialogs", True) is False:
        return True
    return QMessageBox.question(
        parent, "The robot's certificate",
        f"{host} uses a certificate with this fingerprint:\n\n{gw.pretty(fp)}\n\n"
        "It is printed by `rcs_gateway.py gencert` on the robot. Trust it only if they match: "
        "the app keeps it and refuses any other.") == QMessageBox.Yes


class SignInDialog(QDialog):
    """User and password; a token comes back. The certificate is trusted on
    first sight, after the operator compared the fingerprint."""

    def __init__(self, profile, parent=None) -> None:
        super().__init__(parent)
        self.profile = profile
        self.setWindowTitle(f"Sign in to {profile.name}")
        self.result_token = ""
        self.result_role = ""
        v = QVBoxLayout(self)
        v.addWidget(QLabel(f"<b>{profile.name}</b> — {profile.ws_url}"))
        hint = QLabel("This robot asks who you are. An admin creates users on the robot "
                      "(rcs_gateway.py users add) or in the app.")
        hint.setWordWrap(True)
        hint.setProperty("muted", True)
        v.addWidget(hint)
        f = QFormLayout()
        self.user = QLineEdit(profile.user)
        self.password = QLineEdit()
        self.password.setEchoMode(QLineEdit.Password)
        f.addRow("User", self.user)
        f.addRow("Password", self.password)
        v.addLayout(f)
        self.msg = QLabel("")
        self.msg.setWordWrap(True)
        v.addWidget(self.msg)
        self.box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.box.button(QDialogButtonBox.Ok).setText("Sign in")
        self.box.accepted.connect(self.sign_in)
        self.box.rejected.connect(self.reject)
        v.addWidget(self.box)
        (self.password if profile.user else self.user).setFocus()

    def sign_in(self, wait: bool = False) -> None:
        p, user, pw = self.profile, self.user.text().strip(), self.password.text()
        if not user or not pw:
            self.msg.setText(f'<span style="color:{RED}">User and password, please.</span>')
            return
        self.box.setEnabled(False)
        self.msg.setText("signing in…")
        pin = p.tls_pin

        def work():
            fp = pin
            if p.use_ssl and not fp:
                fp = gw.fingerprint(p.host, p.port)
                return ("pin", fp)
            return ("ok", gw.login(p.host, p.port, user, pw, tls=p.use_ssl, pin=fp))

        def done(r, e):
            self.box.setEnabled(True)
            if e is not None:
                self.msg.setText(f'<span style="color:{RED}">{e}</span>')
                return
            kind, val = r
            if kind == "pin":
                if not confirm_certificate(self, p.host, val):
                    self.msg.setText(f'<span style="color:{RED}">Not signed in: certificate not trusted.</span>')
                    return
                p.tls_pin = val
                self.sign_in(wait)
                return
            self.result_token, self.result_role = val
            p.user, p.token, p.role = user, self.result_token, self.result_role
            self.accept()
        if wait:                        # tests: same thread, no dialog loop
            try:
                done(work(), None)
            except Exception as ex:  # noqa: BLE001
                done(None, ex)
        else:
            run_bg(work, done)


class AccountBox(QGroupBox):
    """On the Connection page: who is signed in to the selected robot."""
    changed = Signal(object)            # the profile, its sign-in changed: save it

    confirm_dialogs = True

    def __init__(self, parent=None) -> None:
        super().__init__("Sign-in", parent)
        self.profile = None
        self.gateway: Optional[bool] = None     # None: not asked yet
        v = QVBoxLayout(self)
        self.status = QLabel("—")
        self.status.setWordWrap(True)
        v.addWidget(self.status)
        row = QHBoxLayout()
        self.sign_in_btn = QPushButton("Sign in…")
        self.sign_in_btn.clicked.connect(self.ask_sign_in)
        self.sign_out_btn = QPushButton("Sign out")
        self.sign_out_btn.clicked.connect(self.sign_out)
        self.users_btn = QPushButton("Users and audit log…")
        self.users_btn.clicked.connect(self.open_users)
        self.forget_btn = QPushButton("Forget certificate")
        self.forget_btn.setToolTip("Trust the robot's certificate anew, after it was replaced on purpose")
        self.forget_btn.clicked.connect(self.forget_certificate)
        self.check_btn = QPushButton("Check")
        self.check_btn.setToolTip("Ask the robot whether it wants a sign-in")
        self.check_btn.clicked.connect(lambda: self.probe())
        for b in (self.sign_in_btn, self.sign_out_btn, self.users_btn, self.forget_btn, self.check_btn):
            row.addWidget(b)
        row.addStretch(1)
        v.addLayout(row)
        self._problem = ""

    def set_profile(self, profile) -> None:
        if profile is not self.profile:
            self.gateway = None
            self._problem = ""
        self.profile = profile
        self.refresh()

    def show_problem(self, why: str) -> None:
        self._problem = why
        self.gateway = True
        self.refresh()

    def set_session(self, user: str, role: str) -> None:
        self._problem = ""
        self.gateway = True
        self.refresh()

    def refresh(self) -> None:
        p = self.profile
        if p is None:
            return
        if p.token:
            text = f'<span style="color:{GREEN}">●</span> Signed in as <b>{p.user}</b> ({p.role}: ' \
                   f'{ROLE_TEXT.get(p.role, "")})'
        elif self.gateway:
            text = f'<span style="color:{AMBER}">●</span> This robot asks you to sign in.'
        elif self.gateway is False:
            text = f'<span style="color:{MUTED}">●</span> No sign-in on this robot (its robot side has no gateway).'
        else:
            text = f'<span style="color:{MUTED}">●</span> Sign-in: not checked yet.'
        if self._problem:
            text += f'<br><span style="color:{RED}">{self._problem}</span>'
        if p.use_ssl:
            text += (f'<br><span style="color:{MUTED}">certificate {gw.pretty(p.tls_pin)[:23]}…</span>'
                     if p.tls_pin else f'<br><span style="color:{MUTED}">certificate not known yet</span>')
        self.status.setText(text)
        self.sign_in_btn.setVisible(not p.token and self.gateway is not False)
        self.sign_out_btn.setVisible(bool(p.token))
        self.users_btn.setVisible(bool(p.token) and p.role == "admin")
        self.forget_btn.setVisible(p.use_ssl and bool(p.tls_pin))

    def probe(self, then: Optional[Callable] = None) -> None:
        p = self.profile
        if p is None:
            return

        def done(r, e):
            if p is not self.profile:
                return
            if isinstance(e, gw.GatewayError):
                self._problem = str(e)
                self.gateway = True
            else:
                self.gateway = r is not None
            self.refresh()
            if then is not None:
                then(self.gateway)
        run_bg(lambda: gw.info(p.host, p.port, p.use_ssl, p.tls_pin), done)

    def ask_sign_in(self) -> bool:
        p = self.profile
        if p is None:
            return False
        dlg = SignInDialog(p, self)
        if dlg.exec() == QDialog.Accepted:
            self._problem = ""
            self.gateway = True
            self.refresh()
            self.changed.emit(p)
            return True
        if p.tls_pin:                   # trusted during the attempt: keep it
            self.changed.emit(p)
        return False

    def sign_out(self) -> None:
        p = self.profile
        if p is None or not p.token:
            return
        acct = gw.Account(p)
        run_bg(acct.logout, lambda _r, _e: None)    # the robot forgets it too, if it can be reached
        p.token, p.role = "", ""
        self.refresh()
        self.changed.emit(p)

    def forget_certificate(self) -> None:
        p = self.profile
        if p is None:
            return
        if self.confirm_dialogs and QMessageBox.question(
                self, "Forget the certificate",
                "Trust whatever certificate the robot shows next time? Do this only if it was replaced on "
                "purpose.") != QMessageBox.Yes:
            return
        p.tls_pin = ""
        self._problem = ""
        self.refresh()
        self.changed.emit(p)

    def open_users(self) -> None:
        if self.profile is not None:
            UsersDialog(self.profile, self).exec()


class UsersDialog(QDialog):
    """Admins: the robot's users, and what was done with the robot."""

    def __init__(self, profile, parent=None) -> None:
        super().__init__(parent)
        self.profile = profile
        self.acct = gw.Account(profile)
        self.setWindowTitle(f"Users and audit log — {profile.name}")
        self.resize(860, 560)
        v = QVBoxLayout(self)
        tabs = QTabWidget()
        v.addWidget(tabs)
        # users
        uw = QWidget()
        uv = QVBoxLayout(uw)
        self.users = QTableWidget(0, 4)
        self.users.setHorizontalHeaderLabels(["User", "Role", "Signed in on", "Last seen"])
        self.users.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.users.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.users.setEditTriggers(QAbstractItemView.NoEditTriggers)
        uv.addWidget(self.users, 1)
        row = QHBoxLayout()
        for text, fn in (("Add…", self.add), ("Role…", self.change_role), ("New password…", self.new_password),
                         ("Sign out everywhere", self.sign_out_all), ("Remove", self.remove),
                         ("Refresh", self.load_users)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            row.addWidget(b)
        row.addStretch(1)
        uv.addLayout(row)
        tabs.addTab(uw, "Users")
        # audit
        aw = QWidget()
        av = QVBoxLayout(aw)
        top = QHBoxLayout()
        self.filter = QLineEdit()
        self.filter.setPlaceholderText("filter: user, address or what")
        self.filter.textChanged.connect(self.show_audit)
        self.span = QComboBox()
        for text, days in (("last day", 1), ("last week", 7), ("last month", 31), ("everything", 0)):
            self.span.addItem(text, days)
        self.span.setCurrentIndex(1)
        self.span.currentIndexChanged.connect(lambda _i: self.load_audit())
        exp = QPushButton("Export CSV…")
        exp.clicked.connect(self.export)
        ref = QPushButton("Refresh")
        ref.clicked.connect(self.load_audit)
        for w_ in (self.filter, self.span, ref, exp):
            top.addWidget(w_)
        av.addLayout(top)
        self.audit = QTableWidget(0, 5)
        self.audit.setHorizontalHeaderLabels(["When", "User", "Role", "From", "What"])
        self.audit.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        self.audit.setEditTriggers(QAbstractItemView.NoEditTriggers)
        av.addWidget(self.audit, 1)
        tabs.addTab(aw, "Audit log")
        self.msg = QLabel("")
        self.msg.setWordWrap(True)
        v.addWidget(self.msg)
        close = QDialogButtonBox(QDialogButtonBox.Close)
        close.rejected.connect(self.reject)
        v.addWidget(close)
        self.rows: list = []
        self.user_rows: list = []
        self.load_users()
        self.load_audit()

    # ---- helpers ----

    def say(self, text: str, ok: bool = True) -> None:
        self.msg.setText(f'<span style="color:{GREEN if ok else RED}">{text}</span>')

    def call(self, fn, ok_text: str, then=None) -> None:
        def done(r, e):
            if e is not None:
                self.say(str(e), False)
                return
            if ok_text:
                self.say(ok_text)
            if then:
                then(r)
        run_bg(fn, done)

    def selected(self) -> Optional[str]:
        r = self.users.currentRow()
        return self.users.item(r, 0).text() if r >= 0 and self.users.item(r, 0) else None

    # ---- users ----

    def load_users(self) -> None:
        self.call(self.acct.users, "", self.fill_users)

    def fill_users(self, users: list) -> None:
        self.user_rows = users
        self.users.setRowCount(len(users))
        for i, u in enumerate(users):
            seen = time.strftime("%Y-%m-%d %H:%M", time.localtime(u["last_used"])) if u.get("last_used") else "—"
            for j, val in enumerate((u["name"], u["role"], f"{u['signed_in']} device(s)", seen)):
                self.users.setItem(i, j, QTableWidgetItem(str(val)))

    def add(self) -> None:
        name, ok = QInputDialog.getText(self, "Add a user", "User name:")
        if not ok or not name.strip():
            return
        role, ok = QInputDialog.getItem(self, "Add a user", "Role:", list(ROLES), 1, False)
        if not ok:
            return
        pw, ok = QInputDialog.getText(self, "Add a user", f"Password for {name} (8 characters or more):",
                                      QLineEdit.Password)
        if ok:
            self.call(lambda: self.acct.add_user(name.strip(), role, pw), f"{name} added", lambda _r: self.load_users())

    def change_role(self) -> None:
        name = self.selected()
        if not name:
            return
        role, ok = QInputDialog.getItem(self, "Role", f"Role for {name}:", list(ROLES), 1, False)
        if ok:
            self.call(lambda: self.acct.set_role(name, role), f"{name} is now {role}", lambda _r: self.load_users())

    def new_password(self) -> None:
        name = self.selected()
        if not name:
            return
        pw, ok = QInputDialog.getText(self, "New password", f"New password for {name}:", QLineEdit.Password)
        if ok:
            self.call(lambda: self.acct.set_password(name, pw), f"new password for {name}; signed out everywhere",
                      lambda _r: self.load_users())

    def sign_out_all(self) -> None:
        name = self.selected()
        if name:
            self.call(lambda: self.acct.sign_out_user(name), f"{name} signed out everywhere",
                      lambda _r: self.load_users())

    def remove(self) -> None:
        name = self.selected()
        if not name or QMessageBox.question(self, "Remove", f"Remove {name}?") != QMessageBox.Yes:
            return
        self.call(lambda: self.acct.remove_user(name), f"{name} removed", lambda _r: self.load_users())

    # ---- audit ----

    def load_audit(self) -> None:
        days = self.span.currentData()
        since = time.time() - days * 86400 if days else 0.0
        self.call(lambda: self.acct.audit(since), "", self.fill_audit)

    def fill_audit(self, rows: list) -> None:
        self.rows = list(reversed(rows))        # newest first
        self.show_audit()

    def show_audit(self) -> None:
        f = self.filter.text().strip().lower()
        rows = [r for r in self.rows if not f or f in " ".join(str(r.get(k, "")) for k in
                                                               ("user", "from", "what", "role")).lower()]
        self.audit.setRowCount(len(rows))
        for i, r in enumerate(rows):
            when = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r.get("t", 0)))
            for j, val in enumerate((when, r.get("user", ""), r.get("role", ""), r.get("from", ""), r.get("what", ""))):
                it = QTableWidgetItem(str(val))
                if not r.get("ok", True):
                    it.setForeground(Qt.red)
                self.audit.setItem(i, j, it)

    def export(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Export the audit log",
                                              f"audit_{self.profile.name}_{time.strftime('%Y%m%d')}.csv", "CSV (*.csv)")
        if path:
            self.export_to(path)

    def export_to(self, path: str) -> None:
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time", "user", "role", "from", "what", "ok"])
            for r in self.rows:
                w.writerow([time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r.get("t", 0))), r.get("user"),
                            r.get("role"), r.get("from"), r.get("what"), r.get("ok", True)])
        self.say(f"saved {path}")
