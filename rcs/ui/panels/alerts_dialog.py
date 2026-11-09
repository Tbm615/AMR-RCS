"""Remote alerts settings (Events page): Telegram and/or e-mail."""
from __future__ import annotations

from PySide6.QtWidgets import (
    QCheckBox, QDialog, QDialogButtonBox, QFormLayout, QGroupBox, QLabel, QLineEdit, QPushButton, QSpinBox,
    QVBoxLayout,
)

from ...core.alerts import DEFAULTS


class AlertsDialog(QDialog):
    def __init__(self, cfg: dict, send_test, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Remote alerts")
        self.send_test = send_test
        c = {**DEFAULTS, **cfg}
        v = QVBoxLayout(self)
        hint = QLabel("Sent from this PC while RCS runs, for the events of the chosen levels. The same event at "
                      "most once every 10 minutes, at most 20 messages an hour.")
        hint.setWordWrap(True)
        hint.setProperty("muted", True)
        v.addWidget(hint)
        self.on = QCheckBox("Send alerts")
        self.on.setChecked(bool(c["on"]))
        v.addWidget(self.on)
        self.errors = QCheckBox("Errors (E-STOP, failures, the robot lost)")
        self.warnings = QCheckBox("Warnings (low battery, the robot stopped itself, a retried goal…)")
        self.errors.setChecked("error" in c["levels"])
        self.warnings.setChecked("warn" in c["levels"])
        v.addWidget(self.errors)
        v.addWidget(self.warnings)
        tg = QGroupBox("Telegram")
        f = QFormLayout(tg)
        self.tg_token = QLineEdit(c["telegram_bot_token"])
        self.tg_token.setEchoMode(QLineEdit.Password)
        self.tg_token.setPlaceholderText("from @BotFather")
        self.tg_chat = QLineEdit(str(c["telegram_chat_id"]))
        self.tg_chat.setPlaceholderText("your chat or group id")
        f.addRow("Bot token", self.tg_token)
        f.addRow("Chat id", self.tg_chat)
        v.addWidget(tg)
        em = QGroupBox("E-mail")
        f = QFormLayout(em)
        self.host = QLineEdit(c["smtp_host"])
        self.port = QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(int(c["smtp_port"] or 587))
        self.tls = QCheckBox("STARTTLS")
        self.tls.setChecked(bool(c["smtp_tls"]))
        self.user = QLineEdit(c["smtp_user"])
        self.password = QLineEdit(c["smtp_password"])
        self.password.setEchoMode(QLineEdit.Password)
        self.sender = QLineEdit(c["smtp_from"])
        self.to = QLineEdit(c["smtp_to"])
        self.to.setPlaceholderText("one address, or several separated by commas")
        for label, w in (("Server", self.host), ("Port", self.port), ("", self.tls), ("User", self.user),
                         ("Password", self.password), ("From", self.sender), ("To", self.to)):
            f.addRow(label, w)
        v.addWidget(em)
        self.result = QLabel("")
        self.result.setWordWrap(True)
        v.addWidget(self.result)
        test = QPushButton("Send a test message")
        test.clicked.connect(lambda: self.send_test(self.config()))
        v.addWidget(test)
        box = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        box.accepted.connect(self.accept)
        box.rejected.connect(self.reject)
        v.addWidget(box)

    def config(self) -> dict:
        levels = (["error"] if self.errors.isChecked() else []) + (["warn"] if self.warnings.isChecked() else [])
        return {"on": self.on.isChecked(), "levels": levels,
                "telegram_bot_token": self.tg_token.text().strip(), "telegram_chat_id": self.tg_chat.text().strip(),
                "smtp_host": self.host.text().strip(), "smtp_port": self.port.value(), "smtp_tls": self.tls.isChecked(),
                "smtp_user": self.user.text().strip(), "smtp_password": self.password.text(),
                "smtp_from": self.sender.text().strip(), "smtp_to": self.to.text().strip()}
