"""Logs panel: subscribes to /rosout."""
from __future__ import annotations

import time

from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QColor, QTextCursor
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QHBoxLayout, QLabel, QPlainTextEdit, QPushButton,
    QVBoxLayout, QWidget,
)


_LEVEL = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
_COLOR = {
    "DEBUG": "#8b93a7",
    "INFO": "#c6cad6",
    "WARN": "#ffb547",
    "ERROR": "#ff6a6a",
    "FATAL": "#ff6a6a",
}


class LogsPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._min_level = 20
        self._autoscroll = True
        self._build()

    def _build(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        top = QHBoxLayout()
        self.level = QComboBox()
        self.level.addItems(["DEBUG", "INFO", "WARN", "ERROR", "FATAL"])
        self.level.setCurrentIndex(1)
        self.level.currentIndexChanged.connect(
            lambda i: setattr(self, "_min_level", [10, 20, 30, 40, 50][i]))
        self.autoscroll = QCheckBox("Auto-scroll"); self.autoscroll.setChecked(True)
        self.autoscroll.toggled.connect(lambda b: setattr(self, "_autoscroll", b))
        clear = QPushButton("Clear")
        clear.clicked.connect(lambda: self.text.clear())
        top.addWidget(QLabel("Min level:"))
        top.addWidget(self.level)
        top.addWidget(self.autoscroll)
        top.addStretch(1)
        top.addWidget(clear)
        root.addLayout(top)

        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setStyleSheet("font-family: monospace; font-size: 12px; background:#0b0d12;")
        root.addWidget(self.text, 1)

    def append_log(self, msg: dict) -> None:
        lvl = int(msg.get("level", 20))
        if lvl < self._min_level:
            return
        name = msg.get("name", "?")
        text = msg.get("msg") or msg.get("message") or ""
        lvl_str = _LEVEL.get(lvl, str(lvl))
        color = _COLOR.get(lvl_str, "#c6cad6")
        ts = time.strftime("%H:%M:%S")
        line = f'<span style="color:#5b6378;">{ts}</span>  ' \
               f'<span style="color:{color}; font-weight:600;">[{lvl_str:5}]</span>  ' \
               f'<span style="color:#8b93a7;">{name}</span>  ' \
               f'<span style="color:#e6e8ee;">{text}</span>'
        self.text.appendHtml(line)
        if self._autoscroll:
            self.text.moveCursor(QTextCursor.End)
