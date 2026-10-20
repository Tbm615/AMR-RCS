"""Camera stream panel. Takes frames as QImage (decoded off the GUI thread
by RosClient) or as JPEG bytes."""
from __future__ import annotations

from PySide6.QtCore import QByteArray, Qt
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget


class CameraPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.image_lbl = QLabel("no camera stream")
        self.image_lbl.setAlignment(Qt.AlignCenter)
        self.image_lbl.setMinimumSize(640, 480)
        self.image_lbl.setStyleSheet("background:#000; color:#5b6378;")
        self.hz_lbl = QLabel("—"); self.hz_lbl.setProperty("muted", True)
        v = QVBoxLayout(self)
        v.setContentsMargins(16, 16, 16, 16)
        v.addWidget(self.image_lbl, 1)
        h = QHBoxLayout(); h.addWidget(QLabel("Topic rate:")); h.addWidget(self.hz_lbl); h.addStretch(1)
        v.addLayout(h)
        self._last_pix: QPixmap | None = None

    def set_frame(self, data) -> None:
        if isinstance(data, QImage):
            if data.isNull():
                return
            self._last_pix = QPixmap.fromImage(data)
            self._render()
            return
        pm = QPixmap()
        if pm.loadFromData(QByteArray(data), "JPG"):
            self._last_pix = pm
            self._render()

    def set_rate(self, hz: float) -> None:
        self.hz_lbl.setText(f"{hz:.1f} Hz" if hz > 0.1 else "—")

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._render()

    def _render(self) -> None:
        if not self._last_pix:
            return
        if not self.isVisible():
            return
        self.image_lbl.setPixmap(self._last_pix.scaled(
            self.image_lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
