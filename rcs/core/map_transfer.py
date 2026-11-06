"""Copy a map between this PC's library and the robot (robot/map_exchange.py).

One request at a time, the next only after the answer, so nothing can overrun
the launcher's queue, and a lost message is simply asked for again.
"""
from __future__ import annotations

import base64
import binascii
import hashlib
import uuid
from typing import Optional

from PySide6.QtCore import QObject, QTimer, Signal

from .map_files import from_blob, to_blob
from .maps import MapLibrary

CHUNK = 192 * 1024
ANSWER_MS = 6000
TRIES = 4


class MapTransfer(QObject):
    progress = Signal(int, int)         # chunks done, total
    finished = Signal(bool, str)        # ok, message

    def __init__(self, ros, library: MapLibrary, parent=None) -> None:
        super().__init__(parent)
        self.ros = ros
        self.library = library
        self.answer_ms = ANSWER_MS
        self._xfer = ""
        self._mode = ""                 # "get" | "put"
        self._name = ""
        self._seq = 0
        self._tries = 0
        self._parts: list = []          # get: received; put: to send
        self._sha = ""
        self._overwrite = False
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._no_answer)
        ros.map_data.connect(self._on_data)
        ros.disconnected.connect(lambda: self._end(False, "connection lost, map not copied"))

    @property
    def busy(self) -> bool:
        return bool(self._xfer)

    def download(self, name: str) -> None:
        self._begin("get", name)
        self._parts = []
        self._ask()

    def upload(self, name: str, overwrite: bool = False) -> None:
        try:
            meta, pgm = self.library.read(name)
        except (OSError, ValueError) as e:
            self.finished.emit(False, f"cannot read map {name}: {e}")
            return
        blob = to_blob(name, meta, pgm)
        self._begin("put", name)
        self._overwrite = overwrite
        self._sha = hashlib.sha256(blob).hexdigest()
        self._parts = [blob[i:i + CHUNK] for i in range(0, len(blob), CHUNK)]
        self._ask()

    def _begin(self, mode: str, name: str) -> None:
        self._xfer = uuid.uuid4().hex[:12]
        self._mode, self._name, self._seq, self._tries = mode, name, 0, 0

    def _ask(self) -> None:
        if self._mode == "get":
            self.ros.send_launcher_cmd("map_get", xfer=self._xfer, name=self._name, seq=self._seq)
        else:
            self.ros.send_launcher_cmd(
                "map_put", xfer=self._xfer, name=self._name, seq=self._seq, total=len(self._parts),
                sha256=self._sha, overwrite=self._overwrite,
                data=base64.b64encode(self._parts[self._seq]).decode())
        self._timer.start(self.answer_ms)

    def _no_answer(self) -> None:
        self._tries += 1
        if self._tries >= TRIES:
            self._end(False, "no answer from the robot, map not copied")
        else:
            self._ask()

    def _next(self) -> None:
        self._seq += 1
        self._tries = 0
        self._ask()

    def _on_data(self, ans: dict) -> None:
        if not self._xfer or ans.get("xfer") != self._xfer:
            return
        if "error" in ans:
            self._end(False, str(ans["error"]))
        elif ans.get("seq") != self._seq:
            return                      # an answer to a request we repeated
        elif self._mode == "get":
            self._on_chunk(ans)
        elif ans.get("done"):
            self._end(True, str(ans.get("msg", "map sent")))
        else:
            self.progress.emit(self._seq + 1, len(self._parts))
            self._next()

    def _on_chunk(self, ans: dict) -> None:
        try:
            self._parts.append(base64.b64decode(str(ans.get("data", "")), validate=True))
            total = int(ans.get("total", 0))
        except (binascii.Error, TypeError, ValueError):
            self._end(False, "the map arrived damaged")
            return
        self.progress.emit(self._seq + 1, total)
        if len(self._parts) < total:
            self._next()
            return
        blob = b"".join(self._parts)
        if hashlib.sha256(blob).hexdigest() != ans.get("sha256"):
            self._end(False, "the map arrived damaged")
            return
        try:
            final, new = self.library.add(self._name, *from_blob(blob))
        except (OSError, ValueError) as e:
            self._end(False, f"cannot keep map {self._name}: {e}")
            return
        if not new:
            text = f"this PC already has map {final}"
        elif final != self._name:
            text = f"map {self._name} kept as {final}: another map has that name"
        else:
            text = f"map {final} copied to this PC"
        self._end(True, text)

    def _end(self, ok: bool, text: str) -> None:
        if not self._xfer:
            return
        self._xfer = ""
        self._parts = []
        self._timer.stop()
        self.finished.emit(ok, text)
