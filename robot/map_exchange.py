"""Copying saved maps between the robot and the app, over the launcher's topics.

No ROS in here, so tests/mock_rosbridge.py runs this very code. The app drives
both directions and sends one request at a time, repeating it when the answer
does not come: a chunk that is lost costs a retry, never a broken map.

  app: {"op": "map_get", "xfer": "...", "name": "office", "seq": 0}
  bot: {"xfer", "seq", "total", "sha256", "data": "<base64>"}

  app: {"op": "map_put", "xfer", "name", "seq", "total", "sha256", "overwrite", "data"}
  bot: {"xfer", "seq", "ok": true}  and on the last chunk also "done": true, "msg"

  bot: {"xfer", "error": "..."}     either way, ends the transfer
"""
from __future__ import annotations

import base64
import binascii
import hashlib
import threading
import time
from pathlib import Path
from typing import Callable, Dict

from map_files import MAX_BLOB, check_name, from_blob, read_map, to_blob, write_map

CHUNK = 192 * 1024      # bytes of map per message, before base64
KEEP_S = 120.0          # a transfer left idle this long is forgotten


class MapExchange:
    def __init__(self, maps: Callable[[], Dict[str, str]], save_dir,
                 in_use: Callable[[str], bool] = lambda _name: False) -> None:
        self.maps = maps                # name -> yaml path
        self.save_dir = Path(save_dir)
        self.in_use = in_use            # Nav2 is running on this map
        self._out: dict = {}            # xfer -> {"blob", "sha", "t"}
        self._in: dict = {}             # xfer -> {"name", "total", "sha", "chunks", "t"}
        self._done: dict = {}           # xfer -> (seq, msg, t): to repeat a lost answer
        self._lock = threading.Lock()

    def handle(self, cmd: dict) -> dict:
        xfer = str(cmd.get("xfer", ""))
        try:
            with self._lock:
                self._forget_old()
                if not xfer:
                    raise ValueError("no transfer id")
                if cmd.get("op") == "map_get":
                    return self._get(xfer, cmd)
                return self._put(xfer, cmd)
        except Exception as e:          # a bad request must not touch the launcher
            self._in.pop(xfer, None)
            return {"xfer": xfer, "error": str(e)}

    def _forget_old(self) -> None:
        limit = time.monotonic() - KEEP_S
        for table in (self._out, self._in):
            for k in [k for k, v in table.items() if v["t"] < limit]:
                del table[k]
        for k in [k for k, v in self._done.items() if v[2] < limit]:
            del self._done[k]

    def _get(self, xfer: str, cmd: dict) -> dict:
        out = self._out.get(xfer)
        if out is None:
            name = str(cmd.get("name", ""))
            path = self.maps().get(name)
            if path is None:
                raise ValueError("map not found on the robot")
            blob = to_blob(name, *read_map(path))
            out = {"blob": blob, "sha": hashlib.sha256(blob).hexdigest(), "t": 0.0}
            self._out[xfer] = out
        out["t"] = time.monotonic()
        blob, seq = out["blob"], int(cmd.get("seq", 0))
        total = max(1, -(-len(blob) // CHUNK))
        if not 0 <= seq < total:
            raise ValueError("no such chunk")
        part = blob[seq * CHUNK:(seq + 1) * CHUNK]
        return {"xfer": xfer, "seq": seq, "total": total, "sha256": out["sha"],
                "data": base64.b64encode(part).decode()}

    def _put(self, xfer: str, cmd: dict) -> dict:
        seq, total = int(cmd.get("seq", -1)), int(cmd.get("total", 0))
        if xfer in self._done and self._done[xfer][0] == seq:
            return {"xfer": xfer, "seq": seq, "ok": True, "done": True, "msg": self._done[xfer][1]}
        st = self._in.get(xfer)
        if st is None:
            if seq != 0:
                raise ValueError("the transfer was interrupted, try again")
            name = check_name(str(cmd.get("name", "")))
            if not 0 < total <= -(-MAX_BLOB // CHUNK):
                raise ValueError("map too large")
            if self.in_use(name):
                raise ValueError(f"Nav2 is running on {name}: stop it before replacing the map")
            if name in self.maps() and not cmd.get("overwrite"):
                raise ValueError(f"the robot already has a map called {name}")
            st = {"name": name, "total": total, "sha": str(cmd.get("sha256", "")), "chunks": [], "t": 0.0}
            self._in[xfer] = st
        st["t"] = time.monotonic()
        if seq == len(st["chunks"]) - 1:        # our answer got lost
            return {"xfer": xfer, "seq": seq, "ok": True}
        if seq != len(st["chunks"]) or seq >= st["total"]:
            raise ValueError("the transfer was interrupted, try again")
        try:
            st["chunks"].append(base64.b64decode(str(cmd.get("data", "")), validate=True))
        except binascii.Error as e:
            raise ValueError("damaged chunk") from e
        if sum(map(len, st["chunks"])) > MAX_BLOB:
            raise ValueError("map too large")
        if len(st["chunks"]) < st["total"]:
            return {"xfer": xfer, "seq": seq, "ok": True}
        blob = b"".join(st["chunks"])
        if hashlib.sha256(blob).hexdigest() != st["sha"]:
            raise ValueError("the map arrived damaged, try again")
        meta, pgm = from_blob(blob)
        write_map(self.save_dir, st["name"], meta, pgm)
        del self._in[xfer]
        msg = f"map {st['name']} saved on the robot"
        self._done[xfer] = (seq, msg, time.monotonic())
        return {"xfer": xfer, "seq": seq, "ok": True, "done": True, "msg": msg}
