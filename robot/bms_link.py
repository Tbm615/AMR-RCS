"""Stay connected to the JK-BMS. The link is weak (the BMS board sits inside the
battery), so losing it is normal: this never gives up, and keeps the last
reading with its age so the app can tell old numbers from live ones.

Once the BMS is known it is connected to straight away, by its address, with
no scan first: a scan took up to 12 s on every drop. Only when that fails a
few times in a row does it scan again (the BMS may have a new address, or be
out of reach). After many failures in a row the dongle is opened anew.

Everything that happens to the link (connected, lost and why, what the BMS
sent that is not a frame) is kept as notes for the battery log, so a bad
link can be read back from the robot's files.

No ROS here. The Bluetooth side is a small backend object, so the tests can drop
the link on purpose.
"""
from __future__ import annotations

import asyncio
import json
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from typing import Callable, Optional

import jk_bms

SCAN_S = 12.0           # one scan; repeated until the BMS is heard
RETRY_S = 1.0           # pause after a failure, always the same: no backing off
DIRECT_TRIES = 3        # straight connects to the known address before a scan
RESET_AFTER = 8         # failures in a row before the dongle is opened anew
ASK_AGAIN_S = 5.0       # no frame for this long: ask again
STALE_S = 20.0          # no frame for this long: the link is dead, connect anew
TICK_S = 1.0


class BmsLink:
    def __init__(self, backend, config: Path, address: str = "",
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.backend = backend
        self.config = Path(config)
        self.clock = clock
        saved = self._saved()
        self.address = address.lower() or saved.get("address", "")
        # address with its type as bumble writes it ("98:DA:../P"), to connect without a scan
        self.peer = saved.get("peer", "") if self.address == saved.get("address") else ""
        self.name = saved.get("name", "") if self.address == saved.get("address") else ""
        self.state = "searching"    # searching | connecting | connected | no_adapter
        self.error = ""
        self.rssi: Optional[int] = None
        self.heard = 0              # devices heard in the last scan: 0 = deaf radio
        self.reconnects = 0
        self.bad_frames = 0
        self.failures = 0           # in a row
        self.direct_fails = 0       # straight connects that failed in a row
        self.lost_reasons: dict = {}    # why the link dropped -> how often
        self.notes: deque = deque(maxlen=50)
        self._text_seen = 0
        self.reading: Optional[dict] = None
        self.frame_hex = ""         # the last good frame as received, for the log
        self._reading_t = 0.0
        self._since = 0.0
        self._up = False
        self._assembler = jk_bms.Assembler()

    def _saved(self) -> dict:
        try:
            d = json.loads(self.config.read_text())
            return {"address": str(d.get("address", "")).lower(), "name": str(d.get("name", "")),
                    "peer": str(d.get("peer", ""))}
        except (OSError, ValueError, AttributeError):
            return {}

    def _remember(self) -> None:
        """Once found it is this BMS for good: another robot's may be in range."""
        try:
            self.config.write_text(json.dumps({"address": self.address, "name": self.name,
                                               "peer": self.peer}))
        except OSError:
            pass

    def note(self, text: str) -> None:
        self.notes.append(text)

    def take_notes(self) -> str:
        """What happened since the last call, for one row of the log."""
        out = " | ".join(self.notes)
        self.notes.clear()
        return out

    def status(self) -> dict:
        now = self.clock()
        return {"state": self.state, "address": self.address, "name": self.name, "rssi": self.rssi,
                "heard": self.heard, "reconnects": self.reconnects, "error": self.error,
                "lost_reasons": dict(self.lost_reasons),
                "bad_frames": self.bad_frames + self._assembler.bad,
                "connected_s": round(now - self._since) if self.state == "connected" else 0,
                "age": round(now - self._reading_t, 1) if self.reading else None,
                "reading": self.reading}

    def on_data(self, data: bytes) -> None:
        data = bytes(data)
        if data[:4] != jk_bms.FRAME_START and not self._assembler.buf and self._text_seen < 20:
            # not a frame: the BMS's Bluetooth module talking ("AT\r\n" was seen
            # a few seconds before every drop). Kept for the log, a few per link.
            if data and all(32 <= b < 127 or b in (10, 13) for b in data):
                self._text_seen += 1
                self.note(f"BMS sent text {data.decode('ascii').strip()!r}")
        frame = self._assembler.feed(data)
        if frame is None or frame[4] != jk_bms.TYPE_CELLS:
            return
        try:
            self.reading = jk_bms.parse_cells(frame)
            self.frame_hex = frame.hex()
            self._reading_t = self.clock()
            self.error = ""
        except ValueError as e:
            self.bad_frames += 1
            self.error = str(e)

    async def _find(self):
        """The known BMS straight away; else scan until it is heard, whatever
        that takes."""
        if self.peer and self.direct_fails < DIRECT_TRIES:
            self.state = "connecting"
            return SimpleNamespace(address=self.address, name=self.name, peer=self.peer, direct=True)
        if self.peer:
            self.note(f"{DIRECT_TRIES} direct connects failed, scanning")
        while True:
            self.state = "searching"
            found = await self.backend.scan(SCAN_S, self.address)
            self.heard = found["heard"]
            if found["device"] is not None:
                self.rssi = found["rssi"]
                return found["device"]

    async def _session(self, device) -> None:
        self.state = "connecting"
        self._assembler = jk_bms.Assembler()
        lost = asyncio.Event()
        why: list = []

        def on_lost(reason: str = "") -> None:
            why.append(reason or "the BMS dropped the connection")
            lost.set()

        client = self.backend.client(device, on_lost)
        direct = getattr(device, "direct", False)
        t0 = self.clock()
        try:
            try:
                await client.connect()
            except Exception:
                self.direct_fails += 1
                raise
            self.direct_fails = 0
            await client.start_notify(jk_bms.CHARACTERISTIC, lambda _c, d: self.on_data(d))
            self.address, self.name = device.address.lower(), device.name or self.name
            self.peer = str(getattr(device, "peer", "") or self.peer)
            self._remember()
            self._text_seen = 0
            self.note(f"connected in {self.clock() - t0:.1f} s" + (" (direct)" if direct else ""))
            await client.write(jk_bms.command(jk_bms.DEVICE_INFO))
            await client.write(jk_bms.command(jk_bms.CELL_INFO))
            self.state = "connected"
            self.failures = 0
            self._up = True
            self._since = self.clock()
            start = asked = self.clock()
            while not lost.is_set():
                await asyncio.sleep(TICK_S)
                now = self.clock()
                last = max(self._reading_t, start)
                if now - last > STALE_S:
                    raise TimeoutError(f"no data for {STALE_S:.0f} s")
                if now - last > ASK_AGAIN_S and now - asked > ASK_AGAIN_S:
                    asked = now
                    await client.write(jk_bms.command(jk_bms.CELL_INFO))
            raise ConnectionError(why[0] if why else "the BMS dropped the connection")
        finally:
            await client.close()

    async def run(self) -> None:
        while True:
            self._up = False
            try:
                await self._session(await self._find())
            except asyncio.CancelledError:
                raise
            except Exception as e:      # any failure just means: try again
                self.error = str(e) or type(e).__name__
                self.state = "no_adapter" if getattr(e, "no_adapter", False) else "searching"
                self.reconnects += 1
                self.failures += 1
                if self._up:
                    key = self.error[:60]
                    self.lost_reasons[key] = self.lost_reasons.get(key, 0) + 1
                self.note(f"lost after {self.clock() - self._since:.0f} s: {self.error}" if self._up
                          else f"failed: {self.error}")
                if self.failures % RESET_AFTER == 0 and hasattr(self.backend, "reset"):
                    self.note(f"{self.failures} failures in a row: opening the dongle anew")
                    try:
                        await self.backend.reset()
                    except Exception:
                        pass
            await asyncio.sleep(RETRY_S)
