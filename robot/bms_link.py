"""Stay connected to the JK-BMS. The link is weak (the BMS board sits inside the
battery), so losing it is normal: this never gives up, and keeps the last
reading with its age so the app can tell old numbers from live ones.

No ROS here. The Bluetooth side is a small backend object, so the tests can drop
the link on purpose.
"""
from __future__ import annotations

import asyncio
import json
import time
from pathlib import Path
from typing import Callable, Optional

import jk_bms

SCAN_S = 12.0           # one scan; repeated until the BMS is heard
RETRY_S = 2.0           # pause after a failure, always the same: no backing off
ASK_AGAIN_S = 5.0       # no frame for this long: ask again
STALE_S = 20.0          # no frame for this long: the link is dead, connect anew
TICK_S = 1.0


class BmsLink:
    def __init__(self, backend, config: Path, address: str = "",
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.backend = backend
        self.config = Path(config)
        self.clock = clock
        self.address = address.lower() or self._saved()
        self.name = ""
        self.state = "searching"    # searching | connecting | connected | no_adapter
        self.error = ""
        self.rssi: Optional[int] = None
        self.heard = 0              # devices heard in the last scan: 0 = deaf radio
        self.reconnects = 0
        self.bad_frames = 0
        self.reading: Optional[dict] = None
        self._reading_t = 0.0
        self._since = 0.0
        self._assembler = jk_bms.Assembler()

    def _saved(self) -> str:
        try:
            return str(json.loads(self.config.read_text()).get("address", "")).lower()
        except (OSError, ValueError, AttributeError):
            return ""

    def _remember(self) -> None:
        """Once found it is this BMS for good: another robot's may be in range."""
        try:
            self.config.write_text(json.dumps({"address": self.address, "name": self.name}))
        except OSError:
            pass

    def status(self) -> dict:
        now = self.clock()
        return {"state": self.state, "address": self.address, "name": self.name, "rssi": self.rssi,
                "heard": self.heard, "reconnects": self.reconnects, "error": self.error,
                "bad_frames": self.bad_frames + self._assembler.bad,
                "connected_s": round(now - self._since) if self.state == "connected" else 0,
                "age": round(now - self._reading_t, 1) if self.reading else None,
                "reading": self.reading}

    def on_data(self, data: bytes) -> None:
        frame = self._assembler.feed(bytes(data))
        if frame is None or frame[4] != jk_bms.TYPE_CELLS:
            return
        try:
            self.reading = jk_bms.parse_cells(frame)
            self._reading_t = self.clock()
            self.error = ""
        except ValueError as e:
            self.bad_frames += 1
            self.error = str(e)

    async def _find(self):
        """Scan until our BMS is heard, whatever that takes."""
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
        client = self.backend.client(device, lost.set)
        try:
            await client.connect()
            await client.start_notify(jk_bms.CHARACTERISTIC, lambda _c, d: self.on_data(d))
            self.address, self.name = device.address.lower(), device.name or self.name
            self._remember()
            await client.write(jk_bms.command(jk_bms.DEVICE_INFO))
            await client.write(jk_bms.command(jk_bms.CELL_INFO))
            self.state = "connected"
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
            raise ConnectionError("the BMS dropped the connection")
        finally:
            await client.close()

    async def run(self) -> None:
        while True:
            try:
                await self._session(await self._find())
            except asyncio.CancelledError:
                raise
            except Exception as e:      # any failure just means: try again
                self.error = str(e) or type(e).__name__
                self.state = "no_adapter" if getattr(e, "no_adapter", False) else "searching"
                self.reconnects += 1
            await asyncio.sleep(RETRY_S)
