"""Battery: JK-BMS frames, staying connected over a bad link, the Battery page.

Run:
    python tests/test_battery.py

Needs no BMS: frames are built here from the layout, and a fake Bluetooth
backend loses the link on purpose. This does not prove the layout matches a
given BMS: parse_cells refuses a frame that does not add up, for that reason.
"""
from __future__ import annotations

import asyncio
import json
import os
import struct
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "robot"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_batt_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]

from PySide6.QtWidgets import QApplication

import bms_link
import jk_bms
from bms_link import BmsLink
from rcs.core.battery import alarm, link_text, live
from rcs.ui.main_window import MainWindow

app = QApplication(sys.argv)
PASS = FAIL = 0
TMP = Path(tempfile.mkdtemp(prefix="rcs_batt_"))
bms_link.SCAN_S, bms_link.RETRY_S, bms_link.TICK_S = 0.01, 0.01, 0.01
bms_link.ASK_AGAIN_S, bms_link.STALE_S = 0.05, 0.15


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def frame(cells=(3.300,) * 16, soc=78, voltage=None, ftype=2, current=-4.5) -> bytes:
    f = bytearray(300)
    f[:4] = jk_bms.FRAME_START
    f[4] = ftype
    for i, c in enumerate(cells):
        struct.pack_into("<H", f, 6 + 2 * i, round(c * 1000))
    v = sum(cells) if voltage is None else voltage
    struct.pack_into("<i", f, 150, round(v * 1000))
    struct.pack_into("<i", f, 154, round(abs(v * current) * 1000))
    struct.pack_into("<i", f, 158, round(current * 1000))
    struct.pack_into("<h", f, 144, 312)
    struct.pack_into("<h", f, 162, 255)
    struct.pack_into("<h", f, 164, -30)
    f[173] = soc
    struct.pack_into("<I", f, 174, 78000)
    struct.pack_into("<I", f, 178, 100000)
    struct.pack_into("<I", f, 182, 42)
    f[190], f[198], f[199] = 97, 1, 1
    f[299] = sum(f[:299]) & 0xFF
    return bytes(f)


def raises(fn) -> bool:
    try:
        fn()
    except ValueError:
        return True
    return False


def test_frames() -> None:
    print("\n[frames]")
    c = jk_bms.command(jk_bms.CELL_INFO)
    check("command as the BMS expects", len(c) == 20 and c[:5] == b"\xaa\x55\x90\xeb\x96" and c[19] == sum(c[:19]) & 255)
    check("read commands only: settings refused", raises(lambda: jk_bms.command(0x95)))
    check("nor anything else", raises(lambda: jk_bms.command(0x00)))
    r = jk_bms.parse_cells(frame(current=-4.5))
    check("values", (r["soc"], r["current"], r["cycles"], r["soh"]) == (78, -4.5, 42, 97), f"{r}")
    check("voltage", abs(r["voltage"] - 52.8) < 0.01 and len(r["cells"]) == 16)
    check("signed temperature", r["temps"] == [25.5, -3.0] and r["mos_temp"] == 31.2)
    check("capacity", (r["remaining_ah"], r["nominal_ah"]) == (78.0, 100.0))
    check("cell difference", jk_bms.parse_cells(frame(cells=(3.3,) * 15 + (3.25,)))["delta"] == 0.05)
    check("another layout: pack does not match its cells", raises(lambda: jk_bms.parse_cells(frame(voltage=12.0))))
    check("SOC impossible", raises(lambda: jk_bms.parse_cells(frame(soc=180))))
    check("cell impossible", raises(lambda: jk_bms.parse_cells(frame(cells=(9.0,) * 16))))
    check("no cells", raises(lambda: jk_bms.parse_cells(frame(cells=(), voltage=0.0))))
    check("wrong frame type", raises(lambda: jk_bms.parse_cells(frame(ftype=3))))
    check("JK by name", jk_bms.is_jk("JK_B2A20S", "aa:bb:cc:00:00:00"))
    check("JK by address", jk_bms.is_jk(None, "C8:47:80:00:00:01"))
    check("JK by service", jk_bms.is_jk(None, "aa:bb:cc:00:00:00", [jk_bms.SERVICE.upper()]))
    check("not a phone", not jk_bms.is_jk("Pixel", "aa:bb:cc:00:00:00"))

    print("\n[pieces over a weak link]")
    f = frame()
    a = jk_bms.Assembler()
    out = [a.feed(f[i:i + 20]) for i in range(0, 300, 20)]
    check("20-byte pieces", out[-1] == f and not any(out[:-1]))
    a = jk_bms.Assembler()
    check("one piece", a.feed(f) == f)
    a = jk_bms.Assembler()
    bad = [f[i:i + 20] for i in range(0, 300, 20)]
    del bad[5]                                  # lost in the air
    check("lost piece gives nothing", not any(a.feed(p) for p in bad))
    check("the next frame still works", a.feed(f) == f)
    a = jk_bms.Assembler()
    check("tail of a lost frame is ignored", a.feed(f[40:60]) is None and not a.buf)
    wrong = bytearray(f)
    wrong[100] ^= 0xFF
    a = jk_bms.Assembler()
    check("damaged frame refused", a.feed(bytes(wrong)) is None)
    a.feed(bytes(20))
    check("garbage after it does not stick", a.feed(f) == f)


class Client:
    def __init__(self, backend, device, on_lost):
        self.b, self.device, self.on_lost = backend, device, on_lost
        self.cb = None

    async def connect(self):
        self.b.attempts += 1
        if self.b.connect_failures > 0:
            self.b.connect_failures -= 1
            raise TimeoutError("connect timed out")

    async def start_notify(self, _char, cb):
        self.cb = cb
        self.b.live = self

    async def write(self, data):
        self.b.writes.append(data)
        if self.b.answer and data[4] == jk_bms.CELL_INFO:
            self.cb(None, frame(soc=self.b.soc))

    async def close(self):
        self.b.closed += 1


class Backend:
    def __init__(self, device):
        self.device, self.attempts, self.closed = device, 0, 0
        self.connect_failures, self.answer, self.soc = 0, True, 78
        self.writes, self.live, self.scans, self.asked_for = [], None, 0, []
        self.hidden = 0

    async def scan(self, _s, address):
        self.scans += 1
        self.asked_for.append(address)
        await asyncio.sleep(0)
        if self.hidden > 0:
            self.hidden -= 1
            return {"heard": 3, "device": None, "rssi": None}
        return {"heard": 4, "device": self.device, "rssi": -88}

    def client(self, device, on_lost):
        return Client(self, device, on_lost)


async def until(cond, timeout=3.0) -> bool:
    for _ in range(int(timeout / 0.005)):
        if cond():
            return True
        await asyncio.sleep(0.005)
    return False


async def test_link() -> None:
    print("\n[staying connected]")
    dev = SimpleNamespace(address="C8:47:80:AA:BB:CC", name="JK_B2A")
    cfg = TMP / "bms.json"
    b = Backend(dev)
    b.hidden, b.connect_failures = 5, 3
    link = BmsLink(b, cfg)
    task = asyncio.create_task(link.run())
    check("keeps searching and retrying until it gets through",
          await until(lambda: link.state == "connected" and link.reading), f"{link.status()}")
    check("heard after 5 empty scans, 3 failed connects", b.scans >= 9 and b.attempts == 4, f"{b.scans} {b.attempts}")
    check("asks device info, then cells", [w[4] for w in b.writes[:2]] == [0x97, 0x96])
    check("reading", link.reading["soc"] == 78 and link.status()["age"] is not None)
    check("only read commands ever sent", {w[4] for w in b.writes} <= {0x96, 0x97})
    check("BMS remembered", json.loads(cfg.read_text())["address"] == dev.address.lower())

    print("\n[the BMS drops the link]")
    n = link.reconnects
    b.soc = 77
    b.live.on_lost()
    check("reconnects by itself", await until(lambda: link.reconnects == n + 1 and link.state == "connected"
                                              and link.reading["soc"] == 77), f"{link.status()}")
    check("connection closed before the next", b.closed >= 1)
    check("looks for that BMS only", b.asked_for[-1] == dev.address.lower())

    print("\n[connected but silent, the worst case of a weak link]")
    b.answer = False
    n, w = link.reconnects, len(b.writes)
    await until(lambda: len(b.writes) > w + 2, 0.5)
    check("asks again first", len(b.writes) > w and b.writes[w][4] == 0x96)
    check("gives the connection up when it stays silent", await until(lambda: link.reconnects > n))
    check("old reading kept, with its age", link.reading is not None and link.status()["age"] > 0.1)
    b.answer, b.soc = True, 75
    check("back when data flows", await until(lambda: link.state == "connected" and link.reading["soc"] == 75))

    print("\n[a bad frame]")
    link.on_data(frame(voltage=12.0))
    check("not shown", link.reading["soc"] == 75 and link.bad_frames == 1 and "does not match" in link.error)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    print("\n[after a restart]")
    b2 = Backend(dev)
    l2 = BmsLink(b2, cfg)
    task = asyncio.create_task(l2.run())
    await until(lambda: b2.scans)
    check("goes straight for the same BMS", b2.asked_for[0] == dev.address.lower())
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    check("address given by hand wins", BmsLink(Backend(dev), cfg, "AA:BB:CC:DD:EE:FF").address == "aa:bb:cc:dd:ee:ff")
    check("broken file is no problem", BmsLink(Backend(dev), TMP / "nope.json").address == "")


def report(**kw) -> dict:
    return {"state": "connected", "name": "JK_B2A", "address": "c8", "heard": 4, "reconnects": 2,
            "connected_s": 60, "error": "", "age": 1.0, "reading": jk_bms.parse_cells(frame()), **kw}


def test_page() -> None:
    print("\n[what the operator reads]")
    check("connected", link_text(report())[1] == "ok" and "2 reconnects" in link_text(report())[0])
    check("no service", link_text(None)[1] == "error" and "service" in link_text(None)[0])
    deaf = {"state": "searching", "heard": 0}
    check("deaf radio points at the antenna", "antenna" in link_text(deaf)[0], link_text(deaf)[0])
    check("known BMS out of reach mentions the phone", "phone" in link_text({"state": "searching", "heard": 5, "address": "c8"})[0])
    check("frames refused say why", "does not match" in link_text(report(age=None, reading=None, error="does not match"))[0])
    check("old reading is not live", not live(report(age=60.0)) and live(report()))
    check("alarm levels", (alarm(50), alarm(20), alarm(10), alarm(None)) == ("", "warn", "error", ""))

    print("\n[Battery page]")
    w = MainWindow()
    w.show()
    w._on_battery(report())
    p = w.p_battery
    check("shows charge", p.soc.text() == "78%" and "52.80 V" in p.v["voltage"].value.text())
    check("cells drawn", len(p.bars) == 16)
    check("status bar", "78%" in w.sb_battery.text())
    w._on_battery(report(age=90.0))
    check("old value marked on the status bar", "old" in w.sb_battery.text())
    check("and on the page", "90 s ago" in p.mos.text())
    n = len(w.events.events)
    low = report(reading=jk_bms.parse_cells(frame(soc=18)))
    w._on_battery(low)
    w._on_battery(low)
    check("low battery logged once", sum("battery at 18%" in e.text for e in w.events.events) == 1)
    w._on_battery(report(reading=jk_bms.parse_cells(frame(soc=9))))
    check("critical logged as an error", any(e.level == "error" and "9%" in e.text for e in w.events.events))
    w._on_battery(report(age=60.0, reading=jk_bms.parse_cells(frame(soc=5))))
    check("an old reading raises no alarm", not any("5%" in e.text for e in list(w.events.events)[n:]))
    w._on_battery(report(reading=None, age=None, state="searching", heard=0))
    check("nothing yet", p.soc.text() == "—" and "antenna" in p.link.text())
    check("no F key for this page: F12 stays E-STOP", not w.btns["battery"].toolTip().startswith("F"))
    w.close()


def main() -> int:
    test_frames()
    asyncio.run(test_link())
    test_page()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    os._exit(code)
