"""Battery: JK-BMS frames, staying connected over a bad link, the battery log
on the robot (a file per boot, kept for good), its history in the app, the
Battery page.

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
import time
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
import base64
import zlib

import battery_log
import hci_quirks
import jk_bms
from battery_log import BootLog, LogExchange
from bms_link import BmsLink
from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QImage
from rcs.core.battery import alarm, link_text, live
from rcs.core.battery_history import BatteryLogs, how_it_ended, series, trend
from rcs.ui.main_window import MainWindow
from rcs.ui.widgets.chart import TimeChart, nice_ticks, time_ticks

app = QApplication(sys.argv)
PASS = FAIL = 0
TMP = Path(tempfile.mkdtemp(prefix="rcs_batt_"))
# a cell info frame as received from the robot's BMS
REAL_SOC = 77
REAL_FRAME = (
    "55aaeb9002b3dc0ce00cd80ce00cdd0cd80cdd0cdb0cde0ce10cde0cdd0cdb0cdc0ce30c000000000000000000000000"
    "00000000000000000000000000000000000000000000ff7f0000dd0c09000e02b100b400b500b300b200b400b200b000"
    "b100af00b100b800b800b500b50000000000000000000000000000000000000000000000000000000000000000000000"
    "820100000000f1c000006e6c0000cefdffff49014901000000000000004dfc3a0000314c00001b0000004e3e08006200"
    "00008f84fe0101010000000000000000000000000000ff0001000000a928000009002b4e3f40000000004b1300000000"
    "000101010100a68b380000000000820130f830f8b2282f2be20cee0500008051010000000100000000000000000000fe"
    "ff7fdc0f0108800b010000bc"
)
bms_link.SCAN_S, bms_link.RETRY_S, bms_link.TICK_S = 0.02, 0.02, 0.02
bms_link.ASK_AGAIN_S, bms_link.STALE_S = 0.1, 0.3


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
    check("JK renamed in its app", jk_bms.is_jk("AMR Battery", "98:da:20:0b:36:a1"))

    print("\n[a real frame, JK_BD6A17S6P hw 19H sw 19.01]")
    real = bytes.fromhex(REAL_FRAME)
    r = jk_bms.parse_cells(real)
    check("15 cells of LiFePO4", len(r["cells"]) == 15 and all(3.2 < c < 3.4 for c in r["cells"]))
    check("pack matches its cells", abs(r["voltage"] - sum(r["cells"])) < 0.05, f"{r['voltage']}")
    check("as the BMS read", (r["soc"], r["soh"]) == (REAL_SOC, 98) and r["current"] < 0 and r["cycles"] >= 0, f"{r}")
    check("all of it: average, highest and lowest cell", (r["cell_avg"], r["cell_max_no"], r["cell_min_no"])
          == (3.293, 15, 3), f"{r.get('cell_avg')} {r.get('cell_max_no')} {r.get('cell_min_no')}")
    check("a resistance per cell", len(r["resistances"]) == 15 and all(0.1 < x < 0.3 for x in r["resistances"]))
    check("capacity through it, matches cycles x nominal",
          abs(r["total_cycle_ah"] / r["nominal_ah"] - r["cycles"]) < 1.5, f"{r['total_cycle_ah']}")
    check("balancing, alarms, running time", (r["balancing"], r["alarms"], r["wire_alarms"]) == (0, 0, 0)
          and r["runtime_s"] > 86400)
    made_up = jk_bms.parse_cells(frame())
    check("a frame whose average does not add up: no extra fields", "resistances" not in made_up
          and made_up["soc"] == 78)

    print("\n[clone dongle]")
    ext = hci_quirks.answer(bytes.fromhex("01041001") + b"\x00")
    check("unanswered command gets an answer", ext[:6] == bytes([4, 0x0E, 14, 1, 0x04, 0x10]) and ext[6] == 0, ext.hex())
    check("refused one too", hci_quirks.answer(bytes.fromhex("0105200 6".replace(" ", "")) + bytes(6))[6] == 0)
    check("reset goes to the dongle", hci_quirks.answer(bytes.fromhex("01030c00")) is None)
    check("data goes to the dongle", hci_quirks.answer(bytes.fromhex("0200200000")) is None)

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
        self.b.connected_to.append(getattr(self.device, "peer", None))
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
        self.resets = 0
        self.connected_to = []

    async def reset(self):
        self.resets += 1

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
    # by the clock, not by counting: on Windows a sleep this short returns at once
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        await asyncio.sleep(0.02)
    return bool(cond())


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
    await until(lambda: len(b.writes) > w, 1.0)
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


async def test_direct() -> None:
    print("\n[back fast: straight to the known BMS, no scan]")
    dev = SimpleNamespace(address="98:DA:20:0B:36:A1", name="AMR Battery", peer="98:DA:20:0B:36:A1/P")
    cfg = TMP / "bms_direct.json"
    b = Backend(dev)
    link = BmsLink(b, cfg)
    task = asyncio.create_task(link.run())
    check("found by a scan the first time", await until(lambda: link.state == "connected") and b.scans == 1)
    check("its address and type remembered", json.loads(cfg.read_text())["peer"] == dev.peer)
    scans = b.scans
    for i in range(5):
        n = link.reconnects
        b.live.on_lost("link lost: CONNECTION_TIMEOUT_ERROR (8)")
        await until(lambda: link.reconnects == n + 1 and link.state == "connected")
    check("5 drops, 5 reconnects, not one scan", b.scans == scans and link.state == "connected",
          f"scans {b.scans - scans}")
    check("connects to the remembered address", b.connected_to[-1] == dev.peer)
    check("why it dropped is counted", link.lost_reasons.get("link lost: CONNECTION_TIMEOUT_ERROR (8)") == 5,
          f"{link.lost_reasons}")
    notes = link.take_notes()
    check("notes for the log say what happened", "lost after" in notes and "(direct)" in notes, notes)
    check("and are given once", link.take_notes() == "")
    link.on_data(b"AT\r\n")
    check("text from the BMS's module is noted", "'AT'" in link.take_notes())
    check("and is not taken for a frame", link.reading["soc"] == 78 and link.bad_frames == 0)

    print("\n[the remembered address stops working]")
    scans = b.scans
    b.connect_failures = bms_link.DIRECT_TRIES
    n = link.reconnects
    b.live.on_lost()
    check("after a few failed direct connects, a scan", await until(lambda: b.scans > scans
                                                                    and link.state == "connected"),
          f"{link.status()}")
    check("tried directly first", link.reconnects >= n + bms_link.DIRECT_TRIES)

    print("\n[a dongle that stops answering]")
    b.connect_failures = bms_link.RESET_AFTER + 1
    b.live.on_lost()
    check("opened anew after failures in a row", await until(lambda: b.resets >= 1, 5.0), f"{b.resets}")
    check("and back", await until(lambda: link.state == "connected", 5.0))
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    check("a restart connects with no scan", BmsLink(Backend(dev), cfg).peer == dev.peer)


def status(soc=77, age=0.5, state="connected", reading=True) -> dict:
    r = jk_bms.parse_cells(bytes.fromhex(REAL_FRAME)) if reading else None
    if r:
        r = dict(r, soc=soc)
    return {"state": state, "age": age if reading else None, "reading": r, "reconnects": 2, "rssi": -84,
            "connected_s": 30, "bad_frames": 0, "error": ""}


def test_log() -> None:
    print("\n[battery log: a file per boot, a row a second]")
    folder = TMP / "battlog"
    t = [1_700_000_000.0]
    up = [100.0]

    def clock():
        return t[0]

    def uptime():
        return up[0]

    def step(log, n, **kw):
        for _ in range(n):
            t[0] += 1
            up[0] += 1
            log.write(status(**kw), "", REAL_FRAME)

    a = BootLog(folder, boot="aaaa1111", clock=clock, up=uptime)
    step(a, 3)
    rows = battery_log.read_rows(a.path)
    check("on the disk at once, before any close", len(rows) == 3)
    check("named after the boot", a.name.startswith("battery_") and a.name.endswith("_aaaa1111.csv"), a.name)
    r = rows[0]
    check("every cell and its resistance", len(r["cells"].split(";")) == 15
          and len(r["resistances"].split(";")) == 15, r["resistances"])
    check("all the rest", all(r[k] != "" for k in ("soc", "voltage", "current", "power", "delta", "cell_avg",
                                                    "temp1", "temp2", "mos_temp", "remaining_ah", "nominal_ah",
                                                    "cycles", "total_cycle_ah", "soh", "charging",
                                                    "balancing", "alarms", "runtime_s", "rssi")), f"{r}")
    check("the raw frame, once a minute", rows[0]["frame"] == REAL_FRAME and rows[1]["frame"] == "")
    step(a, 2, reading=False, state="searching")
    rows = battery_log.read_rows(a.path)
    check("BMS not heard: the row is still written", len(rows) == 5 and rows[-1]["state"] == "searching")
    check("with no old numbers in it", rows[-1]["soc"] == "" and rows[-1]["cells"] == "")
    step(a, 1, age=40.0)
    check("a reading too old is not written as current", battery_log.read_rows(a.path)[-1]["soc"] == "")
    a._f.close()                # power cut: no last row

    print("\n[the next boot]")
    up[0] = 20.0
    b = BootLog(folder, boot="bbbb2222", clock=clock, up=uptime)
    check("a new file", b.path != a.path)
    prev = b.previous
    check("knows the last boot did not stop cleanly", prev and not prev["clean_end"], f"{prev}")
    check("and how the battery was then", prev["last"]["soc"] == 77 and prev["last"]["voltage"] > 40)
    check("said plainly", "power cut" in how_it_ended(prev), how_it_ended(prev))
    step(b, 2)
    b.close()
    c = BootLog(folder, boot="cccc3333", clock=clock, up=uptime)
    check("a clean shut down is told apart", c.previous["clean_end"] and "shut down" in how_it_ended(c.previous))
    step(c, 1)
    c2 = BootLog(folder, boot="cccc3333", clock=clock, up=uptime)
    step(c2, 1)
    rows = battery_log.read_rows(c2.path)
    check("service restarted in the same boot: same file, one header", c2.path == c.path and len(rows) == 2)
    with open(c2.path, "a") as f:
        f.write("1700000999.0,2023-")        # cut short by the power going
    check("a last line cut short is dropped", len(battery_log.read_rows(c2.path)) == 2)

    print("\n[kept for good]")
    for i in range(40):
        x = BootLog(folder, boot=f"{i:08x}", clock=clock, up=uptime)
        step(x, 1)
        x.close()
    files = list(folder.glob("battery_*.csv"))
    check("no file is ever deleted by itself", len(files) == 43, f"{len(files)}")
    check("disk use is reported", x.disk()["files"] == 43 and x.disk()["free"] > 0)

    print("\n[the app asks for them]")
    ex = LogExchange(c2)
    ans = ex.handle({"op": "log_list", "req": "1"})
    names = [f["name"] for f in ans["files"]]
    check("listed with how each went", len(names) == 43 and ans["current"] == c2.name
          and any(not f["clean_end"] for f in ans["files"]), f"{len(names)}")
    first = next(f for f in ans["files"] if f["name"] == a.name)
    check("a summary per boot", first["rows"] == 6 and first["soc_end"] == 77 and first["v_min"] > 40
          and first["delta_max"] > 0, f"{first}")
    old_chunk = battery_log.CHUNK
    battery_log.CHUNK = 800         # rows are 300-1000 bytes (the raw frame once a minute)
    got, offset, pieces, whole = b"", 0, 0, True
    while pieces < 50:
        g = ex.handle({"op": "log_get", "req": "2", "name": a.name, "offset": offset})
        part = zlib.decompress(base64.b64decode(g["data"]))
        whole = whole and (part.endswith(b"\n") or g["eof"])
        got += part
        offset += len(part)
        pieces += 1
        if g["eof"]:
            break
    battery_log.CHUNK = old_chunk
    check("fetched in pieces", got == a.path.read_bytes() and pieces >= 2, f"{pieces}")
    check("each of whole lines", whole)
    torn = BootLog(folder, boot="cccc3333", clock=clock, up=uptime)
    torn.write(status())
    rows = battery_log.read_rows(torn.path)
    check("after a row torn by a kill, the next rows are whole", len(rows) == 3 and rows[-1]["soc"] == "77",
          f"{len(rows)}")
    check("refuses another folder", "error" in ex.handle({"op": "log_get", "req": "3", "name": "../bms.json"}))
    check("and any other file", "error" in ex.handle({"op": "log_get", "req": "3", "name": "estop.json"}))
    check("the running boot's file cannot be deleted",
          "cannot be deleted" in ex.handle({"op": "log_delete", "req": "4", "name": c2.name}).get("error", ""))
    d = ex.handle({"op": "log_delete", "req": "5", "name": b.name})
    check("an old one can, when asked", d.get("deleted") == b.name and not b.path.exists()
          and not b.path.with_suffix(".summary.json").exists())

    print("\n[watchdog]")
    from battery_log import Watchdog
    exits = []
    dog = Watchdog(torn, timeout=0.05, exit_fn=exits.append)
    dog.alive(status())
    check("quiet while the service goes round", not dog.check())
    time.sleep(0.1)
    check("ends a hung service", dog.check() and exits == [3])
    check("and says so in the log", "watchdog" in battery_log.read_rows(torn.path)[-1]["note"])


class FakeRos(QObject):
    """The robot's battery service, answering through the real LogExchange."""
    battery_log = Signal(object)
    disconnected = Signal()

    def __init__(self, exchange):
        super().__init__()
        self.exchange = exchange
        self.drop = 0
        self.sent = []

    def is_connected(self):
        return True

    def send_battery_cmd(self, op, **kw):
        cmd = {"op": op, **kw}
        self.sent.append(cmd)
        if self.drop > 0:
            self.drop -= 1
            return kw.get("req")
        ans = self.exchange.handle(cmd)
        QTimer.singleShot(0, lambda: self.battery_log.emit(ans))
        return kw.get("req")


def wait(cond, timeout=5.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.01)
    return bool(cond())


def test_history() -> None:
    print("\n[history in the app]")
    folder = TMP / "battlog2"
    t = [1_700_100_000.0]
    up = [10.0]
    old = BootLog(folder, boot="dddd4444", clock=lambda: t[0], up=lambda: up[0])
    for i in range(120):
        t[0] += 1
        up[0] += 1
        old.write(status(soc=90 - i // 10))
    old._f.close()          # power cut
    up[0] = 10.0
    cur = BootLog(folder, boot="eeee5555", clock=lambda: t[0], up=lambda: up[0])
    for i in range(30):
        t[0] += 1
        up[0] += 1
        cur.write(status(soc=60))
    ros = FakeRos(LogExchange(cur))
    logs = BatteryLogs(ros, TMP / "pc_copies")
    logs.answer_ms = 100
    got = {}
    logs.listed.connect(lambda d: got.setdefault("list", d))
    logs.loaded.connect(lambda n, rows: got.__setitem__(n, rows))
    logs.refresh_list()
    check("boots listed", wait(lambda: "list" in got) and len(got["list"]["files"]) == 2)
    logs.load(old.name)
    check("a boot fetched", wait(lambda: old.name in got) and len(got[old.name]) == 120)
    check("kept on this PC", (TMP / "pc_copies" / old.name).read_bytes() == old.path.read_bytes())
    n = len(ros.sent)
    got.pop(old.name)
    logs.load(old.name)
    check("complete already: not asked again", wait(lambda: old.name in got) and len(ros.sent) == n)
    logs.load(cur.name)
    check("the running boot", wait(lambda: cur.name in got) and len(got[cur.name]) == 30)
    for i in range(10):
        t[0] += 1
        up[0] += 1
        cur.write(status(soc=59))
    got.pop(cur.name)
    logs.load(cur.name)
    check("then only what was added", wait(lambda: cur.name in got) and len(got[cur.name]) == 40
          and ros.sent[-1]["offset"] > 0, f"{ros.sent[-1]}")
    ts, vs = series(got[cur.name], "soc")
    check("as a series", len(ts) == 40 and vs[-1] == 59)
    ts, vs = series(got[cur.name], "delta")
    check("cell difference in mV", 5 < vs[0] < 20, f"{vs[0]}")
    ts, vs = trend(got["list"]["files"], "soc")
    check("a trend over boots", len(ts) == 2 and vs[0] == 79, f"{vs}")
    ros.drop = 1
    got.pop("list")
    logs.refresh_list()
    check("a lost answer is asked again", wait(lambda: "list" in got))
    ros.drop = 99
    fails = []
    logs.failed.connect(fails.append)
    logs.refresh_list()
    check("no battery service: says so", wait(lambda: fails) and "battery service" in fails[0])
    ros.drop = 0

    print("\n[chart]")
    check("ticks", nice_ticks(0, 100) == [0, 20, 40, 60, 80, 100], f"{nice_ticks(0, 100)}")
    tt = time_ticks(1_700_000_000, 1_700_000_000 + 3 * 3600)
    check("time ticks on the hour or half", len(tt) >= 3 and all(x % 1800 == 0 for x in tt), f"{tt}")
    ch = TimeChart()
    ch.resize(600, 300)
    many = list(range(100_000))
    vals = [50 + (i % 600) / 60 for i in many]
    vals[500:520] = [None] * 20
    ch.set_data([1.7e9 + i for i in many], vals, "Charge", "%", lines=[(20, "low")])
    img = QImage(600, 300, QImage.Format_ARGB32)
    t0 = time.monotonic()
    ch.render(img)
    check("100 000 points drawn fast", time.monotonic() - t0 < 1.0, f"{time.monotonic() - t0:.2f} s")
    i = ch.nearest(300)
    check("hover finds a value", i is not None and ch.vs[i] is not None)
    ch.set_data([], [], "Charge", "%", empty="Loading…")
    ch.render(img)
    check("empty chart says so", not ch.values())

    print("\n[History tab]")
    w = MainWindow()
    w.show()
    w.battery_logs.ros = ros
    w.battery_logs.answer_ms = 200
    ros.battery_log.connect(w.battery_logs._on_answer)
    w._show("battery")
    p = w.p_battery
    p.tabs.setCurrentIndex(1)
    p.refresh()
    check("boots in the list", wait(lambda: p.boots.rowCount() == 2), f"{p.boots.rowCount()}")
    ended = [p.boots.item(r, 7).text() for r in range(2)]
    check("how each ended", any("power cut" in e for e in ended) and any("running" in e for e in ended), f"{ended}")
    p.select_boot(old.name)
    check("a boot drawn", wait(lambda: len(p.chart.values()) == 120))
    check("can be deleted", p.delete_btn.isEnabled())
    p._set_metric("voltage")
    check("another measure", p.chart.title == "Voltage" and p.chart.unit == "V")
    p.select_boot(cur.name)
    check("the running boot cannot be deleted", wait(lambda: p.chart.values()) and not p.delete_btn.isEnabled())
    p.select_boot("__trend__")
    check("trend: a point per boot", p.chart.points and len(p.chart.values()) == 2)
    p.confirm_delete(old.name)
    check("deleted on the robot and this PC", wait(lambda: p.boots.rowCount() == 1)
          and not old.path.exists() and not (w.battery_logs.folder / old.name).exists())
    w._on_battery(dict(report(), log={"file": cur.name, "rows": 30, "disk": {"low": True, "free": 10**8,
                                                                              "used": 10**6, "files": 2},
                                      "previous": dict(LogExchange(cur).handle({"op": "log_list"})["files"][0],
                                                       clean_end=False)}))
    check("Now tab tells of the last power cut", "power cut" in p.prev_boot.text() or "no clean stop"
          in p.prev_boot.text(), p.prev_boot.text())
    check("and a disk getting full", "nearly full" in p.prev_boot.text())
    check("an event too, once", sum("last boot ended" in e.text for e in w.events.events) == 1)
    w._on_battery(dict(report(), log={"previous": {"name": "x", "clean_end": False}}))
    w._on_battery(dict(report(), log={"previous": {"name": "x", "clean_end": False}}))
    check("once per boot", sum("last boot ended" in e.text for e in w.events.events) == 2)
    w.close()


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
    asyncio.run(test_direct())
    test_log()
    test_history()
    test_page()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    os._exit(code)
