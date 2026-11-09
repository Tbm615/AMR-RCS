#!/usr/bin/env python3
"""Battery service: keeps a Bluetooth link to the JK-BMS, publishes what it
reads on /rcs/battery (std_msgs/String, JSON, 1 Hz) for the RCS app, and writes
it to the battery log every second (battery_log.py: one file per boot).

Runs on the robot next to rcs_launcher.py (see start_rcs_robot.sh), so the
battery can be watched from anywhere on the network, not only from a phone next
to the robot. Reads only: see jk_bms.py.

The dongle is driven over USB by bumble, not through BlueZ: see hci_quirks.py.
The card inside the Jetson has no antenna and is not used.

A watchdog thread checks that the service is still going round: if it hangs
(a USB call that never returns) for 20 s, it writes a last row saying
so and exits, and start_rcs_robot.sh starts it again.

Topics:
  /rcs/battery       robot -> app   status and reading, 1 Hz
  /rcs/battery/cmd   app -> robot   {"op": "log_list" | "log_get", ...}
  /rcs/battery/log   robot -> app   the answers, see battery_log.py

Environment: RCS_BMS_ADDRESS fixes the BMS instead of finding it;
RCS_BMS_TRANSPORT picks the dongle (default usb:0a12:0001); RCS_BATTERY_LOG
is the log folder (default $RCS_WS/battery_log).
The BMS found first is remembered in bms.json next to this file.
"""
from __future__ import annotations

import asyncio
import json
import os
import signal
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy
from bumble.core import AdvertisingData, PhysicalTransport
from bumble.device import Device, Peer
from bumble.hci import HCI_Constant, OwnAddressType
from bumble.transport import open_transport
from rclpy.node import Node
from std_msgs.msg import String

import jk_bms
from battery_log import BootLog, LogExchange, Watchdog
from bms_link import BmsLink
from hci_quirks import HciQuirks

TRANSPORT = os.environ.get("RCS_BMS_TRANSPORT", "usb:0a12:0001")
WS = Path(os.environ.get("RCS_WS", str(Path.home() / "tbm_test"))).expanduser()
LOG_DIR = Path(os.environ.get("RCS_BATTERY_LOG", str(WS / "battery_log"))).expanduser()


class NoAdapter(Exception):
    no_adapter = True


def _name(adv) -> str:
    raw = adv.data.get(AdvertisingData.COMPLETE_LOCAL_NAME) \
        or adv.data.get(AdvertisingData.SHORTENED_LOCAL_NAME) or ""
    return raw if isinstance(raw, str) else bytes(raw).decode("latin1")


def _reason(code) -> str:
    try:
        return f"link lost: {HCI_Constant.error_name(int(code))} ({int(code)})"
    except (TypeError, ValueError):
        return "link lost"


class _Client:
    def __init__(self, backend, device, on_lost) -> None:
        self.backend, self.device, self.on_lost = backend, device, on_lost
        self.conn = self.peer = self.char = None

    async def connect(self) -> None:
        d = await self.backend.open()
        try:
            # device.peer: the address the scan heard, or the one remembered
            # from last time ("98:DA:../P"): then there is no scan at all
            self.conn = await asyncio.wait_for(
                d.connect(self.device.peer, transport=PhysicalTransport.LE,
                          own_address_type=OwnAddressType.PUBLIC, timeout=15), 20)
            self.conn.on("disconnection", lambda reason=None, *_a: self.on_lost(_reason(reason)))
            self.peer = Peer(self.conn)
            # only the BMS's own service: discovering them all cost seconds per reconnect
            services = await asyncio.wait_for(self.peer.discover_service(jk_bms.SERVICE), 15)
            if not services:
                services = await asyncio.wait_for(self.peer.discover_services(), 15)
            for service in services:
                await asyncio.wait_for(service.discover_characteristics(), 15)
                for c in service.characteristics:
                    if "FFE1" in str(c.uuid).upper():
                        self.char = c
            if self.char is None:
                raise ConnectionError("this device is not a JK-BMS: no FFE1")
            await asyncio.wait_for(self.peer.discover_descriptors(self.char), 15)
        except asyncio.TimeoutError as e:
            raise TimeoutError("the BMS did not answer in time") from e

    async def start_notify(self, _char, cb) -> None:
        await asyncio.wait_for(self.peer.subscribe(self.char, lambda v: cb(None, v)), 15)

    async def write(self, data: bytes) -> None:
        await self.peer.write_value(self.char, data, with_response=False)

    async def close(self) -> None:
        try:
            if self.conn is not None:
                await asyncio.wait_for(self.conn.disconnect(), 5)
        except Exception:       # already gone
            pass


class Backend:
    """One open dongle, opened again if it goes away."""

    def __init__(self) -> None:
        self.transport = None
        self.device = None

    async def open(self):
        if self.device is not None:
            return self.device
        try:
            self.transport = await open_transport(TRANSPORT)
            quirks = HciQuirks(self.transport.sink)
            d = Device.with_hci("rcs", "F0:F1:F2:F3:F4:F5", self.transport.source, quirks)
            quirks.host = d.host
            await asyncio.wait_for(d.power_on(), 15)
        except Exception as e:
            await self.reset()
            raise NoAdapter(f"no Bluetooth dongle ({e or type(e).__name__})") from e
        self.device = d
        return d

    async def reset(self) -> None:
        t, self.transport, self.device = self.transport, None, None
        if t is not None:
            try:
                await asyncio.wait_for(t.close(), 5)
            except Exception:
                pass

    async def scan(self, seconds: float, address: str) -> dict:
        d = await self.open()
        seen: dict = {}
        hit = asyncio.Event()

        def wanted(addr: str, name: str) -> bool:
            return addr == address if address else jk_bms.is_jk(name, addr)

        def on_adv(adv) -> None:
            addr = str(adv.address).split("/")[0].lower()
            name = _name(adv) or (seen[addr].name if addr in seen else "")
            seen[addr] = SimpleNamespace(address=addr, name=name, peer=str(adv.address), rssi=adv.rssi)
            if wanted(addr, name):
                hit.set()

        d.on("advertisement", on_adv)
        try:
            await d.start_scanning(active=True, filter_duplicates=False,
                                   own_address_type=OwnAddressType.PUBLIC)
            try:
                await asyncio.wait_for(hit.wait(), seconds)
            except asyncio.TimeoutError:
                pass
            await d.stop_scanning()
        except Exception:
            await self.reset()      # the dongle may have gone: start clean
            raise
        finally:
            d.remove_listener("advertisement", on_adv)
        best = max((v for v in seen.values() if wanted(v.address, v.name)),
                   key=lambda v: v.rssi, default=None)
        return {"heard": len(seen), "device": best, "rssi": best.rssi if best else None}

    def client(self, device, on_lost) -> _Client:
        return _Client(self, device, on_lost)


def _status(link: BmsLink, log: BootLog) -> dict:
    st = link.status()
    prev = log.previous
    if log.rows % 60 == 0 or not hasattr(log, "_disk"):
        log._disk = log.disk()      # walks the folder: once a minute is enough
    st["log"] = {"file": log.name, "rows": log.rows, "disk": log._disk,
                 "previous": None if prev is None else {
                     "name": prev.get("name"), "end": prev.get("end"), "clean_end": prev.get("clean_end"),
                     "last": prev.get("last")}}
    return st


async def serve(node: Node, log: BootLog, dog: Watchdog) -> None:
    pub = node.create_publisher(String, "/rcs/battery", 1)
    answers = node.create_publisher(String, "/rcs/battery/log", 5)
    exchange = LogExchange(log)
    loop = asyncio.get_running_loop()
    pending: list = []

    def on_cmd(msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(cmd, dict):
            pending.append(cmd)

    node.create_subscription(String, "/rcs/battery/cmd", on_cmd, 10)
    config = Path(__file__).with_name("bms.json")
    link = BmsLink(Backend(), config, os.environ.get("RCS_BMS_ADDRESS", ""))
    task = asyncio.create_task(link.run())
    node.get_logger().info(f"battery service ready, BMS {link.address or 'not known yet'}, "
                           f"log {log.path}")
    prev = log.previous
    if prev and not prev.get("clean_end"):
        last = prev.get("last") or {}
        node.get_logger().warn(
            f"the last boot ({prev.get('name')}) ended without a clean stop: power cut or crash. "
            f"Battery then: {last.get('soc')}% {last.get('voltage')} V")
    log.write(link.status(), "service started")

    async def answer(cmd: dict) -> None:
        out = await loop.run_in_executor(None, exchange.handle, cmd)
        answers.publish(String(data=json.dumps(out)))

    last_line, next_row = "", time.monotonic()
    while not task.done():
        rclpy.spin_once(node, timeout_sec=0)
        while pending:
            asyncio.create_task(answer(pending.pop(0)))
        now = time.monotonic()
        if now >= next_row:
            next_row += 1.0
            if now - next_row > 5:      # fell behind (a hang): start counting again
                next_row = now + 1.0
            st = _status(link, log)
            pub.publish(String(data=json.dumps(st)))
            log.write(st, link.take_notes(), link.frame_hex)
            dog.alive(st)
            line = f"{st['state']} {st['address']} heard={st['heard']} {st['error']}"
            if line != last_line:
                last_line = line
                node.get_logger().info(line)
        await asyncio.sleep(0.05)
    task.result()


def main() -> None:
    log = BootLog(LOG_DIR)
    dog = Watchdog(log)
    dog.start()

    def stop(_sig, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)     # shutdown: a clean last row
    rclpy.init()
    node = Node("rcs_battery")
    note = "stopped"
    try:
        asyncio.run(serve(node, log, dog))
    except KeyboardInterrupt:
        pass
    except Exception as e:      # not a clean stop: the last row says what it was
        note = f"crashed: {e or type(e).__name__}"
        raise
    finally:
        log.close(note)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
