#!/usr/bin/env python3
"""Battery service: keeps a Bluetooth link to the JK-BMS and publishes what it
reads on /rcs/battery (std_msgs/String, JSON, 1 Hz) for the RCS app.

Runs on the robot next to rcs_launcher.py (see start_rcs_robot.sh), so the
battery can be watched from anywhere on the network, not only from a phone next
to the robot. Reads only: see jk_bms.py.

Environment: RCS_BMS_ADDRESS fixes the BMS instead of finding it; RCS_BMS_ADAPTER
picks the Bluetooth adapter (hci1 for a USB dongle with a better antenna).
The BMS found first is remembered in bms.json next to this file.
"""
from __future__ import annotations

import asyncio
import json
import os
import subprocess
from pathlib import Path

import rclpy
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError
from rclpy.node import Node
from std_msgs.msg import String

import jk_bms
from bms_link import BmsLink

ADAPTER = os.environ.get("RCS_BMS_ADAPTER", "")
BLUEZ = {"adapter": ADAPTER} if ADAPTER else {}


class NoAdapter(Exception):
    no_adapter = True


class _Client:
    def __init__(self, device, on_lost) -> None:
        self.c = BleakClient(device, disconnected_callback=lambda _c: on_lost(), timeout=25.0, bluez=BLUEZ)

    async def connect(self) -> None:
        await self.c.connect()

    async def start_notify(self, char, cb) -> None:
        await self.c.start_notify(char, cb)

    async def write(self, data: bytes) -> None:
        await self.c.write_gatt_char(jk_bms.CHARACTERISTIC, data)

    async def close(self) -> None:
        try:
            await self.c.disconnect()
        except Exception:       # already gone
            pass


class Backend:
    async def scan(self, seconds: float, address: str) -> dict:
        seen: dict = {}
        hit = asyncio.Event()

        def on_adv(device, adv) -> None:
            seen[device.address.lower()] = (device, adv.rssi, adv.service_uuids)
            if (device.address.lower() == address if address
                    else jk_bms.is_jk(device.name, device.address, adv.service_uuids)):
                hit.set()

        try:
            async with BleakScanner(on_adv, bluez=BLUEZ):
                try:
                    await asyncio.wait_for(hit.wait(), seconds)
                except asyncio.TimeoutError:
                    pass
        except BleakError as e:
            # adapter off or blocked: AutoEnable does not cover everything
            subprocess.run(["bluetoothctl", "power", "on"], capture_output=True, timeout=8)
            raise NoAdapter(str(e)) from e
        match = [v for k, v in seen.items()
                 if (k == address if address else jk_bms.is_jk(v[0].name, v[0].address, v[2]))]
        best = max(match, key=lambda v: v[1], default=None)
        return {"heard": len(seen), "device": best[0] if best else None, "rssi": best[1] if best else None}

    def client(self, device, on_lost) -> _Client:
        return _Client(device, on_lost)


async def serve(node: Node) -> None:
    pub = node.create_publisher(String, "/rcs/battery", 1)
    config = Path(__file__).with_name("bms.json")
    link = BmsLink(Backend(), config, os.environ.get("RCS_BMS_ADDRESS", ""))
    task = asyncio.create_task(link.run())
    node.get_logger().info(f"battery service ready, BMS {link.address or 'not known yet'}")
    last = ""
    while not task.done():
        st = link.status()
        pub.publish(String(data=json.dumps(st)))
        line = f"{st['state']} {st['address']} heard={st['heard']} {st['error']}"
        if line != last:
            last = line
            node.get_logger().info(line)
        await asyncio.sleep(1.0)
    task.result()


def main() -> None:
    rclpy.init()
    node = Node("rcs_battery")
    try:
        asyncio.run(serve(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
