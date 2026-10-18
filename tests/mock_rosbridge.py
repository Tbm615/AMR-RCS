"""Minimal rosbridge-compatible WebSocket server for end-to-end testing RCS
without a real ROS 2 stack.

Implements the subset of the rosbridge v2.0 protocol that RCS actually uses:
  - "advertise" / "publish" / "unadvertise" (both directions)
  - "subscribe" / "unsubscribe"
  - "call_service" (server echoes an empty success response)
  - periodic server-side publishes of /map, /scan_filtered, /odom, /tf

Records every message received so a test can assert on it.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import struct
import time
from typing import Dict, List

import numpy as np
import websockets


class Hub:
    def __init__(self) -> None:
        self.subs_by_topic: Dict[str, set] = {}
        self.ever_subscribed: set = set()
        self.received_publishes: List[dict] = []
        self.service_calls: List[dict] = []
        self.advertised_by_client: Dict[str, str] = {}

    def record_publish(self, msg: dict) -> None:
        self.received_publishes.append(msg)


HUB = Hub()


async def send(ws, obj) -> None:
    await ws.send(json.dumps(obj))


def _oc_grid(now_s: float) -> dict:
    """A 50 x 50 map with a square obstacle; resolution 0.1 m."""
    w = h = 50
    data = [-1] * (w * h)
    for y in range(10, 20):
        for x in range(15, 25):
            data[y * w + x] = 100
    return {
        "op": "publish",
        "topic": "/map",
        "msg": {
            "header": {"frame_id": "map", "stamp": {"sec": int(now_s), "nanosec": 0}},
            "info": {
                "resolution": 0.1, "width": w, "height": h,
                "origin": {"position": {"x": -2.5, "y": -2.5, "z": 0.0},
                           "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}},
            },
            "data": data,
        },
    }


def _scan(now_s: float) -> dict:
    n = 360
    ranges = [1.5 + 0.3 * math.sin(math.radians(i * 2)) for i in range(n)]
    return {
        "op": "publish",
        "topic": "/scan_filtered",
        "msg": {
            "header": {"frame_id": "laser_frame",
                       "stamp": {"sec": int(now_s), "nanosec": 0}},
            "angle_min": -math.pi, "angle_max": math.pi,
            "angle_increment": 2 * math.pi / n,
            "time_increment": 0.0, "scan_time": 0.05,
            "range_min": 0.1, "range_max": 10.0,
            "ranges": ranges, "intensities": [],
        },
    }


def _odom(now_s: float, x: float, y: float, yaw: float, vx: float, wz: float) -> dict:
    return {
        "op": "publish",
        "topic": "/odom",
        "msg": {
            "header": {"frame_id": "odom", "stamp": {"sec": int(now_s), "nanosec": 0}},
            "child_frame_id": "base_link",
            "pose": {"pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": {"x": 0, "y": 0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)},
            }, "covariance": [0.0] * 36},
            "twist": {"twist": {
                "linear": {"x": vx, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": wz},
            }, "covariance": [0.0] * 36},
        },
    }


def _tf(now_s: float, x: float, y: float, yaw: float) -> dict:
    stamp = {"sec": int(now_s), "nanosec": 0}
    def tr(parent, child, tx, ty, qz, qw):
        return {
            "header": {"stamp": stamp, "frame_id": parent},
            "child_frame_id": child,
            "transform": {
                "translation": {"x": tx, "y": ty, "z": 0.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
            },
        }
    return {
        "op": "publish", "topic": "/tf",
        "msg": {"transforms": [
            tr("map", "odom", 0.0, 0.0, 0.0, 1.0),
            tr("odom", "base_link", x, y, math.sin(yaw / 2), math.cos(yaw / 2)),
            tr("base_link", "laser_frame", 0.40, 0.0, 0.0, 1.0),
        ]},
    }


async def ticker():
    """Publish periodic topics to any subscriber."""
    x, y, yaw = 0.0, 0.0, 0.0
    vx, wz = 0.0, 0.0
    last_cmd_t = 0.0
    tick = 0
    while True:
        now = time.time()
        # read latest /cmd_vel_safe publish if any
        for p in reversed(HUB.received_publishes):
            if p.get("topic") == "/cmd_vel_safe":
                vx = float(p["msg"]["linear"].get("x", 0.0))
                wz = float(p["msg"]["angular"].get("z", 0.0))
                last_cmd_t = now
                break
        if now - last_cmd_t > 0.5:
            vx, wz = 0.0, 0.0
        dt = 0.05
        yaw += wz * dt
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt

        msgs = [_tf(now, x, y, yaw), _odom(now, x, y, yaw, vx, wz)]
        if tick % 4 == 0:
            msgs.append(_scan(now))
        if tick % 20 == 0:
            msgs.append(_oc_grid(now))
        for topic, m in [(mm["topic"], mm) for mm in msgs]:
            subs = HUB.subs_by_topic.get(topic, set())
            for ws in list(subs):
                try:
                    await ws.send(json.dumps(m))
                except Exception:
                    HUB.subs_by_topic[topic].discard(ws)
        tick += 1
        await asyncio.sleep(dt)


async def handle(ws):
    peer = getattr(ws, "remote_address", "?")
    print(f"[mock] CONNECTED {peer}", flush=True)
    try:
        async for raw in ws:
            try:
                m = json.loads(raw)
            except json.JSONDecodeError:
                continue
            op = m.get("op")
            if op == "subscribe":
                topic = m["topic"]
                HUB.subs_by_topic.setdefault(topic, set()).add(ws)
                HUB.ever_subscribed.add(topic)   # durable record across disconnect
            elif op == "unsubscribe":
                topic = m.get("topic", "")
                HUB.subs_by_topic.get(topic, set()).discard(ws)
            elif op == "advertise":
                HUB.advertised_by_client[m["topic"]] = m.get("type", "")
            elif op == "unadvertise":
                HUB.advertised_by_client.pop(m.get("topic", ""), None)
            elif op == "publish":
                HUB.record_publish({"topic": m["topic"], "msg": m.get("msg", {}), "t": time.time()})
            elif op == "call_service":
                await send(ws, {"op": "service_response",
                                "service": m.get("service"),
                                "id": m.get("id"),
                                "values": {},
                                "result": True})
    except websockets.ConnectionClosed:
        pass
    finally:
        print(f"[mock] DISCONNECTED {peer}", flush=True)
        for subs in HUB.subs_by_topic.values():
            subs.discard(ws)


async def main_async(host: str, port: int, duration: float | None):
    async with websockets.serve(handle, host, port):
        print(f"mock rosbridge listening on ws://{host}:{port}")
        task = asyncio.create_task(ticker())
        if duration:
            await asyncio.sleep(duration)
            task.cancel()
        else:
            await task


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9090)
    ap.add_argument("--duration", type=float, default=None,
                    help="exit after this many seconds (for CI)")
    args = ap.parse_args()
    asyncio.run(main_async(args.host, args.port, args.duration))


if __name__ == "__main__":
    main()
