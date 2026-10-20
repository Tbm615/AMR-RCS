"""Minimal rosbridge-compatible WebSocket server for end-to-end testing RCS
without a real ROS 2 stack.

Implements the subset of the rosbridge v2.0 protocol that RCS actually uses:
  - "advertise" / "publish" / "unadvertise" (both directions)
  - "subscribe" / "unsubscribe"
  - "call_service" (server echoes an empty success response)
  - periodic server-side publishes of /map, /scan_filtered, /odom, /tf
  - throttle_rate on subscriptions, like the real rosbridge

With --heavy it also sends robot-sized traffic: a 1000 x 1000 SLAM map and
global costmap, 4995-point scans at 20 Hz, a local costmap, and a synthetic
RGB-D camera (tests/synthetic_rgbd.py) whose frames arrive CAMERA_DELAY late,
as they do over WiFi.

Records every message received so a test can assert on it.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import os
import struct
import sys
import time
from typing import Dict, List

import numpy as np
import websockets

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # synthetic_rgbd


HEAVY = False
CAMERA_DELAY = 0.3      # s between capture (header stamp) and delivery


def _stamp(t: float) -> dict:
    return {"sec": int(t), "nanosec": int((t - int(t)) * 1e9)}


class Hub:
    def __init__(self) -> None:
        self.subs_by_topic: Dict[str, set] = {}
        self.throttle: Dict[tuple, float] = {}     # (ws, topic) -> s
        self.last_sent: Dict[tuple, float] = {}
        self.ever_subscribed: set = set()
        self.received_publishes: List[dict] = []
        self.service_calls: List[dict] = []
        self.advertised_by_client: Dict[str, str] = {}

    def record_publish(self, msg: dict) -> None:
        self.received_publishes.append(msg)


HUB = Hub()


class FakeLauncher:
    """Stands in for robot/rcs_launcher.py: same JSON protocol, instant state changes."""

    def __init__(self) -> None:
        self.stacks = {k: {"state": "stopped", "detail": "", "since": time.time(), "args": {}, "log": []}
                       for k in ("bringup", "nav")}
        self.maps = ["map_a", "map_b"]
        self.result = {"id": "", "ok": True, "msg": ""}

    def _set(self, k, state, args=None):
        self.stacks[k].update(state=state, since=time.time(), args=args or {})

    def command(self, cmd: dict) -> None:
        op, k = cmd.get("op"), cmd.get("stack")
        ok, msg = True, ""
        if op == "start" and k == "bringup":
            self._set("bringup", "running"); msg = "bringup starting"
        elif op == "start" and k == "nav":
            if self.stacks["bringup"]["state"] != "running":
                ok, msg = False, "start bringup first"
            else:
                self._set("nav", "running", {kk: v for kk, v in cmd.items() if kk in ("mode", "map")})
                msg = "navigation starting"
        elif op == "stop":
            if k == "bringup":
                self._set("nav", "stopped")
            self._set(k, "stopped"); msg = f"{k} stopped"
        elif op == "save_map":
            if self.stacks["nav"]["state"] != "running":
                ok, msg = False, "no /map to save (start navigation first)"
            else:
                self.maps = sorted(set(self.maps) | {cmd.get("name", "")}); msg = f"map saved as {cmd.get('name')}"
        self.result = {"id": cmd.get("id", ""), "ok": ok, "msg": msg}

    def state_msg(self) -> dict:
        return {"op": "publish", "topic": "/rcs/launcher/state",
                "msg": {"data": json.dumps({"version": 1, "time": time.time(), "stacks": self.stacks,
                                            "maps": self.maps, "busy": "", "result": self.result})}}


LAUNCHER = FakeLauncher()


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
            "header": {"frame_id": "map", "stamp": _stamp(now_s)},
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
            "header": {"frame_id": "laser_frame", "stamp": _stamp(now_s)},
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
            "header": {"frame_id": "odom", "stamp": _stamp(now_s)},
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
    stamp = _stamp(now_s)
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


_BIG: Dict[str, str] = {}


def _big_grid(topic: str, kind: str) -> str:
    """1000 x 1000 OccupancyGrid at 5 cm, JSON-encoded once (as big as a real SLAM map)."""
    if topic not in _BIG:
        w = h = 1000
        yy, xx = np.mgrid[0:h, 0:w]
        g = np.full((h, w), -1, np.int8)
        room = (abs(xx - 500) < 160) & (abs(yy - 500) < 120)
        g[room] = 0
        wall = room & ((abs(xx - 500) >= 158) | (abs(yy - 500) >= 118))
        g[wall] = 100
        if kind == "cost":
            g = np.where(room, np.where(wall, 100, 0), -1).astype(np.int8)
        _BIG[topic] = json.dumps({"op": "publish", "topic": topic, "msg": {
            "header": {"frame_id": "map", "stamp": _stamp(time.time())},
            "info": {"resolution": 0.05, "width": w, "height": h,
                     "origin": {"position": {"x": -25.0, "y": -25.0, "z": 0.0},
                                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
            "data": g.reshape(-1).tolist()}})
    return _BIG[topic]


def _heavy_scan(now_s: float) -> dict:
    m = _scan(now_s)
    n = 4995
    m["msg"]["angle_increment"] = 2 * math.pi / n
    m["msg"]["ranges"] = [None if i % 97 == 0 else 2.0 + 0.5 * math.sin(i * 0.01) for i in range(n)]
    return m


def _local_costmap(now_s: float, x: float, y: float) -> dict:
    w = h = 120
    return {"op": "publish", "topic": "/local_costmap/costmap", "msg": {
        "header": {"frame_id": "odom", "stamp": _stamp(now_s)},
        "info": {"resolution": 0.05, "width": w, "height": h,
                 "origin": {"position": {"x": x - 3, "y": y - 3, "z": 0.0},
                            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
        "data": [(i * 7) % 100 for i in range(w * h)]}}


def _wants(ws, topic: str, now: float) -> bool:
    """rosbridge throttle_rate: at most one message per interval per subscriber."""
    key = (ws, topic)
    thr = HUB.throttle.get(key, 0.0)
    if thr and now - HUB.last_sent.get(key, 0.0) < thr:
        return False
    HUB.last_sent[key] = now
    return True


async def _deliver(topic: str, payload) -> None:
    now = time.time()
    raw = None
    for ws in list(HUB.subs_by_topic.get(topic, set())):
        if not _wants(ws, topic, now):
            continue
        if raw is None:
            raw = payload if isinstance(payload, str) else json.dumps(payload)
        try:
            await ws.send(raw)
        except Exception:
            HUB.subs_by_topic[topic].discard(ws)


def _subscribed(topic: str) -> bool:
    return bool(HUB.subs_by_topic.get(topic))


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
        if HEAVY:
            msgs.append(_heavy_scan(now))
            if tick % 40 == 0:
                await _deliver("/map", _big_grid("/map", "map"))
            if tick % 20 == 0:
                await _deliver("/global_costmap/costmap", _big_grid("/global_costmap/costmap", "cost"))
            if tick % 4 == 0:
                msgs.append(_local_costmap(now, x, y))
            await _camera(now, x, y, yaw, tick)
        else:
            if tick % 4 == 0:
                msgs.append(_scan(now))
            if tick % 20 == 0:
                msgs.append(_oc_grid(now))
        if tick % 10 == 0:
            msgs.append(LAUNCHER.state_msg())
        for m in msgs:
            await _deliver(m["topic"], m)
        tick += 1
        await asyncio.sleep(dt)


_CAM_QUEUE: list = []     # (deliver_at, topic, msg)


async def _camera(now: float, x: float, y: float, yaw: float, tick: int) -> None:
    import synthetic_rgbd as cam
    if tick % 40 == 0 or not hasattr(_camera, "static_sent"):
        _camera.static_sent = True
        await _deliver("/tf_static", {"op": "publish", "topic": "/tf_static",
                                      "msg": cam.static_tf_msg(_stamp(now))})
    depth_t, color_t = "/camera/depth/image_raw/compressedDepth", "/camera/color/image_raw/compressed"
    if tick % 2 == 0 and (_subscribed(depth_t) or _subscribed(color_t)):
        st = _stamp(now)
        depth, color = cam.render(x, y, yaw)
        if _subscribed(depth_t):
            _CAM_QUEUE.append((now + CAMERA_DELAY, depth_t, cam.compressed_depth_msg(depth, st)))
        if _subscribed(color_t):
            _CAM_QUEUE.append((now + CAMERA_DELAY, color_t, cam.jpeg_msg(color, st)))
    if tick % 10 == 0:
        await _deliver("/camera/depth/camera_info", {"op": "publish", "topic": "/camera/depth/camera_info",
                                                      "msg": cam.camera_info_msg(_stamp(now))})
    while _CAM_QUEUE and _CAM_QUEUE[0][0] <= now:
        _, topic, msg = _CAM_QUEUE.pop(0)
        await _deliver(topic, {"op": "publish", "topic": topic, "msg": msg})


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
                HUB.throttle[(ws, topic)] = float(m.get("throttle_rate") or 0) / 1000.0
                if topic == "/tf_static" and HEAVY:
                    import synthetic_rgbd as cam
                    await send(ws, {"op": "publish", "topic": "/tf_static",
                                    "msg": cam.static_tf_msg(_stamp(time.time()))})
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
                if m["topic"] == "/rcs/launcher/cmd":
                    try:
                        LAUNCHER.command(json.loads(m.get("msg", {}).get("data", "")))
                    except ValueError:
                        pass
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
    ap.add_argument("--heavy", action="store_true",
                    help="robot-sized traffic: big map/costmap, 4995-point scans, RGB-D camera")
    args = ap.parse_args()
    global HEAVY
    HEAVY = args.heavy
    asyncio.run(main_async(args.host, args.port, args.duration))


if __name__ == "__main__":
    main()
