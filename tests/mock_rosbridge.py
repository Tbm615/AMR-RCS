"""Minimal rosbridge-compatible WebSocket server for end-to-end testing RCS
without a real ROS 2 stack.

Implements the subset of the rosbridge v2.0 protocol that RCS actually uses:
  - "advertise" / "publish" / "unadvertise" (both directions)
  - "subscribe" / "unsubscribe"
  - "call_service" (server echoes an empty success response)
  - periodic server-side publishes of /map, /scan_filtered, /odom, /tf
  - throttle_rate on subscriptions, like the real rosbridge

With --robot it is also the robot's own services, running their real code:
the battery service (robot/battery_log.py writes a boot log in a temp folder
and answers /rcs/battery/cmd; the BMS reading is made up) and the E-STOP gate
(robot/estop_gate.py: /amr/estop, heartbeats, /rcs/estop/state; the base then
follows only what the gate lets through). --name gives the robot a name, so
tests can tell several mocks apart.

A message a client publishes also goes to the clients subscribed to that
topic, as with the real rosbridge.

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
import re
import struct
import sys
import tempfile
import time
from pathlib import Path
from typing import Dict, List

import numpy as np
import websockets

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # synthetic_rgbd
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "robot"))
import map_files                                                 # noqa: E402
import map_preview                                               # noqa: E402
from map_exchange import MapExchange                             # noqa: E402
import battery_log                                               # noqa: E402
import estop_gate                                                # noqa: E402
import zone_masks                                                # noqa: E402


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
# the release of the robot side, as the real rcs_launcher.py says it
_m = re.search(r'ROBOT_SIDE = "([^"]+)"', (Path(__file__).resolve().parent.parent / "robot" / "rcs_launcher.py")
               .read_text(encoding="utf-8"))
ROBOT_SIDE = _m.group(1) if _m else ""


class FakeLauncher:
    """Stands in for robot/rcs_launcher.py: same JSON protocol, instant state changes."""

    def __init__(self) -> None:
        self.stacks = {k: {"state": "stopped", "detail": "", "since": time.time(), "args": {}, "log": []}
                       for k in ("bringup", "nav")}
        self.maps = ["map_a", "map_b"]
        self.old = False            # True: behaves like the v0.4 launcher (no previews)
        self.preview_out: list = []     # answers waiting to go on /rcs/launcher/preview
        self.preview_asked: list = []
        self.result = {"id": "", "ok": True, "msg": ""}
        # map_a is a real file so that it can be copied; map_b stands for a
        # map the launcher cannot read
        self.map_dir = tempfile.mkdtemp(prefix="rcs_mock_maps_")
        img = np.full((60, 80), 254, np.uint8)
        img[:, 40] = 0
        map_files.write_map(self.map_dir, "map_a", {
            "resolution": 0.05, "origin": [-1.0, -2.0, 0.0], "negate": 0, "occupied_thresh": 0.65,
            "free_thresh": 0.25, "mode": "trinary"}, b"P5\n80 60\n255\n" + img.tobytes())
        self.camera = {"color": True, "depth": True, "publishers": 1}
        self.nav_map = ""           # map Nav2 runs on, refused as a target
        self.drop_next = 0          # answers to lose on the way back
        self.exchange = MapExchange(self._map_paths, self.map_dir, lambda n: n == self.nav_map)
        self.zones: dict = {}       # map name -> zones, as the robot keeps them
        self.zone_cmds: list = []

    def _map_paths(self) -> dict:
        return {m: os.path.join(self.map_dir, m + ".yaml") for m in self.maps}

    def _set(self, k, state, args=None):
        self.stacks[k].update(state=state, since=time.time(), args=args or {})

    def command(self, cmd: dict) -> None:
        op, k = cmd.get("op"), cmd.get("stack")
        ok, msg = True, ""
        if op == "map_preview":
            self.preview_asked.append(cmd.get("name"))
            if not self.old:
                self.preview_out.append(self._preview(cmd.get("name", "")))
            return
        if op in ("zones_get", "zones_put"):
            if self.old:
                return
            self.zone_cmds.append(cmd)
            name = cmd.get("name", "")
            ans = {"id": cmd.get("id", ""), "name": name, "ok": True, "msg": "", "zones": [], "applied": False}
            try:
                if name not in self.maps:
                    raise ValueError(f"map {name!r} not found on the robot")
                if op == "zones_put":
                    self.zones[name] = zone_masks.check_zones(cmd.get("zones"))
                    nav = self.stacks["nav"]
                    ans["applied"] = nav["state"] == "running" and nav["args"].get("map") == name
                    ans["msg"] = f"{len(self.zones[name])} zone(s) saved on the robot" + (
                        ", in use now" if ans["applied"] else "")
                ans["zones"] = self.zones.get(name, [])
            except ValueError as e:
                ans.update(ok=False, msg=str(e))
            self.preview_out.append({"op": "publish", "topic": "/rcs/launcher/zones",
                                     "msg": {"data": json.dumps(ans)}})
            return
        if op in ("map_get", "map_put"):
            if self.old:
                return
            ans = self.exchange.handle(cmd)
            if ans.get("done"):
                self.maps = sorted(set(self.maps) | {cmd.get("name", "")})
            if self.drop_next > 0:
                self.drop_next -= 1
            else:
                self.preview_out.append({"op": "publish", "topic": "/rcs/launcher/map_data",
                                         "msg": {"data": json.dumps(ans)}})
            return
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

    def _preview(self, name: str) -> dict:
        if name == "map_b":
            out = {"name": name, "error": "no preview for .png map images"}
        else:
            img = np.full((60, 80), 254, np.uint8)
            img[:, 40] = 0
            out = {"name": name, "png": base64.b64encode(map_preview.png_gray(img)).decode(), "saved": False,
                   "width_m": 4.0, "height_m": 3.0, "resolution": 0.05, "mtime": 1000.0}
        return {"op": "publish", "topic": "/rcs/launcher/preview", "msg": {"data": json.dumps(out)}}

    def state_msg(self) -> dict:
        st = {"version": 1, "rcs": ROBOT_SIDE, "time": time.time(), "stacks": self.stacks,
              "maps": self.maps, "busy": "", "result": self.result}
        if not self.old:
            st["map_mtimes"] = {m: 1000.0 for m in self.maps}
            st["caps"] = ["map_transfer", "zones"]
            st["camera"] = self.camera
        return {"op": "publish", "topic": "/rcs/launcher/state", "msg": {"data": json.dumps(st)}}


LAUNCHER = FakeLauncher()


class FakeRobot:
    """The robot's battery service and E-STOP gate, with their real code."""

    def __init__(self, name: str = "mock", soc: int = 80) -> None:
        self.name, self.soc = name, soc
        self.log_dir = tempfile.mkdtemp(prefix=f"rcs_mock_battlog_{name}_")
        # a boot before this one that ended in a power cut: no "stopped" row
        old = battery_log.BootLog(self.log_dir, boot="0000aaaa", clock=lambda: time.time() - 7200,
                                  up=lambda: 3600)
        for i in range(30):
            old.write(self.status(age=0.5, soc=self.soc + 5))
        old._f.close()
        self.log = battery_log.BootLog(self.log_dir, boot="1111bbbb")
        self.exchange = battery_log.LogExchange(self.log)
        self.gate = estop_gate.Gate(os.path.join(self.log_dir, "estop.json"))
        self.cancels = 0
        self.out: list = []          # publishes waiting to go out

    def status(self, age=0.5, soc=None) -> dict:
        cells = [3.30 + 0.002 * i for i in range(15)]
        r = {"soc": self.soc if soc is None else soc, "soh": 98, "voltage": round(sum(cells), 3),
             "current": -5.2, "power": 257.0, "cells": cells, "delta": round(cells[-1] - cells[0], 3),
             "temps": [31.0, 32.0], "mos_temp": 38.0, "remaining_ah": 15.0, "nominal_ah": 19.5,
             "cycles": 27, "charging": True, "discharging": True, "cell_avg": 3.314,
             "resistances": [0.18] * 15, "balancing": 0, "balance_current": 0.0,
             "total_cycle_ah": 540.0, "runtime_s": 33457295, "alarms": 0, "wire_alarms": 0}
        return {"state": "connected", "address": "98:da:20:0b:36:a1", "name": f"{self.name} battery",
                "rssi": -80, "heard": 3, "reconnects": 1, "error": "", "bad_frames": 0, "connected_s": 60,
                "age": age, "reading": r}

    def tick(self) -> list:
        st = self.status()
        st["log"] = {"file": self.log.name, "rows": self.log.rows, "disk": self.log.disk(),
                     "previous": self.log.previous}
        self.log.write(st)
        if "cancel_nav" in self.gate.tick(False):
            self.cancels += 1
        g = dict(self.gate.state(), host=self.name, cancels=self.cancels)
        return [{"op": "publish", "topic": "/rcs/battery", "msg": {"data": json.dumps(st)}},
                {"op": "publish", "topic": "/rcs/estop/state", "msg": {"data": json.dumps(g)}}]

    def on_publish(self, topic: str, msg: dict) -> None:
        if topic == "/rcs/battery/cmd":
            try:
                ans = self.exchange.handle(json.loads(msg.get("data", "")))
            except ValueError:
                return
            self.out.append({"op": "publish", "topic": "/rcs/battery/log", "msg": {"data": json.dumps(ans)}})
        elif topic == "/amr/estop":
            if "cancel_nav" in self.gate.estop(bool(msg.get("data")), by="/amr/estop"):
                self.cancels += 1
        elif topic == "/rcs/app/heartbeat":
            try:
                self.gate.heartbeat(json.loads(msg.get("data", "")))
            except ValueError:
                pass


ROBOT = None


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
                if ROBOT is not None:       # the base follows the gate, as on the robot
                    vx, wz = ROBOT.gate.filter(vx, wz)
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
        if ROBOT is not None and tick % 20 == 0:
            msgs += ROBOT.tick()
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
                # to the clients subscribed to it, as rosbridge does
                await _deliver(m["topic"], {"op": "publish", "topic": m["topic"], "msg": m.get("msg", {})})
                if ROBOT is not None:
                    ROBOT.on_publish(m["topic"], m.get("msg", {}))
                    while ROBOT.out:
                        p = ROBOT.out.pop(0)
                        await _deliver(p["topic"], p)
                if m["topic"] == "/rcs/launcher/cmd":
                    try:
                        LAUNCHER.command(json.loads(m.get("msg", {}).get("data", "")))
                    except ValueError:
                        pass
                    while LAUNCHER.preview_out:
                        p = LAUNCHER.preview_out.pop(0)
                        await _deliver(p["topic"], p)
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
    ap.add_argument("--robot", action="store_true",
                    help="also the robot's battery service and E-STOP gate (their real code)")
    ap.add_argument("--name", default="mock", help="robot name, with --robot")
    ap.add_argument("--soc", type=int, default=80, help="battery charge, with --robot")
    ap.add_argument("--heavy", action="store_true",
                    help="robot-sized traffic: big map/costmap, 4995-point scans, RGB-D camera")
    args = ap.parse_args()
    global HEAVY, ROBOT
    HEAVY = args.heavy
    if args.robot:
        ROBOT = FakeRobot(args.name, args.soc)
        print(f"mock robot {args.name}: battery log in {ROBOT.log_dir}", flush=True)
    asyncio.run(main_async(args.host, args.port, args.duration))


if __name__ == "__main__":
    main()
