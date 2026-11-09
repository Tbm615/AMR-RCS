"""Several robots on one screen: a light connection to each watched robot.

Each RobotWatch has its own rosbridge connection but takes only small topics,
throttled: battery, programs (launcher), E-STOP, speed and position, Nav2's
state. No map, scan or camera, so watching ten robots costs less than driving
one. The robot being driven keeps its full connection (RosClient) as before.

Callbacks run on roslibpy's thread and only write a dict under a lock; the
GUI reads a copy of it (snapshot) on a timer.
"""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from typing import Dict, Optional

try:
    import roslibpy
except ImportError:  # pragma: no cover
    roslibpy = None

from PySide6.QtCore import QObject, QTimer, Signal

from .battery import alarm, live
from .profile import RobotProfile

LOG = logging.getLogger("rcs.fleet")
STALE_S = 5.0           # a topic quiet this long is shown as not arriving
AMCL_POSE = "/amcl_pose"
_NAV = {1: "navigating", 2: "navigating", 3: "canceling", 4: "goal reached", 5: "canceled", 6: "aborted"}


class RobotWatch:
    def __init__(self, profile: RobotProfile) -> None:
        self.profile = profile
        self.started_token = profile.token      # a new sign-in starts the watch again
        self._lock = threading.Lock()
        self._data: dict = {}
        self._seen: Dict[str, float] = {}
        self._ros = None
        self._connected = False
        self._pubs: dict = {}

    # ---- connection ----

    def start(self) -> None:
        if roslibpy is None or self._ros is not None:
            return
        p = self.profile
        headers = {"Authorization": f"Bearer {p.token}"} if p.token else None
        if p.use_ssl:
            from . import gateway
            gateway.pin_ws(p.host, p.port, p.tls_pin)
        try:
            ros = roslibpy.Ros(host=p.host, port=p.port, is_secure=p.use_ssl, headers=headers)
        except Exception as e:  # noqa: BLE001
            LOG.warning("watch %s: %s", p.name, e)
            return
        fac = ros.factory
        fac.initialDelay, fac.maxDelay, fac.factor, fac.maxRetries = 0.5, 5.0, 1.6, None
        ros.on("ready", lambda *_: self._on_ready(ros))
        ros.on("close", lambda *_: self._on_close(ros))
        self._ros = ros
        ros.factory.manager.run()

    def stop(self) -> None:
        ros, self._ros = self._ros, None
        self._connected = False
        self._pubs = {}
        if ros is None:
            return
        try:
            ros.factory.stopTrying()
        except Exception:  # noqa: BLE001
            pass
        threading.Thread(target=lambda: _close(ros), daemon=True).start()

    def connected(self) -> bool:
        return self._connected and self._ros is not None and bool(self._ros.is_connected)

    def _on_close(self, ros) -> None:
        if ros is self._ros:
            self._connected = False
            self._pubs = {}

    def _on_ready(self, ros) -> None:
        if ros is not self._ros:
            return
        self._connected = True
        self._pubs = {}
        t = self.profile.topics
        subs = [("/rcs/battery", "std_msgs/msg/String", 2000, self._json("battery")),
                ("/rcs/launcher/state", "std_msgs/msg/String", 2000, self._json("launcher")),
                ("/rcs/estop/state", "std_msgs/msg/String", 1000, self._json("gate")),
                (t.estop_topic, "std_msgs/msg/Bool", 0, lambda m: self._put("estop", bool(m.get("data")))),
                (t.odom, "nav_msgs/msg/Odometry", 1000, self._on_odom),
                (AMCL_POSE, "geometry_msgs/msg/PoseWithCovarianceStamped", 1000, self._on_amcl)]
        if t.nav_action:
            subs.append((f"{t.nav_action}/_action/status", "action_msgs/msg/GoalStatusArray", 1000, self._on_nav))
        for name, typ, thr, cb in subs:
            if not name:
                continue
            try:
                roslibpy.Topic(ros, name, typ, throttle_rate=thr, queue_size=1,
                               reconnect_on_close=False).subscribe(cb)
            except Exception as e:  # noqa: BLE001
                LOG.warning("watch %s %s: %s", self.profile.name, name, e)

    # ---- data (network thread) ----

    def _put(self, key: str, value) -> None:
        with self._lock:
            self._data[key] = value
            self._seen[key] = time.monotonic()

    def _json(self, key: str):
        def cb(m: dict) -> None:
            try:
                d = json.loads(m.get("data", ""))
            except (TypeError, ValueError):
                return
            if isinstance(d, dict):
                self._put(key, d)
        return cb

    def _on_odom(self, m: dict) -> None:
        tw = m.get("twist", {}).get("twist", {})
        pose = m.get("pose", {}).get("pose", {})
        pos = pose.get("position", {})
        self._put("odom", {"v": float(tw.get("linear", {}).get("x", 0.0)),
                           "w": float(tw.get("angular", {}).get("z", 0.0)),
                           "x": float(pos.get("x", 0.0)), "y": float(pos.get("y", 0.0)),
                           "yaw": _yaw(pose.get("orientation", {}))})

    def _on_amcl(self, m: dict) -> None:
        pose = m.get("pose", {}).get("pose", {})
        pos = pose.get("position", {})
        self._put("map_pose", {"x": float(pos.get("x", 0.0)), "y": float(pos.get("y", 0.0)),
                               "yaw": _yaw(pose.get("orientation", {}))})

    def _on_nav(self, m: dict) -> None:
        sl = m.get("status_list") or []
        if not sl:
            return
        latest = max(sl, key=lambda s: (s.get("goal_info", {}).get("stamp", {}).get("sec", 0),
                                        s.get("goal_info", {}).get("stamp", {}).get("nanosec", 0)))
        self._put("nav", _NAV.get(int(latest.get("status", 0)), ""))

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            d = dict(self._data)
            ages = {k: now - t for k, t in self._seen.items()}
        d["_ages"] = ages
        d["_connected"] = self.connected()
        return d

    # ---- commands ----

    def estop(self) -> bool:
        """Engage only: releasing is done on the robot's own page, by its operator."""
        if not self.connected() or not self.profile.topics.estop_topic:
            return False
        try:
            pub = self._pubs.get("estop")
            if pub is None:
                pub = roslibpy.Topic(self._ros, self.profile.topics.estop_topic, "std_msgs/msg/Bool")
                pub.advertise()
                self._pubs["estop"] = pub
            pub.publish(roslibpy.Message({"data": True}))
            return True
        except Exception as e:  # noqa: BLE001
            LOG.warning("E-STOP %s: %s", self.profile.name, e)
            return False


def _yaw(q: dict) -> float:
    x, y, z, w = (float(q.get(k, 0.0)) for k in ("x", "y", "z", "w"))
    if not any((x, y, z, w)):
        return 0.0
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def heading(d: dict) -> Optional[float]:
    """Where the robot faces: on the map when AMCL says so, else by odometry."""
    ages = d.get("_ages", {})
    for key in ("map_pose", "odom"):
        if key in d and ages.get(key, 1e9) <= STALE_S:
            return d[key].get("yaw")
    return None


def overall(desc: dict) -> str:
    """One level for a robot's card: the worst of its lines that matter."""
    levels = [lvl for key, (_t, lvl) in desc.items() if key in ("link", "battery", "estop", "programs")]
    if "error" in levels:
        return "error"
    if "warn" in levels:
        return "warn"
    return "ok" if desc.get("link", ("", "off"))[1] == "ok" else "off"


def _close(ros) -> None:
    try:
        ros.close(timeout=3)
    except Exception:  # noqa: BLE001
        pass


def describe(d: dict) -> dict:
    """What a robot's card shows: text and a level (ok | warn | error | off)
    per line. No Qt here, for the tests."""
    ages = d.get("_ages", {})

    def fresh(key: str) -> bool:
        return key in d and ages.get(key, 1e9) <= STALE_S

    out = {}
    if not d.get("_connected"):
        return {"link": ("offline: no answer from its rosbridge", "error")}
    out["link"] = ("online", "ok")
    b = d.get("battery") if fresh("battery") else None
    if b and b.get("reading"):
        r = b["reading"]
        lvl = (alarm(r.get("soc")) or "ok") if live(b) else "off"
        out["battery"] = (f"{r['soc']}% · {r['voltage']:.1f} V · {r['current']:+.1f} A"
                          + ("" if live(b) else " (old)"), lvl)
    elif b:
        out["battery"] = (f"BMS {b.get('state', '?')}", "warn")
    else:
        out["battery"] = ("no battery service", "off")
    gate = d.get("gate") if fresh("gate") else None
    if (gate and gate.get("engaged")) or d.get("estop"):
        out["estop"] = ("E-STOP ENGAGED", "error")
    elif gate:
        last = gate.get("last_stop")
        out["estop"] = ("clear" + (f" · stopped itself {gate.get('stops')}× (app lost)" if last else ""), "ok")
    else:
        out["estop"] = ("clear · no E-STOP service on the robot", "warn")
    st = (d.get("launcher") or {}).get("stacks", {}) if fresh("launcher") else {}
    if st:
        base = st.get("bringup", {}).get("state", "?")
        nav = st.get("nav", {})
        nav_txt = nav.get("state", "?")
        args = nav.get("args") or {}
        if nav_txt in ("starting", "running"):
            nav_txt += f" on {args.get('map')}" if args.get("mode") == "map" else " (SLAM)"
        lvl = "error" if "failed" in (base, nav.get("state")) else "ok" if base == "running" else "off"
        out["programs"] = (f"base {base} · navigation {nav_txt}", lvl)
    else:
        out["programs"] = ("no launcher", "off")
    o = d.get("odom") if fresh("odom") else None
    nav = d.get("nav", "")
    if o:
        moving = abs(o["v"]) > 0.02 or abs(o["w"]) > 0.05
        out["motion"] = (f"{'moving' if moving else 'still'} {o['v']:+.2f} m/s"
                         + (f" · {nav}" if nav else ""), "ok")
    else:
        out["motion"] = ("no odometry" + (f" · {nav}" if nav else ""), "off")
    return out


class FleetMonitor(QObject):
    updated = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.watches: Dict[str, RobotWatch] = {}
        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self.updated.emit)
        self._timer.start()

    def watch(self, profiles: list, ids: set) -> None:
        """Watch exactly these robots."""
        by_id = {p.id: p for p in profiles}
        for rid in list(self.watches):
            w = self.watches[rid]
            p = by_id.get(rid)
            if rid not in ids or p is None or (p.host, p.port, p.use_ssl, p.token) != \
                    (w.profile.host, w.profile.port, w.profile.use_ssl, w.started_token):
                w.stop()
                del self.watches[rid]
        for rid in ids:
            if rid in by_id and rid not in self.watches:
                w = RobotWatch(by_id[rid])
                self.watches[rid] = w
                w.start()
            elif rid in self.watches:
                self.watches[rid].profile = by_id[rid]

    def snapshot(self, rid: str) -> Optional[dict]:
        w = self.watches.get(rid)
        return w.snapshot() if w else None

    def estop_all(self) -> list:
        """Names of the robots the E-STOP reached."""
        return [w.profile.name for w in self.watches.values() if w.estop()]

    def stop(self) -> None:
        for w in self.watches.values():
            w.stop()
        self.watches.clear()
