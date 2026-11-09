"""Who may do what with the robot: users, sign-in tokens, the rules per role and
the audit log. No ROS and no network here: rcs_gateway.py uses it, the tests
run it as it is.

Roles, each with everything of the one before:
  viewer     sees everything; may stop the robot (E-STOP, cancel the goal)
  operator   drives: teleop, goals, initial pose, missions, start/stop
             bringup and Nav2, save and send maps, release the E-STOP
  admin      users, deleting battery logs, any other topic or service

Anything not listed is admin only: a new topic is closed until someone opens it.

Users file (rcs_users.json, mode 600): passwords as PBKDF2-SHA256 with a salt
of their own, tokens as their SHA-256 only, so the file does not let anyone in.
"""
from __future__ import annotations

import base64
import hashlib
import hmac
import json
import os
import secrets
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

ROLES = ("viewer", "operator", "admin")
RANK = {r: i for i, r in enumerate(ROLES)}
PBKDF2_ROUNDS = 200_000
TOKEN_DAYS = 30
NAME_OK = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-")
MIN_PASSWORD = 8


class AccessError(ValueError):
    """Shown to the person as it is."""


def _hash(password: str, salt: bytes, rounds: int = PBKDF2_ROUNDS) -> str:
    return hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, rounds).hex()


def _token_id(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def check_name(name: str) -> str:
    name = str(name).strip()
    if not 1 <= len(name) <= 40 or not set(name) <= NAME_OK:
        raise AccessError("a user name is 1 to 40 letters, digits, '.', '_' or '-'")
    return name


def check_password(password: str) -> str:
    if len(password) < MIN_PASSWORD:
        raise AccessError(f"a password has at least {MIN_PASSWORD} characters")
    return password


class Users:
    def __init__(self, path, clock=time.time) -> None:
        self.path = Path(path)
        self.clock = clock
        self._lock = threading.Lock()
        self.data = {"users": {}, "tokens": {}}
        self._mtime = None
        self.reload()

    # ---- file ----

    def reload(self) -> None:
        """Read again if the file changed (users added from the command line)."""
        try:
            st = self.path.stat()
        except OSError:
            return
        if self._mtime == st.st_mtime_ns:
            return
        try:
            d = json.loads(self.path.read_text())
        except (OSError, ValueError):
            return
        if isinstance(d, dict):
            self.data = {"users": dict(d.get("users") or {}), "tokens": dict(d.get("tokens") or {})}
            self._mtime = st.st_mtime_ns

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_name(self.path.name + ".tmp")
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as f:
            json.dump(self.data, f, indent=1)
        os.replace(tmp, self.path)
        try:
            os.chmod(self.path, 0o600)
        except OSError:
            pass
        self._mtime = self.path.stat().st_mtime_ns

    def enabled(self) -> bool:
        """Sign-in is asked for once there is a user."""
        self.reload()
        return bool(self.data["users"])

    # ---- users ----

    def add(self, name: str, password: str, role: str) -> None:
        name = check_name(name)
        if role not in ROLES:
            raise AccessError(f"role is one of {', '.join(ROLES)}")
        with self._lock:
            self.reload()
            if name in self.data["users"]:
                raise AccessError(f"{name} exists already")
            salt = secrets.token_bytes(16)
            self.data["users"][name] = {"role": role, "salt": salt.hex(), "rounds": PBKDF2_ROUNDS,
                                        "hash": _hash(check_password(password), salt), "created": self.clock()}
            self.save()

    def set_password(self, name: str, password: str) -> None:
        with self._lock:
            self.reload()
            u = self._user(name)
            salt = secrets.token_bytes(16)
            u.update(salt=salt.hex(), rounds=PBKDF2_ROUNDS, hash=_hash(check_password(password), salt))
            self._revoke(name)                  # a new password signs everyone out
            self.save()

    def set_role(self, name: str, role: str) -> None:
        if role not in ROLES:
            raise AccessError(f"role is one of {', '.join(ROLES)}")
        with self._lock:
            self.reload()
            u = self._user(name)
            if u["role"] == "admin" and role != "admin" and self._admins() == 1:
                raise AccessError("the last admin stays an admin")
            u["role"] = role
            self.save()

    def remove(self, name: str) -> None:
        with self._lock:
            self.reload()
            u = self._user(name)
            if u["role"] == "admin" and self._admins() == 1:
                raise AccessError("the last admin cannot be removed")
            del self.data["users"][name]
            self._revoke(name)
            self.save()

    def list(self) -> list:
        self.reload()
        now = self.clock()
        out = []
        for name, u in sorted(self.data["users"].items()):
            toks = [t for t in self.data["tokens"].values() if t["user"] == name and t["expires"] > now]
            out.append({"name": name, "role": u["role"], "created": u.get("created"),
                        "signed_in": len(toks), "last_used": max((t.get("last_used") or 0 for t in toks), default=0)})
        return out

    def _user(self, name: str) -> dict:
        u = self.data["users"].get(name)
        if u is None:
            raise AccessError(f"no user {name}")
        return u

    def _admins(self) -> int:
        return sum(1 for u in self.data["users"].values() if u["role"] == "admin")

    # ---- signing in ----

    def login(self, name: str, password: str, days: float = TOKEN_DAYS) -> Tuple[str, str]:
        """(token, role); raises AccessError with one reason for any failure."""
        with self._lock:
            self.reload()
            u = self.data["users"].get(str(name))
            # the same work for a wrong name: no telling names apart by time
            salt = bytes.fromhex(u["salt"]) if u else b"\0" * 16
            rounds = int(u.get("rounds", PBKDF2_ROUNDS)) if u else PBKDF2_ROUNDS
            ok = hmac.compare_digest(_hash(password, salt, rounds), u["hash"] if u else "0" * 64)
            if not (u and ok):
                raise AccessError("wrong user name or password")
            token = secrets.token_urlsafe(32)
            now = self.clock()
            self.data["tokens"] = {k: t for k, t in self.data["tokens"].items() if t["expires"] > now}
            self.data["tokens"][_token_id(token)] = {"user": name, "created": now, "expires": now + days * 86400,
                                                     "last_used": now}
            self.save()
            return token, u["role"]

    def check(self, token: Optional[str]) -> Optional[Tuple[str, str]]:
        """(user, role) for a live token, else None."""
        if not token:
            return None
        self.reload()
        t = self.data["tokens"].get(_token_id(token))
        now = self.clock()
        if t is None or t["expires"] <= now:
            return None
        u = self.data["users"].get(t["user"])
        if u is None:
            return None
        if now - (t.get("last_used") or 0) > 600:       # written now and then, not on every connect
            with self._lock:
                t["last_used"] = now
                try:
                    self.save()
                except OSError:
                    pass
        return t["user"], u["role"]

    def logout(self, token: str) -> None:
        with self._lock:
            self.reload()
            if self.data["tokens"].pop(_token_id(token), None) is not None:
                self.save()

    def revoke(self, name: str) -> int:
        with self._lock:
            self.reload()
            n = self._revoke(name)
            self.save()
            return n

    def _revoke(self, name: str) -> int:
        gone = [k for k, t in self.data["tokens"].items() if t["user"] == name]
        for k in gone:
            del self.data["tokens"][k]
        return len(gone)


def basic_auth(header: str) -> Tuple[str, str]:
    """(user, password) of an "Authorization: Basic ..." header."""
    if not header or not header.lower().startswith("basic "):
        raise AccessError("sign in with a user name and password")
    try:
        user, _, password = base64.b64decode(header[6:].strip()).decode("utf-8").partition(":")
    except (ValueError, UnicodeDecodeError) as e:
        raise AccessError("sign in with a user name and password") from e
    return user, password


def bearer(header: str) -> Optional[str]:
    if header and header.lower().startswith("bearer "):
        return header[7:].strip() or None
    return None


class Throttle:
    """Slows down guessing: after FAILS wrong passwords from one address within
    WINDOW s, that address waits LOCK s."""
    FAILS, WINDOW, LOCK = 5, 300.0, 60.0

    def __init__(self, clock=time.monotonic) -> None:
        self.clock = clock
        self.fails: Dict[str, list] = {}
        self.locked: Dict[str, float] = {}

    def wait(self, who: str) -> float:
        left = self.locked.get(who, 0.0) - self.clock()
        return max(0.0, left)

    def failed(self, who: str) -> None:
        now = self.clock()
        f = [t for t in self.fails.get(who, []) if now - t < self.WINDOW] + [now]
        self.fails[who] = f
        if len(f) >= self.FAILS:
            self.locked[who] = now + self.LOCK
            self.fails[who] = []

    def ok(self, who: str) -> None:
        self.fails.pop(who, None)
        self.locked.pop(who, None)


# ---------------------------------------------------------------- the rules

class Rules:
    """What a role may send to the robot, one rosbridge message at a time."""

    def __init__(self, estop: str = "/amr/estop",
                 cmd_vel=("/cmd_vel_safe", "/cmd_vel", "/cmd_vel_nav", "/cmd_vel_teleop"),
                 goal: str = "/goal_pose", initial_pose: str = "/initialpose",
                 nav_action: str = "/navigate_to_pose") -> None:
        self.estop = estop
        self.cmd_vel = set(cmd_vel)
        self.goal = goal
        self.initial_pose = initial_pose
        self.nav_action = nav_action
        self.viewer_topics = {"/rcs/app/heartbeat"}
        self.read_services = {"/rosapi/topics", "/rosapi/services", "/rosapi/nodes", "/rosapi/topic_type",
                              "/rosapi/topics_for_type", "/rosapi/service_type", "/rosapi/get_time",
                              "/rosapi/topics_and_raw_types", "/map_server/map"}
        self.launcher = {"start": "operator", "stop": "operator", "save_map": "operator",
                         "map_preview": "viewer", "map_get": "viewer", "map_put": "operator"}
        self.battery = {"log_list": "viewer", "log_get": "viewer", "log_delete": "admin"}

    def need(self, msg: dict) -> Tuple[str, str]:
        """(least role, what it is, for the audit: "" = not worth a line)."""
        op = msg.get("op")
        if op in ("subscribe", "unsubscribe", "advertise", "unadvertise", "set_level", "status",
                  "advertise_service", "unadvertise_service"):
            return ("viewer" if op not in ("advertise_service", "unadvertise_service") else "admin"), ""
        if op == "publish":
            return self._publish(str(msg.get("topic", "")), msg.get("msg") or {})
        if op == "call_service":
            s = str(msg.get("service", ""))
            if s in self.read_services:
                return "viewer", ""
            if s == f"{self.nav_action}/_action/cancel_goal":
                return "viewer", "cancelled the navigation goal"
            return "admin", f"called {s}"
        if op == "send_action_goal":
            return "operator", f"sent an action goal to {msg.get('action', '?')}"
        if op == "cancel_action_goal":
            return "viewer", f"cancelled action {msg.get('action', '?')}"
        return "admin", f"sent {op!r}"

    def _publish(self, topic: str, m: dict) -> Tuple[str, str]:
        if topic == self.estop:
            return ("viewer", "E-STOP engaged") if m.get("data") else ("operator", "E-STOP released")
        if topic in self.viewer_topics:
            return "viewer", ""
        if topic in self.cmd_vel:
            return "operator", ""                       # teleop: summed up by the session
        if topic == self.goal:
            p = (m.get("pose") or {}).get("position") or {}
            return "operator", f"goal x {_num(p.get('x'))} y {_num(p.get('y'))}"
        if topic == self.initial_pose:
            p = ((m.get("pose") or {}).get("pose") or {}).get("position") or {}
            return "operator", f"initial pose x {_num(p.get('x'))} y {_num(p.get('y'))}"
        if topic in ("/rcs/launcher/cmd", "/rcs/battery/cmd"):
            try:
                cmd = json.loads(m.get("data", ""))
            except (TypeError, ValueError):
                return "admin", f"bad command on {topic}"
            if not isinstance(cmd, dict):
                return "admin", f"bad command on {topic}"
            table = self.launcher if topic == "/rcs/launcher/cmd" else self.battery
            op = str(cmd.get("op", ""))
            role = table.get(op, "admin")
            if op in ("map_get", "map_put") and cmd.get("seq", 0):
                return role, ""                         # one line per map, not per piece
            what = {"start": f"started {cmd.get('stack')}" + (f" on {cmd.get('map')}" if cmd.get("map") else ""),
                    "stop": f"stopped {cmd.get('stack')}", "save_map": f"saved map {cmd.get('name')}",
                    "map_put": f"sent map {cmd.get('name')} to the robot",
                    "map_get": f"copied map {cmd.get('name')} from the robot",
                    "log_delete": f"deleted battery log {cmd.get('name')}"}.get(op, "")
            return role, what
        return "admin", f"published on {topic}"

    @staticmethod
    def allows(role: str, need: str) -> bool:
        return RANK.get(role, -1) >= RANK[need]


def _num(v) -> str:
    try:
        return f"{float(v):.2f}"
    except (TypeError, ValueError):
        return "?"


class Audit:
    """One JSON line per thing done, a file per month, kept until deleted."""

    def __init__(self, folder, clock=time.time) -> None:
        self.folder = Path(folder)
        self.clock = clock
        self._lock = threading.Lock()

    def write(self, user: str, role: str, addr: str, what: str, ok: bool = True) -> None:
        t = self.clock()
        line = json.dumps({"t": round(t, 3), "user": user, "role": role, "from": addr, "what": what, "ok": ok})
        path = self.folder / f"audit-{time.strftime('%Y-%m', time.localtime(t))}.jsonl"
        with self._lock:
            try:
                self.folder.mkdir(parents=True, exist_ok=True)
                with open(path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except OSError:
                pass

    def read(self, since: float = 0.0, limit: int = 5000) -> list:
        rows = []
        for path in sorted(self.folder.glob("audit-*.jsonl")):
            try:
                for line in path.read_text(encoding="utf-8").splitlines():
                    try:
                        r = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(r, dict) and r.get("t", 0) >= since:
                        rows.append(r)
            except OSError:
                continue
        return rows[-limit:]
