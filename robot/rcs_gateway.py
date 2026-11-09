#!/usr/bin/env python3
"""The robot's front door for RCS apps: sign-in, a role per user, an audit log.

rosbridge listens on 127.0.0.1 only; this takes the public port (9090) and
passes on what each signed-in user may do (rcs_access.py has the rules).
Without it anyone on the robot's network could drive the robot.

  app --ws (Authorization: Bearer <token>)--> rcs_gateway :9090 --> rosbridge 127.0.0.1:9091

On the same port, plain HTTP GET:
  /rcs/info                         is this a gateway, is sign-in on, TLS
  /rcs/login      Basic user:pass   a token for 30 days and the role
  /rcs/logout, /rcs/whoami          Bearer
  /rcs/users[/add|/role|/password|/remove]?name=&role=   admin (password in X-RCS-Password)
  /rcs/audit?since=<unix time>      admin

Messages the app cannot send for its role are not passed on: the app hears why
on /rcs/gateway/denied, and its role on /rcs/gateway/session.

Run (start_rcs_robot.sh does, once there is a user):
  rcs_gateway.py serve [--listen 0.0.0.0:9090] [--upstream ws://127.0.0.1:9091] [--cert C --key K]
  rcs_gateway.py users add NAME --role admin|operator|viewer      (asks the password)
  rcs_gateway.py users list | passwd NAME | role NAME ROLE | remove NAME | signout NAME
  rcs_gateway.py gencert                                           (self-signed, for wss://)
Files: $RCS_WS/rcs_users.json, $RCS_WS/audit/, $RCS_WS/rcs_tls/
"""
from __future__ import annotations

import argparse
import asyncio
import getpass
import json
import logging
import os
import ssl
import subprocess
import sys
import time
from http import HTTPStatus
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcs_access import ROLES, AccessError, Audit, Rules, Throttle, Users, basic_auth, bearer  # noqa: E402

VERSION = 1
WS = Path(os.environ.get("RCS_WS", str(Path.home() / "tbm_test"))).expanduser()
USERS = Path(os.environ.get("RCS_USERS", str(WS / "rcs_users.json")))
AUDIT = Path(os.environ.get("RCS_AUDIT", str(WS / "audit")))
TLS_DIR = WS / "rcs_tls"
SESSION_TOPIC = "/rcs/gateway/session"
DENIED_TOPIC = "/rcs/gateway/denied"
TELEOP_IDLE_S = 2.0
RECHECK_S = 5.0
LOG = logging.getLogger("rcs_gateway")


def _json_response(connection, status: HTTPStatus, body: dict):
    from websockets.datastructures import Headers
    from websockets.http11 import Response
    data = json.dumps(body).encode("utf-8")
    headers = Headers([("Content-Type", "application/json"), ("Content-Length", str(len(data))),
                       ("Cache-Control", "no-store")])
    return Response(status.value, status.phrase, headers, data)


class Gateway:
    def __init__(self, users: Users, audit: Audit, rules: Rules, upstream: str, tls: bool = False) -> None:
        self.users, self.audit, self.rules = users, audit, rules
        self.upstream = upstream
        self.tls = tls
        self.throttle = Throttle()
        self.sessions: set = set()
        self._denied_logged: dict = {}

    # ---- HTTP ----

    async def process_request(self, connection, request):
        url = urlparse(request.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        addr = _addr(connection)
        auth = request.headers.get("Authorization", "")
        if url.path == "/rcs/info":
            return _json_response(connection, HTTPStatus.OK, {"rcs_gateway": VERSION, "login": True, "tls": self.tls,
                                                              "users": self.users.enabled()})
        if url.path == "/rcs/login":
            return await self._login(connection, auth, addr)
        if url.path.startswith("/rcs/"):
            who = self.users.check(bearer(auth))
            if who is None:
                return _json_response(connection, HTTPStatus.UNAUTHORIZED, {"ok": False, "error": "sign in first"})
            return await self._api(connection, url.path, q, request.headers, who, addr, bearer(auth))
        # a WebSocket: the token decides
        who = self.users.check(bearer(auth))
        if who is None:
            msg = "sign in first" if self.users.enabled() else "no users yet: add one on the robot (rcs_gateway.py)"
            return _json_response(connection, HTTPStatus.UNAUTHORIZED, {"ok": False, "error": msg})
        connection.rcs_who = who
        connection.rcs_token = bearer(auth)
        return None

    async def _login(self, connection, auth: str, addr: str):
        wait = self.throttle.wait(addr)
        if wait:
            return _json_response(connection, HTTPStatus.TOO_MANY_REQUESTS,
                                  {"ok": False, "error": f"too many wrong passwords: wait {wait:.0f} s"})
        try:
            user, password = basic_auth(auth)
            token, role = await asyncio.get_running_loop().run_in_executor(None, self.users.login, user, password)
        except AccessError as e:
            self.throttle.failed(addr)
            self.audit.write(locals().get("user", "?") or "?", "", addr, "sign-in refused: wrong user or password",
                             ok=False)
            return _json_response(connection, HTTPStatus.UNAUTHORIZED, {"ok": False, "error": str(e)})
        self.throttle.ok(addr)
        self.audit.write(user, role, addr, "signed in")
        return _json_response(connection, HTTPStatus.OK, {"ok": True, "token": token, "user": user, "role": role,
                                                          "days": 30})

    async def _api(self, connection, path: str, q: dict, headers, who, addr: str, token: str):
        user, role = who
        ok = lambda **kw: _json_response(connection, HTTPStatus.OK, {"ok": True, **kw})     # noqa: E731
        try:
            if path == "/rcs/whoami":
                return ok(user=user, role=role)
            if path == "/rcs/logout":
                self.users.logout(token)
                self.audit.write(user, role, addr, "signed out")
                return ok()
            if path == "/rcs/users/password" and q.get("name", user) == user:
                pw = headers.get("X-RCS-Password", "")
                await asyncio.get_running_loop().run_in_executor(None, self.users.set_password, user, pw)
                self.audit.write(user, role, addr, "changed own password")
                return ok()
            if role != "admin":
                return _json_response(connection, HTTPStatus.FORBIDDEN, {"ok": False, "error": "admins only"})
            name = q.get("name", "")
            if path == "/rcs/users":
                return ok(users=self.users.list(), roles=list(ROLES))
            if path == "/rcs/users/add":
                pw = headers.get("X-RCS-Password", "")
                await asyncio.get_running_loop().run_in_executor(None, self.users.add, name, pw, q.get("role", ""))
                self.audit.write(user, role, addr, f"added user {name} ({q.get('role')})")
                return ok()
            if path == "/rcs/users/role":
                self.users.set_role(name, q.get("role", ""))
                self.audit.write(user, role, addr, f"made {name} {q.get('role')}")
                return ok()
            if path == "/rcs/users/password":
                pw = headers.get("X-RCS-Password", "")
                await asyncio.get_running_loop().run_in_executor(None, self.users.set_password, name, pw)
                self.audit.write(user, role, addr, f"set a new password for {name}")
                return ok()
            if path == "/rcs/users/remove":
                self.users.remove(name)
                self.audit.write(user, role, addr, f"removed user {name}")
                return ok()
            if path == "/rcs/users/signout":
                n = self.users.revoke(name)
                self.audit.write(user, role, addr, f"signed {name} out everywhere")
                return ok(signed_out=n)
            if path == "/rcs/audit":
                try:
                    since = float(q.get("since", 0))
                except ValueError:
                    since = 0.0
                return ok(rows=self.audit.read(since, limit=int(q.get("limit", 5000) or 5000)))
        except AccessError as e:
            return _json_response(connection, HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(e)})
        return _json_response(connection, HTTPStatus.NOT_FOUND, {"ok": False, "error": "no such thing"})

    # ---- WebSocket ----

    async def handler(self, connection) -> None:
        from websockets.asyncio.client import connect
        from websockets.exceptions import ConnectionClosed
        user, role = connection.rcs_who
        addr = _addr(connection)
        s = Session(self, connection, user, role, addr)
        self.sessions.add(s)
        self.audit.write(user, role, addr, "connected")
        try:
            async with connect(self.upstream, max_size=None, ping_interval=None, open_timeout=10) as up:
                s.up = up
                tasks = [asyncio.create_task(s.downstream()), asyncio.create_task(s.upstream()),
                         asyncio.create_task(s.watch())]
                done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                for t in pending:
                    t.cancel()
                for t in done:
                    if t.exception() and not isinstance(t.exception(), ConnectionClosed):
                        LOG.warning("session %s: %r", user, t.exception())
        except (OSError, asyncio.TimeoutError) as e:
            LOG.warning("rosbridge not reachable at %s: %s", self.upstream, e)
            await connection.close(1013, "rosbridge is not running on the robot")
        finally:
            s.teleop_end()
            self.sessions.discard(s)
            self.audit.write(user, s.role, addr, "disconnected")

    def denied_once(self, user: str, what: str) -> bool:
        """Log a refusal once per 10 s per user and thing: teleop sends 20 a second."""
        key, now = (user, what), time.monotonic()
        if now - self._denied_logged.get(key, -1e9) < 10.0:
            return False
        self._denied_logged[key] = now
        return True


class Session:
    def __init__(self, gw: Gateway, conn, user: str, role: str, addr: str) -> None:
        self.gw, self.conn, self.user, self.role, self.addr = gw, conn, user, role, addr
        self.up = None
        self.teleop_since: Optional[float] = None
        self.teleop_last = 0.0

    async def downstream(self) -> None:
        """App -> robot: each message checked."""
        async for raw in self.conn:
            if isinstance(raw, bytes):
                if self.role == "admin":
                    await self.up.send(raw)
                continue
            try:
                msg = json.loads(raw)
            except ValueError:
                continue
            if not isinstance(msg, dict):
                continue
            op, topic = msg.get("op"), msg.get("topic")
            if op == "subscribe" and topic in (SESSION_TOPIC, DENIED_TOPIC):
                if topic == SESSION_TOPIC:
                    await self.tell(SESSION_TOPIC, {"user": self.user, "role": self.role, "gateway": VERSION})
                continue                                # ours: nothing for rosbridge
            if op == "unsubscribe" and topic in (SESSION_TOPIC, DENIED_TOPIC):
                continue
            need, what = self.gw.rules.need(msg)
            if not Rules.allows(self.role, need):
                text = what or f"{op} {topic or msg.get('service') or ''}".strip()
                if self.gw.denied_once(self.user, text):
                    self.gw.audit.write(self.user, self.role, self.addr, f"refused ({need} only): {text}", ok=False)
                    await self.tell(DENIED_TOPIC, {"what": text, "need": need, "role": self.role})
                continue
            if op == "publish" and topic in self.gw.rules.cmd_vel:
                self.teleop((msg.get("msg") or {}))
            if what:
                self.gw.audit.write(self.user, self.role, self.addr, what)
            await self.up.send(raw)

    async def upstream(self) -> None:
        """Robot -> app: as it comes."""
        async for raw in self.up:
            await self.conn.send(raw)

    async def watch(self) -> None:
        """A user removed, signed out or given another role: it applies now."""
        while True:
            await asyncio.sleep(1.0)
            if self.teleop_since is not None and time.monotonic() - self.teleop_last > TELEOP_IDLE_S:
                self.teleop_end()
            self._recheck = getattr(self, "_recheck", 0) + 1
            if self._recheck % int(RECHECK_S) == 0:
                who = self.gw.users.check(self.conn.rcs_token)
                if who is None:
                    self.gw.audit.write(self.user, self.role, self.addr, "dropped: signed out or removed")
                    await self.conn.close(4001, "signed out")
                    return
                if who[1] != self.role:
                    self.role = who[1]
                    await self.tell(SESSION_TOPIC, {"user": self.user, "role": self.role, "gateway": VERSION})

    async def tell(self, topic: str, data: dict) -> None:
        await self.conn.send(json.dumps({"op": "publish", "topic": topic, "msg": {"data": json.dumps(data)}}))

    def teleop(self, twist: dict) -> None:
        lin = abs(float(((twist.get("linear") or {}).get("x")) or 0.0))
        ang = abs(float(((twist.get("angular") or {}).get("z")) or 0.0))
        if lin < 1e-4 and ang < 1e-4:
            return
        now = time.monotonic()
        if self.teleop_since is None:
            self.teleop_since = now
            self.gw.audit.write(self.user, self.role, self.addr, "teleop started")
        self.teleop_last = now

    def teleop_end(self) -> None:
        if self.teleop_since is not None:
            took = self.teleop_last - self.teleop_since
            self.gw.audit.write(self.user, self.role, self.addr, f"teleop ended ({took:.0f} s)")
            self.teleop_since = None


def _addr(connection) -> str:
    try:
        host = connection.remote_address[0]
    except (TypeError, IndexError, AttributeError):
        host = "?"
    return str(host)


async def serve(listen: str, upstream: str, cert: Optional[str], key: Optional[str],
                users: Users, audit: Audit, ready=None) -> None:
    from websockets.asyncio.server import serve as ws_serve
    host, _, port = listen.rpartition(":")
    ctx = None
    if cert and key:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert, key)
    gw = Gateway(users, audit, Rules(), upstream, tls=ctx is not None)
    async with ws_serve(gw.handler, host or "0.0.0.0", int(port), process_request=gw.process_request,
                        ssl=ctx, max_size=None, ping_interval=20, ping_timeout=20) as server:
        LOG.info("RCS gateway on %s%s -> %s, %d users", listen, " (TLS)" if ctx else "", upstream,
                 len(users.list()))
        if ready is not None:
            ready(server, gw)
        await asyncio.Future()


# ---------------------------------------------------------------- command line

def _ask_password() -> str:
    if not sys.stdin.isatty():
        return sys.stdin.readline().rstrip("\n")
    a = getpass.getpass("password: ")
    if a != getpass.getpass("again: "):
        raise AccessError("the two passwords differ")
    return a


def gencert(folder: Path) -> None:
    folder.mkdir(parents=True, exist_ok=True)
    key, cert = folder / "key.pem", folder / "cert.pem"
    subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "3650",
                    "-subj", "/CN=rcs-robot", "-keyout", str(key), "-out", str(cert)], check=True,
                   capture_output=True)
    os.chmod(key, 0o600)
    fp = subprocess.run(["openssl", "x509", "-in", str(cert), "-noout", "-fingerprint", "-sha256"],
                        capture_output=True, text=True, check=True).stdout.strip()
    print(f"{cert}\n{key}\n{fp}\nThe app shows this fingerprint the first time it connects: check it matches.")


def main(argv=None) -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] [rcs_gateway] %(message)s")
    ap = argparse.ArgumentParser(prog="rcs_gateway.py")
    sub = ap.add_subparsers(dest="cmd")
    sv = sub.add_parser("serve")
    sv.add_argument("--listen", default="0.0.0.0:9090")
    sv.add_argument("--upstream", default="ws://127.0.0.1:9091")
    sv.add_argument("--cert", default=str(TLS_DIR / "cert.pem") if (TLS_DIR / "cert.pem").exists() else None)
    sv.add_argument("--key", default=str(TLS_DIR / "key.pem") if (TLS_DIR / "key.pem").exists() else None)
    us = sub.add_parser("users")
    us.add_argument("action", choices=("add", "list", "passwd", "role", "remove", "signout"))
    us.add_argument("name", nargs="?")
    us.add_argument("role_arg", nargs="?")
    us.add_argument("--role", default=None)
    sub.add_parser("gencert")
    a = ap.parse_args(argv)
    users = Users(USERS)
    try:
        if a.cmd == "users":
            if a.action == "list":
                for u in users.list():
                    print(f"{u['name']:20s} {u['role']:9s} signed in on {u['signed_in']} device(s)")
                return 0
            if not a.name:
                raise AccessError("which user?")
            if a.action == "add":
                users.add(a.name, _ask_password(), a.role or a.role_arg or "operator")
            elif a.action == "passwd":
                users.set_password(a.name, _ask_password())
            elif a.action == "role":
                users.set_role(a.name, a.role or a.role_arg or "")
            elif a.action == "remove":
                users.remove(a.name)
            elif a.action == "signout":
                print(f"{users.revoke(a.name)} sign-in(s) ended")
            Audit(AUDIT).write("(robot console)", "admin", "localhost", f"users {a.action} {a.name}")
            print("done")
            return 0
        if a.cmd == "gencert":
            gencert(TLS_DIR)
            return 0
        if a.cmd in (None, "serve"):
            if a.cmd is None:
                a = ap.parse_args(["serve"])
            asyncio.run(serve(a.listen, a.upstream, a.cert, a.key, users, Audit(AUDIT)))
            return 0
    except AccessError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
