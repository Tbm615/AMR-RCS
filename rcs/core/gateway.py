"""The app's side of the robot's front door (robot/rcs_gateway.py).

A robot with the gateway asks every app to sign in; the app keeps the token it
gets (30 days) in the robot's profile and sends it when it connects. Without a
gateway (older robot side) nothing changes.

With wss:// the robot's certificate is self-signed: the first time, the app
shows its fingerprint to the operator and keeps it (the profile's tls_pin);
after that a robot showing another certificate is refused, on the HTTPS calls
and on the WebSocket alike (pin_ws below).

No Qt here: the calls block, the pages run them off the GUI thread.
"""
from __future__ import annotations

import base64
import hashlib
import http.client
import json
import socket
import ssl
from typing import Dict, Optional, Tuple
from urllib.parse import quote

TIMEOUT = 6.0


class GatewayError(Exception):
    """Shown as it is. status: the HTTP status, 0 when the robot did not answer."""

    def __init__(self, text: str, status: int = 0) -> None:
        super().__init__(text)
        self.status = status


def _ctx() -> ssl.SSLContext:
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE     # self-signed: checked by its fingerprint instead
    return ctx


def fingerprint(host: str, port: int, timeout: float = TIMEOUT) -> str:
    """SHA-256 of the robot's certificate, hex."""
    with socket.create_connection((host, port), timeout=timeout) as raw:
        with _ctx().wrap_socket(raw, server_hostname=host) as s:
            return hashlib.sha256(s.getpeercert(binary_form=True)).hexdigest()


def pretty(fp: str) -> str:
    return ":".join(fp[i:i + 2] for i in range(0, len(fp), 2)).upper()


def request(host: str, port: int, path: str, tls: bool = False, pin: str = "", token: str = "",
            auth: str = "", headers: Optional[Dict[str, str]] = None, timeout: float = TIMEOUT) -> dict:
    """GET one of the gateway's /rcs/ paths; its JSON, or GatewayError."""
    try:
        if tls:
            conn = http.client.HTTPSConnection(host, port, timeout=timeout, context=_ctx())
            conn.connect()
            fp = hashlib.sha256(conn.sock.getpeercert(binary_form=True)).hexdigest()
            if pin and fp != pin:
                conn.close()
                raise GatewayError("the robot's certificate is not the one this app knows: someone may be in "
                                   "between. If the robot got a new certificate, forget the old one on the "
                                   "Connection page.", 495)
        else:
            conn = http.client.HTTPConnection(host, port, timeout=timeout)
        h = dict(headers or {})
        if token:
            h["Authorization"] = f"Bearer {token}"
        elif auth:
            h["Authorization"] = auth
        conn.request("GET", path, headers=h)
        r = conn.getresponse()
        raw = r.read()
        conn.close()
    except GatewayError:
        raise
    except (OSError, http.client.HTTPException, ssl.SSLError) as e:
        raise GatewayError(f"no answer from {host}:{port} ({e})") from e
    try:
        body = json.loads(raw.decode("utf-8")) if raw else {}
    except (UnicodeDecodeError, ValueError):
        body = {}
    if not isinstance(body, dict):
        body = {}
    if r.status != 200:
        raise GatewayError(body.get("error") or f"{r.status} {r.reason}", r.status)
    return body


def info(host: str, port: int, tls: bool = False, pin: str = "", timeout: float = 3.0) -> Optional[dict]:
    """The gateway's /rcs/info, or None: no gateway (plain rosbridge, older robot)."""
    try:
        d = request(host, port, "/rcs/info", tls=tls, pin=pin, timeout=timeout)
    except GatewayError as e:
        if e.status == 495:
            raise
        return None
    return d if d.get("rcs_gateway") else None


def login(host: str, port: int, user: str, password: str, tls: bool = False, pin: str = "") -> Tuple[str, str]:
    """(token, role)."""
    auth = "Basic " + base64.b64encode(f"{user}:{password}".encode("utf-8")).decode("ascii")
    d = request(host, port, "/rcs/login", tls=tls, pin=pin, auth=auth)
    return d["token"], d["role"]


class Account:
    """The calls for one signed-in profile."""

    def __init__(self, profile) -> None:
        self.p = profile

    def _get(self, path: str, headers=None) -> dict:
        p = self.p
        return request(p.host, p.port, path, tls=p.use_ssl, pin=p.tls_pin, token=p.token, headers=headers)

    def whoami(self) -> Tuple[str, str]:
        d = self._get("/rcs/whoami")
        return d["user"], d["role"]

    def logout(self) -> None:
        self._get("/rcs/logout")

    def users(self) -> list:
        return self._get("/rcs/users")["users"]

    def add_user(self, name: str, role: str, password: str) -> None:
        self._get(f"/rcs/users/add?name={quote(name)}&role={quote(role)}", {"X-RCS-Password": password})

    def set_role(self, name: str, role: str) -> None:
        self._get(f"/rcs/users/role?name={quote(name)}&role={quote(role)}")

    def set_password(self, name: str, password: str) -> None:
        self._get(f"/rcs/users/password?name={quote(name)}", {"X-RCS-Password": password})

    def remove_user(self, name: str) -> None:
        self._get(f"/rcs/users/remove?name={quote(name)}")

    def sign_out_user(self, name: str) -> int:
        return int(self._get(f"/rcs/users/signout?name={quote(name)}").get("signed_out", 0))

    def audit(self, since: float = 0.0) -> list:
        return self._get(f"/rcs/audit?since={since}")["rows"]


# ---------------------------------------------------------------- wss:// pinning

PINS: Dict[Tuple[str, int], str] = {}     # (host, port) -> fingerprint the WebSocket must show
_patched = False


def pin_ws(host: str, port: int, fp: str) -> None:
    """The WebSocket to host:port only opens with this certificate."""
    _patch()
    if fp:
        PINS[(host, int(port))] = fp
    else:
        PINS.pop((host, int(port)), None)


def _patch() -> None:
    """roslibpy opens wss:// with Twisted's default, which checks nothing: give
    its connections a context that checks the fingerprint."""
    global _patched
    if _patched:
        return
    try:
        import roslibpy.comm.comm_autobahn as ca
        from OpenSSL import SSL
        from twisted.internet import ssl as tssl
    except ImportError:
        return
    original = ca.connectWS

    class Pinned(tssl.ClientContextFactory):
        def __init__(self, fp: str) -> None:
            self.fp = fp

        def getContext(self):
            ctx = SSL.Context(SSL.TLS_CLIENT_METHOD)
            want = self.fp

            def verify(_conn, cert, _errno, depth, _ok):
                if depth != 0:
                    return True                 # the chain: the leaf decides
                got = cert.digest("sha256").decode("ascii").replace(":", "").lower()
                return got == want
            ctx.set_verify(SSL.VERIFY_PEER, verify)
            return ctx

    def connect_ws(factory, contextFactory=None, *args, **kwargs):
        if factory.isSecure and contextFactory is None:
            fp = PINS.get((factory.host, int(factory.port)))
            if fp:
                contextFactory = Pinned(fp)
        return original(factory, contextFactory, *args, **kwargs)

    ca.connectWS = connect_ws
    _patched = True
