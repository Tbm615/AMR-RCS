"""Alerts away from the screen: Telegram and/or e-mail when something goes wrong
(E-STOP, a robot that stopped itself, low battery, a failed mission, the link
lost...). They go from this PC while the app runs.

Every warning or error of the event log can go out, chosen by level. The same
text is sent at most once per REPEAT_S, and at most MAX_PER_HOUR messages an
hour, so a flapping link does not flood a phone. Sending is done on a thread
of its own: a slow mail server never holds the app up.

Settings (settings.json, key "alerts"): on, levels, robot name prefix,
telegram_bot_token, telegram_chat_id, smtp_host, smtp_port, smtp_user,
smtp_password, smtp_from, smtp_to, smtp_tls. Diagnostic bundles leave out
every key with "token" or "password" in its name.
"""
from __future__ import annotations

import json
import queue
import smtplib
import ssl
import threading
import time
import urllib.parse
import urllib.request
from email.message import EmailMessage
from typing import Callable, Optional

REPEAT_S = 600.0
MAX_PER_HOUR = 20
DEFAULTS = {"on": False, "levels": ["error"], "telegram_bot_token": "", "telegram_chat_id": "",
            "smtp_host": "", "smtp_port": 587, "smtp_user": "", "smtp_password": "", "smtp_from": "",
            "smtp_to": "", "smtp_tls": True}


class AlertError(Exception):
    pass


def send_telegram(token: str, chat_id: str, text: str, timeout: float = 10.0) -> None:
    if not token or not chat_id:
        raise AlertError("Telegram: the bot token and the chat id are both needed")
    data = urllib.parse.urlencode({"chat_id": chat_id, "text": text}).encode()
    req = urllib.request.Request(f"https://api.telegram.org/bot{token}/sendMessage", data=data)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            body = json.loads(r.read().decode("utf-8") or "{}")
    except urllib.error.HTTPError as e:
        try:
            body = json.loads(e.read().decode("utf-8") or "{}")
        except ValueError:
            body = {}
        raise AlertError(f"Telegram: {body.get('description') or e}") from e
    except (OSError, ValueError) as e:
        raise AlertError(f"Telegram: {e}") from e
    if not body.get("ok"):
        raise AlertError(f"Telegram: {body.get('description', 'refused')}")


def send_email(cfg: dict, subject: str, text: str, timeout: float = 15.0) -> None:
    host, to = cfg.get("smtp_host", ""), cfg.get("smtp_to", "")
    if not host or not to:
        raise AlertError("E-mail: the server and the address to send to are both needed")
    msg = EmailMessage()
    msg["Subject"] = subject
    msg["From"] = cfg.get("smtp_from") or cfg.get("smtp_user") or "rcs@localhost"
    msg["To"] = to
    msg.set_content(text)
    try:
        with smtplib.SMTP(host, int(cfg.get("smtp_port") or 587), timeout=timeout) as s:
            if cfg.get("smtp_tls", True):
                s.starttls(context=ssl.create_default_context())
            if cfg.get("smtp_user"):
                s.login(cfg["smtp_user"], cfg.get("smtp_password", ""))
            s.send_message(msg)
    except (OSError, smtplib.SMTPException) as e:
        raise AlertError(f"E-mail: {e}") from e


class Alerts:
    """Takes events, sends the ones chosen. report(ok, text) is called (from
    the sending thread) after each attempt."""

    def __init__(self, cfg: Optional[dict] = None, clock=time.monotonic,
                 report: Optional[Callable[[bool, str], None]] = None,
                 telegram=send_telegram, email=send_email) -> None:
        self.cfg = {**DEFAULTS, **(cfg or {})}
        self.clock = clock
        self.report = report
        self._telegram, self._email = telegram, email
        self._last: dict = {}           # text -> when sent
        self._sent: list = []           # times of the last hour's messages
        self._q: "queue.Queue" = queue.Queue()
        self._thread: Optional[threading.Thread] = None

    def configure(self, cfg: dict) -> None:
        self.cfg = {**DEFAULTS, **cfg}

    def channels(self) -> list:
        c = self.cfg
        out = []
        if c.get("telegram_bot_token") and c.get("telegram_chat_id"):
            out.append("telegram")
        if c.get("smtp_host") and c.get("smtp_to"):
            out.append("email")
        return out

    def event(self, level: str, source: str, text: str, robot: str = "") -> bool:
        """An event of the log: True if it is going out."""
        c = self.cfg
        if not c.get("on") or level not in c.get("levels", []) or not self.channels():
            return False
        line = f"[RCS{' · ' + robot if robot else ''}] {level.upper()} {source}: {text}"
        now = self.clock()
        key = (source, text)
        if now - self._last.get(key, -1e9) < REPEAT_S:
            return False
        self._sent = [t for t in self._sent if now - t < 3600]
        if len(self._sent) >= MAX_PER_HOUR:
            return False
        self._last[key] = now
        self._sent.append(now)
        self._put(line)
        return True

    def test(self) -> None:
        self._put("[RCS] test message: alerts from this PC work.")

    def _put(self, line: str) -> None:
        self._q.put((dict(self.cfg), line))
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def _run(self) -> None:
        while True:
            try:
                cfg, line = self._q.get(timeout=5.0)
            except queue.Empty:
                return
            for ch in self.channels_of(cfg):
                try:
                    if ch == "telegram":
                        self._telegram(cfg["telegram_bot_token"], cfg["telegram_chat_id"], line)
                    else:
                        self._email(cfg, line[:120], line)
                    ok, msg = True, f"sent by {ch}"
                except AlertError as e:
                    ok, msg = False, str(e)
                except Exception as e:  # noqa: BLE001 - never kill the thread
                    ok, msg = False, f"{ch}: {e}"
                if self.report:
                    self.report(ok, msg)

    @staticmethod
    def channels_of(cfg: dict) -> list:
        return Alerts(cfg).channels()
