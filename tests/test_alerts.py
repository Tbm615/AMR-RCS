"""Remote alerts: which events go out, not too often, by Telegram and e-mail
(both faked here: nothing is sent), failures reported without looping.

Run:
    python tests/test_alerts.py
"""
from __future__ import annotations

import io
import json
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = os.environ["APPDATA"] = tempfile.mkdtemp(prefix="rcs_alert_cfg_")

from rcs.core import alerts  # noqa: E402

PASS = FAIL = 0


def check(name, cond, extra=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def wait(cond, sec=5.0):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def main() -> int:
    print("\n[which events, how often]")
    t = [0.0]
    sent, reports = [], []
    a = alerts.Alerts({"on": True, "levels": ["error"], "telegram_bot_token": "T", "telegram_chat_id": "42"},
                      clock=lambda: t[0], report=lambda ok, m: reports.append((ok, m)),
                      telegram=lambda tok, chat, text: sent.append(("tg", chat, text)))
    check("an error goes out", a.event("error", "E-STOP", "engaged", robot="AMR 1"))
    check("by Telegram, with the robot's name", wait(lambda: sent) and "AMR 1" in sent[0][2] and sent[0][1] == "42",
          str(sent))
    check("a warning does not (errors only)", not a.event("warn", "Battery", "low"))
    check("the same error again: not within 10 min", not a.event("error", "E-STOP", "engaged"))
    t[0] += 601
    check("after 10 min it does", a.event("error", "E-STOP", "engaged"))
    for i in range(30):
        a.event("error", "Mission", f"failure {i}")
    check("at most 20 an hour", wait(lambda: len(sent) >= 20) and len(sent) <= 20 + 2, str(len(sent)))
    off = alerts.Alerts({"on": False, "levels": ["error"], "telegram_bot_token": "T", "telegram_chat_id": "1"})
    check("off: nothing", not off.event("error", "E-STOP", "engaged"))
    none = alerts.Alerts({"on": True, "levels": ["error"]})
    check("no channel set: nothing", not none.event("error", "E-STOP", "engaged"))

    print("\n[Telegram and e-mail, faked]")
    calls = []

    class Resp(io.BytesIO):
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    real_open = alerts.urllib.request.urlopen
    alerts.urllib.request.urlopen = lambda req, timeout=0: (calls.append(req), Resp(b'{"ok": true}'))[1]
    alerts.send_telegram("ABC", "7", "hello")
    check("Telegram: the bot API with chat id and text", calls and "botABC/sendMessage" in calls[0].full_url
          and b"chat_id=7" in calls[0].data and b"text=hello" in calls[0].data)
    alerts.urllib.request.urlopen = lambda req, timeout=0: Resp(b'{"ok": false, "description": "chat not found"}')
    try:
        alerts.send_telegram("ABC", "7", "hello")
        check("Telegram refusal said", False)
    except alerts.AlertError as e:
        check("Telegram refusal said", "chat not found" in str(e))
    alerts.urllib.request.urlopen = real_open
    mails = []

    class FakeSMTP:
        def __init__(self, host, port, timeout=0):
            mails.append(("connect", host, port))

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

        def starttls(self, context=None):
            mails.append(("tls",))

        def login(self, u, p):
            mails.append(("login", u))

        def send_message(self, msg):
            mails.append(("send", msg["To"], msg["Subject"], msg.get_content()))
    real_smtp = alerts.smtplib.SMTP
    alerts.smtplib.SMTP = FakeSMTP
    alerts.send_email({"smtp_host": "mail.example", "smtp_port": 587, "smtp_user": "u", "smtp_password": "p",
                       "smtp_to": "ops@example", "smtp_tls": True}, "subj", "body text")
    check("e-mail: TLS, login, to the address", ("tls",) in mails and ("login", "u") in mails
          and any(m[0] == "send" and m[1] == "ops@example" and "body text" in m[3] for m in mails), str(mails))
    alerts.smtplib.SMTP = real_smtp

    print("\n[in the app]")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    from rcs.core.profile import load_settings
    from rcs.ui.main_window import MainWindow
    w = MainWindow()
    got = []
    w.alerts._telegram = lambda tok, chat, text: got.append(text)
    w.set_alerts({"on": True, "levels": ["error", "warn"], "telegram_bot_token": "secret-bot", "telegram_chat_id": "9"})
    check("saved", load_settings().get("alerts", {}).get("telegram_chat_id") == "9")
    w._log("error", "E-STOP", "engaged from the test")
    check("an app event goes out", wait(lambda: any("engaged from the test" in g for g in got)), str(got))

    def failing(tok, chat, text):
        raise alerts.AlertError("Telegram: no network")
    w.alerts._telegram = failing
    w._log("error", "Mission", "aborted at waypoint 3")
    ok = wait(lambda: w._alert_results, 5)
    w._drain_alert_results()
    check("a failed send is in the event log", ok and any(e.source == "Alerts" and "no network" in e.text
                                                          for e in w.events.events))
    check("and does not send an alert about itself", not any("Alerts" in g for g in got))
    out = w.save_bundle(str(Path(tempfile.mkdtemp()) / "d.zip"))
    import zipfile
    with zipfile.ZipFile(out) as z:
        blob = z.read("settings.json")
    check("the bot token stays out of diagnostic bundles", b"secret-bot" not in blob)
    w.close()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
