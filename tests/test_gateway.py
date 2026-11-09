"""The robot's front door (robot/rcs_gateway.py): sign-in, roles, the audit log,
in front of a mock rosbridge. The app itself through it is at the end.

Run:
    python tests/test_gateway.py
"""
from __future__ import annotations

import asyncio
import base64
import json
import os
import shutil
import ssl
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "robot"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
CFG = tempfile.mkdtemp(prefix="rcs_gw_cfg_")
os.environ["XDG_CONFIG_HOME"] = CFG
os.environ["APPDATA"] = CFG

import roslibpy  # noqa: E402

import rcs_access  # noqa: E402
import rcs_gateway  # noqa: E402
from tests import mock_rosbridge  # noqa: E402

UP, GW, GW_TLS = 9211, 9212, 9213
TMP = Path(tempfile.mkdtemp(prefix="rcs_gw_tmp_"))
PASS = FAIL = 0
GATEWAYS: dict = {}


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def wait(cond, timeout=8.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def http(path: str, port: int = GW, auth: str = "", headers=None, tls: bool = False):
    req = urllib.request.Request(f"{'https' if tls else 'http'}://127.0.0.1:{port}{path}")
    if auth:
        req.add_header("Authorization", auth)
    for k, v in (headers or {}).items():
        req.add_header(k, v)
    ctx = None
    if tls:
        ctx = ssl.create_default_context()
        ctx.check_hostname, ctx.verify_mode = False, ssl.CERT_NONE
    try:
        with urllib.request.urlopen(req, timeout=10, context=ctx) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def basic(user: str, pw: str) -> str:
    return "Basic " + base64.b64encode(f"{user}:{pw}".encode()).decode()


def login(user: str, pw: str, port: int = GW, tls: bool = False):
    st, body = http("/rcs/login", port, basic(user, pw), tls=tls)
    return body.get("token") if st == 200 else None


class Client:
    """A rosbridge client through the gateway, like the app."""

    def __init__(self, token, port=GW, tls=False):
        self.session, self.denied = [], []
        headers = {"Authorization": f"Bearer {token}"} if token else None
        self.ros = roslibpy.Ros("127.0.0.1", port, is_secure=tls, headers=headers)
        self.ros.run(timeout=5) if token else None

    def connected(self) -> bool:
        return bool(self.ros.is_connected)

    def listen(self):
        roslibpy.Topic(self.ros, rcs_gateway.SESSION_TOPIC, "std_msgs/msg/String").subscribe(
            lambda m: self.session.append(json.loads(m["data"])))
        roslibpy.Topic(self.ros, rcs_gateway.DENIED_TOPIC, "std_msgs/msg/String").subscribe(
            lambda m: self.denied.append(json.loads(m["data"])))

    def pub(self, topic, typ, msg):
        t = roslibpy.Topic(self.ros, topic, typ)
        t.publish(roslibpy.Message(msg))

    def close(self):
        try:
            self.ros.close()
        except Exception:  # noqa: BLE001
            pass


def published(topic: str) -> list:
    return [m for m in mock_rosbridge.HUB.received_publishes if m.get("topic") == topic]


def start_servers(users, audit, cert=None, key=None):
    threading.Thread(target=lambda: asyncio.run(mock_rosbridge.main_async("127.0.0.1", UP, duration=None)),
                     daemon=True).start()
    ready = threading.Event()
    gws = {}

    def run(port, c, k):
        asyncio.run(rcs_gateway.serve(f"127.0.0.1:{port}", f"ws://127.0.0.1:{UP}", c, k, users, audit,
                                      ready=lambda s, gw: (gws.__setitem__(port, gw), ready.set())))
    threading.Thread(target=run, args=(GW, None, None), daemon=True).start()
    ready.wait(10)
    if cert:
        ready.clear()
        threading.Thread(target=run, args=(GW_TLS, cert, key), daemon=True).start()
        ready.wait(10)
    time.sleep(0.5)
    return gws


def test_access() -> None:
    print("\n[users and rules, no network]")
    t = [1000.0]
    u = rcs_access.Users(TMP / "u.json", clock=lambda: t[0])
    check("no users: sign-in not on yet", not u.enabled())
    u.add("boss", "correct horse", "admin")
    check("file only for its owner", os.name == "nt" or (os.stat(TMP / "u.json").st_mode & 0o077) == 0)
    raw = (TMP / "u.json").read_text()
    check("no password in the file", "correct horse" not in raw)
    tok, role = u.login("boss", "correct horse")
    check("signs in", role == "admin" and u.check(tok) == ("boss", "admin"))
    check("no token in the file either", tok not in (TMP / "u.json").read_text())
    for bad in (("boss", "wrong pass"), ("nobody", "correct horse")):
        try:
            u.login(*bad)
            check(f"refused {bad[0]}", False)
        except rcs_access.AccessError as e:
            check(f"refused {bad[0]}, one reason for both", str(e) == "wrong user name or password")
    try:
        u.add("x", "short", "viewer")
        check("short password refused", False)
    except rcs_access.AccessError:
        check("short password refused", True)
    try:
        u.add("../etc", "long enough pw", "viewer")
        check("odd name refused", False)
    except rcs_access.AccessError:
        check("odd name refused", True)
    for fn, what in ((lambda: u.remove("boss"), "removed"), (lambda: u.set_role("boss", "viewer"), "demoted")):
        try:
            fn()
            check(f"the last admin cannot be {what}", False)
        except rcs_access.AccessError:
            check(f"the last admin cannot be {what}", True)
    t[0] += 31 * 86400
    check("a token runs out after 30 days", u.check(tok) is None)
    tok, _ = u.login("boss", "correct horse")
    u.set_password("boss", "another long one")
    check("a new password signs everyone out", u.check(tok) is None)
    th = rcs_access.Throttle(clock=lambda: t[0])
    for _ in range(5):
        th.failed("1.2.3.4")
    check("five wrong passwords: wait", th.wait("1.2.3.4") > 0 and th.wait("5.6.7.8") == 0)
    r = rcs_access.Rules()
    cases = [
        ({"op": "subscribe", "topic": "/map"}, "viewer"),
        ({"op": "publish", "topic": "/amr/estop", "msg": {"data": True}}, "viewer"),
        ({"op": "publish", "topic": "/amr/estop", "msg": {"data": False}}, "operator"),
        ({"op": "publish", "topic": "/cmd_vel_safe", "msg": {}}, "operator"),
        ({"op": "publish", "topic": "/goal_pose", "msg": {}}, "operator"),
        ({"op": "publish", "topic": "/rcs/launcher/cmd", "msg": {"data": '{"op": "start", "stack": "nav"}'}},
         "operator"),
        ({"op": "publish", "topic": "/rcs/launcher/cmd", "msg": {"data": '{"op": "map_preview"}'}}, "viewer"),
        ({"op": "publish", "topic": "/rcs/battery/cmd", "msg": {"data": '{"op": "log_delete"}'}}, "admin"),
        ({"op": "publish", "topic": "/rcs/battery/cmd", "msg": {"data": '{"op": "log_list"}'}}, "viewer"),
        ({"op": "publish", "topic": "/rcs/launcher/cmd", "msg": {"data": "not json"}}, "admin"),
        ({"op": "publish", "topic": "/some/new/topic", "msg": {}}, "admin"),
        ({"op": "call_service", "service": "/rosapi/topics"}, "viewer"),
        ({"op": "call_service", "service": "/navigate_to_pose/_action/cancel_goal"}, "viewer"),
        ({"op": "call_service", "service": "/rosapi/set_param"}, "admin"),
        ({"op": "send_action_goal", "action": "/navigate_to_pose"}, "operator"),
        ({"op": "whatever"}, "admin"),
    ]
    bad = [(c, need, r.need(c)[0]) for c, need in cases if r.need(c)[0] != need]
    check("each message needs the right role", not bad, str(bad[:2]))


def test_gateway() -> None:
    print("\n[the gateway in front of rosbridge]")
    users = rcs_access.Users(TMP / "users.json")
    audit = rcs_access.Audit(TMP / "audit")
    have_openssl = shutil.which("openssl") is not None
    cert = key = None
    if have_openssl:
        os.environ["RCS_WS"] = str(TMP)
        rcs_gateway.gencert(TMP / "tls")
        cert, key = str(TMP / "tls" / "cert.pem"), str(TMP / "tls" / "key.pem")
    GATEWAYS.update(start_servers(users, audit, cert, key))
    st, info = http("/rcs/info")
    check("says it is a gateway", st == 200 and info.get("rcs_gateway") == 1 and not info.get("users"), str(info))
    c = Client(None)
    try:
        c.ros.run(timeout=3)
    except Exception:  # noqa: BLE001
        pass
    check("no token: no connection", not c.connected())
    c.close()
    users.add("ann", "admin password", "admin")
    users.add("op", "operator password", "operator")
    users.add("view", "viewer password", "viewer")
    check("users count now", http("/rcs/info")[1].get("users") is True)
    check("wrong password refused", login("op", "nope nope") is None)
    tv, to, ta = login("view", "viewer password"), login("op", "operator password"), login("ann", "admin password")
    check("each signs in", tv and to and ta)
    st, me = http("/rcs/whoami", auth=f"Bearer {to}")
    check("knows who you are", me.get("user") == "op" and me.get("role") == "operator", str(me))

    v = Client(tv)
    v.listen()
    check("viewer connected", v.connected())
    check("told its role", wait(lambda: v.session and v.session[-1]["role"] == "viewer"), str(v.session))
    n0 = len(published("/cmd_vel_safe"))
    v.pub("/cmd_vel_safe", "geometry_msgs/msg/Twist", {"linear": {"x": 0.3}, "angular": {"z": 0}})
    check("viewer cannot drive", not wait(lambda: len(published("/cmd_vel_safe")) > n0, 1.5))
    check("and is told why", wait(lambda: v.denied and v.denied[-1]["need"] == "operator"), str(v.denied))
    v.pub("/amr/estop", "std_msgs/msg/Bool", {"data": True})
    check("viewer can engage the E-STOP", wait(lambda: any(m["msg"]["data"] is True for m in published("/amr/estop"))))
    v.pub("/amr/estop", "std_msgs/msg/Bool", {"data": False})
    check("but not release it", not wait(lambda: any(m["msg"]["data"] is False for m in published("/amr/estop")), 1.5))
    got = []
    roslibpy.Topic(v.ros, "/odom", "nav_msgs/msg/Odometry").subscribe(got.append)
    check("viewer sees the robot", wait(lambda: len(got) > 0))

    o = Client(to)
    o.pub("/cmd_vel_safe", "geometry_msgs/msg/Twist", {"linear": {"x": 0.3}, "angular": {"z": 0}})
    check("operator drives", wait(lambda: len(published("/cmd_vel_safe")) > n0))
    o.pub("/goal_pose", "geometry_msgs/msg/PoseStamped", {"pose": {"position": {"x": 1.5, "y": 2.0}}})
    check("operator sends goals", wait(lambda: len(published("/goal_pose")) > 0))
    o.pub("/amr/estop", "std_msgs/msg/Bool", {"data": False})
    check("operator releases the E-STOP", wait(lambda: any(m["msg"]["data"] is False for m in published("/amr/estop"))))
    o.pub("/rcs/battery/cmd", "std_msgs/msg/String", {"data": json.dumps({"op": "log_delete", "name": "x"})})
    check("operator cannot delete battery logs",
          not wait(lambda: any("log_delete" in m["msg"]["data"] for m in published("/rcs/battery/cmd")), 1.5))
    a = Client(ta)
    a.pub("/rcs/battery/cmd", "std_msgs/msg/String", {"data": json.dumps({"op": "log_delete", "name": "x"})})
    check("admin can", wait(lambda: any("log_delete" in m["msg"]["data"] for m in published("/rcs/battery/cmd"))))

    print("\n[users from the app, the audit log]")
    st, body = http("/rcs/users", auth=f"Bearer {to}")
    check("only admins manage users", st == 403)
    st, body = http("/rcs/users", auth=f"Bearer {ta}")
    check("admin lists users", st == 200 and {u["name"] for u in body["users"]} == {"ann", "op", "view"})
    st, body = http("/rcs/users/add?name=new&role=operator", auth=f"Bearer {ta}",
                    headers={"X-RCS-Password": "fresh password"})
    check("admin adds a user", st == 200 and login("new", "fresh password"), str(body))
    st, body = http("/rcs/users/role?name=view&role=operator", auth=f"Bearer {ta}")
    check("role changed", st == 200)
    check("applies to the open connection", wait(lambda: v.session and v.session[-1]["role"] == "operator", 8),
          str(v.session[-1:]))
    st, body = http("/rcs/users/password?name=op", auth=f"Bearer {to}", headers={"X-RCS-Password": "op new password"})
    check("a user changes their own password", st == 200 and login("op", "op new password"))
    check("which signed them out", wait(lambda: not o.connected(), 10))
    st, body = http("/rcs/users/remove?name=view", auth=f"Bearer {ta}")
    check("removed user is dropped", st == 200 and wait(lambda: not v.connected(), 10))
    st, body = http("/rcs/audit", auth=f"Bearer {ta}")
    whats = [r["what"] for r in body.get("rows", [])]
    for w in ("signed in", "E-STOP engaged", "E-STOP released", "teleop started", "goal x 1.50 y 2.00",
              "deleted battery log x", "added user new (operator)", "removed user view"):
        check(f"audit: {w}", any(w in x for x in whats))
    check("audit: refusals too", any(r["what"].startswith("refused") and not r["ok"] for r in body["rows"]))
    check("audit: wrong passwords", any("sign-in refused" in x for x in whats))
    for _ in range(6):
        login("ann", "guessing guess")
    st, body = http("/rcs/login", auth=basic("ann", "admin password"))
    check("after five wrong passwords, even the right one waits", st == 429, str(body))

    if cert:
        print("\n[wss://]")
        tt = login("new", "fresh password", GW_TLS, tls=True)
        check("signs in over https", tt is not None)
        s = Client(tt, GW_TLS, tls=True)
        s.listen()
        check("connects over wss", s.connected() and wait(lambda: s.session))
        s.close()
    else:
        print("  (no openssl: TLS skipped)")
    for cl in (v, o, a):
        cl.close()


def test_app() -> None:
    print("\n[the app through the gateway]")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    from rcs.core import pack as packs
    from rcs.core.profile import PROFILES_PATH, RobotProfile
    from rcs.ui.main_window import MainWindow
    from rcs.ui.panels.account import SignInDialog, UsersDialog

    def pump(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            app.processEvents()
            time.sleep(0.01)

    def until(cond, sec=10.0):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            app.processEvents()
            if cond():
                return True
            time.sleep(0.02)
        return False

    users = rcs_access.Users(TMP / "users.json")
    users.add("vee", "viewer pass 1", "viewer")
    for g in GATEWAYS.values():         # the guessing test above locked this address for a minute
        g.throttle = rcs_access.Throttle()
    w = MainWindow()
    w.show()
    prof = w.profiles.active()
    prof.host, prof.port = "127.0.0.1", GW
    w.profiles.upsert(prof)
    w.p_conn.refresh()
    who = {"user": "vee", "pw": "viewer pass 1"}
    asked = []

    def fake_sign_in():                 # what the dialog does, without its event loop
        asked.append(1)
        dlg = SignInDialog(prof, w)
        dlg.user.setText(who["user"])
        dlg.password.setText(who["pw"])
        dlg.sign_in(wait=True)
        if prof.token:
            w.p_conn.account.changed.emit(prof)
        return bool(prof.token)
    w.p_conn.account.ask_sign_in = fake_sign_in
    w._on_connect(prof)
    check("asked to sign in", until(lambda: asked, 10))
    check("then connected", until(w.ros.is_connected, 10))
    check("as a viewer", until(lambda: w.ros.role == "viewer", 8), w.ros.role)
    check("status bar says who", "vee" in w.sb_user.text() and "viewer" in w.sb_user.text(), w.sb_user.text())
    check("teleop greyed out", not w.p_teleop.can_drive and not w.p_teleop.joy.isEnabled())
    check("programs greyed out", not w.p_robot.can_operate)
    check("missions cannot start", not w.p_mission.can_run and not w.p_mission.play_btn.isEnabled())
    n0 = len(published("/cmd_vel_safe"))
    w.ros.publish_cmd_vel(0.3, 0.0)
    w.ros.send_nav_goal(1.0, 1.0, 0.0)
    pump(1.0)
    check("nothing to drive with leaves the app", len(published("/cmd_vel_safe")) == n0)
    check("and the operator is told", any(e.source == "Access" for e in w.events.events))
    w.ros.publish_estop(True)
    check("a viewer still stops the robot", until(lambda: any(m["msg"]["data"] is True for m in published("/amr/estop"))))
    check("token kept for next time", prof.token and RobotProfile.from_dict(
        json.loads(PROFILES_PATH.read_text())["profiles"][0]).token == prof.token)
    check("profiles file private", os.name == "nt" or (os.stat(PROFILES_PATH).st_mode & 0o077) == 0)
    out = packs.export_pack(TMP / "share", robot_name="x", profile=prof)
    import zipfile
    with zipfile.ZipFile(out) as z:
        shared = json.loads(z.read("profile.json"))
    check("a shared profile carries no sign-in", not shared.get("token") and not shared.get("user"))

    print("\n[made operator while connected, then signed out by an admin]")
    ta = login("ann", "admin password")
    http("/rcs/users/role?name=vee&role=operator", auth=f"Bearer {ta}")
    check("the app hears its new role", until(lambda: w.ros.role == "operator", 12), w.ros.role)
    check("teleop open now", w.p_teleop.can_drive)
    w.ros.publish_cmd_vel(0.25, 0.0)
    check("and drives", until(lambda: len(published("/cmd_vel_safe")) > n0))
    asked.clear()
    who["pw"] = "wrong password!"
    http("/rcs/users/signout?name=vee", auth=f"Bearer {ta}")
    check("dropped, the app says why", until(lambda: "ended" in w.p_conn.account.status.text(), 20),
          w.p_conn.account.status.text()[-120:])
    check("its token forgotten", not prof.token)
    check("and back to all greyed out? no: nothing is connected", not w.ros._want_connected)

    print("\n[an admin in the app]")
    who.update(user="ann", pw="admin password")
    prof.user = "ann"
    check("signs in", w.p_conn.account.ask_sign_in() and until(lambda: w.ros.role == "admin", 10), w.ros.role)
    w.p_conn.account.refresh()
    check("sees the users button", not w.p_conn.account.users_btn.isHidden())
    dlg = UsersDialog(prof, w)
    check("the users list", until(lambda: dlg.users.rowCount() >= 3, 10), str(dlg.users.rowCount()))
    check("the audit log", until(lambda: dlg.audit.rowCount() > 5, 10), str(dlg.audit.rowCount()))
    dlg.filter.setText("vee")
    check("filtered", all("vee" in " ".join(dlg.audit.item(r, c).text() for c in range(5))
                          for r in range(dlg.audit.rowCount())))
    dlg.export_to(str(TMP / "audit.csv"))
    check("exported as CSV", (TMP / "audit.csv").read_text().startswith("time,user,role,from,what,ok"))
    dlg.close()
    w.p_conn.account.sign_out()
    check("sign out: disconnected, token gone", until(lambda: not w.ros._want_connected and not prof.token, 5))
    w.close()


def main() -> int:
    test_access()
    test_gateway()
    test_app()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
