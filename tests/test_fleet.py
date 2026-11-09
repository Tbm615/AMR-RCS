"""Several robots on one screen: the Fleet page against two mock robots (each a
mock rosbridge running the robot's real battery log and E-STOP gate) and one
robot that is not there.

Run:
    python tests/test_fleet.py
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_fleet_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]

from PySide6.QtWidgets import QApplication

from rcs.core.fleet import FleetMonitor, describe
from rcs.core.profile import RobotProfile, load_settings
from rcs.ui.main_window import MainWindow

app = QApplication(sys.argv)
PASS = FAIL = 0
PORTS = {"alpha": (9201, 70), "beta": (9202, 15)}
GONE = 9203


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def wait(cond, timeout=10.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if cond():
            return True
        time.sleep(0.03)
    return bool(cond())


def profile(name: str, port: int) -> RobotProfile:
    p = RobotProfile.new(name=name)
    p.host, p.port = "127.0.0.1", port
    return p


def main() -> int:
    mocks = [subprocess.Popen([sys.executable, str(HERE / "mock_rosbridge.py"), "--port", str(port), "--robot",
                               "--name", name, "--soc", str(soc), "--duration", "90"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
             for name, (port, soc) in PORTS.items()]
    time.sleep(1.5)
    try:
        run()
    finally:
        for m in mocks:
            m.kill()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


def run() -> None:
    print("\n[what a card says]")
    check("offline", describe({"_connected": False})["link"][1] == "error")
    d = describe({"_connected": True, "_ages": {"battery": 1, "gate": 1},
                  "battery": {"reading": {"soc": 15, "voltage": 47.1, "current": -3}, "age": 1.0},
                  "gate": {"engaged": False, "stops": 2, "last_stop": {"reason": "x"}}})
    check("low battery", d["battery"][1] == "warn" and "15%" in d["battery"][0], f"{d['battery']}")
    check("stopped itself, counted", "2×" in d["estop"][0])
    d = describe({"_connected": True, "_ages": {"battery": 60}, "battery": {"reading": {"soc": 50}}})
    check("old data is not shown as live", d["battery"][0] == "no battery service")
    check("no E-STOP service on an old robot is said", "no E-STOP service" in describe(
        {"_connected": True, "_ages": {}})["estop"][0])

    print("\n[watching three robots]")
    profs = {n: profile(n, port) for n, (port, _s) in PORTS.items()}
    profs["gone"] = profile("gone", GONE)
    mon = FleetMonitor()
    mon.watch(list(profs.values()), {p.id for p in profs.values()})

    def card(name):
        snap = mon.snapshot(profs[name].id)
        return describe(snap) if snap else {}

    check("both robots online", wait(lambda: all(card(n).get("link", ("", ""))[1] == "ok" for n in PORTS)))
    check("their battery, each its own", wait(lambda: "70%" in card("alpha").get("battery", ("",))[0]
                                              and "15%" in card("beta").get("battery", ("",))[0]),
          f"{card('alpha').get('battery')} {card('beta').get('battery')}")
    check("low one flagged", wait(lambda: card("beta")["battery"][1] in ("warn", "error")))
    check("their E-STOP gate", wait(lambda: card("alpha").get("estop", ("",))[0].startswith("clear")))
    check("motion", wait(lambda: "m/s" in card("alpha").get("motion", ("",))[0]))
    check("the missing one offline", card("gone")["link"][1] == "error")

    print("\n[E-STOP all]")
    names = mon.estop_all()
    check("reached the two that are there", sorted(names) == ["alpha", "beta"], f"{names}")
    check("both robots engaged it", wait(lambda: all(card(n)["estop"][0] == "E-STOP ENGAGED" for n in PORTS)))
    mon.watch(list(profs.values()), {profs["alpha"].id})
    check("unwatched: its connection closed", set(mon.watches) == {profs["alpha"].id})
    mon.stop()

    print("\n[Fleet page]")
    w = MainWindow()
    w.show()
    for p in profs.values():
        w.profiles.upsert(p)
    w._show("fleet")
    page = w.p_fleet
    check("a card per robot", wait(lambda: len(page.cards) == len(w.profiles.profiles)))
    for n in ("alpha", "beta", "gone"):
        page.cards[profs[n].id].watch.setChecked(True)
    check("watching is remembered", set(load_settings().get("fleet_watch", [])) == {p.id for p in profs.values()})
    alpha = page.cards[profs["alpha"].id]
    check("cards fill in", wait(lambda: "%" in alpha.vals["battery"].text()), alpha.vals["battery"].text())
    check("still engaged from before: shown in red", "ENGAGED" in alpha.vals["estop"].text())
    check("the missing robot's card says offline", "offline" in page.cards[profs["gone"].id].vals["link"].text())
    check("no E-STOP button for a robot that is not there",
          wait(lambda: not page.cards[profs["gone"].id].estop_btn.isEnabled()))
    page.estop(profs["beta"].id)
    check("E-STOP one", "sent to beta" in page.result.text())
    check("logged", any("beta" in e.text and e.source == "E-STOP" for e in w.events.events))
    page.cards[profs["alpha"].id].open_btn.click()
    check("Open connects the app to that robot", wait(lambda: w.ros.is_connected()
                                                      and w.ros.profile.id == profs["alpha"].id))
    check("its card says it is the one open", wait(lambda: "open in the app" in alpha.note.text()))
    check("the robot's E-STOP shows in the app too", wait(lambda: w._estop_on))
    page.estop_all()
    check("E-STOP all from the page", "alpha" in page.result.text() and "could not reach gone" in page.result.text(),
          page.result.text())
    w.close()


if __name__ == "__main__":
    code = main()
    os._exit(code)
