"""What may reach the drives: the robot's own E-STOP. No ROS here, so the tests
and tests/mock_rosbridge.py run this very code; rcs_safety.py is the ROS shell.

Every velocity command passes through here on its way to the driver:

    Nav2 -> CBF -> /cmd_vel_safe  <- robot gamepad (flydigi), old RCS apps
                        |
                        |    /cmd_vel_teleop  <- RCS teleop (first)
                        |      |
                   rcs_safety.py   (this gate)
                        |
                 /rcs/cmd_vel_gated -> diff_drive_controller

E-STOP (/amr/estop true, from any RCS app or the web console) latches here, on
the robot: commands are replaced by zero, Nav2 goals are cancelled, and it
stays so until an operator releases it (/amr/estop false). The latch is kept
in a file, so it survives a restart of the service or of the robot.

Losing the app: each RCS app sends a heartbeat twice a second, saying whether
it is driving the robot (a mission, a goal it sent, teleop). If one that was
driving goes silent for HEARTBEAT_S (Wi-Fi gone, app crashed), the robot stops
by itself: its Nav2 goal is cancelled and zero is sent for HOLD_S. Without this
a goal sent by an app went on to the end with nobody watching. An app that is
only watching, or that says goodbye when it closes, stops nothing.

Teleop first: teleop comes on its own topic (/cmd_vel_teleop, see rcs_safety.py),
Nav2 and missions on the shared one. While teleop commands arrive, and for
TELEOP_HOLD_S after the last one, the shared topic is not let through: two
streams at once would make the driver follow each in turn and the robot
jerk between them. Nav2's goal is not cancelled: once the operator lets go, it
drives on.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Callable, Optional

HEARTBEAT_S = 3.0       # a driving app silent this long is gone
HOLD_S = 1.5            # zero after losing it, then commands pass again
CANCEL_EVERY_S = 1.0    # while engaged, a goal Nav2 still runs is cancelled again
TELEOP_HOLD_S = 0.5     # after the last teleop command, Nav2's stay out this long


class Gate:
    def __init__(self, path: Optional[Path] = None, clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time) -> None:
        self.path = Path(path) if path else None
        self.clock, self.wall = clock, wall
        self.engaged = False
        self.since: Optional[float] = None      # unix time
        self.by = ""
        self.stops = 0                          # stops because an app was lost
        self.last_stop: Optional[dict] = None
        self.clients: dict = {}                 # id -> {"t", "driving", "host"}
        self.hold_until = 0.0
        self._last_cancel = -1e9
        self.teleop_t = -1e9                    # last teleop command (clock)
        self.overridden_t = -1e9                # last Nav2 command dropped for teleop
        self.overrides = 0                      # Nav2 commands dropped for teleop
        self._load()

    # ---- the latch ----

    def _load(self) -> None:
        if not self.path:
            return
        try:
            d = json.loads(self.path.read_text())
        except (OSError, ValueError):
            return
        if isinstance(d, dict) and d.get("engaged"):
            self.engaged, self.since = True, d.get("since")
            self.by = str(d.get("by", "")) + " (kept through a restart)"

    def _save(self) -> None:
        if not self.path:
            return
        try:
            tmp = self.path.with_name(self.path.name + ".tmp")
            tmp.write_text(json.dumps({"engaged": self.engaged, "since": self.since, "by": self.by}))
            tmp.replace(self.path)
        except OSError:
            pass

    def estop(self, on: bool, by: str = "") -> list:
        """Returns what to do now: ["cancel_nav"] when it engages."""
        if on == self.engaged:
            return []
        self.engaged = on
        self.since = self.wall() if on else None
        self.by = by if on else ""
        self._save()
        if on:
            self._last_cancel = self.clock()
            return ["cancel_nav"]
        return []

    # ---- apps ----

    def heartbeat(self, msg: dict) -> None:
        cid = str(msg.get("id", ""))
        if not cid:
            return
        if msg.get("bye"):
            self.clients.pop(cid, None)     # closed on purpose: it cancelled its own goal
            return
        self.clients[cid] = {"t": self.clock(), "driving": bool(msg.get("driving")),
                             "host": str(msg.get("host", ""))[:60]}

    def tick(self, nav_active: bool = False) -> list:
        """Called a few times a second. Returns what to do: "cancel_nav"."""
        now = self.clock()
        out = []
        for cid, c in list(self.clients.items()):
            if now - c["t"] <= HEARTBEAT_S:
                continue
            del self.clients[cid]
            if c["driving"]:
                self.stops += 1
                self.hold_until = now + HOLD_S
                self.last_stop = {"n": self.stops, "t": self.wall(), "app": cid,
                                  "reason": f"lost the RCS app on {c['host'] or 'a PC'} while it was "
                                            f"driving (no heartbeat for {HEARTBEAT_S:.0f} s)"}
                out.append("cancel_nav")
        if self.engaged and nav_active and now - self._last_cancel >= CANCEL_EVERY_S:
            self._last_cancel = now
            out.append("cancel_nav")
        return sorted(set(out))

    # ---- commands ----

    def blocking(self) -> bool:
        return self.engaged or self.clock() < self.hold_until

    def filter(self, lin: float, ang: float):
        return (0.0, 0.0) if self.blocking() else (lin, ang)

    def teleop_active(self) -> bool:
        return self.clock() - self.teleop_t < TELEOP_HOLD_S

    def teleop(self, lin: float, ang: float):
        """A teleop command (RCS app, gamepad): what goes to the driver."""
        self.teleop_t = self.clock()
        return self.filter(lin, ang)

    def nav(self, lin: float, ang: float):
        """A command on the shared topic (Nav2 through the CBF filter): what goes
        to the driver, or None while teleop drives (nothing goes out for it)."""
        if self.blocking():
            return 0.0, 0.0
        if self.teleop_active():
            self.overrides += 1
            self.overridden_t = self.clock()
            return None
        return lin, ang

    def state(self) -> dict:
        now = self.clock()
        return {"engaged": self.engaged, "since": self.since, "by": self.by, "gate": True,
                "holding": now < self.hold_until, "stops": self.stops, "last_stop": self.last_stop,
                "apps": len(self.clients), "teleop": self.teleop_active(),
                "nav_overridden": now - self.overridden_t < 1.0, "overrides": self.overrides,
                "driving_apps": sum(1 for c in self.clients.values() if c["driving"])}
