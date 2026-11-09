"""Going to charge by itself: below a charge, the mission pauses and the robot
drives to its charging place; charged again, the mission goes on from where it
was. No Qt here: the main window feeds step() once a second and does what it
returns.

The robot must charge at that place by itself (a dock, or a person plugging it
in): the app only drives it there and waits. Charging is seen from the
battery's current (positive), or from the charge rising.
"""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from typing import List, Optional

CHARGING_A = 0.3        # current above this: the battery is being charged
RETRY_S = 30.0
MAX_TRIES = 3


@dataclass
class ChargeConfig:
    on: bool = False
    low: float = 25.0           # % : go below this
    resume: float = 90.0        # % : back to work above this
    place: str = ""             # id of a saved place

    @classmethod
    def from_dict(cls, d) -> "ChargeConfig":
        d = d if isinstance(d, dict) else {}
        c = cls(on=bool(d.get("on", False)), place=str(d.get("place", "")))
        try:
            c.low = min(80.0, max(5.0, float(d.get("low", 25.0))))
            c.resume = min(100.0, max(c.low + 5.0, float(d.get("resume", 90.0))))
        except (TypeError, ValueError):
            pass
        return c

    def to_dict(self) -> dict:
        return asdict(self)


class ChargeKeeper:
    """idle -> going (to the charger) -> charging -> idle (mission resumed)."""

    def __init__(self, cfg: Optional[ChargeConfig] = None, clock=time.monotonic) -> None:
        self.cfg = cfg or ChargeConfig()
        self.clock = clock
        self.state = "idle"
        self.resume_mission = False
        self.tries = 0
        self._retry_at = 0.0
        self.why = ""                   # what it is doing, for the page

    def take_over(self) -> None:
        """The operator drove, sent a goal or started a mission: their call."""
        if self.state != "idle":
            self.state, self.resume_mission, self.why = "idle", False, "stopped: the operator took over"

    def step(self, soc: Optional[float], current: Optional[float], mission_running: bool, nav: str,
             estop: bool, place_ok: bool) -> List[str]:
        """Actions: pause_mission, go_charger, resume_mission, failed, arrived, charged."""
        c = self.cfg
        if not c.on or not place_ok:
            if self.state != "idle":
                self.state, self.resume_mission = "idle", False
            self.why = "off" if not c.on else "no charging place chosen"
            return []
        if soc is None:
            self.why = "waiting for the battery"
            return []
        now = self.clock()
        out: List[str] = []
        if self.state == "idle":
            self.why = f"on: below {c.low:.0f}% it goes to charge"
            if soc <= c.low and not estop:
                self.resume_mission = mission_running
                if mission_running:
                    out.append("pause_mission")
                out.append("go_charger")
                self.state, self.tries, self._retry_at = "going", 1, now + RETRY_S
                self.why = f"battery {soc:.0f}%: going to charge"
        elif self.state == "going":
            if nav == "succeeded":
                self.state = "charging"
                self.why = "at the charging place"
                out.append("arrived")
            elif nav in ("aborted", "canceled", "no_response", "") and now >= self._retry_at and not estop:
                if self.tries >= MAX_TRIES:
                    self.state = "idle"
                    self.why = "could not reach the charging place"
                    out.append("failed")
                else:
                    self.tries += 1
                    self._retry_at = now + RETRY_S
                    out.append("go_charger")
        elif self.state == "charging":
            charging = current is not None and current > CHARGING_A
            self.why = f"charging: {soc:.0f}% of {c.resume:.0f}%" if charging else \
                f"at the charging place, {soc:.0f}%: is it charging?"
            if soc >= c.resume:
                self.state = "idle"
                out.append("charged")
                if self.resume_mission:
                    out.append("resume_mission")
                self.resume_mission = False
        return out
