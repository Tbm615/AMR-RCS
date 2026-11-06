"""What the robot's battery service reports, as an operator reads it."""
from __future__ import annotations

from typing import Optional, Tuple

STALE_S = 15.0          # older than this is shown as the last known value
LOW, CRITICAL = 20, 10  # state of charge, %


def live(b: dict) -> bool:
    age = b.get("age")
    return bool(b.get("reading")) and age is not None and age <= STALE_S


def link_text(b: Optional[dict]) -> Tuple[str, str]:
    """(text, level); level: ok | warn | error."""
    if not b:
        return ("No answer from the battery service on the robot. Update it: "
                "see Battery in robot/README.md.", "error")
    state, name = b.get("state"), b.get("name") or b.get("address") or "the BMS"
    if state == "connected":
        if live(b):
            return (f"Connected to {name} · {b.get('connected_s', 0)} s · "
                    f"{b.get('reconnects', 0)} reconnects", "ok")
        err = b.get("error")
        return (f"Connected to {name} but its frames cannot be used: {err}" if err
                else f"Connected to {name}, waiting for data…", "warn")
    if state == "no_adapter":
        return f"The robot's Bluetooth is off: {b.get('error', '')}", "error"
    if state == "connecting":
        return f"Connecting to {name}…", "warn"
    if not b.get("heard"):
        return ("Searching, and the robot's Bluetooth hears no device at all. Check its antenna, or "
                "fit a USB Bluetooth adapter.", "error")
    if b.get("address"):
        return (f"Searching for {name}: it is out of reach, or a phone is connected to it "
                "(the BMS takes one connection at a time).", "warn")
    return (f"Searching: {b.get('heard')} devices heard, none is a JK-BMS. If a phone is "
            "connected to the BMS, disconnect it.", "warn")


def alarm(soc: Optional[int]) -> str:
    if soc is None:
        return ""
    return "error" if soc <= CRITICAL else "warn" if soc <= LOW else ""
