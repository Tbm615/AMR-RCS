"""Is the robot all right? One line per part, a level and what to do about it,
from what the app already knows (no extra traffic). The Status page shows it,
diagnostic bundles keep it. No Qt here.

ctx keys (all optional): connected, launcher_online, launcher (state dict),
gate (rcs_safety state), estop (bool), battery (rcs_battery dict), ages
({stat key: seconds since last}), localized (bool), camera_problem (str),
robot_note (str), role, user, gateway (bool | None), nav_state (str).
"""
from __future__ import annotations

import math

from typing import List, NamedTuple


LOC_UNSURE_M = 0.30        # AMCL's spread (1 sigma) above which the pose is called unsure
LOC_UNSURE_DEG = 12.0


class Check(NamedTuple):
    key: str
    title: str
    level: str          # ok | warn | error | off
    text: str
    hint: str = ""


def _fresh(ctx: dict, key: str, within: float = 3.0) -> bool:
    return ctx.get("ages", {}).get(key, 1e9) <= within


def checks(ctx: dict) -> List[Check]:
    out: List[Check] = []
    if not ctx.get("connected"):
        out.append(Check("link", "Connection", "error", "not connected",
                         "Is the robot on, rosbridge (or its gateway) running, this PC on its network? "
                         "Connection page."))
        return out
    out.append(Check("link", "Connection", "ok", "connected"))
    if ctx.get("gateway"):
        role = ctx.get("role")
        out.append(Check("signin", "Sign-in", "ok" if role else "warn",
                         f"{ctx.get('user')} ({role})" if role else "not signed in",
                         "" if role else "Sign in on the Connection page."))
    note = ctx.get("robot_note", "")
    if not ctx.get("launcher_online"):
        out.append(Check("launcher", "Robot launcher", "error", "not answering",
                         "rcs_launcher.py is not running on the robot: robot/README.md, start_rcs_robot.sh."))
    else:
        out.append(Check("robotside", "Robot side", "warn" if note else "ok",
                         note or f"v{ctx.get('launcher', {}).get('rcs', '?')}"))
        stacks = ctx.get("launcher", {}).get("stacks", {})
        for key, title, hint in (("bringup", "Robot base", "Robot programs page: start it; its log says why it failed."),
                                 ("nav", "Navigation", "Robot programs page: start Nav2 on a saved map, or SLAM.")):
            st = stacks.get(key, {})
            state = "external" if st.get("external") else st.get("state", "stopped")
            level = {"running": "ok", "external": "ok", "starting": "warn", "failed": "error"}.get(state, "off")
            out.append(Check(key, title, level, state, hint if level in ("off", "error") else ""))
    base_up = ctx.get("launcher", {}).get("stacks", {}).get("bringup", {}).get("state") == "running" \
        or ctx.get("launcher", {}).get("stacks", {}).get("bringup", {}).get("external")
    for key, title, hint in (("odom", "Odometry", "The drives or ros2_control are not running (Robot base log)."),
                             ("scan", "Lidar", "The lidar driver or the scan filter is down (Robot base log); "
                                               "check the lidar's cable and power.")):
        if _fresh(ctx, key):
            out.append(Check(key, title, "ok", "arriving"))
        elif base_up or key in ctx.get("ages", {}):
            out.append(Check(key, title, "error", "not arriving", hint))
        else:
            out.append(Check(key, title, "off", "robot base not running"))
    nav = ctx.get("launcher", {}).get("stacks", {}).get("nav", {}).get("state")
    amcl = ctx.get("amcl")
    if ctx.get("localized") and amcl and (amcl[0] > LOC_UNSURE_M or amcl[1] > math.radians(LOC_UNSURE_DEG)):
        # AMCL has a pose but its particles are spread out: the drawn robot can be off
        # (seen on the robot: 0.15 m and 4.5 degrees off on a map built from one spot)
        out.append(Check("loc", "Localization", "warn",
                         f"on the map, but unsure (±{amcl[0]:.2f} m, ±{math.degrees(amcl[1]):.0f}°)",
                         "Drive a little so AMCL can match the lidar to the map, or set the initial pose "
                         "again. If it stays unsure, the map may be out of date: build it again (SLAM)."))
    elif ctx.get("localized"):
        out.append(Check("loc", "Localization", "ok", "the robot is on the map"))
    elif nav == "running":
        out.append(Check("loc", "Localization", "warn", "not on the map yet",
                         "Set the initial pose on the 2D Map (or Use last position)."))
    gate = ctx.get("gate")
    if ctx.get("estop") or (gate and gate.get("engaged")):
        out.append(Check("estop", "E-STOP", "error", "ENGAGED", "Release it when it is safe (sidebar)."))
    elif gate:
        stops = gate.get("stops") or 0
        out.append(Check("estop", "E-STOP", "ok", "clear" + (f", stopped itself {stops}× (app lost)" if stops else "")))
    else:
        out.append(Check("estop", "E-STOP", "warn", "no E-STOP service on the robot",
                         "rcs_safety.py is not running: the E-STOP is only the app's (robot/README.md)."))
    b = ctx.get("battery")
    if b and b.get("reading") and _fresh(ctx, "battery", 15.0):
        soc = b["reading"].get("soc")
        level = "error" if soc is not None and soc <= 15 else "warn" if soc is not None and soc <= 30 else "ok"
        out.append(Check("battery", "Battery", level, f"{soc}% · {b['reading'].get('voltage', 0):.1f} V",
                         "Charge the robot." if level != "ok" else ""))
    elif b:
        out.append(Check("battery", "Battery", "warn", f"BMS {b.get('state', '?')}",
                         "The robot's Bluetooth link to the BMS is down (Battery page says why)."))
    disk = ((b or {}).get("log") or {}).get("disk") or {}
    free = disk.get("free")
    if isinstance(free, (int, float)) and free < 2e9:
        out.append(Check("disk", "Robot disk", "warn", f"{free / 1e9:.1f} GB free",
                         "Delete old battery logs or maps on the robot."))
    cam = ctx.get("camera_problem")
    if cam:
        out.append(Check("camera", "Camera", "warn", "no picture", cam))
    return out


def summary(cs: List[Check]) -> str:
    """One word for the whole robot: the worst level."""
    levels = [c.level for c in cs]
    for lvl in ("error", "warn"):
        if lvl in levels:
            return lvl
    return "ok"
