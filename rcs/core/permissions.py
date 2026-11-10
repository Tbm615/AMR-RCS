"""What the signed-in user may do, as the robot's gateway sees it
(robot/rcs_access.py has the same table). The gateway is what enforces it; the
app checks first so a viewer is told at once instead of nothing happening.

role "" means the robot has no gateway (older robot side): everything is allowed,
as before.
"""
from __future__ import annotations

RANK = {"viewer": 0, "operator": 1, "admin": 2}

NEED = {
    "drive": "operator",            # teleop
    "goal": "operator",             # Nav2 goals, missions, saved places
    "pose": "operator",             # initial pose
    "launch": "operator",           # start/stop bringup and Nav2
    "save_map": "operator",
    "send_map": "operator",         # a map from this PC to the robot
    "zones": "operator",            # keepout and speed zones of a map
    "release_estop": "operator",
    "delete_log": "admin",          # a battery log on the robot
    "users": "admin",
    "estop": "viewer",              # anyone may stop the robot
    "cancel": "viewer",
}

WHAT = {
    "drive": "drive the robot", "goal": "send goals or run missions", "pose": "set the initial pose",
    "launch": "start or stop programs on the robot", "save_map": "save maps", "send_map": "send maps to the robot",
    "zones": "change the zones of a map",
    "release_estop": "release the E-STOP", "delete_log": "delete battery logs", "users": "manage users",
}

LAUNCHER = {"start": "launch", "stop": "launch", "save_map": "save_map", "map_put": "send_map",
            "zones_put": "zones"}
BATTERY = {"log_delete": "delete_log"}


def can(role: str, action: str) -> bool:
    if not role:
        return True
    return RANK.get(role, -1) >= RANK[NEED.get(action, "admin")]


def refusal(role: str, action: str) -> str:
    return f"Signed in as {role}: you cannot {WHAT.get(action, action)} ({NEED.get(action, 'admin')} only)."
