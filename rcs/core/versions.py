"""Versions: the app's, the robot side's (rcs_launcher.py says it), GitHub's
newest release.

"0.6.0b1" < "0.6.0" < "0.6.1"; a tag "v0.6.0-beta.1" reads as "0.6.0b1".
"""
from __future__ import annotations

import json
import re
import urllib.request
from typing import Optional, Tuple

from .. import __version__

RELEASES = "https://api.github.com/repos/Tbm615/AMR-RCS/releases?per_page=15"
PAGE = "https://github.com/Tbm615/AMR-RCS/releases"
# the robot side this app needs at least: older ones lack what it uses
ROBOT_SIDE_MIN = "0.6.0b2"


def parse(v: str) -> Tuple[int, int, int, int, int]:
    """(major, minor, patch, 0 for a beta / 1 for a release, beta number)."""
    v = str(v or "").strip().lstrip("vV")
    m = re.match(r"(\d+)\.(\d+)(?:\.(\d+))?(?:[-.]?(?:b|beta)\.?(\d+))?", v)
    if not m:
        return (0, 0, 0, 0, 0)
    major, minor, patch, beta = m.group(1), m.group(2), m.group(3) or 0, m.group(4)
    return (int(major), int(minor), int(patch), 0 if beta else 1, int(beta or 0))


def is_beta(v: str) -> bool:
    return parse(v)[3] == 0


def robot_side_note(state: dict, app: str = __version__) -> str:
    """What to tell the operator about the robot side, "" if all is well."""
    robot = state.get("rcs")
    if not robot:
        return (f"The robot side is older than v{ROBOT_SIDE_MIN} (it does not say its version): copy this "
                "release's robot/ folder to the robot (robot/README.md).")
    if parse(robot) < parse(ROBOT_SIDE_MIN):
        return f"The robot side is v{robot}, this app needs v{ROBOT_SIDE_MIN} or newer: update it (robot/README.md)."
    if parse(robot) > parse(app):
        return f"The robot side (v{robot}) is newer than this app (v{app}): update the app."
    return ""


def newest(releases: list, current: str = __version__) -> Optional[dict]:
    """The release to offer, or None. A beta app is offered betas too; a stable
    app only stable releases."""
    want_beta = is_beta(current)
    best = None
    for r in releases:
        if r.get("draft") or (r.get("prerelease") and not want_beta):
            continue
        tag = r.get("tag_name", "")
        if parse(tag) <= parse(current):
            continue
        if best is None or parse(tag) > parse(best["tag_name"]):
            best = r
    return best


def check(timeout: float = 6.0) -> Optional[dict]:
    """GitHub's answer: {tag_name, html_url, ...} of a newer release, or None.
    Blocks: call off the GUI thread. Sends nothing about this PC."""
    req = urllib.request.Request(RELEASES, headers={"Accept": "application/vnd.github+json",
                                                    "User-Agent": f"RCS/{__version__}"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        data = json.loads(r.read().decode("utf-8"))
    return newest(data if isinstance(data, list) else [])
