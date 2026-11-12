"""Is the Astra+ on the USB bus? Reads sysfs only: no ROS, no root, and cheap
enough for the launcher's 1 Hz state.

The camera is two USB devices behind its own small hub: color (0536) and depth
(0636). The driver looks for the depth one. It can fail to come up at boot while
color does, and the driver then waits forever without an error, so bringup
looks fine with no camera at all. This is what lets the app say so.
"""
from __future__ import annotations

from pathlib import Path

ORBBEC = "2bc5"
COLOR, DEPTH = "0536", "0636"


def camera_usb(root="/sys/bus/usb/devices") -> dict:
    """{"color", "depth": on the bus, "depth_usb": its link speed in Mbit/s or 0}.
    The depth sensor worked as USB 2.0 (480); as USB 3 (5000) on this robot it
    streamed a few seconds, then the kernel reported -71 (EPROTO) and depth stopped."""
    found = {}
    for dev in Path(root).glob("*"):
        try:
            if (dev / "idVendor").read_text().strip() == ORBBEC:
                found[(dev / "idProduct").read_text().strip()] = dev
        except OSError:         # interfaces have no idVendor; devices can vanish
            continue
    speed = 0
    if DEPTH in found:
        try:
            speed = int(float((found[DEPTH] / "speed").read_text().strip()))
        except (OSError, ValueError):
            pass
    return {"color": COLOR in found, "depth": DEPTH in found, "depth_usb": speed}
