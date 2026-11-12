"""Why there is no picture, in words an operator can act on.

No frames looks the same from here whatever the cause, and the usual one is not
in the app: see robot/camera_check.py.
"""
from __future__ import annotations

from typing import Optional

NO_DEPTH_USB = ("The robot sees the color camera but not its depth sensor on USB, so the camera "
                "driver cannot start. Unplug the camera and plug it back in: rebooting rarely helps. "
                "If it keeps happening, connect it through a USB 2.0 cable or another USB port.")
NO_USB = "The robot does not see the camera on USB. Check its cable, then reboot the robot."


DEPTH_STOPPED = ("The camera's color picture works but its depth sensor sends nothing, so the 3D map "
                 "gets no frames. Unplug the camera and plug it back in, then restart Robot base.")
DEPTH_STOPPED_USB3 = ("The camera's color picture works but its depth sensor stopped on the robot (USB "
                      "errors), so the 3D map gets no frames. The camera is connected as USB 3 now; it "
                      "worked as USB 2.0. Plug it into a USB 2.0 port or through a USB 2.0 cable, then "
                      "restart Robot base.")
DRIVER_GONE = ("The camera driver is not running on the robot: it stops when the camera drops off USB. "
               "Restart Robot base; if it happens again, connect the camera as USB 2.0 (a USB 2.0 port or cable).")
DRIVER_GONE_S = 40.0        # bringup up this long and still no camera publisher
DEPTH_GRACE_S = 15.0        # the driver can take this long after bringup to send depth


def depth_problem(state: dict) -> Optional[str]:
    """The depth sensor has stopped while the camera driver runs (robot side
    counts depth frames since this change). The color picture can go on meanwhile."""
    cam = state.get("camera")
    if not isinstance(cam, dict) or "depth_hz" not in cam or not cam.get("depth", True):
        return None
    base = state.get("stacks", {}).get("bringup", {})
    if base.get("state") != "running" and not base.get("external"):
        return None
    try:
        up_s = float(state.get("time", 0)) - float(base.get("since", 0))
        hz = float(cam.get("depth_hz") or 0.0)
    except (TypeError, ValueError):
        return None
    if not cam.get("publishers") or hz > 0.0 or (not base.get("external") and up_s < DEPTH_GRACE_S):
        return None
    return DEPTH_STOPPED_USB3 if int(cam.get("depth_usb") or 0) >= 5000 else DEPTH_STOPPED


def usb_problem(state: dict) -> Optional[str]:
    """A camera that is missing from the robot's USB, whatever is running."""
    cam = state.get("camera")
    if not isinstance(cam, dict) or cam.get("depth", True):
        return None
    return NO_DEPTH_USB if cam.get("color") else NO_USB


def _base_up(state: dict) -> bool:
    base = state.get("stacks", {}).get("bringup", {})
    return bool(base.get("external")) or base.get("state") in ("starting", "running")


def camera_problem(connected: bool, state: dict, frames: bool, topic: str) -> Optional[str]:
    """What to tell the operator; None while frames arrive."""
    if frames:
        return None
    if not connected:
        return "Not connected to a robot."
    usb = usb_problem(state)
    if usb:
        return usb
    if not state:
        return f"No frames from {topic}, and the RCS launcher on the robot does not answer."
    if not _base_up(state):
        return "The camera is off: start Robot base on the Robot programs page."
    cam = state.get("camera")
    if not isinstance(cam, dict):
        return f"No frames from {topic}. Update rcs_launcher.py on the robot to see why here."
    if not cam.get("publishers"):
        base = state.get("stacks", {}).get("bringup", {})
        try:
            up_s = float(state.get("time", 0)) - float(base.get("since", 0))
        except (TypeError, ValueError):
            up_s = 0.0
        if base.get("state") == "running" and "time" in state and up_s > DRIVER_GONE_S:
            return DRIVER_GONE
        return "The camera driver is starting…"
    return (f"The camera is running on the robot but no frames arrive from {topic}. "
            "Check that topic on the Connection page, and the Wi-Fi.")
