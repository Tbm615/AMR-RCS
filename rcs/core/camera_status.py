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
        return "The camera driver is starting…"
    return (f"The camera is running on the robot but no frames arrive from {topic}. "
            "Check that topic on the Connection page, and the Wi-Fi.")
