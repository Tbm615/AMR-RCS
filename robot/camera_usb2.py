#!/usr/bin/env python3
"""Keep the Orbbec Astra+ on USB 2.0: run by udev (99-rcs-camera-usb2.rules) when
the camera's own USB 3 hub (Genesys 05e3:0620) appears.

The depth sensor worked for weeks as USB 2.0, when the camera's USB 3 link
did not come up. Since it does (same ports, same cable), that link fails after
seconds to minutes (-71, "Cannot enable. Maybe the USB cable is bad?") and
depth stops while the color picture goes on. This sets the USB 3 link of the
port the camera hub hangs on to SS.Disabled (USB 3 hub class request
SetPortFeature(PORT_LINK_STATE)): the camera hub then runs on its USB 2 half
only, and the depth sensor comes up behind it as USB 2.0 (480 Mbit/s is plenty
for 640x480@30 depth plus MJPG color). Other ports and devices are untouched,
and a re-plug or reboot runs this again.

    camera_usb2.py <sysfs devpath of the camera hub, e.g. /devices/.../2-1.1>

Standard library only (udev runs it as root, without the user's packages).
"""
from __future__ import annotations

import ctypes
import os
import struct
import sys
from pathlib import Path

USBDEVFS_CONTROL = 0xC0185500           # _IOWR('U', 0, struct usbdevfs_ctrltransfer) on 64-bit
SET_FEATURE, PORT_LINK_STATE, SS_DISABLED = 3, 5, 4


def parent_and_port(dev: Path):
    """(sysfs dir of the hub it is plugged into, port number) from a name like 2-1.4,
    or 2-1 for a root hub port. In sysfs a device sits inside its hub's directory."""
    name = dev.name
    port = name.rsplit(".", 1)[1] if "." in name else name.split("-", 1)[1]
    return dev.parent, int(port)


def disable_ss_link(hub: Path, port: int) -> None:
    import fcntl                        # Linux only; the tests import this file on Windows too
    node = "/dev/bus/usb/%03d/%03d" % (int((hub / "busnum").read_text()), int((hub / "devnum").read_text()))
    req = struct.pack("@BBHHHI", 0x23, SET_FEATURE, PORT_LINK_STATE, (SS_DISABLED << 8) | port, 0, 1000)
    buf = ctypes.create_string_buffer(req + b"\0" * 4 + struct.pack("@Q", 0))
    fd = os.open(node, os.O_RDWR)
    try:
        fcntl.ioctl(fd, USBDEVFS_CONTROL, buf, True)
    finally:
        os.close(fd)


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    dev = (Path("/sys" + sys.argv[1]) if sys.argv[1].startswith("/devices") else Path(sys.argv[1])).resolve()
    try:
        if int((dev / "speed").read_text()) < 5000:
            return 0                    # already USB 2
        hub, port = parent_and_port(dev)
        disable_ss_link(hub, port)
    except (OSError, ValueError) as e:
        print(f"camera_usb2: {dev}: {e}", file=sys.stderr)
        return 1
    print(f"camera_usb2: USB 3 link of {hub.name} port {port} disabled: the camera runs as USB 2.0")
    return 0


if __name__ == "__main__":
    sys.exit(main())
