"""Answer for a cheap Bluetooth dongle where it fails to.

Clones of the CSR8510 (0a12:0001) say they support commands they ignore or
refuse. On the robot's kernel 5.15 that is why BlueZ cannot use one at all:
"Set Event Filter" fails and the adapter never comes up. So the battery service
talks to the dongle over USB with bumble, and this stands in the way of the few
commands that would hang or fail. No Bluetooth stack in here, for the tests.
"""
from __future__ import annotations

import asyncio

READ_EXTENDED_FEATURES = 0x1004     # never answered
LE_SET_RANDOM_ADDRESS = 0x2005      # refused: we use the dongle's own address


def answer(packet: bytes):
    """The Command Complete event to give instead, or None to pass it on."""
    if packet[:1] != b"\x01":
        return None
    op = packet[1] | packet[2] << 8
    if op == READ_EXTENDED_FEATURES:
        ret = bytes([0, packet[4] if len(packet) > 4 else 0, 0]) + bytes(8)
    elif op == LE_SET_RANDOM_ADDRESS:
        ret = b"\x00"
    else:
        return None
    return bytes([4, 0x0E, 3 + len(ret), 1, packet[1], packet[2]]) + ret


class HciQuirks:
    """Sits where bumble expects the controller's sink."""

    def __init__(self, sink) -> None:
        self.sink = sink
        self.host = None

    def on_packet(self, packet) -> None:
        packet = bytes(packet)
        event = answer(packet)
        if event is None:
            self.sink.on_packet(packet)
        else:
            asyncio.get_running_loop().call_soon(self.host.on_packet, event)
