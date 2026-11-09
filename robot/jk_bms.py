"""JK-BMS over BLE: commands and frames. No Bluetooth, no ROS, so it can be
tested on any PC. Layout after RondaYummy/bms-monitor-ble (32-cell firmware).

Only the two read commands exist here on purpose: nothing in RCS may change a
setting of the BMS. A frame that does not add up is refused, not shown: JK has
several layouts, and a wrong number is worse than none.
"""
from __future__ import annotations

from typing import Optional

CHARACTERISTIC = "0000ffe1-0000-1000-8000-00805f9b34fb"
SERVICE = "0000ffe0-0000-1000-8000-00805f9b34fb"
CELL_INFO, DEVICE_INFO = 0x96, 0x97
FRAME_START = b"\x55\xaa\xeb\x90"
FRAME_LEN, FRAME_MAX = 300, 320
TYPE_CELLS = 0x02
JK_OUI = ("c8:47:80",)


def _crc(data: bytes) -> int:
    return sum(data) & 0xFF


def command(cmd: int) -> bytes:
    if cmd not in (CELL_INFO, DEVICE_INFO):
        raise ValueError("only read commands are allowed")
    frame = bytearray(20)
    frame[:4] = b"\xaa\x55\x90\xeb"
    frame[4] = cmd
    frame[19] = _crc(frame[:19])
    return bytes(frame)


def is_jk(name: Optional[str], address: str, services=()) -> bool:
    """Worth a try. A JK BMS can be given any name in its app, ours is "AMR
    Battery", and connecting finds out: a device without FFE1 is not one."""
    upper = (name or "").upper()
    return upper.startswith("JK") or "BMS" in upper or "BATTERY" in upper \
        or address.lower().startswith(JK_OUI) or SERVICE in [s.lower() for s in services]


class Assembler:
    """Notifications -> whole frames. A weak link loses pieces, so a frame only
    counts once its checksum is right."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data: bytes) -> Optional[bytes]:
        if data[:4] == FRAME_START:
            self.buf = bytearray()
        elif not self.buf:
            return None             # the tail of a frame whose start was lost
        self.buf += data
        n = len(self.buf)
        if n < FRAME_LEN:
            return None
        for size in (FRAME_LEN, n):
            if size <= FRAME_MAX and n >= size and _crc(self.buf[:size - 1]) == self.buf[size - 1]:
                frame = bytes(self.buf[:size])
                self.buf = bytearray()
                return frame
        if n >= FRAME_MAX:
            self.bad += 1
            self.buf = bytearray()
        return None


def _u(data: bytes, pos: int, size: int) -> int:
    return int.from_bytes(data[pos:pos + size], "little")


def _s(data: bytes, pos: int, size: int) -> int:
    return int.from_bytes(data[pos:pos + size], "little", signed=True)


def parse_cells(f: bytes) -> dict:
    """A cell info frame -> reading; raises ValueError if it does not add up."""
    if len(f) < FRAME_LEN or f[4] != TYPE_CELLS:
        raise ValueError("not a cell info frame")
    cells = [v * 0.001 for v in (_u(f, 6 + 2 * i, 2) for i in range(32)) if v]
    voltage = _s(f, 150, 4) * 0.001
    soc = f[173]
    if not cells or not all(0.5 <= c <= 5.0 for c in cells):
        raise ValueError("cell voltages out of range")
    if soc > 100:
        raise ValueError(f"state of charge {soc}")
    if abs(voltage - sum(cells)) > max(1.0, 0.05 * voltage):
        raise ValueError(f"pack {voltage:.2f} V does not match its cells ({sum(cells):.2f} V)")
    n = len(cells)
    avg = _u(f, 74, 2) * 0.001
    extra = {}
    # Checked against a real frame (tests/test_battery.py REAL_FRAME): the
    # average the BMS sends must match the cells, else these are left out.
    if abs(avg - sum(cells) / n) < 0.02:
        extra = {
            "cell_avg": round(avg, 3),
            "cell_max_no": f[78] + 1 if f[78] < n else None,
            "cell_min_no": f[79] + 1 if f[79] < n else None,
            "resistances": [round(_u(f, 80 + 2 * i, 2) * 0.001, 3) for i in range(n)],
            "balance_current": round(_s(f, 170, 2) * 0.001, 3),
            "balancing": f[172],            # 0 off, 1 charging balance, 2 discharging balance
            "total_cycle_ah": round(_u(f, 186, 4) * 0.001, 1),
            "runtime_s": _u(f, 194, 4),
            "alarms": _u(f, 166, 4),        # raw bits: their meaning is not checked yet
            "wire_alarms": _u(f, 146, 4),
        }
    return {
        **extra,
        "soc": soc,
        "soh": f[190],
        "voltage": round(voltage, 3),
        "current": round(_s(f, 158, 4) * 0.001, 3),
        "power": round(_s(f, 154, 4) * 0.001, 1),
        "cells": [round(c, 3) for c in cells],
        "delta": round(max(cells) - min(cells), 3),
        "temps": [round(_s(f, p, 2) * 0.1, 1) for p in (162, 164)],
        "mos_temp": round(_s(f, 144, 2) * 0.1, 1),
        "remaining_ah": round(_u(f, 174, 4) * 0.001, 2),
        "nominal_ah": round(_u(f, 178, 4) * 0.001, 2),
        "cycles": _u(f, 182, 4),
        "charging": bool(f[198]),
        "discharging": bool(f[199]),
    }
