"""End-to-end test of the built Windows rcs.exe under wine against our
mock rosbridge server. Runs the mock in a background thread, spawns the
Windows exe (via wine) with --autoconnect, and asserts the mock saw
real WebSocket subscribe + publish traffic from the Windows binary.

This is the strongest Windows-portability evidence we can produce without
a Windows machine: the real PE32+ x86-64 binary is loaded by the Windows
loader (wine), its Qt event loop starts, its roslibpy opens a WebSocket
on Windows paths, and our mock records what the Windows binary sent.
"""
from __future__ import annotations

import asyncio
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

PORT = 9393
MOCK_DURATION = 20  # seconds the mock stays up
EXE_RUN_SECONDS = 12
WINE = ROOT / ".build-tools" / "wine-11.19-amd64" / "bin" / "wine"
EXE = ROOT / "dist-win" / "rcs" / "rcs.exe"


def _start_mock():
    from tests import mock_rosbridge
    def run():
        asyncio.run(mock_rosbridge.main_async("127.0.0.1", PORT, duration=MOCK_DURATION))
    t = threading.Thread(target=run, daemon=True)
    t.start()
    time.sleep(0.5)
    return mock_rosbridge


def main() -> int:
    if not EXE.exists():
        print(f"NOT FOUND: {EXE} — build it first", file=sys.stderr)
        return 2
    if not WINE.exists():
        print(f"NOT FOUND: {WINE} — provision wine first", file=sys.stderr)
        return 2

    mock = _start_mock()
    env = {**os.environ,
           "WINEPREFIX": str(ROOT / ".build-tools" / "wine-prefix"),
           "WINEARCH": "win64",
           "WINEDEBUG": "-all",
           "WINEDLLOVERRIDES": "mscoree,mshtml=",
           "QT_QPA_PLATFORM": "offscreen"}
    env.pop("DISPLAY", None)

    args = [str(WINE), str(EXE), "--autoconnect", "--host", f"127.0.0.1:{PORT}"]
    print(f"spawning Windows binary via wine (duration ~{EXE_RUN_SECONDS}s)...")
    proc = subprocess.Popen(args, cwd=str(ROOT), env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(EXE_RUN_SECONDS)
    finally:
        # Terminate the wine launcher AND the real rcs.exe child
        subprocess.call(["pkill", "-f", "rcs.exe"])
        proc.terminate()
        try:
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            proc.kill()

    hub = mock.HUB
    subscribed = hub.ever_subscribed       # durable across client disconnect
    advertised = hub.advertised_by_client
    publishes = hub.received_publishes

    print(f"\n--- Windows binary ↔ mock rosbridge ---")
    print(f"  topics the Windows client subscribed to: {len(subscribed)}")
    for t in sorted(subscribed):
        print(f"    - {t}")
    print(f"  topics the Windows client advertised: {len(advertised)}")
    for t, ty in advertised.items():
        print(f"    - {t}  ({ty})")
    print(f"  messages the Windows client published: {len(publishes)}")

    ok = True
    def check(name, cond):
        nonlocal ok
        print(f"  {'OK' if cond else 'FAIL'}  {name}")
        if not cond:
            ok = False

    print()
    check("Windows binary opened WebSocket + subscribed to ≥5 topics", len(subscribed) >= 5)
    check("Windows binary subscribed to /scan_filtered", "/scan_filtered" in subscribed)
    check("Windows binary subscribed to /odom", "/odom" in subscribed)
    check("Windows binary subscribed to /map", "/map" in subscribed)
    check("Windows binary subscribed to /tf", "/tf" in subscribed)
    check("Windows binary advertised /cmd_vel_safe", "/cmd_vel_safe" in advertised)
    check("Windows binary published ≥10 messages", len(publishes) >= 10)
    check("Windows binary published cmd_vel", any(p["topic"] == "/cmd_vel_safe" for p in publishes))

    print(f"\n{'PASSED' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
