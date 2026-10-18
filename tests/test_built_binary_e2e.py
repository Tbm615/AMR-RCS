"""End-to-end test that spawns the Linux/Windows built binary as a subprocess
and verifies it opens a WebSocket to the mock rosbridge and publishes
subscribes + cmd_vel. Switch which binary via RCS_BINARY env var.

    RCS_BINARY=dist/rcs/rcs                                          # Linux
    RCS_BINARY="wine dist-win/rcs/rcs.exe"                           # Windows under wine
"""
from __future__ import annotations

import asyncio
import os
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

PORT = 9521
EXE = os.environ.get("RCS_BINARY", str(ROOT / "dist" / "rcs" / "rcs"))


def _start_mock():
    from tests import mock_rosbridge
    def run():
        asyncio.run(mock_rosbridge.main_async("0.0.0.0", PORT, duration=15))
    t = threading.Thread(target=run, daemon=True)
    t.start()
    time.sleep(0.6)
    return mock_rosbridge


def main() -> int:
    mock = _start_mock()
    env = dict(os.environ)
    env["QT_QPA_PLATFORM"] = "offscreen"
    env.pop("DISPLAY", None)
    # Point the host at loopback so wine localhost works OR a LAN IP for wine.
    host_arg = os.environ.get("RCS_MOCK_HOST", "127.0.0.1")
    args = shlex.split(EXE) + ["--autoconnect", "--host", f"{host_arg}:{PORT}"]
    print(f"spawning: {' '.join(args)}")
    proc = subprocess.Popen(args, cwd=str(ROOT), env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        time.sleep(10)
    finally:
        subprocess.call(["pkill", "-f", shlex.split(EXE)[-1]])
        proc.terminate()
        try: proc.wait(timeout=4)
        except subprocess.TimeoutExpired: proc.kill()

    out, _ = proc.communicate()
    hub = mock.HUB
    subscribed = hub.ever_subscribed
    publishes = hub.received_publishes
    advertised = hub.advertised_by_client

    print(f"\n--- binary ↔ mock ---")
    print(f"  subscribed topics: {len(subscribed)}  -> {sorted(subscribed)}")
    print(f"  advertised topics: {len(advertised)}  -> {list(advertised.keys())}")
    print(f"  publishes received: {len(publishes)}")

    ok = True
    def check(name, cond):
        nonlocal ok
        print(f"  {'OK' if cond else 'FAIL'}  {name}")
        if not cond: ok = False

    check("WebSocket opened and ≥5 topics subscribed", len(subscribed) >= 5)
    check("/scan_filtered subscribed", "/scan_filtered" in subscribed)
    check("/odom subscribed", "/odom" in subscribed)
    check("/map subscribed", "/map" in subscribed)
    check("/cmd_vel_safe advertised", "/cmd_vel_safe" in advertised)
    check("≥10 cmd_vel publishes", sum(1 for p in publishes if p["topic"] == "/cmd_vel_safe") >= 10)

    if not ok and out:
        print("\n--- binary stdout/stderr (last 30 lines) ---")
        for line in out.decode(errors="replace").splitlines()[-30:]:
            print(f"  | {line}")

    print(f"\n{'PASSED' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
