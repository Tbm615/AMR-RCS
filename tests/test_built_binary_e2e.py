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
_DEFAULT_EXE = ROOT / "dist" / "rcs" / ("rcs.exe" if sys.platform == "win32" else "rcs")
EXE = os.environ.get("RCS_BINARY", str(_DEFAULT_EXE))


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
    # A plain path (incl. Windows paths with backslashes) is used as-is;
    # anything else, e.g. "wine path/to/rcs.exe", is split like a shell would.
    exe_argv = [EXE] if os.path.exists(EXE) else shlex.split(EXE)
    args = exe_argv + ["--autoconnect", "--host", f"{host_arg}:{PORT}"]
    print(f"spawning: {' '.join(args)}")
    proc = subprocess.Popen(args, cwd=str(ROOT), env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        time.sleep(10)
    finally:
        # Under wine the real rcs.exe is a grandchild of the launcher.
        if len(exe_argv) > 1 and sys.platform != "win32":
            subprocess.call(["pkill", "-f", exe_argv[-1]])
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
    def check(name, cond, detail=""):
        nonlocal ok
        print(f"  {'OK' if cond else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))
        if not cond: ok = False

    check("WebSocket opened and ≥5 topics subscribed", len(subscribed) >= 5)
    check("/scan_filtered subscribed", "/scan_filtered" in subscribed)
    check("/odom subscribed", "/odom" in subscribed)
    check("/map subscribed", "/map" in subscribed)
    check("/tf_static subscribed (static lidar mount)", "/tf_static" in subscribed)
    check("Nav2 action status subscribed", "/navigate_to_pose/_action/status" in subscribed)
    # Idle teleop must stay silent: a constant zero stream on /cmd_vel_safe
    # would override Nav2 on the real robot.
    check("no cmd_vel published while idle",
          not any(p["topic"] == "/cmd_vel_safe" for p in publishes),
          f"{sum(1 for p in publishes if p['topic'] == '/cmd_vel_safe')} msgs")

    if not ok and out:
        print("\n--- binary stdout/stderr (last 30 lines) ---")
        for line in out.decode(errors="replace").splitlines()[-30:]:
            print(f"  | {line}")

    print(f"\n{'PASSED' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
