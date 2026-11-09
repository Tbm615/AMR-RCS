#!/bin/bash
# Install or update the robot side of RCS in one go, on the robot:
#   bash install_robot.sh            copy, check, start at boot, start now
#   bash install_robot.sh --dry-run  say what it would do, change nothing
# Run it from the robot/ folder of the release (copied to the robot), as the
# user that runs ROS (not root). Safe to run again: it updates in place, keeps
# the E-STOP latch, users, audit and battery logs, and adds the boot entry once.
#
# Env: RCS_WS (default ~/tbm_test), the workspace with bringup and navigation.
set -euo pipefail

DRY=0
[ "${1:-}" = "--dry-run" ] && DRY=1
HERE="$(cd "$(dirname "$0")" && pwd)"
RCS_WS="${RCS_WS:-$HOME/tbm_test}"
export RCS_WS
DEST="$RCS_WS/rcs_robot"
VERSION="$(sed -n 's/^ROBOT_SIDE = "\([^"]*\)".*/\1/p' "$HERE/rcs_launcher.py")"
say() { echo "• $*"; }
run() { if [ "$DRY" = 1 ]; then echo "  would run: $*"; else "$@"; fi; }
warn() { echo "! $*" >&2; }

say "RCS robot side v$VERSION -> $DEST (workspace $RCS_WS)"
[ "$(id -u)" = 0 ] && { warn "run it as the robot's user, not root"; exit 1; }
[ -f /opt/ros/humble/setup.bash ] || { warn "ROS 2 Humble is not installed (/opt/ros/humble)"; exit 1; }
[ -f "$RCS_WS/install/setup.bash" ] || warn "no $RCS_WS/install: build the robot workspace first (colcon build)"

# what the launcher and the app need from apt
missing=()
for pkg in ros-humble-rosbridge-suite ros-humble-compressed-image-transport; do
    dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
done
if [ ${#missing[@]} -gt 0 ]; then
    warn "missing: ${missing[*]}"
    warn "install them: sudo apt install ${missing[*]}"
fi

# Python modules: websockets (gateway), bumble (battery over a USB dongle)
python3 - <<'PY' || true
import importlib, sys
need = {"websockets": "websockets>=13 (sign-in gateway)", "bumble": "bumble (battery, only with a USB dongle)"}
for mod, why in need.items():
    try:
        m = importlib.import_module(mod)
        if mod == "websockets" and int(m.__version__.split(".")[0]) < 13:
            print(f"! websockets {m.__version__} is too old: pip3 install --user 'websockets>=13'", file=sys.stderr)
    except ImportError:
        print(f"! no {why}: pip3 install --user {why.split(' ')[0]!r}", file=sys.stderr)
PY

if [ -d "$DEST" ] && [ "$(cd "$HERE" && pwd -P)" = "$(cd "$DEST" && pwd -P)" ]; then
    say "already in $DEST: nothing to copy"
else
    say "copying the files"
    run mkdir -p "$DEST"
    for f in "$HERE"/*.py "$HERE"/*.sh "$HERE"/*.rules "$HERE"/README.md; do
        [ -e "$f" ] || continue
        run cp "$f" "$DEST/"
    done
fi
run chmod +x "$DEST"/*.py "$DEST"/*.sh

ENTRY="@reboot sleep 10 && RCS_WS=$RCS_WS $DEST/start_rcs_robot.sh"
if crontab -l 2>/dev/null | grep -Fq "$DEST/start_rcs_robot.sh"; then
    say "start at boot: already set"
else
    say "start at boot: adding the crontab entry"
    if [ "$DRY" = 1 ]; then echo "  would add: $ENTRY"; else (crontab -l 2>/dev/null; echo "$ENTRY") | crontab -; fi
fi

say "restarting the robot side"
if [ "$DRY" = 0 ]; then
    pkill -f start_rcs_robot.sh || true
    pkill -f rcs_launcher.py || true
    pkill -f rcs_battery.py || true
    pkill -f rcs_gateway.py || true
    pkill -INT -f rosbridge_websocket || true      # SIGINT: a plain kill leaves it holding the port
    pkill -f rcs_safety.py || true                 # last: the drives stay stopped meanwhile
    sleep 3
    nohup setsid "$DEST/start_rcs_robot.sh" >/dev/null 2>&1 &
    sleep 1
else
    echo "  would stop the old programs and start $DEST/start_rcs_robot.sh"
fi
if [ -s "$RCS_WS/rcs_users.json" ]; then
    say "sign-in: on (users exist). Manage them: python3 $DEST/rcs_gateway.py users list"
else
    say "sign-in: off, anyone on the network can drive. To turn it on:"
    say "  python3 $DEST/rcs_gateway.py users add admin --role admin, then run this again"
fi
say "done. Logs: $RCS_WS/runlog/. In the app: Robot programs should say 'Robot launcher online'."
