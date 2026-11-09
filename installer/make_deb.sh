#!/bin/bash
# Make rcs_<version>_amd64.deb from the PyInstaller build in dist/rcs:
#   /opt/rcs/            the app
#   /usr/bin/rcs         starts it
#   /usr/share/applications/rcs.desktop, an icon
# Usage: installer/make_deb.sh [out dir]      (run python build.py first)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
RCS="$(cd "$HERE/.." && pwd)"
OUT="${1:-$RCS/dist}"
VERSION="$(sed -n 's/^__version__ = "\(.*\)"/\1/p' "$RCS/rcs/__init__.py")"
DEBV="${VERSION/b/~beta}"          # 0.6.0b1 -> 0.6.0~beta1: sorts before 0.6.0
[ -x "$RCS/dist/rcs/rcs" ] || { echo "no dist/rcs/rcs: run python build.py first"; exit 1; }
ROOT="$(mktemp -d)"
trap 'rm -rf "$ROOT"' EXIT
mkdir -p "$ROOT/DEBIAN" "$ROOT/opt" "$ROOT/usr/bin" "$ROOT/usr/share/applications" "$ROOT/usr/share/pixmaps"
cp -a "$RCS/dist/rcs" "$ROOT/opt/rcs"
ln -s /opt/rcs/rcs "$ROOT/usr/bin/rcs"
ICON="$(ls "$RCS"/rcs/assets/icons/*.png 2>/dev/null | head -1 || true)"
[ -n "$ICON" ] && cp "$ICON" "$ROOT/usr/share/pixmaps/rcs.png"
cat > "$ROOT/usr/share/applications/rcs.desktop" <<DESK
[Desktop Entry]
Type=Application
Name=RCS
Comment=Robot Control System for AMRs
Exec=/opt/rcs/rcs
Icon=rcs
Terminal=false
Categories=Utility;Development;
DESK
SIZE="$(du -sk "$ROOT/opt" | cut -f1)"
cat > "$ROOT/DEBIAN/control" <<CTRL
Package: rcs
Version: $DEBV
Section: misc
Priority: optional
Architecture: amd64
Installed-Size: $SIZE
Maintainer: b4lanc3 <tranbinhminh615@gmail.com>
Depends: libgl1, libegl1, libxkbcommon0, libxkbcommon-x11-0, libxcb-cursor0, libdbus-1-3
Description: RCS, the operator console for AMRs
 Drive, map, navigate and watch ROS 2 robots through rosbridge. No ROS 2 or
 Python needed on this PC.
CTRL
mkdir -p "$OUT"
dpkg-deb --root-owner-group --build "$ROOT" "$OUT/rcs_${DEBV}_amd64.deb" >/dev/null
echo "$OUT/rcs_${DEBV}_amd64.deb"
