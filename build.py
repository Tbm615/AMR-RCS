"""Build a single-file RCS executable with PyInstaller.

Runs on the current platform (produces .exe on Windows, no extension on Linux).
Install dev deps first:
    pip install -r requirements.txt pyinstaller
Then:
    python build.py
Artifact lands in ./dist/.
"""
from __future__ import annotations

import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).parent
NAME = "rcs"


def main() -> int:
    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        print("PyInstaller not installed. Run: pip install pyinstaller", file=sys.stderr)
        return 1

    # Only remove our own build output — never the whole dist/ dir, which
    # may also contain packaged archives (.tar.gz, .zip) and sibling
    # OS-specific builds (e.g. rcs-windows-x86_64/).
    build = HERE / "build"
    if build.exists():
        shutil.rmtree(build)
    out_dir = HERE / "dist" / NAME
    if out_dir.exists():
        shutil.rmtree(out_dir)

    hidden = [
        "pyqtgraph.opengl",
        "OpenGL.platform.egl",
        "OpenGL.platform.glx",
        "OpenGL.platform.osmesa",
        "OpenGL.platform.wgl",
        "OpenGL.arrays",
        "OpenGL.GL",
    ]
    # Collecting whole packages instead of relying on static import scanning —
    # roslibpy, twisted and autobahn do a lot of runtime discovery that
    # PyInstaller's analyzer misses. Without --collect-all the frozen binary
    # launches but silently fails to open a WebSocket.
    collect_all = [
        "roslibpy",
        "twisted",
        "autobahn",
        "pyqtgraph",
        "OpenGL",
    ]
    mode = "--onefile" if os.environ.get("RCS_ONEFILE") else "--onedir"
    # Keep --console on Windows: PyInstaller's --windowed/WinMain subsystem
    # breaks Qt timers under wine (so cross-build verification would fail)
    # and the console flash on launch is a minor cost for a dev-grade tool.
    # Users can repackage with RCS_WINDOWED=1 on native Windows if desired.
    chrome = "--console" if os.environ.get("RCS_WINDOWED") != "1" else "--windowed"
    cmd = [
        sys.executable, "-m", "PyInstaller",
        "--noconfirm",
        "--clean",
        chrome,
        mode,
        "--name", NAME,
        "--paths", str(HERE),
    ]
    for h in hidden:
        cmd += ["--hidden-import", h]
    for c in collect_all:
        cmd += ["--collect-all", c]
    # Only include assets if there is something to include (empty dirs confuse PyInstaller)
    assets = HERE / "rcs" / "assets"
    if assets.exists():
        has_files = any(p.is_file() for p in assets.rglob("*"))
        if has_files:
            sep = ";" if platform.system() == "Windows" else ":"
            cmd += ["--add-data", f"{assets}{sep}rcs/assets"]
    cmd += [str(HERE / "rcs" / "__main__.py")]
    print("+", " ".join(cmd))
    return subprocess.call(cmd, cwd=HERE)


if __name__ == "__main__":
    raise SystemExit(main())
