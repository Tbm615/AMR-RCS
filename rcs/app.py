"""Qt application entrypoint."""
from __future__ import annotations

import argparse
import io as _io
import os
import sys
import tempfile
import traceback

# PyInstaller --windowed on Windows sets sys.stdout and sys.stderr to None.
# Several libraries (autobahn's logger, twisted's startLogging) assume a
# file-like object and crash silently when they .write() to None, killing
# the WebSocket protocol before it ever opens. Shim with a /dev/null-ish
# writer so every "stdout write" lands somewhere valid, and mirror into a
# log file so --windowed Windows builds leave a trail we can inspect.
if sys.stdout is None:
    sys.stdout = _io.StringIO()
if sys.stderr is None:
    sys.stderr = _io.StringIO()
if sys.stdin is None:
    sys.stdin = _io.StringIO()

try:
    _logf = open(os.path.join(tempfile.gettempdir(), "rcs.log"), "a", buffering=1)
    _logf.write(f"\n--- RCS launched pid={os.getpid()} argv={sys.argv!r} ---\n")

    class _Tee:
        def __init__(self, a, b):
            self.a, self.b = a, b
        def write(self, s):
            for x in (self.a, self.b):
                try: x.write(s)
                except Exception: pass
            return len(s) if isinstance(s, str) else 0
        def flush(self):
            for x in (self.a, self.b):
                try: x.flush()
                except Exception: pass

    sys.stdout = _Tee(sys.stdout, _logf)
    sys.stderr = _Tee(sys.stderr, _logf)

    def _excepthook(exctype, value, tb):
        _logf.write("UNCAUGHT EXCEPTION:\n")
        traceback.print_exception(exctype, value, tb, file=_logf)
        _logf.flush()
    sys.excepthook = _excepthook
except Exception:
    pass

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QApplication

from . import APP_NAME, ORG_NAME
from .ui.main_window import MainWindow
from .ui import theme


def _parse_argv(argv):
    ap = argparse.ArgumentParser(prog="rcs", description=f"{APP_NAME} — robot control console")
    ap.add_argument("--autoconnect", action="store_true",
                    help="connect to the active robot profile on launch")
    ap.add_argument("--host", type=str,
                    help="override active profile's host:port (e.g. 192.168.1.10:9090)")
    # Qt swallows its own args; we drop unknowns so --platform etc. keep working.
    args, remaining = ap.parse_known_args(argv[1:])
    return args, [argv[0]] + remaining


def main() -> int:
    args, qt_argv = _parse_argv(sys.argv)
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QApplication(qt_argv)
    app.setApplicationName(APP_NAME)
    app.setOrganizationName(ORG_NAME)
    theme.apply(app)

    win = MainWindow()

    if args.host:
        host, _, port = args.host.partition(":")
        p = win.profiles.active()
        if p:
            p.host = host
            if port:
                try:
                    p.port = int(port)
                except ValueError:
                    pass
            win.profiles.upsert(p)

    win.show()

    if args.autoconnect:
        prof = win.profiles.active()
        if prof:
            QTimer.singleShot(200, lambda p=prof: win._on_connect(p))

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
