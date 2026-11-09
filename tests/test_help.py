"""The "?" button: the guide of the page on screen, in the app's language.

Run:
    python tests/test_help.py
"""
from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = os.environ["APPDATA"] = tempfile.mkdtemp(prefix="rcs_help_cfg_")

from PySide6.QtWidgets import QApplication, QTextBrowser  # noqa: E402

app = QApplication(sys.argv)
from rcs import help as helptext, i18n  # noqa: E402
from rcs.ui.main_window import PAGES, MainWindow  # noqa: E402

PASS = FAIL = 0


def check(name, cond, extra=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def main() -> int:
    keys = [k for k, _ in PAGES]
    check("a guide for every page, in English", all(k in helptext.EN for k in keys),
          str([k for k in keys if k not in helptext.EN]))
    check("and in Vietnamese", all(k in helptext.VI for k in keys), str([k for k in keys if k not in helptext.VI]))
    w = MainWindow()
    w.show()
    check("the ? button is there", w.help_btn.isVisible() and w.help_btn.text() == "?")
    w._show("teleop")
    app.processEvents()
    w.help_btn.click()
    app.processEvents()
    dlg = w._help_dialog
    text = dlg.findChild(QTextBrowser).toPlainText()
    check("it opens the guide of the open page", dlg.isVisible() and "Teleop" in dlg.windowTitle()
          and "joystick" in text, dlg.windowTitle())
    check("with what works anywhere", "E-STOP" in text and "Layout" in text)
    dlg.close()
    w._show("battery")
    app.processEvents()
    dlg = w.show_help()
    check("another page, another guide", "Battery" in dlg.windowTitle()
          and "Charging" in dlg.findChild(QTextBrowser).toPlainText())
    dlg.close()
    i18n.set_language("vi")
    dlg = w.show_help("mission")
    check("in Vietnamese when the app is", "Nhiệm vụ" in dlg.windowTitle()
          and "điểm dừng" in dlg.findChild(QTextBrowser).toPlainText())
    i18n.set_language("en")
    dlg.close()
    w.close()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
