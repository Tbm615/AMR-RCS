"""The app in Vietnamese: every page builds, the texts come out translated,
texts with numbers through their patterns, untranslated ones stay English.

Run:
    python tests/test_i18n.py            (SHOT=1 also saves screenshots)
"""
from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = os.environ["APPDATA"] = tempfile.mkdtemp(prefix="rcs_i18n_cfg_")

from PySide6.QtWidgets import QApplication, QLabel, QPushButton  # noqa: E402

app = QApplication(sys.argv)
from rcs import i18n  # noqa: E402

PASS = FAIL = 0


def check(name, cond, extra=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def main() -> int:
    print("\n[tr]")
    check("English: as it is", i18n.tr("Connect") == "Connect")
    i18n.install("vi")
    check("a text", i18n.tr("Connect") == "Kết nối")
    check("a text with a number", i18n.tr("Events  (3)") == "Sự kiện  (3)")
    check("a text with a name", i18n.tr("connected — ws://1.2.3.4:9090") == "đã kết nối — ws://1.2.3.4:9090")
    check("not in the catalog: English", i18n.tr("Something new") == "Something new")
    check("not text: untouched", i18n.tr(None) is None and i18n.tr(3) == 3)
    check("spaces kept", i18n.tr("  Connect ") == "  Kết nối ")

    print("\n[the window in Vietnamese]")
    from rcs.ui import theme
    from rcs.ui.main_window import PAGES, MainWindow
    theme.apply(app)
    w = MainWindow()
    w.resize(1400, 900)
    w.show()
    app.processEvents()
    check("sidebar", w.btns["conn"].text().strip().startswith("Kết nối") and w.btns["teleop"].text().strip().startswith("Lái tay"),
          f"{w.btns['conn'].text()!r} {w.btns['teleop'].text()!r}")
    texts = {b.text() for b in w.findChildren(QPushButton)}
    for t in ("Lưu thay đổi", "Ngắt kết nối", "▶ Chạy", "Lưu gói chẩn đoán…", "Cảnh báo từ xa…"):
        check(f"button {t}", t in texts)
    labels = " ".join(lb.text() for lb in w.findChildren(QLabel))
    check("labels too", "Giới hạn tốc độ" in " ".join(g.title() for g in w.findChildren(__import__("PySide6.QtWidgets", fromlist=["QGroupBox"]).QGroupBox))
          or "Giới hạn" in labels)
    check("combo items", w.p_conn.model_combo.itemText(1) == "Hinson AMR (có sẵn trong app)", w.p_conn.model_combo.itemText(1))
    check("tooltips", "Dùng" in w.p_conn.update_ck.text() or w.p_conn.update_ck.text().startswith("Báo"))
    w._log("info", "Navigation", "goal sent to (1.00, 2.00)")
    check("status bar through a pattern", "đã gửi đích tới" in w.statusBar().currentMessage(), w.statusBar().currentMessage())
    shots = os.environ.get("SHOT")
    for key, _name in PAGES:
        w._show(key)
        app.processEvents()
        if shots:
            w.grab().save(str(Path(tempfile.gettempdir()) / f"rcs_vi_{key}.png"))
    check("every page builds and shows", True)
    w._show("status")
    w.p_status._refresh()
    hs = [r[1].text() for r in w.p_status.h_rows if not r[1].isHidden()]
    check("health lines translated", "Kết nối" in hs, str(hs))
    check("language choice offered", w.p_conn.lang.count() == 2 and w.p_conn.lang.findData("vi") >= 0)
    zb = w.p_map.zones
    check("zones box", zb.title() == "Vùng cấm và vùng giảm tốc" and zb.add_keep.text() == "+ Vùng cấm"
          and zb.save_btn.text() == "Lưu lên robot" and "Kết nối robot" in zb.where.text(), zb.where.text())
    zb.connected, zb.available, zb.map_name, zb.loaded = True, True, "kho", True
    zb.add_drawn("keepout", [[0, 0], [1, 0], [1, 1]])
    check("a new zone named in Vietnamese, listed in Vietnamese", zb.zones[0]["name"] == "Vùng cấm 1"
          and zb.list.item(0).text().endswith("Vùng cấm 1 · cấm"), zb.list.item(0).text())
    w.p_teleop.set_zone_note("info", ["In the slow zone Hall: Nav2 drives at most 40 % here.",
                                      "Manual driving is not limited by zones."])
    check("Teleop zone note, sentence by sentence", w.p_teleop.zone_note.text()
          == "Đang trong vùng chậm Hall: Nav2 chạy tối đa 40 % ở đây. Lái tay không bị giới hạn bởi vùng.",
          w.p_teleop.zone_note.text())
    w.close()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
