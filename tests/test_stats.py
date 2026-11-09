"""Statistics from the run reports and the event log, and the PDF report.

Run:
    python tests/test_stats.py
"""
from __future__ import annotations

import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = os.environ["APPDATA"] = tempfile.mkdtemp(prefix="rcs_stats_cfg_")

from rcs.core import stats  # noqa: E402
from rcs.core.events import Event  # noqa: E402
from rcs.core.runs import Run  # noqa: E402

PASS = FAIL = 0


def check(name, cond, extra=""):
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def run(i, mission, result, start, dur, dist, soc=(80, 70), reason=""):
    return Run(id=str(i), mission_id=mission, mission=mission, start=start, end=start + dur, result=result,
               distance_m=dist, reached=3, retries=1 if result == "failed" else 0, soc_start=soc[0], soc_end=soc[1],
               reason=reason)


def main() -> int:
    now = time.time()
    day = 86400
    runs = [run(1, "Rounds", "complete", now - 3600, 600, 120.0),
            run(2, "Rounds", "failed", now - 7200, 300, 40.0),
            run(3, "Dock", "complete", now - 2 * day, 900, 300.0),
            run(4, "Rounds", "skipped", now - 3 * day, 0, 0.0, (None, None), reason="battery 12% (needs 30%)"),
            run(5, "Old", "complete", now - 40 * day, 600, 999.0)]
    events = [Event(now - 100, "error", "E-STOP", "engaged"), Event(now - 200, "warn", "E-STOP",
              "the robot stopped itself: lost the RCS app"), Event(now - 300, "warn", "Charging",
              "battery 20%: going to charge at Dock"), Event(now - 50 * day, "error", "E-STOP", "engaged")]
    print("\n[the numbers]")
    s = stats.summarize(runs, events, now - 7 * day, now + 1)
    check("only the period's runs", s["runs"] == 4, str(s["runs"]))
    check("how they ended", s["by_result"]["complete"] == 2 and s["by_result"]["failed"] == 1
          and s["by_result"]["skipped"] == 1)
    check("success over the runs tried", abs(s["success"] - 2 / 3) < 1e-9, str(s["success"]))
    check("distance", abs(s["distance_m"] - 460.0) < 1e-9)
    check("time at work, not the skipped one", abs(s["work_s"] - 1800) < 1e-6)
    check("battery used", abs(s["battery_used"] - 30) < 1e-9, str(s["battery_used"]))
    check("E-STOPs, self stops, trips to charge", (s["estops"], s["self_stops"], s["charge_trips"]) == (1, 1, 1))
    check("per mission", s["missions"]["Rounds"]["runs"] == 3 and s["missions"]["Dock"]["complete"] == 1)
    check("why runs were skipped", s["skipped_why"] == ["battery 12% (needs 30%)"])
    empty = stats.summarize([], [], now - day, now)
    check("nothing run: no success rate", empty["success"] is None and empty["runs"] == 0)
    d = stats.daily(runs, now - 6 * day, 7)
    check("a row per day", len(d) == 7 and sum(r["runs"] for r in d) == 4, str([r["runs"] for r in d]))
    h = stats.report_html("AMR <1>", s, d)
    check("the report", "AMR &lt;1&gt;" in h and "0.46 km" in h and "67%" in h and "Rounds" in h)

    print("\n[the Schedule page]")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication(sys.argv)
    from rcs.ui.main_window import MainWindow
    w = MainWindow()
    w.runs.runs.extend(runs)
    w.events.add("error", "E-STOP", "engaged")
    w.p_schedule.period.setCurrentIndex(w.p_schedule.period.findData(30))
    w.p_schedule.show_stats()
    text = w.p_schedule.stats_view.toPlainText()
    check("shown on the page", "how it did" in text and "Rounds" in text and "E-STOP engaged" in text, text[:120])
    out = w.p_schedule.save_pdf(str(Path(tempfile.mkdtemp()) / "report"))
    data = Path(out).read_bytes()
    check("saved as PDF", out.endswith(".pdf") and data.startswith(b"%PDF") and len(data) > 2000, f"{len(data)} bytes")
    w.close()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
