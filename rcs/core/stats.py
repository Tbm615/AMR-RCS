"""How the robot did over a period: missions run and how they ended, distance,
time at work, battery used, E-STOPs, stops for a lost app, trips to charge,
errors. Built from the run reports (runs.py) and the event log, both kept per
robot on this PC. No Qt here; report_html() is what the Statistics page shows
and saves as PDF.
"""
from __future__ import annotations

import html
import time
from collections import OrderedDict
from typing import Iterable, List

ENDED_BADLY = ("failed", "stopped", "interrupted")


def summarize(runs: Iterable, events: Iterable, since: float, until: float) -> dict:
    rs = [r for r in runs if since <= (r.start or 0) < until]
    es = [e for e in events if since <= e.t < until]
    by = {k: sum(1 for r in rs if r.result == k) for k in ("complete", "failed", "stopped", "interrupted",
                                                           "skipped", "running")}
    tried = by["complete"] + sum(by[k] for k in ENDED_BADLY)
    used = [r.soc_start - r.soc_end for r in rs if r.soc_start is not None and r.soc_end is not None]
    missions: "OrderedDict[str, dict]" = OrderedDict()
    for r in sorted(rs, key=lambda r: r.start):
        m = missions.setdefault(r.mission, {"runs": 0, "complete": 0, "time": 0.0, "distance": 0.0})
        m["runs"] += 1
        m["complete"] += r.result == "complete"
        m["time"] += r.duration() if r.result != "skipped" else 0.0
        m["distance"] += r.distance_m or 0.0

    def count(source: str, word: str, level: str = "") -> int:
        return sum(1 for e in es if e.source == source and word in e.text and (not level or e.level == level))
    return {
        "since": since, "until": until, "runs": len(rs), "by_result": by,
        "success": (by["complete"] / tried) if tried else None,
        "distance_m": sum(r.distance_m or 0.0 for r in rs),
        "work_s": sum(r.duration() for r in rs if r.result not in ("skipped",)),
        "reached": sum(r.reached for r in rs), "retries": sum(r.retries for r in rs),
        "battery_used": sum(used), "missions": missions,
        "estops": count("E-STOP", "engaged"), "self_stops": count("E-STOP", "stopped itself"),
        "charge_trips": count("Charging", "going to charge"),
        "errors": sum(1 for e in es if e.level == "error"), "warnings": sum(1 for e in es if e.level == "warn"),
        "skipped_why": [r.reason for r in rs if r.result == "skipped" and r.reason][:10],
    }


def daily(runs: Iterable, since: float, days: int) -> List[dict]:
    """One row per day: runs, complete, distance (local days)."""
    rs = list(runs)
    out = []
    for d in range(days):
        start = time.mktime(time.localtime(since + d * 86400)[:3] + (0, 0, 0, 0, 0, -1))
        end = start + 86400
        day = [r for r in rs if start <= (r.start or 0) < end]
        out.append({"day": time.strftime("%Y-%m-%d", time.localtime(start)), "runs": len(day),
                    "complete": sum(1 for r in day if r.result == "complete"),
                    "distance_m": sum(r.distance_m or 0.0 for r in day)})
    return out


def _hms(s: float) -> str:
    s = int(s)
    return f"{s // 3600}h {s % 3600 // 60:02d}m" if s >= 3600 else f"{s // 60}m {s % 60:02d}s"


def report_html(robot: str, s: dict, days: List[dict]) -> str:
    """The report as HTML (shown on the page, printed to PDF)."""
    e = html.escape
    period = (f"{time.strftime('%Y-%m-%d', time.localtime(s['since']))} — "
              f"{time.strftime('%Y-%m-%d', time.localtime(s['until'] - 1))}")
    succ = f"{s['success'] * 100:.0f}%" if s["success"] is not None else "—"
    by = s["by_result"]
    rows = "".join(f"<tr><td>{e(name)}</td><td>{m['runs']}</td><td>{m['complete']}</td>"
                   f"<td>{_hms(m['time'])}</td><td>{m['distance'] / 1000:.2f} km</td></tr>"
                   for name, m in s["missions"].items()) or "<tr><td colspan=5>no mission run</td></tr>"
    drows = "".join(f"<tr><td>{d['day']}</td><td>{d['runs']}</td><td>{d['complete']}</td>"
                    f"<td>{d['distance_m'] / 1000:.2f} km</td></tr>" for d in days if d["runs"])
    skipped = "".join(f"<li>{e(w)}</li>" for w in s["skipped_why"])
    return f"""<html><head><style>
body {{ font-family: sans-serif; }} h1 {{ font-size: 18pt; }} h2 {{ font-size: 13pt; margin-top: 14px; }}
table {{ border-collapse: collapse; }} td, th {{ border: 1px solid #999; padding: 3px 8px; text-align: left; }}
.k {{ color: #555; }}
</style></head><body>
<h1>{e(robot)}: how it did</h1>
<p class=k>{period} · made by RCS on {time.strftime('%Y-%m-%d %H:%M')}</p>
<h2>Missions</h2>
<table><tr><th>Runs</th><th>Complete</th><th>Failed</th><th>Stopped</th><th>Interrupted</th><th>Skipped</th><th>Success</th></tr>
<tr><td>{s['runs']}</td><td>{by['complete']}</td><td>{by['failed']}</td><td>{by['stopped']}</td>
<td>{by['interrupted']}</td><td>{by['skipped']}</td><td>{succ}</td></tr></table>
<p>Distance <b>{s['distance_m'] / 1000:.2f} km</b> · at work <b>{_hms(s['work_s'])}</b> ·
waypoints reached {s['reached']} · goals retried {s['retries']} · battery used {s['battery_used']:.0f}%</p>
<h2>Safety and incidents</h2>
<p>E-STOP engaged {s['estops']}× · stopped itself (app lost) {s['self_stops']}× · trips to charge
{s['charge_trips']} · errors {s['errors']} · warnings {s['warnings']}</p>
<h2>Per mission</h2>
<table><tr><th>Mission</th><th>Runs</th><th>Complete</th><th>Time</th><th>Distance</th></tr>{rows}</table>
{f'<h2>Per day</h2><table><tr><th>Day</th><th>Runs</th><th>Complete</th><th>Distance</th></tr>{drows}</table>' if drows else ''}
{f'<h2>Why runs were skipped</h2><ul>{skipped}</ul>' if skipped else ''}
</body></html>"""
