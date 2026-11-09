"""Scheduled missions and run reports.

Run:
    python tests/test_schedule.py

Needs no robot: the clock is faked for the schedule, and the mission executor
is driven by moving the robot pose onto each waypoint.
"""
from __future__ import annotations

import csv
import json
import os
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["XDG_CONFIG_HOME"] = tempfile.mkdtemp(prefix="rcs_sched_cfg_")
os.environ["APPDATA"] = os.environ["XDG_CONFIG_HOME"]

from PySide6.QtWidgets import QApplication

from rcs.core.mission import Mission, Waypoint
from rcs.core.runs import RunLog
from rcs.core.schedule import Schedule, Scheduler, ScheduleStore
from rcs.ui.main_window import MainWindow

app = QApplication(sys.argv)
PASS = FAIL = 0
TMP = Path(tempfile.mkdtemp(prefix="rcs_sched_"))


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def at(y, mo, d, h, mi, s=0) -> float:
    return datetime(y, mo, d, h, mi, s).timestamp()


def test_times() -> None:
    print("\n[when a schedule runs]")
    s = Schedule.new("m1", "08:30")             # Mon-Fri by default
    # 2026-11-06 is a Friday
    check("its time on a weekday", s.times_between(at(2026, 11, 6, 8, 0), at(2026, 11, 6, 9, 0))
          == [at(2026, 11, 6, 8, 30)])
    check("not on Saturday", s.times_between(at(2026, 11, 7, 8, 0), at(2026, 11, 7, 9, 0)) == [])
    check("next after Friday is Monday", s.next_time(at(2026, 11, 6, 9, 0)) == at(2026, 11, 9, 8, 30))
    check("the second itself counts once", s.times_between(at(2026, 11, 6, 8, 29, 59), at(2026, 11, 6, 8, 30))
          == [at(2026, 11, 6, 8, 30)] and s.times_between(at(2026, 11, 6, 8, 30), at(2026, 11, 6, 8, 31)) == [])
    s.days = list(range(7))
    check("every day", s.days_text() == "every day" and len(s.times_between(at(2026, 11, 1, 0, 0),
                                                                            at(2026, 11, 8, 0, 0))) == 7)
    s.days = []
    check("no day: never", s.next_time(time.time()) is None)
    try:
        Schedule.from_dict({"id": "x", "mission_id": "m", "at": "25:00"})
        bad = False
    except ValueError:
        bad = True
    check("a time that is not a time is refused", bad)
    store = ScheduleStore(TMP / "schedules.json")
    store.upsert(Schedule.new("m1", "07:00"))
    (TMP / "s2.json").write_text(json.dumps({"schedules": [{"id": "a", "mission_id": "m", "at": "9:99"},
                                                           {"id": "b", "mission_id": "m", "at": "10:00"}]}))
    check("kept on disk", len(ScheduleStore(TMP / "schedules.json").schedules) == 1)
    check("one bad entry does not lose the others", [s.id for s in ScheduleStore(TMP / "s2.json").schedules] == ["b"])

    print("\n[the scheduler]")
    clock = [at(2026, 11, 6, 8, 29, 50)]
    started = []
    store = ScheduleStore(TMP / "sch3.json")
    a = Schedule.new("m1", "08:30")
    b = Schedule.new("m2", "08:30")
    b.enabled = False
    store.upsert(a)
    store.upsert(b)
    sc = Scheduler(store, lambda s: started.append(s.mission_id) or "", clock=lambda: clock[0])
    fired = []
    sc.fired.connect(lambda s, why: fired.append((s.mission_id, why)))
    sc.tick()
    check("before its time: nothing", not started)
    clock[0] += 15
    sc.tick()
    check("at its time: started", started == ["m1"] and fired == [("m1", "")])
    clock[0] += 5
    sc.tick()
    check("once", started == ["m1"])
    check("a schedule that is off does not run", "m2" not in started)
    clock[0] = at(2026, 11, 9, 8, 35)           # PC asleep over the weekend, woke 5 min late
    sc.tick()
    check("a time missed while asleep is skipped, not run late", started == ["m1"]
          and "asleep" in fired[-1][1], f"{fired[-1]}")
    sc2 = Scheduler(store, lambda s: started.append("late") or "", clock=lambda: at(2026, 11, 10, 9, 0))
    sc2.tick()
    check("times before the app started are not made up for", "late" not in started)


def test_runs() -> None:
    print("\n[run reports]")
    m = Mission.new("tour")
    m.waypoints = [Waypoint.new("A", 0, 0), Waypoint.new("B", 1, 0)]
    path = TMP / "runs.json"
    log = RunLog(path)
    r = log.start(m, "schedule", "s1", 80)
    log.reached("A")
    log.retry("aborted at B, retry 1")
    log.add_distance(3.21)
    check("written at each step", json.loads(path.read_text())["runs"][0]["reached"] == 1)
    log2 = RunLog(path)                     # the app crashed here
    check("a run cut by a crash is reported as interrupted", log2.runs[0].result == "interrupted"
          and "crash" in log2.runs[0].reason)
    log.reached("B")
    log.lap()
    done = log.finish("complete", soc=74)
    check("finished", done.result == "complete" and done.laps == 1 and done.reached == 2 and done.retries == 1
          and done.distance_m == 3.2 and (done.soc_start, done.soc_end) == (80, 74))
    log.skipped("m9", "night", "the battery was at 12%, below 30%", "s2", 12)
    out = TMP / "runs.csv"
    n = log.export_csv(out)
    rows = list(csv.DictReader(open(out, encoding="utf-8")))
    check("exported", n == 2 and rows[0]["result"] == "complete" and rows[1]["result"] == "skipped"
          and "12%" in rows[1]["reason"], f"{rows[-1]}")
    for i in range(300):
        log.skipped("m", "x", "test")
    check("kept for good, never trimmed", len(RunLog(path).runs) == 302)


def drive_to(w, wp, ticks=8) -> None:
    w.p_mission.update_robot_pose(wp.x, wp.y, 0.0, draw=False)
    w.p_mission.on_nav_status("succeeded")
    w.p_mission._nav_state = "succeeded"
    for _ in range(ticks):
        w.p_mission._exec_tick()


def test_app() -> None:
    print("\n[scheduled run in the app]")
    w = MainWindow()
    w.show()
    m = Mission.new("tour")
    m.waypoints = [Waypoint.new("A", 2.0, 0.0), Waypoint.new("B", 2.0, 2.0)]
    w.missions.upsert(m)
    w.p_mission.reload()
    s = Schedule.new(m.id, "08:00")
    s.min_soc = 30
    w.p_schedule.save(s)
    check("listed with its next run", w.p_schedule.table.rowCount() == 1
          and w.p_schedule.table.item(0, 5).text() != "—")

    why = w._start_scheduled(s)
    check("not connected: skipped, and why", "not connected" in why and w.runs.runs[-1].result == "skipped")
    ready = {"why": ""}
    w._not_ready = lambda _s: ready["why"]
    w._battery = {"reading": {"soc": 64}, "age": 1.0}
    why = w._start_scheduled(s)
    check("ready: it starts", why == "" and w.p_mission._exec_running, why)
    run = w.runs.active
    check("its report is open", run is not None and run.trigger == "schedule" and run.schedule_id == s.id
          and run.soc_start == 64)
    check("a second one while it runs is skipped", "another mission" in w._start_scheduled(s))
    w._track_run_distance((0.0, 0.0, 0.0))
    w._track_run_distance((0.5, 0.0, 0.0))
    w._track_run_distance((5.5, 0.0, 0.0))      # AMCL jumped: not driving
    drive_to(w, m.waypoints[0])
    drive_to(w, m.waypoints[1])
    check("mission complete", not w.p_mission._exec_running and w.p_mission._exec_idx == -1)
    done = run
    check("reported: complete, both waypoints", done.result == "complete" and done.reached == 2 and done.laps == 1,
          f"{done}")
    check("distance without the jump", abs(done.distance_m - 0.5) < 0.01, f"{done.distance_m}")
    w.p_schedule.refresh_reports()
    rows = [(w.p_schedule.reports.item(r, 3).text(), w.p_schedule.reports.item(r, 2).text())
            for r in range(w.p_schedule.reports.rowCount())]
    check("on the page, newest first", rows[0][0] == "skipped" and ("complete", "schedule") in rows, f"{rows}")

    print("\n[the robot not ready at the time]")
    del w._not_ready
    w.ros.is_connected = lambda: True
    w.ros.profile = w._data_profile()
    w._launcher = {"stacks": {"nav": {"state": "running"}}}
    w._battery = {"reading": {"soc": 12}, "age": 1.0}
    check("low battery", "12%" in w._start_scheduled(s))
    w._battery = None
    check("battery unknown", "not known" in w._start_scheduled(s))
    s.min_soc = 0
    w._launcher = {"stacks": {"nav": {"state": "stopped"}}}
    check("navigation not running", "navigation" in w._start_scheduled(s))
    w._launcher = {"stacks": {"nav": {"state": "running"}}}
    w._estop_on = True
    check("E-STOP", "E-STOP" in w._start_scheduled(s))
    w._estop_on = False
    m2 = Mission.new("elsewhere")
    m2.map = "other_map"
    m2.waypoints = [Waypoint.new("C", 1, 1)]
    w.missions.upsert(m2)
    w.p_mission.reload()
    w.p_mission.set_map("this_map")
    s2 = Schedule.new(m2.id, "09:00")
    s2.min_soc = 0
    check("a mission for another map", "other_map" in w._start_scheduled(s2), w.runs.runs[-1].reason)
    skipped = [r for r in w.runs.runs if r.result == "skipped"]
    check("each skip is in the reports", len(skipped) == 7, f"{len(skipped)}")
    w._on_schedule_fired(s, "the battery was at 12%, below 30%")
    check("and in the events", any("skipped" in e.text for e in w.events.events))

    print("\n[a run stopped by hand, a run that fails]")
    w.p_mission.set_map("")
    w.p_mission.open_mission(m.id)
    w.p_mission._play(0, 0)
    w.p_mission.stop_mission()
    check("stopped by the operator", w.runs.runs[-1].result == "stopped"
          and "operator" in w.runs.runs[-1].reason)
    w.p_mission._play(0, 0)
    w.p_mission.update_robot_pose(-5, -5, 0, draw=False)
    for _ in range(w.p_mission.MAX_RETRIES + 1):
        w.p_mission.on_nav_status("aborted")
    r = w.runs.runs[-1]
    check("failed, with the retries", r.result == "failed" and r.retries == w.p_mission.MAX_RETRIES
          and "could not reach" in r.reason, f"{r.result} {r.retries} {r.reason}")
    w.close()

    print("\n[per robot]")
    w2 = MainWindow()
    check("reports and schedules come back after a restart", len(w2.runs.runs) == len(w.runs.runs)
          and len(w2.schedules.schedules) == 1)
    w2.close()


def main() -> int:
    test_times()
    test_runs()
    test_app()
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    os._exit(code)
