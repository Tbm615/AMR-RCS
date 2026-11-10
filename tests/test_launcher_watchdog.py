"""robot/rcs_launcher.py when Nav2 never comes up: restarted once by itself,
then failed with the reason; the nav stack's processes start staggered and a
stop during the stagger leaves nothing behind.

Run (needs ROS 2 Humble, no robot):
    bash -c "source /opt/ros/humble/setup.bash && python3 tests/test_launcher_watchdog.py"

Own ROS domain, loopback only; the "Nav2" processes are sleeps.
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).parent
TMP = Path(tempfile.mkdtemp(prefix="rcs_watchdog_"))
os.environ.update(ROS_DOMAIN_ID="82", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                  RCS_WS=str(TMP / "ws"), RCS_NAV_UP_TIMEOUT="2", RCS_ZONES_DIR=str(TMP / "zones"))
os.environ.pop("CYCLONEDDS_URI", None)
sys.path.insert(0, str(HERE.parent / "robot"))
try:
    import rclpy
    import rcs_launcher as rl
except ImportError:
    print("needs ROS 2: source /opt/ros/humble/setup.bash first")
    sys.exit(2)

PASS = FAIL = 0
MARK = f"rcs_watchdog_{os.getpid()}"


def check(name: str, cond, extra: str = "") -> None:
    global PASS, FAIL
    PASS += bool(cond)
    FAIL += not cond
    print(f"  {'OK  ' if cond else 'FAIL'} {name} {extra}", flush=True)


def sleeper(i: int, *extra: str) -> list:
    return ["bash", "-c", f"exec -a {MARK}_{i} sleep 300", *extra]


def alive() -> list:
    out = subprocess.run(["pgrep", "-af", MARK], capture_output=True, text=True).stdout.splitlines()
    return [ln for ln in out if not ln.startswith(f"{os.getpid()} ")]


def wait(cond, sec: float) -> bool:
    end = time.time() + sec
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def main() -> int:
    rclpy.init()
    node = rl.Launcher()
    node.count_publishers = lambda topic: 0         # Nav2 never answers
    nav = rl.STACKS["nav"]
    try:
        print("\n[staggered start]")
        t0 = time.time()
        nav.start([sleeper(1), sleeper(2), sleeper(3)], {"mode": "map", "map": "x"}, stagger=0.4)
        took = time.time() - t0
        check("three processes, 0.4 s apart", len(nav.procs) == 3 and 0.7 < took < 1.6, f"{took:.2f} s")
        check("starting", nav.state == "starting" and nav.retries == 0)
        first = [p.pid for p in nav.procs]

        print("\n[never comes up: restarted, twice]")
        for k in (1, 2):
            time.sleep(2.2)
            node._update_ready()
            check(f"restarted by itself ({k})", wait(lambda: nav.retries == k and nav.state == "starting"
                                                     and len(nav.procs) == 3 and node.busy == "", 25),
                  f"{nav.state} retries {nav.retries} busy {node.busy!r}")
            check(f"it says so ({k})", f"restarted ({k})" in nav.detail, nav.detail)
            if k == 1:
                check("the first processes are gone", not set(first) & {p.pid for p in nav.procs}
                      and all(subprocess.run(["kill", "-0", str(pid)], capture_output=True).returncode
                              for pid in first))
        print("\n[still not up: failed, with the reason]")
        time.sleep(2.2)
        node._update_ready()
        check("failed", wait(lambda: nav.state == "failed" and node.busy == "", 25), nav.state)
        check("and why", "did not come up 3 times" in nav.detail and "not up in 2 s" in nav.detail, nav.detail)
        check("nothing left running", wait(lambda: not alive(), 10) and not nav.procs, str(alive()))

        print("\n[a lifecycle manager gives up: restarted at once, not after the timeout]")
        rl.NAV_UP_TIMEOUT_S = 60.0
        quit_cmd = ["bash", "-c", "echo '[lifecycle_manager_navigation]: Failed to bring up all requested nodes. "
                    f"Aborting bringup.'; exec -a {MARK}_q sleep 300"]
        nav.start([quit_cmd], {"mode": "map", "map": "x"})
        check("seen in its log", wait(lambda: nav.aborted, 5))
        node._update_ready()
        check("restarted at once", wait(lambda: nav.retries == 1 and nav.state == "starting" and node.busy == "", 25),
              f"{nav.state} {nav.retries} after {time.time() - nav.since:.1f} s")
        nav.stop()
        print("\n[a stopped start's dying words do not mark the next one]")
        late = ["bash", "-c", "trap 'echo \"Failed to bring up all requested nodes. Aborting bringup.\"; exit' INT; "
                f"exec -a {MARK}_late sleep 300 & wait"]
        nav.start([late], {"mode": "map", "map": "x"})
        time.sleep(0.5)
        nav.stop()                                  # it prints the line while it stops
        nav.start([sleeper(17)], {"mode": "map", "map": "x"})
        time.sleep(1.5)
        check("the new start is not taken for aborted", not nav.aborted)
        nav.stop()
        rl.NAV_UP_TIMEOUT_S = 2.0

        print("\n[its topics exist, but its lifecycle managers never say active (a hung activation)]")
        node.count_publishers = lambda topic: 1
        nav.start([sleeper(11), sleeper(12, "localization_launch.py")], {"mode": "map", "map": "x"})
        time.sleep(1.0)
        node._update_ready()
        check("not taken for running", nav.state == "starting", nav.state)
        time.sleep(1.5)
        node._update_ready()
        check("restarted when the time is up", wait(lambda: nav.retries == 1 and nav.state == "starting"
                                                    and node.busy == "", 25), f"{nav.state} {nav.retries}")
        nav.stop()

        print("\n[its lifecycle managers say active: running]")
        from rclpy.executors import MultiThreadedExecutor
        from std_srvs.srv import Trigger
        fake = rclpy.create_node("fake_managers")
        answers = {"lifecycle_manager_navigation": True, "lifecycle_manager_localization": False}

        def server(name):
            def cb(_req, res):
                res.success = answers[name]
                return res
            return fake.create_service(Trigger, f"/{name}/is_active", cb)
        services = [server(m) for m in answers]
        ex = MultiThreadedExecutor()
        ex.add_node(node)
        ex.add_node(fake)
        import threading as th
        th.Thread(target=ex.spin, daemon=True).start()
        os.environ["RCS_NAV_UP_TIMEOUT"] = "60"
        rl.NAV_UP_TIMEOUT_S = 60.0
        answers["lifecycle_manager_navigation"] = False         # waits for AMCL's pose, as on a saved map
        nav.start([sleeper(13), sleeper(14, "localization_launch.py")], {"mode": "map", "map": "x"})
        time.sleep(3.5)
        check("localization not active yet: still starting", nav.state == "starting", nav.state)
        answers["lifecycle_manager_localization"] = True
        check("localization active, navigation waiting for the pose: running (the operator sets the pose)",
              wait(lambda: nav.state == "running", 8), nav.state)
        rl.NAV_ACTIVE_AFTER_POSE_S = 3.0
        time.sleep(4.0)
        check("no pose yet: navigation may wait as long as it likes", nav.state == "running" and nav.retries == 0,
              f"{nav.state} {nav.retries}")
        node._pose_t = time.time()                  # AMCL got its pose, navigation stays inactive
        check("pose came, navigation still not active: restarted", wait(lambda: nav.retries == 1, 15),
              f"{nav.state} {nav.retries} {nav.detail}")
        check("it says why", wait(lambda: node.busy == "" and nav.state == "starting", 15), nav.detail)
        nav.stop()
        print("\n[pose came and navigation became active: stays running]")
        answers["lifecycle_manager_navigation"] = True
        node._pose_t = 0.0
        nav.start([sleeper(15), sleeper(16, "localization_launch.py")], {"mode": "map", "map": "x"})
        check("running", wait(lambda: nav.state == "running", 8), nav.state)
        node._pose_t = time.time()
        time.sleep(4.5)
        check("not restarted", nav.state == "running" and nav.retries == 0, f"{nav.state} {nav.retries}")
        nav.stop()
        rl.NAV_ACTIVE_AFTER_POSE_S = 45.0
        ex.shutdown(timeout_sec=2)
        rl.NAV_UP_TIMEOUT_S = 2.0

        print("\n[an operator's start begins with no restarts used]")
        node.count_publishers = lambda topic: 0
        nav.start([sleeper(4)], {"mode": "map", "map": "x"}, stagger=0.4)
        check("retries back to 0", nav.retries == 0 and nav.detail == "")
        nav.stop()

        print("\n[stopped while still staggering]")
        import threading
        th = threading.Thread(target=nav.start, args=([sleeper(i) for i in range(5, 10)], {"mode": "map"}, 0.5))
        th.start()
        time.sleep(0.8)
        nav.stop()
        th.join(5)
        time.sleep(0.5)
        check("the start gave up: nothing left", not alive() and not nav.procs and nav.state == "stopped",
              f"{alive()} {nav.state}")
        print("\n[Nav2 on a saved map: one launch, with the zones]")
        (TMP / "maps").mkdir(exist_ok=True)
        img = b"P5\n10 10\n255\n" + b"\xfe" * 100
        (TMP / "maps" / "m.pgm").write_bytes(img)
        (TMP / "maps" / "m.yaml").write_text("image: m.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
        cfg = Path(os.environ["RCS_WS"]) / "install" / "navigation" / "share" / "navigation" / "config"
        cfg.mkdir(parents=True, exist_ok=True)
        (cfg / "nav2_smac_mppi.yml").write_text("global_costmap:\n  global_costmap:\n    ros__parameters: {}\n")
        cmds = rl._nav_commands("map", str(TMP / "maps" / "m.yaml"))
        flat = " ".join(cmds[0])
        check("one launch: nav_map.launch.py, with the masks and the CBF parameters",
              len(cmds) == 1 and cmds[0][2].endswith("nav_map.launch.py") and "keepout_mask:=" in flat
              and "cbf_params_file:=" in flat and "robot_navigation" not in flat, flat[:200])
        nav.cmds = cmds
        check("its lifecycle managers: localization, navigation, zones",
              set(node._managers(nav)) == {"lifecycle_manager_localization", "lifecycle_manager_navigation",
                                            "lifecycle_manager_zones"}, str(node._managers(nav)))
    finally:
        nav.stop()
        node.destroy_node()
        rclpy.try_shutdown()
        for line in alive():
            subprocess.run(["kill", "-9", line.split()[0]])
    print(f"\n--- {PASS} passed, {FAIL} failed ---", flush=True)
    return 1 if FAIL else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
