"""Which saved map, and where on it, the robot stands: one lidar scan matched
against each map (brute force on a likelihood field). Gives the --map and
--pose for tests/test_real_robot.py.

    python3 tests/locate_robot.py [--host 192.168.10.10] [map names...]

Needs numpy, scipy and roslibpy (system Python 3 with ROS has them). Maps are
read from the workspace's src/navigation/maps (the robot's copies are the same
files); the lidar is taken as 0.4 m ahead of base_link, as in the URDF.
"""
import argparse
import math
import re
import sys
import time
from pathlib import Path

import numpy as np
import roslibpy
from scipy.ndimage import distance_transform_edt

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="192.168.10.10")
ap.add_argument("--maps-dir", default=str(Path(__file__).resolve().parents[2] / "src/navigation/maps"))
ap.add_argument("maps", nargs="*")
args = ap.parse_args()
MAPS = args.maps_dir
ros = roslibpy.Ros(host=args.host, port=9090)
ros.run(timeout=10)
scans = []
roslibpy.Topic(ros, "/scan_filtered", "sensor_msgs/LaserScan", throttle_rate=500).subscribe(scans.append)
t0 = time.time()
while not scans and time.time() - t0 < 10: time.sleep(0.1)
ros.terminate()
if not scans:
    sys.exit("no scan from /scan_filtered: is the robot base (bringup) running?")
m = scans[-1]
r = np.array([np.nan if v is None else v for v in m["ranges"]], float)
a = m["angle_min"] + np.arange(r.size) * m["angle_increment"]
ok = np.isfinite(r) & (r > 0.1) & (r < min(m["range_max"], 25))
LX = 0.4   # laser 0.4 m ahead of base_link, same yaw
pts = np.stack([LX + r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)[::4]
print(f"scan: {ok.sum()} valid of {r.size}, using {len(pts)}")
def load(name):
    y = open(f"{MAPS}/{name}.yaml").read()
    res = float(re.search(r"resolution:\s*([\d.]+)", y).group(1))
    org = [float(v) for v in re.search(r"origin:\s*\[([^\]]+)\]", y).group(1).split(",")[:2]]
    raw = open(f"{MAPS}/{name}.pgm", "rb").read()
    hdr = re.match(rb"P5\s+(?:#.*\s+)*(\d+)\s+(\d+)\s+(\d+)\s", raw)
    w, h = int(hdr.group(1)), int(hdr.group(2))
    img = np.frombuffer(raw[hdr.end():hdr.end() + w * h], np.uint8).reshape(h, w)[::-1]  # row 0 = y origin
    return img, res, org
best_all = []
for name in args.maps or sorted(p.stem for p in Path(MAPS).glob("*.yaml")):
    img, res, (ox, oy) = load(name)
    occ = img < 50; free = img > 250
    dist = distance_transform_edt(~occ) * res
    field = np.exp(-(np.minimum(dist, 1.0) ** 2) / (2 * 0.1 ** 2))
    H, W = img.shape
    fy, fx = np.nonzero(free)
    sel = np.arange(0, fx.size, max(1, fx.size // 6000))   # candidate positions
    cand = np.stack([ox + (fx[sel] + 0.5) * res, oy + (fy[sel] + 0.5) * res], 1)
    best = (-1, None)
    for th in np.radians(np.arange(0, 360, 3)):
        c, s = math.cos(th), math.sin(th)
        rp = pts @ np.array([[c, s], [-s, c]])
        sub = rp[::3]
        gx = ((cand[:, None, 0] + sub[None, :, 0] - ox) / res).astype(int)
        gy = ((cand[:, None, 1] + sub[None, :, 1] - oy) / res).astype(int)
        inb = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        sc = np.where(inb, field[gy.clip(0, H - 1), gx.clip(0, W - 1)], 0).mean(1)
        i = sc.argmax()
        if sc[i] > best[0]: best = (sc[i], (cand[i, 0], cand[i, 1], th))
    # refine
    sc0, (bx, by, bth) = best
    for step, dstep in ((0.05, 1.0), (0.02, 0.3)):
        for _ in range(3):
            for dx in (-2, -1, 0, 1, 2):
                for dy in (-2, -1, 0, 1, 2):
                    for dt in (-2, -1, 0, 1, 2):
                        x, y, th = bx + dx * step, by + dy * step, bth + math.radians(dt * dstep)
                        c, s = math.cos(th), math.sin(th)
                        rp = pts @ np.array([[c, s], [-s, c]]) + [x, y]
                        gx = ((rp[:, 0] - ox) / res).astype(int); gy = ((rp[:, 1] - oy) / res).astype(int)
                        inb = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
                        v = np.where(inb, field[gy.clip(0, H - 1), gx.clip(0, W - 1)], 0).mean()
                        if v > sc0: sc0, (bx, by, bth) = v, (x, y, th)
    c, s = math.cos(bth), math.sin(bth)
    rp = pts @ np.array([[c, s], [-s, c]]) + [bx, by]
    gx = ((rp[:, 0] - ox) / res).astype(int); gy = ((rp[:, 1] - oy) / res).astype(int)
    inb = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
    on = (np.where(inb, dist[gy.clip(0, H - 1), gx.clip(0, W - 1)], 9) < 0.1).mean()
    print(f"{name:16s} score {sc0:.3f}  within 10 cm of a wall {on:.0%}  pose x {bx:.2f} y {by:.2f} yaw {math.degrees(bth):.0f}°", flush=True)
    best_all.append((sc0, name, bx, by, bth))
sc_, name_, bx_, by_, bth_ = max(best_all)
print(f"best: --map {name_} --pose={bx_:.3f},{by_:.3f},{math.degrees(bth_):.0f}")
