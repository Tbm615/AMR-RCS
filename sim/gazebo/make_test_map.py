#!/usr/bin/env python3
"""The test building: 32 x 22 m, four large rooms on a central corridor, with
doors between neighbouring rooms too, so Nav2 always has a second way round.

Writes the map Nav2 loads and the Gazebo world from one description, so they
agree. What is in NOT_ON_MAP exists only in Gazebo: the lidar sees it, the saved
map does not, and the costmap and controller have to deal with it live.

Everything is taller than the lidar plane (0.17 m above the floor), or the robot
could not see it either.

    make_test_map.py <maps dir> <world dir>
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

from make_world import world

NAME = "test_rooms"
RES = 0.05
MARGIN = 1.0                # m of unknown around the building
W, H = 32.0, 22.0
T = 0.2                     # wall thickness
CORRIDOR = (9.0, 13.0)      # y of its two wall lines
MIDDLE = 16.0               # x of the wall between west and east rooms
DOOR = 2.4
ROBOT_RADIUS = 0.55         # corner of the 0.9 x 0.6 footprint, from its centre
HEIGHT = 2.2

GREY, WOOD, BLUE, GREEN, RED = ("Gazebo/Grey", "Gazebo/Wood", "Gazebo/Blue", "Gazebo/Green", "Gazebo/Red")
ORANGE, DARK, YELLOW = "Gazebo/Orange", "Gazebo/DarkGrey", "Gazebo/Yellow"

# name: centre, facing the corridor or a neighbour through these doors
ROOMS = {"Warehouse": (8.0, 6.5), "Columns": (24.0, 7.5), "Offices": (8.0, 14.5), "Workshop": (24.0, 14.5)}
START = (3.0, 4.0)

# doors: (centre along the wall)
DOORS_SOUTH = (7.2, 24.8)       # in the wall y=9: Warehouse, Columns
DOORS_NORTH = (10.2, 22.2)      # in the wall y=13: Offices, Workshop
DOOR_SOUTH_MID = 4.7            # in the wall x=16, between Warehouse and Columns
DOOR_NORTH_MID = 17.2           # between Offices and Workshop

# (kind, x, y, sx, sy, height, material); for "round", sx is the radius
ON_MAP = [
    # Warehouse: shelves along the south wall, crates, barrels
    ("box", 3.5, 1.2, 4.0, 0.6, 1.8, ORANGE), ("box", 11.5, 1.2, 4.0, 0.6, 1.8, ORANGE),
    ("box", 7.5, 4.5, 1.0, 1.0, 0.9, WOOD), ("box", 12.5, 6.8, 1.2, 0.8, 0.8, WOOD),
    ("round", 2.5, 7.3, 0.35, 0, 1.0, YELLOW),
    # Columns: a hall of pillars and one crate
    ("round", 20.0, 2.5, 0.3, 0, HEIGHT, DARK), ("round", 28.0, 2.5, 0.3, 0, HEIGHT, DARK),
    ("round", 20.0, 6.5, 0.3, 0, HEIGHT, DARK), ("round", 28.0, 6.5, 0.3, 0, HEIGHT, DARK),
    ("box", 24.0, 2.0, 1.2, 0.8, 1.0, WOOD),
    # Offices: desks and a cabinet
    ("box", 4.0, 16.0, 1.6, 0.8, 0.75, WOOD), ("box", 4.0, 19.5, 1.6, 0.8, 0.75, WOOD),
    ("box", 9.0, 19.5, 1.6, 0.8, 0.75, WOOD), ("box", 12.5, 15.5, 1.6, 0.8, 0.75, WOOD),
    ("box", 14.8, 20.8, 1.6, 0.5, 1.9, GREY),
    # Workshop: machines, a crate, a barrel
    ("box", 20.0, 19.5, 2.0, 1.5, 1.5, GREEN), ("box", 27.5, 15.8, 1.5, 1.5, 1.5, GREEN),
    ("box", 24.0, 20.5, 1.0, 1.0, 1.0, WOOD), ("round", 29.8, 20.2, 0.35, 0, 1.0, YELLOW),
]
NOT_ON_MAP = [
    ("box", 13.0, 10.3, 0.8, 0.6, 1.0, RED),        # a cart left in the corridor
    ("box", 10.0, 7.2, 0.8, 0.8, 1.0, RED),         # a crate by the Warehouse door
    ("round", 24.0, 5.0, 0.3, 0, 1.7, RED),         # someone standing in Columns
    ("round", 18.8, 15.0, 0.3, 0, 1.7, RED),        # someone standing in Workshop
]


def walls() -> list:
    """(x0, y0, x1, y1, material) of every wall piece, doors left open."""
    h = T / 2
    out = []

    def horizontal(y, x0, x1, doors, material):
        edges = [x0]
        for d in sorted(doors):
            edges += [d - DOOR / 2, d + DOOR / 2]
        edges.append(x1)
        for a, b in zip(edges[::2], edges[1::2]):
            out.append((a, y - h, b, y + h, material))

    def vertical(x, y0, y1, doors, material):
        edges = [y0]
        for d in sorted(doors):
            edges += [d - DOOR / 2, d + DOOR / 2]
        edges.append(y1)
        for a, b in zip(edges[::2], edges[1::2]):
            out.append((x - h, a, x + h, b, material))

    horizontal(0.0, 0.0, W, [], GREY)
    horizontal(H, 0.0, W, [], BLUE)
    vertical(0.0, 0.0, H, [], GREY)
    vertical(W, 0.0, H, [], WOOD)
    horizontal(CORRIDOR[0], 0.0, W, DOORS_SOUTH, GREY)
    horizontal(CORRIDOR[1], 0.0, W, DOORS_NORTH, BLUE)
    vertical(MIDDLE, 0.0, CORRIDOR[0], [DOOR_SOUTH_MID], WOOD)
    vertical(MIDDLE, CORRIDOR[1], H, [DOOR_NORTH_MID], GREEN)
    return out


def rasterize(pieces, things) -> np.ndarray:
    """Occupied cells; row 0 = lowest y."""
    rows, cols = int((H + 2 * MARGIN) / RES), int((W + 2 * MARGIN) / RES)
    occ = np.zeros((rows, cols), bool)
    ys, xs = np.mgrid[0:rows, 0:cols]
    xs = (xs + 0.5) * RES - MARGIN
    ys = (ys + 0.5) * RES - MARGIN
    for x0, y0, x1, y1, _m in pieces:
        occ |= (xs >= x0) & (xs <= x1) & (ys >= y0) & (ys <= y1)
    for kind, x, y, sx, sy, _h, _m in things:
        if kind == "round":
            occ |= np.hypot(xs - x, ys - y) <= sx
        else:
            occ |= (np.abs(xs - x) <= sx / 2) & (np.abs(ys - y) <= sy / 2)
    return occ


def write_map(occ: np.ndarray, folder: Path) -> None:
    img = np.full(occ.shape, 205, np.uint8)         # unknown outside the building
    m = int(MARGIN / RES)
    img[m:-m, m:-m] = 254
    img[occ] = 0
    folder.mkdir(parents=True, exist_ok=True)
    Image.fromarray(img[::-1, :]).save(folder / f"{NAME}.pgm")
    (folder / f"{NAME}.yaml").write_text(
        f"image: {NAME}.pgm\nmode: trinary\nresolution: {RES}\norigin: [{-MARGIN}, {-MARGIN}, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def _cell(x: float, y: float) -> tuple:
    return int((y + MARGIN) / RES), int((x + MARGIN) / RES)


def check_reachable(occ: np.ndarray) -> None:
    """The robot's centre must reach every room with everything in place."""
    clearance = ndimage.distance_transform_edt(~occ) * RES
    labels, _n = ndimage.label(clearance > ROBOT_RADIUS)
    home = labels[_cell(*START)]
    if not home:
        raise SystemExit(f"start {START} is too close to something")
    for room, (x, y) in ROOMS.items():
        if labels[_cell(x, y)] != home:
            raise SystemExit(f"{room} cannot be reached from the start")


def geometry(kind: str, sx: float, sy: float, height: float) -> str:
    if kind == "round":
        return f"<geometry><cylinder><radius>{sx}</radius><length>{height}</length></cylinder></geometry>"
    return f"<geometry><box><size>{sx:.3f} {sy:.3f} {height:.2f}</size></box></geometry>"


def piece(i: int, kind: str, x: float, y: float, sx: float, sy: float, height: float, material: str) -> str:
    pose = f"<pose>{x:.3f} {y:.3f} {height / 2:.3f} 0 0 0</pose>"
    shape = geometry(kind, sx, sy, height)
    script = f"<script><uri>file://media/materials/scripts/gazebo.material</uri><name>{material}</name></script>"
    return (f'        <collision name="c{i}">{pose}{shape}</collision>\n'
            f'        <visual name="v{i}">{pose}{shape}<material>{script}</material></visual>')


def main() -> None:
    maps, worlds = Path(sys.argv[1]), Path(sys.argv[2])
    pieces = walls()
    occ_map = rasterize(pieces, ON_MAP)
    check_reachable(rasterize(pieces, ON_MAP + NOT_ON_MAP))
    write_map(occ_map, maps)

    parts = [("box", (a + c) / 2, (b + d) / 2, c - a, d - b, HEIGHT, m) for a, b, c, d, m in pieces] + ON_MAP
    fixed = "\n".join(piece(i, *p) for i, p in enumerate(parts))
    loose = "\n".join(piece(i, *p) for i, p in enumerate(NOT_ON_MAP))
    extra = f'    <model name="not_on_map"><static>true</static><link name="link">\n{loose}\n    </link></model>'
    (worlds / f"{NAME}.world").write_text(world(fixed, W / 2, H / 2, max(W, H) + 10.0, extra))
    print(f"{NAME}: {W:.0f} x {H:.0f} m, {len(ROOMS)} rooms, {len(ON_MAP)} objects on the map, "
          f"{len(NOT_ON_MAP)} not on it; every room reachable")


if __name__ == "__main__":
    main()
