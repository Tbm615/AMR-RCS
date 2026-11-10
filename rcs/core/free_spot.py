"""A free place for the robot near the middle of a map: where to draw it
before it is localized, where the simulator puts it on a map without a known
start. Numpy only (sim/gazebo/free_spot.py uses it outside the app)."""
from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


def _erode(free: np.ndarray, cells: int) -> np.ndarray:
    """Free cells at least `cells` away from anything not free (square steps)."""
    out = free.copy()
    for _ in range(cells):
        e = out.copy()
        e[1:, :] &= out[:-1, :]
        e[:-1, :] &= out[1:, :]
        e[:, 1:] &= out[:, :-1]
        e[:, :-1] &= out[:, 1:]
        out = e
        if not out.any():
            break
    return out


def nearest_free(grid: np.ndarray, res: float, ox: float, oy: float,
                 clearance: float = 0.6) -> Optional[Tuple[float, float]]:
    """(x, y) in the map frame: the free cell with the most clearance up to
    `clearance` metres, nearest the middle of the mapped area. grid is the
    occupancy grid, rows from the bottom (as /map): 0 free, 100 occupied, -1
    unknown. None when the map has no free cell."""
    g = np.asarray(grid)
    if g.ndim != 2 or g.size == 0:
        return None
    free = (g >= 0) & (g < 50)
    if not free.any():
        return None
    known = np.argwhere(g >= 0)
    (r0, c0), (r1, c1) = known.min(axis=0), known.max(axis=0)
    cr, cc = (r0 + r1) / 2.0, (c0 + c1) / 2.0
    ok = free
    for cells in range(max(1, int(round(clearance / res))), 0, -1):
        e = _erode(free, cells)
        if e.any():
            ok = e
            break
    rows, cols = np.nonzero(ok)
    i = int(np.argmin((rows - cr) ** 2 + (cols - cc) ** 2))
    return ox + (cols[i] + 0.5) * res, oy + (rows[i] + 0.5) * res


def from_pgm(pgm: bytes, res: float, ox: float, oy: float, negate: bool = False,
             clearance: float = 0.6) -> Optional[Tuple[float, float]]:
    """nearest_free() for a map image (.pgm bytes, as map_server reads it)."""
    head, i, tokens = pgm[:4096], 0, []
    while len(tokens) < 4:
        while head[i:i + 1].isspace():
            i += 1
        if head[i:i + 1] == b"#":
            i = head.index(b"\n", i) + 1
            continue
        j = i
        while not head[j:j + 1].isspace():
            j += 1
        tokens.append(head[i:j])
        i = j
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(pgm[i + 1:i + 1 + w * h], np.uint8).reshape(h, w)[::-1]
    occ = (255 - img if not negate else img) / 255.0
    grid = np.where(occ > 0.65, 100, np.where(occ < 0.25, 0, -1)).astype(np.int8)
    return nearest_free(grid, res, ox, oy, clearance)
