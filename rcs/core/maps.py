"""Maps kept on this PC. They are not tied to a robot: robots on one site use
the same map, and this is the way a map gets from one robot to another."""
from __future__ import annotations

import time
from pathlib import Path
from typing import List, Tuple

from .map_files import check_name, digest, pgm_size, read_map, write_map
from .profile import CONFIG_DIR

MAPS_DIR = CONFIG_DIR / "maps"


class MapLibrary:
    def __init__(self, root: Path = MAPS_DIR) -> None:
        self.root = Path(root)

    def names(self) -> List[str]:
        return sorted(p.stem for p in self.root.glob("*.yaml"))

    def read(self, name: str) -> Tuple[dict, bytes]:
        return read_map(self.root / f"{check_name(name)}.yaml")

    def info(self, name: str) -> dict:
        meta, pgm = self.read(name)
        w, h = pgm_size(pgm)
        res = meta["resolution"]
        mtime = (self.root / f"{name}.yaml").stat().st_mtime
        return {"width_m": w * res, "height_m": h * res, "resolution": res,
                "date": time.strftime("%Y-%m-%d %H:%M", time.localtime(mtime)), "pgm": pgm}

    def _same(self, name: str, wanted: str) -> bool:
        try:
            return digest(*self.read(name)) == wanted
        except (OSError, ValueError):
            return False

    def add(self, name: str, meta: dict, pgm: bytes) -> Tuple[str, bool]:
        """Store a map; returns (its name here, whether it is new). A different
        map under a name already taken gets another name: nothing is replaced."""
        check_name(name)
        wanted, have = digest(meta, pgm), set(self.names())
        cand, n = name, 2
        while cand in have:
            if self._same(cand, wanted):
                return cand, False
            tail = f"_{n}"
            cand, n = name[:40 - len(tail)] + tail, n + 1
        write_map(self.root, cand, meta, pgm)
        return cand, True

    def remove(self, name: str) -> None:
        check_name(name)
        for suffix in (".yaml", ".pgm"):
            (self.root / f"{name}{suffix}").unlink(missing_ok=True)
