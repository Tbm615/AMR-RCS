"""The robot's body for the 3D Map page. Made by tools/build_robot_model.py from
the URDF. Hinson only: another robot gets the plain box of its own profile."""
from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import numpy as np

PATH = Path(__file__).resolve().parent.parent / "assets" / "robot_model.npz"
FIT = 0.25              # how far a profile's footprint may be from the model


def load(path: Path = PATH) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """(triangles (n, 3, 3), colors (n, 4)), or None: the app works without it."""
    try:
        with np.load(path) as z:
            return z["triangles"], z["colors"]
    except (OSError, ValueError, KeyError):
        return None


def fits(triangles: np.ndarray, half_length: float, half_width: float) -> bool:
    size = np.ptp(triangles.reshape(-1, 3), axis=0)
    return (abs(2 * half_length - size[0]) <= FIT * size[0]
            and abs(2 * half_width - size[1]) <= FIT * size[1])
