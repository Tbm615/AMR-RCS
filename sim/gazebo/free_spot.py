"""Where to put the simulated robot on a map without a known start: the free
place nearest the middle of the map (not (0, 0), often inside a wall).

    python3 free_spot.py <map.yaml>      prints "x y yaw"
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from rcs.core.free_spot import from_pgm  # noqa: E402


def main() -> None:
    yp = Path(sys.argv[1])
    text = yp.read_text()

    def field(key: str, default: str = "") -> str:
        m = re.search(rf"^{key}\s*:\s*(.+?)\s*(#.*)?$", text, re.M)
        return m.group(1).strip("'\"") if m else default
    img = Path(field("image"))
    img = img if img.is_absolute() else yp.parent / img
    origin = [float(v) for v in re.findall(r"-?[\d.]+(?:e-?\d+)?", field("origin", "[0, 0, 0]"))[:2]]
    spot = None
    if img.suffix.lower() == ".pgm":
        spot = from_pgm(img.read_bytes(), float(field("resolution", "0.05")), origin[0], origin[1],
                        negate=field("negate", "0") == "1")
    x, y = spot if spot else (0.0, 0.0)
    print(f"{x:.2f} {y:.2f} 0.0")


if __name__ == "__main__":
    main()
