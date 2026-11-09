"""Robot bodies: the 3D Map, the 2D maps and the Fleet page draw the robot with one.

A model is a list of triangles in the robot's base frame (base_link), in metres,
lifted so it stands on z = 0, with a colour and a part (body, drive wheel,
caster, sensor) per triangle. Stored as .npz (no pickles):

  triangles (n, 3, 3) float32   colors (n, 4) float32   parts (n,) uint8
  name, source                  text

Built in: assets/robot_model.npz, the Hinson AMR, made by
tools/build_robot_model.py from the robot's URDF. Imported: robots/<id>/model.npz,
made in the app from a URDF (meshes STL, OBJ or DAE; boxes, cylinders, spheres),
from one STL/OBJ, or from a .npz another operator exported. No ROS is needed,
except to expand a .xacro, which needs the `xacro` command.

Which model a robot uses is its profile's `model`: "auto" (the Hinson if the
profile's footprint is about its size, else a box), "hinson", "box", or
"imported".
"""
from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

PATH = Path(__file__).resolve().parent.parent / "assets" / "robot_model.npz"
MODEL_FILE = "model.npz"                # in a robot's own folder
FIT = 0.25              # how far a profile's footprint may be from the model ("auto")
KINDS = ("auto", "hinson", "box", "imported")

BODY, WHEEL, CASTER, SENSOR = 0, 1, 2, 3
PART_NAMES = {BODY: "body", WHEEL: "drive wheels", CASTER: "casters", SENSOR: "sensors"}

MAX_TRIANGLES = 120_000         # kept for drawing; more is simplified
MAX_LOAD_TRIANGLES = 2_000_000  # a stored model larger than this is refused
MAX_FILE = 256 * 1024 * 1024    # one mesh or model file
MAX_SIZE_M = 50.0               # a robot is not larger than this
DEFAULT_COLOR = (0.62, 0.66, 0.74, 1.0)


class ModelError(ValueError):
    """Why a file cannot be made into a model; the text is shown as is."""


# ---------------------------------------------------------------- the model

@dataclass
class Model:
    triangles: np.ndarray               # (n, 3, 3) float32
    colors: np.ndarray                  # (n, 4) float32
    parts: np.ndarray                   # (n,) uint8
    name: str = ""
    source: str = ""
    _top: Dict[float, "TopView"] = field(default_factory=dict, repr=False, compare=False)

    def __len__(self) -> int:
        return len(self.triangles)

    def bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        pts = self.triangles.reshape(-1, 3)
        return pts.min(0), pts.max(0)

    def size(self) -> np.ndarray:
        lo, hi = self.bounds()
        return hi - lo

    def footprint(self) -> Tuple[float, float]:
        """Half length and half width of a rectangle centred on base_link that
        holds the whole robot seen from above."""
        lo, hi = self.bounds()
        return float(max(-lo[0], hi[0])), float(max(-lo[1], hi[1]))

    def height(self) -> float:
        return float(self.bounds()[1][2])

    def count(self, part: int) -> int:
        return int((self.parts == part).sum())

    def describe(self) -> str:
        sx, sy, sz = self.size()
        bits = [f"{sx:.2f} × {sy:.2f} × {sz:.2f} m", f"{len(self):,} triangles"]
        wheels = self.count(WHEEL)
        if wheels:
            bits.append("drive wheels found")
        return ", ".join(bits)

    def top_view(self, cell: float = 0.005) -> "TopView":
        tv = self._top.get(cell)
        if tv is None:
            tv = self._top[cell] = top_view(self, cell)
        return tv


def fits(model, half_length: float, half_width: float) -> bool:
    """Whether a profile's footprint is about the size of this model ("auto")."""
    tris = model.triangles if isinstance(model, Model) else model
    size = np.ptp(np.asarray(tris).reshape(-1, 3), axis=0)
    return (abs(2 * half_length - size[0]) <= FIT * size[0]
            and abs(2 * half_width - size[1]) <= FIT * size[1])


# ---------------------------------------------------------------- files

def _text(z, key: str) -> str:
    if key not in z.files:
        return ""
    a = z[key]
    if a.dtype.kind not in "US" or a.size != 1:
        return ""
    return str(a.reshape(-1)[0])[:200]


def checked(tris, colors, parts=None, name: str = "", source: str = "") -> Model:
    """A Model from arrays that may come from anyone: shapes, numbers and size
    are checked, colours and parts are clamped."""
    t = np.asarray(tris)
    if t.ndim != 3 or t.shape[1:] != (3, 3) or t.dtype.kind != "f":
        raise ModelError("the triangles are not (n, 3, 3) numbers")
    if not 0 < len(t) <= MAX_LOAD_TRIANGLES:
        raise ModelError(f"{len(t):,} triangles: a model has 1 to {MAX_LOAD_TRIANGLES:,}")
    t = t.astype(np.float32)
    if not np.isfinite(t).all():
        raise ModelError("some points are not numbers")
    if np.abs(t).max() > MAX_SIZE_M:
        raise ModelError(f"larger than {MAX_SIZE_M:.0f} m: not in metres?")
    c = np.asarray(colors)
    if c.shape != (len(t), 4) or c.dtype.kind not in "fiu":
        raise ModelError("the colours do not match the triangles")
    c = np.nan_to_num(c.astype(np.float32), nan=0.6)
    c = np.clip(c, 0.0, 1.0)
    if parts is None:
        p = np.zeros(len(t), np.uint8)
    else:
        p = np.asarray(parts)
        if p.shape != (len(t),) or p.dtype.kind not in "iu":
            raise ModelError("the parts do not match the triangles")
        p = np.clip(p, 0, SENSOR).astype(np.uint8)
    return Model(t, c, p, str(name)[:200], str(source)[:200])


def load(path: Path = PATH) -> Optional[Model]:
    """The model in this file, or None: the app works without one."""
    try:
        with np.load(path, allow_pickle=False) as z:
            return checked(z["triangles"], z["colors"], z["parts"] if "parts" in z.files else None,
                           _text(z, "name"), _text(z, "source"))
    except (OSError, ValueError, KeyError, EOFError):
        return None


def read_file(path) -> Model:
    """Like load() but says why a file is no good (import, packs)."""
    path = Path(path)
    try:
        if path.stat().st_size > MAX_FILE:
            raise ModelError("the file is too large")
        with np.load(path, allow_pickle=False) as z:
            if "triangles" not in z.files or "colors" not in z.files:
                raise ModelError("this is not an RCS robot model")
            return checked(z["triangles"], z["colors"], z["parts"] if "parts" in z.files else None,
                           _text(z, "name"), _text(z, "source"))
    except ModelError:
        raise
    except (OSError, ValueError, KeyError, EOFError) as e:
        raise ModelError(f"cannot read it: {e}") from e


def from_bytes(data: bytes) -> Model:
    """A model from the bytes of a .npz (a member of a pack)."""
    import io
    if len(data) > MAX_FILE:
        raise ModelError("the model is too large")
    try:
        with np.load(io.BytesIO(data), allow_pickle=False) as z:
            if "triangles" not in z.files or "colors" not in z.files:
                raise ModelError("this is not an RCS robot model")
            return checked(z["triangles"], z["colors"], z["parts"] if "parts" in z.files else None,
                           _text(z, "name"), _text(z, "source"))
    except ModelError:
        raise
    except (OSError, ValueError, KeyError, EOFError) as e:
        raise ModelError(f"cannot read it: {e}") from e


def to_bytes(model: Model) -> bytes:
    import io
    buf = io.BytesIO()
    np.savez_compressed(buf, triangles=model.triangles.astype(np.float32),
                        colors=model.colors.astype(np.float32), parts=model.parts.astype(np.uint8),
                        name=np.array(model.name), source=np.array(model.source))
    return buf.getvalue()


def save(model: Model, path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp.npz")
    np.savez_compressed(tmp, triangles=model.triangles.astype(np.float32),
                        colors=model.colors.astype(np.float32), parts=model.parts.astype(np.uint8),
                        name=np.array(model.name), source=np.array(model.source))
    os.replace(tmp, path)
    return path


_BUILTIN: Dict[str, Optional[Model]] = {}


def builtin() -> Optional[Model]:
    """The Hinson AMR that comes with the app (read once)."""
    if "hinson" not in _BUILTIN:
        _BUILTIN["hinson"] = load(PATH)
    return _BUILTIN["hinson"]


_FILES: Dict[str, Tuple[tuple, Optional[Model]]] = {}


def load_cached(path: Path) -> Optional[Model]:
    """load(), the same object again while the file is unchanged: the views only
    rebuild what they draw when the model really changed."""
    try:
        st = path.stat()
        key = (st.st_mtime_ns, st.st_size)
    except OSError:
        _FILES.pop(str(path), None)
        return None
    hit = _FILES.get(str(path))
    if hit is not None and hit[0] == key:
        return hit[1]
    m = load(path)
    _FILES[str(path)] = (key, m)
    return m


def for_profile(kind: str, folder: Optional[Path], half_length: float, half_width: float) -> Optional[Model]:
    """The model a robot is drawn with, None for a box of its footprint."""
    if kind == "box":
        return None
    if kind == "imported":
        return load_cached(folder / MODEL_FILE) if folder is not None else None
    m = builtin()
    if kind == "hinson":
        return m
    return m if m is not None and fits(m, half_length, half_width) else None


# ---------------------------------------------------------------- geometry

def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _floats(text: Optional[str], n: int, default: Sequence[float]) -> List[float]:
    if text is None:
        return list(default)
    try:
        vals = [float(v) for v in text.split()]
    except ValueError as e:
        raise ModelError(f"not numbers: {text!r}") from e
    if len(vals) == 1 and n == 3:
        vals = vals * 3
    if len(vals) != n or not all(math.isfinite(v) for v in vals):
        raise ModelError(f"expected {n} numbers: {text!r}")
    return vals


def origin(el) -> np.ndarray:
    m = np.eye(4)
    if el is not None:
        m[:3, 3] = _floats(el.get("xyz"), 3, (0, 0, 0))
        m[:3, :3] = rpy_matrix(*_floats(el.get("rpy"), 3, (0, 0, 0)))
    return m


def box(sx: float, sy: float, sz: float) -> np.ndarray:
    c = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)]) * (sx, sy, sz)
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, cc, d in faces:
        tris += [(a, b, cc), (a, cc, d)]
    return c[np.array(tris)]


def cylinder(radius: float, length: float, n: int = 32) -> np.ndarray:
    """Axis along z, centred, like URDF."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([radius * np.cos(a), radius * np.sin(a)], -1)
    nxt = np.roll(ring, -1, 0)
    h = length / 2
    top = [((0, 0, h), (*p, h), (*q, h)) for p, q in zip(ring, nxt)]
    bot = [((0, 0, -h), (*q, -h), (*p, -h)) for p, q in zip(ring, nxt)]
    side = []
    for p, q in zip(ring, nxt):
        side += [((*p, -h), (*q, -h), (*q, h)), ((*p, -h), (*q, h), (*p, h))]
    return np.array(top + bot + side, float)


def sphere(radius: float, n: int = 12) -> np.ndarray:
    th = np.linspace(0, np.pi, n + 1)
    ph = np.linspace(0, 2 * np.pi, 2 * n + 1)
    p = np.array([[[np.sin(t) * np.cos(f), np.sin(t) * np.sin(f), np.cos(t)] for f in ph] for t in th]) * radius
    tris = []
    for i in range(n):
        for j in range(2 * n):
            a, b, c, d = p[i, j], p[i + 1, j], p[i + 1, j + 1], p[i, j + 1]
            if i > 0:
                tris.append((a, b, d))
            if i < n - 1:
                tris.append((b, c, d))
    return np.array(tris, float)


def simplify(tris: np.ndarray, grid: float) -> np.ndarray:
    """Merge the corners that fall in one grid cell; drop what collapses."""
    keys = np.round(tris.reshape(-1, 3) / grid).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    idx = inv.reshape(-1, 3)
    good = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])
    keep = np.unique(np.sort(idx[good], axis=1), axis=0, return_index=True)[1]
    kept = idx[good][np.sort(keep)]
    return (uniq[kept] * grid).astype(np.float64)


# ---------------------------------------------------------------- mesh files

def _check_size(path: Path) -> bytes:
    try:
        if path.stat().st_size > MAX_FILE:
            raise ModelError(f"{path.name} is too large")
        return path.read_bytes()
    except OSError as e:
        raise ModelError(f"cannot read {path.name}: {e.strerror or e}") from e


def read_stl(path: Path) -> np.ndarray:
    """(n, 3, 3) triangles of a binary or text STL."""
    data = _check_size(path)
    if len(data) >= 84:
        n = int.from_bytes(data[80:84], "little")
        if len(data) == 84 + 50 * n and n:
            rec = np.frombuffer(data, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), n, 84)
            return rec["v"].astype(np.float64)
    v = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
    if not v or len(v) % 3:
        raise ModelError(f"{path.name} is not an STL")
    try:
        return np.array(v, dtype=np.float64).reshape(-1, 3, 3)
    except ValueError as e:
        raise ModelError(f"{path.name} is a damaged STL") from e


def read_obj(path: Path) -> np.ndarray:
    verts, tris = [], []
    for line in _check_size(path).decode("utf-8", "replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "v" and len(parts) >= 4:
            verts.append([float(x) for x in parts[1:4]])
        elif parts[0] == "f" and len(parts) >= 4:
            idx = []
            for tok in parts[1:]:
                i = int(tok.split("/")[0])
                idx.append(i - 1 if i > 0 else len(verts) + i)
            for k in range(1, len(idx) - 1):
                tris.append((idx[0], idx[k], idx[k + 1]))
    if not tris:
        raise ModelError(f"{path.name} has no faces")
    v = np.array(verts, float)
    t = np.array(tris)
    if t.min() < 0 or t.max() >= len(v):
        raise ModelError(f"{path.name} points to corners it does not have")
    return v[t]


def _dae_matrix(node) -> np.ndarray:
    ns = node.tag.split("}")[0] + "}" if node.tag.startswith("{") else ""
    m = np.eye(4)
    for child in node:
        tag = child.tag[len(ns):]
        vals = [float(x) for x in (child.text or "").split()]
        if tag == "matrix" and len(vals) == 16:
            m = m @ np.array(vals).reshape(4, 4)
        elif tag == "translate" and len(vals) == 3:
            t = np.eye(4)
            t[:3, 3] = vals
            m = m @ t
        elif tag == "scale" and len(vals) == 3:
            m = m @ np.diag(vals + [1.0])
        elif tag == "rotate" and len(vals) == 4:
            ax = np.array(vals[:3])
            ax = ax / (np.linalg.norm(ax) or 1)
            a = math.radians(vals[3])
            k = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            r = np.eye(4)
            r[:3, :3] = np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k
            m = m @ r
    return m


def read_dae(path: Path) -> np.ndarray:
    """Triangles of a COLLADA file: its geometries placed by the scene's nodes,
    in metres (asset/unit), z up."""
    try:
        root = ET.fromstring(_check_size(path))
    except ET.ParseError as e:
        raise ModelError(f"{path.name} is not a COLLADA file") from e
    ns = root.tag.split("}")[0] + "}" if root.tag.startswith("{") else ""

    def find(el, tag):
        return el.find(ns + tag)

    unit, up = 1.0, "Z_UP"
    asset = find(root, "asset")
    if asset is not None:
        u = find(asset, "unit")
        if u is not None and u.get("meter"):
            unit = float(u.get("meter"))
        a = find(asset, "up_axis")
        if a is not None and a.text:
            up = a.text.strip()

    geoms: Dict[str, np.ndarray] = {}
    for g in root.iter(ns + "geometry"):
        mesh = find(g, "mesh")
        if mesh is None:
            continue
        sources = {}
        for s in mesh.findall(ns + "source"):
            fa = find(s, "float_array")
            if fa is not None and fa.text:
                sources[s.get("id")] = np.array(fa.text.split(), float)
        verts_src = {}
        for vt in mesh.findall(ns + "vertices"):
            for inp in vt.findall(ns + "input"):
                if inp.get("semantic") == "POSITION":
                    verts_src[vt.get("id")] = inp.get("source", "").lstrip("#")
        out = []
        for prim in list(mesh):
            tag = prim.tag[len(ns):]
            if tag not in ("triangles", "polylist", "polygons"):
                continue
            inputs = prim.findall(ns + "input")
            stride = max(int(i.get("offset", 0)) for i in inputs) + 1 if inputs else 1
            pos = None
            for inp in inputs:
                if inp.get("semantic") == "VERTEX":
                    src = verts_src.get(inp.get("source", "").lstrip("#"))
                    pos = (sources.get(src), int(inp.get("offset", 0)))
            if pos is None or pos[0] is None:
                continue
            pts = pos[0].reshape(-1, 3)
            p_els = prim.findall(ns + "p")
            if tag == "polygons":
                polys = [np.array(p.text.split(), int).reshape(-1, stride)[:, pos[1]] for p in p_els if p.text]
            else:
                if not p_els or not p_els[0].text:
                    continue
                idx = np.array(p_els[0].text.split(), int).reshape(-1, stride)[:, pos[1]]
                if tag == "triangles":
                    out.append(pts[idx.reshape(-1, 3)])
                    continue
                vc = find(prim, "vcount")
                counts = np.array(vc.text.split(), int) if vc is not None and vc.text else np.full(len(idx) // 3, 3)
                polys, k = [], 0
                for c in counts:
                    polys.append(idx[k:k + c])
                    k += c
            for poly in polys:
                for j in range(1, len(poly) - 1):
                    out.append(pts[[poly[0], poly[j], poly[j + 1]]][None])
        if out:
            geoms[g.get("id")] = np.concatenate(out)

    placed = []

    def walk(node, m):
        m = m @ _dae_matrix(node)
        for ig in node.findall(ns + "instance_geometry"):
            t = geoms.get(ig.get("url", "").lstrip("#"))
            if t is not None:
                v = t.reshape(-1, 3) @ m[:3, :3].T + m[:3, 3]
                placed.append(v.reshape(-1, 3, 3))
        for child in node.findall(ns + "node"):
            walk(child, m)

    scenes = list(root.iter(ns + "visual_scene"))
    for sc in scenes:
        for node in sc.findall(ns + "node"):
            walk(node, np.eye(4))
    if not placed:                      # no scene: the geometries as they are
        placed = list(geoms.values())
    if not placed:
        raise ModelError(f"{path.name} has no triangles this app can read")
    t = np.concatenate(placed) * unit
    x, y, z = t[..., 0], t[..., 1], t[..., 2]
    if up == "Y_UP":                    # +90° about x: y up becomes z up
        t = np.stack([x, -z, y], -1)
    elif up == "X_UP":                  # -90° about y: x up becomes z up
        t = np.stack([-z, y, x], -1)
    return t


def read_mesh(path: Path) -> np.ndarray:
    ext = path.suffix.lower()
    readers = {".stl": read_stl, ".obj": read_obj, ".dae": read_dae}
    if ext not in readers:
        raise ModelError(f"{path.name}: {ext or 'no extension'} meshes are not supported (STL, OBJ, DAE)")
    try:
        t = readers[ext](path)
    except ModelError:
        raise
    except (ValueError, IndexError, KeyError, TypeError) as e:
        raise ModelError(f"{path.name} is damaged ({type(e).__name__}: {e})") from e
    t = np.asarray(t, float).reshape(-1, 3, 3)
    if not np.isfinite(t).all():
        t = t[np.isfinite(t).all(axis=(1, 2))]
    if not len(t):
        raise ModelError(f"{path.name} has no triangles")
    return t


# ---------------------------------------------------------------- URDF

def part_of(link: str) -> int:
    n = link.lower()
    if "caster" in n or "castor" in n:
        return CASTER
    if "wheel" in n:
        return WHEEL
    if any(k in n for k in ("lidar", "laser", "camera", "hokuyo", "rplidar", "sensor", "sick", "velodyne",
                             "ouster", "realsense", "kinect", "astra", "imu", "sonar", "ultrasonic")):
        return SENSOR
    return BODY


def find_mesh(uri: str, urdf_dir: Path, search: Sequence[Path] = ()) -> Optional[Path]:
    """Where a mesh named in a URDF is on this PC."""
    uri = uri.strip()
    rel = None
    pkg = None
    if uri.startswith("package://"):
        pkg, _, rel = uri[len("package://"):].partition("/")
    elif uri.startswith("file://"):
        p = Path(uri[len("file://"):])
        return p if p.is_file() else None
    else:
        p = Path(uri)
        if p.is_absolute():
            return p if p.is_file() else None
        rel = uri
    roots = [urdf_dir, *search]
    up = urdf_dir
    for _ in range(4):
        up = up.parent
        roots.append(up)
    for r in roots:
        cands = [r / rel]
        if pkg:
            cands = [r / pkg / rel, r / "share" / pkg / rel, r / "src" / pkg / rel] + (
                [r / rel] if r.name == pkg else [])
        for c in cands:
            if c.is_file():
                return c
    # last resort: the same file name somewhere near the URDF
    name = Path(rel).name.lower()
    base = urdf_dir.parent
    seen = 0
    for dirpath, dirs, files in os.walk(base):
        dirs[:] = [d for d in dirs if not d.startswith(".") and d not in ("build", "log", "node_modules")]
        for f in files:
            seen += 1
            if f.lower() == name:
                return Path(dirpath) / f
        if seen > 20000:
            break
    return None


def expand_xacro(path: Path, args: Sequence[str] = ()) -> str:
    exe = shutil.which("xacro")
    if exe is None:
        raise ModelError(f"{path.name} is a xacro file. Expand it first on a PC with ROS 2: "
                         f"xacro {path.name} > robot.urdf, then import robot.urdf.")
    try:
        r = subprocess.run([exe, str(path), *args], capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired) as e:
        raise ModelError(f"xacro failed: {e}") from e
    if r.returncode != 0:
        raise ModelError(f"xacro failed: {(r.stderr or r.stdout).strip()[-400:]}")
    return r.stdout


Replacement = Tuple[np.ndarray, Tuple[float, float, float, float]]


def from_urdf(path, *, text: Optional[str] = None, base_frame: str = "base_link",
              search: Sequence[Path] = (), replace: Optional[Dict[str, Replacement]] = None,
              xacro_args: Sequence[str] = (),
              progress: Optional[Callable[[str], None]] = None) -> Tuple[Model, List[str]]:
    """The robot of a URDF, with the joints at zero, in base_frame (or the
    root link). Returns the model and what could not be used, to show."""
    path = Path(path)
    if text is None:
        raw = _check_size(path).decode("utf-8", "replace")
        if path.suffix.lower() == ".xacro" or "xmlns:xacro" in raw or "<xacro:" in raw:
            raw = expand_xacro(path, xacro_args)
        text = raw
    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        raise ModelError(f"{path.name} is not valid XML: {e}") from e
    if root.tag != "robot":
        raise ModelError(f"{path.name} is not a URDF (no <robot>)")
    warnings: List[str] = []
    replace = replace or {}
    materials = {}
    for m in root.findall("material"):
        c = m.find("color")
        if c is not None and m.get("name"):
            materials[m.get("name")] = tuple(_floats(c.get("rgba"), 4, DEFAULT_COLOR))
    links = {lk.get("name"): lk for lk in root.findall("link")}
    if not links:
        raise ModelError(f"{path.name} has no links")
    parent: Dict[str, Tuple[str, np.ndarray]] = {}
    for j in root.findall("joint"):
        c, p = j.find("child"), j.find("parent")
        if c is None or p is None:
            continue
        parent[c.get("link")] = (p.get("link"), origin(j.find("origin")))

    def pose(link: str) -> np.ndarray:
        m = np.eye(4)
        seen = set()
        while link in parent:
            if link in seen:
                raise ModelError(f"the joints of {path.name} go round in a loop")
            seen.add(link)
            link, t = parent[link]
            m = t @ m
        return m

    roots = [n for n in links if n not in parent]
    base = base_frame if base_frame in links else (roots[0] if roots else next(iter(links)))
    if base != base_frame:
        warnings.append(f"no link {base_frame}: placed from {base}")
    to_base = np.linalg.inv(pose(base))

    tris, cols, parts = [], [], []
    missing = set()
    for name, link in links.items():
        if name in replace:
            raw, color = replace[name]
            visuals = [(raw, np.eye(4), color)]
        else:
            visuals = []
            for vis in link.findall("visual"):
                geo = vis.find("geometry")
                if geo is None or len(geo) == 0:
                    continue
                shape = geo[0]
                try:
                    if shape.tag == "mesh":
                        uri = shape.get("filename", "")
                        mp = find_mesh(uri, path.parent, search)
                        if mp is None:
                            missing.add(uri)
                            continue
                        if progress:
                            progress(f"reading {mp.name}")
                        raw = read_mesh(mp) * np.array(_floats(shape.get("scale"), 3, (1, 1, 1)))
                    elif shape.tag == "box":
                        raw = box(*_floats(shape.get("size"), 3, (0.1, 0.1, 0.1)))
                    elif shape.tag == "cylinder":
                        raw = cylinder(float(shape.get("radius", 0.05)), float(shape.get("length", 0.1)))
                    elif shape.tag == "sphere":
                        raw = sphere(float(shape.get("radius", 0.05)))
                    else:
                        warnings.append(f"{name}: <{shape.tag}> is not drawn")
                        continue
                except (ModelError, ValueError, IndexError, KeyError, TypeError) as e:
                    warnings.append(f"{name}: {e}")     # a broken mesh: the rest is still drawn
                    continue
                color = DEFAULT_COLOR
                mat = vis.find("material")
                if mat is not None:
                    c = mat.find("color")
                    if c is not None:
                        color = tuple(_floats(c.get("rgba"), 4, DEFAULT_COLOR))
                    elif mat.get("name") in materials:
                        color = materials[mat.get("name")]
                visuals.append((raw, origin(vis.find("origin")), color))
        if not visuals:
            continue
        m_link = to_base @ pose(name)
        kind = part_of(name)
        for raw, local, color in visuals:
            if len(raw) == 0:
                continue
            m = m_link @ local
            v = raw.reshape(-1, 3) @ m[:3, :3].T + m[:3, 3]
            tris.append(v.reshape(-1, 3, 3))
            cols.append(np.tile(np.array(color, np.float32), (len(raw), 1)))
            parts.append(np.full(len(raw), kind, np.uint8))
    for uri in sorted(missing):
        warnings.append(f"mesh not found: {uri}")
    if not tris:
        raise ModelError(f"nothing to draw in {path.name}" + (f" ({warnings[0]})" if warnings else ""))
    t = np.concatenate(tris)
    c = np.concatenate(cols)
    p = np.concatenate(parts)
    t, c, p = _budget(t, c, p, warnings)
    t[:, :, 2] -= t[:, :, 2].min()
    return checked(t, c, p, root.get("name") or path.stem, path.name), warnings


def from_mesh(path) -> Tuple[Model, List[str]]:
    """One STL/OBJ/DAE as the whole robot: its own origin is base_link."""
    path = Path(path)
    t = read_mesh(path)
    warnings = ["one mesh: its origin is taken as base_link, x forward"]
    if np.abs(t).max() > 5.0:
        t = t * 0.001
        warnings.append("read as millimetres")
    c = np.tile(np.array(DEFAULT_COLOR, np.float32), (len(t), 1))
    p = np.zeros(len(t), np.uint8)
    t, c, p = _budget(t, c, p, warnings)
    t[:, :, 2] -= t[:, :, 2].min()
    return checked(t, c, p, path.stem, path.name), warnings


def _budget(t: np.ndarray, c: np.ndarray, p: np.ndarray, warnings: List[str]):
    """At most MAX_TRIANGLES: merge close corners, coarser until it fits."""
    if len(t) <= MAX_TRIANGLES:
        return t, c, p
    n0 = len(t)
    size = float(np.ptp(t.reshape(-1, 3), axis=0).max()) or 1.0
    grid = size / 400
    while True:
        out_t, out_c, out_p = [], [], []
        for kind in np.unique(p):
            sel = p == kind
            for col in np.unique(c[sel], axis=0):
                s2 = sel & (c == col).all(1)
                st = simplify(t[s2], grid)
                out_t.append(st)
                out_c.append(np.tile(col, (len(st), 1)))
                out_p.append(np.full(len(st), kind, np.uint8))
        nt, nc, npart = np.concatenate(out_t), np.concatenate(out_c), np.concatenate(out_p)
        if len(nt) <= MAX_TRIANGLES or grid > size / 20:
            if len(nt) > MAX_TRIANGLES:     # does not merge (scattered): keep the largest
                area = np.linalg.norm(np.cross(nt[:, 1] - nt[:, 0], nt[:, 2] - nt[:, 0]), axis=1)
                keep = np.sort(np.argsort(area)[-MAX_TRIANGLES:])
                nt, nc, npart = nt[keep], nc[keep], npart[keep]
            warnings.append(f"simplified from {n0:,} to {len(nt):,} triangles")
            return nt, nc, npart
        grid *= 1.6


def import_file(path, base_frame: str = "base_link",
                progress: Optional[Callable[[str], None]] = None) -> Tuple[Model, List[str]]:
    """A model from what an operator picked: .urdf/.xacro, .stl/.obj/.dae or .npz."""
    path = Path(path)
    ext = path.suffix.lower()
    if ext == ".npz":
        return read_file(path), []
    if ext in (".stl", ".obj", ".dae"):
        return from_mesh(path)
    if ext in (".urdf", ".xacro", ".xml"):
        return from_urdf(path, base_frame=base_frame, progress=progress)
    raise ModelError(f"{path.name}: pick a URDF, a mesh (STL, OBJ, DAE) or an RCS model (.npz)")


# ---------------------------------------------------------------- seen from above

@dataclass
class TopView:
    """The robot seen from above, per part: masks[part][row, col] is True where
    that part is, at (x0 + col * cell, y0 + row * cell) in base_link."""
    masks: Dict[int, np.ndarray]
    x0: float
    y0: float
    cell: float

    def shape(self) -> Tuple[int, int]:
        return next(iter(self.masks.values())).shape


def _raster(tris2: np.ndarray, x0: float, y0: float, cell: float, shape: Tuple[int, int]) -> np.ndarray:
    """Fill (n, 3, 2) triangles into a boolean grid (a cell is in when its
    centre is). Triangles are done in batches by size, the few large ones alone."""
    h, w = shape
    out = np.zeros(shape, bool)
    g = (tris2 - (x0, y0)) / cell - 0.5          # cell-centre coordinates
    lo = np.floor(g.min(1)).astype(int)
    hi = np.ceil(g.max(1)).astype(int)
    span = (hi - lo).max(1) + 1
    # every corner and centre, so that thin slivers still leave a mark
    for pts in (g.reshape(-1, 2), g.mean(1)):
        ij = np.round(pts).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < w) & (ij[:, 1] >= 0) & (ij[:, 1] < h)
        out[ij[ok, 1], ij[ok, 0]] = True
    a, b, c = g[:, 0], g[:, 1], g[:, 2]
    d = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    todo = (span > 1) & (np.abs(d) > 1e-12)

    def fill(sel: np.ndarray, k: int) -> None:
        if not sel.any():
            return
        idx = np.nonzero(sel)[0]
        for chunk in np.array_split(idx, max(1, len(idx) * k * k // 2_000_000 + 1)):
            off = np.arange(k)
            xs = lo[chunk, 0][:, None, None] + off[None, None, :]
            ys = lo[chunk, 1][:, None, None] + off[None, :, None]
            aa, bb, cc, dd = a[chunk], b[chunk], c[chunk], d[chunk]
            l1 = ((bb[:, 1] - cc[:, 1])[:, None, None] * (xs - cc[:, 0, None, None])
                  + (cc[:, 0] - bb[:, 0])[:, None, None] * (ys - cc[:, 1, None, None])) / dd[:, None, None]
            l2 = ((cc[:, 1] - aa[:, 1])[:, None, None] * (xs - cc[:, 0, None, None])
                  + (aa[:, 0] - cc[:, 0])[:, None, None] * (ys - cc[:, 1, None, None])) / dd[:, None, None]
            inside = (l1 >= -1e-6) & (l2 >= -1e-6) & (l1 + l2 <= 1 + 1e-6)
            inside &= (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
            out[np.broadcast_to(ys, inside.shape)[inside], np.broadcast_to(xs, inside.shape)[inside]] = True

    done = ~todo
    for k in (2, 4, 8, 16, 32):
        sel = ~done & (span <= k)
        fill(sel, k)
        done |= sel
    for t in np.nonzero(~done)[0]:              # large triangles, one at a time
        x_lo, x_hi = max(lo[t, 0], 0), min(hi[t, 0], w - 1)
        y_lo, y_hi = max(lo[t, 1], 0), min(hi[t, 1], h - 1)
        if x_lo > x_hi or y_lo > y_hi:
            continue
        xs, ys = np.meshgrid(np.arange(x_lo, x_hi + 1), np.arange(y_lo, y_hi + 1))
        (ax, ay), (bx, by), (cx, cy) = g[t]
        l1 = ((by - cy) * (xs - cx) + (cx - bx) * (ys - cy)) / d[t]
        l2 = ((cy - ay) * (xs - cx) + (ax - cx) * (ys - cy)) / d[t]
        inside = (l1 >= -1e-6) & (l2 >= -1e-6) & (l1 + l2 <= 1 + 1e-6)
        out[ys[inside], xs[inside]] = True
    return out


def _close(m: np.ndarray, r: int = 1) -> np.ndarray:
    """Fill pinholes: grow then shrink by r cells."""
    def grow(a):
        b = a.copy()
        b[1:] |= a[:-1]; b[:-1] |= a[1:]
        b[:, 1:] |= a[:, :-1]; b[:, :-1] |= a[:, 1:]
        return b
    for _ in range(r):
        m = grow(m)
    for _ in range(r):
        m = ~grow(~m)
    return m


def top_view(model: Model, cell: float = 0.005) -> TopView:
    lo, hi = model.bounds()
    x0, y0 = float(lo[0]) - 2 * cell, float(lo[1]) - 2 * cell
    w = int(math.ceil((hi[0] - x0) / cell)) + 3
    h = int(math.ceil((hi[1] - y0) / cell)) + 3
    if w * h > 4_000_000:               # a huge robot: coarser cells
        return top_view(model, cell * math.sqrt(w * h / 4_000_000) * 1.01)
    t2 = model.triangles[:, :, :2].astype(np.float64)
    masks = {}
    for part in (BODY, WHEEL, CASTER, SENSOR):
        sel = model.parts == part
        if sel.any():
            masks[part] = _close(_raster(t2[sel], x0, y0, cell, (h, w)))
    return TopView(masks, x0, y0, cell)
