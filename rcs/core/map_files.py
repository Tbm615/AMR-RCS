"""Checking, reading and packing saved maps (map_server .yaml + .pgm).

The same file is in rcs/core/ (the app) and robot/ (the launcher), so both ends
accept exactly the same maps; tests/test_data.py fails if the copies differ.
Standard library only: it has to run on the robot as is.

A map from a file or from another PC is not trusted. It is never used as it
came: the numbers are checked and the .yaml is written again from them, so an
`image:` pointing somewhere else on the disk cannot get through.
"""
from __future__ import annotations

import hashlib
import io
import math
import os
import re
import zipfile
from pathlib import Path
from typing import Tuple

NAME_RE = re.compile(r"[A-Za-z0-9_-]{1,40}")
MAX_BLOB = 64 * 1024 * 1024         # one map, packed or unpacked
MODES = ("trinary", "scale", "raw")
_SPACE = b" \t\r\n"


def check_name(name: str) -> str:
    if not NAME_RE.fullmatch(name):
        raise ValueError("map name: letters, digits, - and _ only")
    return name


def _field(text: str, key: str) -> str:
    m = re.search(rf"^\s*{key}\s*:\s*(.+?)\s*(?:#.*)?$", text, re.M)
    return m.group(1).strip("'\"") if m else ""


def _number(text: str, key: str, default: float, lo: float, hi: float) -> float:
    raw = _field(text, key)
    v = float(raw) if raw else default
    if not (math.isfinite(v) and lo <= v <= hi):
        raise ValueError(f"map {key} out of range: {raw}")
    return v


def parse_yaml(text: str) -> dict:
    """The fields of a map .yaml, checked; raises ValueError if one is wrong."""
    m = re.fullmatch(r"\[(.*)\]", _field(text, "origin"))
    if not m:
        raise ValueError("map origin is missing")
    origin = [float(v) for v in m.group(1).split(",")]
    if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
        raise ValueError("map origin must be [x, y, yaw]")
    if not _field(text, "resolution"):
        raise ValueError("map resolution is missing")
    mode = _field(text, "mode") or "trinary"
    if mode not in MODES:
        raise ValueError(f"unknown map mode {mode!r}")
    return {
        "resolution": _number(text, "resolution", 0.05, 1e-4, 100.0),
        "origin": origin,
        "negate": int(_number(text, "negate", 0, 0, 1)),
        "occupied_thresh": _number(text, "occupied_thresh", 0.65, 0.0, 1.0),
        "free_thresh": _number(text, "free_thresh", 0.25, 0.0, 1.0),
        "mode": mode,
    }


def canonical_yaml(name: str, meta: dict) -> str:
    o = meta["origin"]
    return (f"image: {name}.pgm\nmode: {meta['mode']}\nresolution: {meta['resolution']}\n"
            f"origin: [{o[0]}, {o[1]}, {o[2]}]\nnegate: {meta['negate']}\n"
            f"occupied_thresh: {meta['occupied_thresh']}\nfree_thresh: {meta['free_thresh']}\n")


def pgm_size(data: bytes) -> Tuple[int, int]:
    """(width, height) of a binary 8-bit PGM; raises ValueError for anything else."""
    n, i, tokens = min(len(data), 4096), 0, []
    while len(tokens) < 4:              # P5, width, height, maxval
        while i < n and data[i] in _SPACE:
            i += 1
        if i < n and data[i:i + 1] == b"#":
            i = data.find(b"\n", i, n) + 1
            if i == 0:
                raise ValueError("not a binary PGM")
            continue
        j = i
        while j < n and data[j] not in _SPACE:
            j += 1
        if j == i or j >= n:
            raise ValueError("not a binary PGM")
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b"P5":
        raise ValueError("not a binary PGM")
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if w <= 0 or h <= 0 or not 0 < maxval <= 255:
        raise ValueError("not an 8-bit PGM")
    if len(data) < i + 1 + w * h:
        raise ValueError("the PGM is cut short")
    return w, h


def read_map(yaml_path) -> Tuple[dict, bytes]:
    """(fields, pgm bytes) of a map on this disk."""
    yp = Path(yaml_path)
    text = yp.read_text(encoding="utf-8")
    image = _field(text, "image")
    img = Path(image) if Path(image).is_absolute() else yp.parent / image
    if img.suffix.lower() != ".pgm":
        raise ValueError(f"only .pgm maps can be copied, not {img.suffix or 'this one'}")
    meta, pgm = parse_yaml(text), img.read_bytes()
    pgm_size(pgm)
    return meta, pgm


def _replace(path: Path, data: bytes) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def write_map(folder, name: str, meta: dict, pgm: bytes) -> Path:
    """Write name.pgm, then name.yaml: a map is listed by its .yaml, so a
    half-written one never shows up."""
    check_name(name)
    pgm_size(pgm)
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    _replace(folder / f"{name}.pgm", pgm)
    _replace(folder / f"{name}.yaml", canonical_yaml(name, meta).encode())
    return folder / f"{name}.yaml"


def digest(meta: dict, pgm: bytes) -> str:
    """Same value for the same map, whatever it is called."""
    return hashlib.sha256(canonical_yaml("map", meta).encode() + pgm).hexdigest()


def to_blob(name: str, meta: dict, pgm: bytes) -> bytes:
    """One map as bytes to send: fixed member names, so nothing in it is a path."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("map.yaml", canonical_yaml(name, meta))
        z.writestr("map.pgm", pgm)
    return buf.getvalue()


def from_blob(blob: bytes) -> Tuple[dict, bytes]:
    if len(blob) > MAX_BLOB:
        raise ValueError("map too large")
    try:
        with zipfile.ZipFile(io.BytesIO(blob)) as z:
            if any(z.getinfo(n).file_size > MAX_BLOB for n in ("map.yaml", "map.pgm")):
                raise ValueError("map too large")
            text, pgm = z.read("map.yaml").decode("utf-8"), z.read("map.pgm")
    except (zipfile.BadZipFile, KeyError, UnicodeDecodeError) as e:
        raise ValueError(f"not a map: {e}") from e
    meta = parse_yaml(text)
    pgm_size(pgm)
    return meta, pgm
