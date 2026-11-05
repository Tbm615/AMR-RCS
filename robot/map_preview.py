"""Small PNG of a saved map for the app's Robot programs page.

Only needs numpy and the standard library, so it runs on the robot as is and
can be tested without ROS.
"""
from __future__ import annotations

import base64
import re
import struct
import zlib
from pathlib import Path

import numpy as np

MAX_PX = 800        # longest side of the preview


def _yaml_field(text: str, key: str) -> str:
    m = re.search(rf"^\s*{key}\s*:\s*(.+?)\s*$", text, re.M)
    return m.group(1).strip("'\"") if m else ""


def read_pgm(path: Path) -> np.ndarray:
    """Binary PGM (what map_saver writes) -> (h, w) uint8."""
    data = Path(path).read_bytes()
    tokens, i = [], 0
    while len(tokens) < 4:              # P5, width, height, maxval
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":       # comment line
            i = data.index(b"\n", i) + 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b"P5":
        raise ValueError("not a binary PGM")
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError("16-bit PGM")
    return np.frombuffer(data, np.uint8, w * h, i + 1).reshape(h, w)


def shrink(img: np.ndarray, max_px: int = MAX_PX) -> np.ndarray:
    """Scale down by a whole factor, keeping the darkest pixel of each block
    so that one-cell walls stay visible."""
    k = -(-max(img.shape) // max_px)
    if k <= 1:
        return img
    h, w = img.shape
    pad = np.pad(img, ((0, -h % k), (0, -w % k)), mode="edge")
    return pad.reshape(pad.shape[0] // k, k, pad.shape[1] // k, k).min(axis=(1, 3))


def png_gray(img: np.ndarray) -> bytes:
    h, w = img.shape

    def chunk(tag: bytes, body: bytes) -> bytes:
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", zlib.crc32(tag + body))

    raw = np.concatenate([np.zeros((h, 1), np.uint8), img], axis=1).tobytes()    # filter 0 per row
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def map_preview(yaml_path: str) -> dict:
    """Preview and facts of a map, from its .yaml; raises if it cannot be read."""
    yp = Path(yaml_path)
    text = yp.read_text()
    image = _yaml_field(text, "image")
    img_path = Path(image) if Path(image).is_absolute() else yp.parent / image
    if img_path.suffix.lower() != ".pgm":
        raise ValueError(f"no preview for {img_path.suffix or 'this'} map images")
    img = read_pgm(img_path)
    res = float(_yaml_field(text, "resolution") or 0.05)
    h, w = img.shape
    return {
        "png": base64.b64encode(png_gray(np.ascontiguousarray(shrink(img)))).decode(),
        "width_m": round(w * res, 2),
        "height_m": round(h * res, 2),
        "resolution": res,
        "mtime": yp.stat().st_mtime,
    }
