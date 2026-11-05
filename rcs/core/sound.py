"""Short alert sounds, with no audio library: the tones are written as WAV
files and played with what the system already has (winsound on Windows,
paplay or aplay on Linux). If there is no player the alert is simply silent;
the Events page still has it.
"""
from __future__ import annotations

import math
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import wave

RATE = 22050
# name -> list of (frequency in Hz, seconds)
TONES = {
    "alarm": [(880, 0.16), (620, 0.16), (880, 0.16), (620, 0.16)],
    "warn": [(660, 0.14), (660, 0.14)],
}
MIN_GAP_S = 1.0     # a burst of events makes one sound, not a siren


def _wav_path(name: str) -> str:
    path = os.path.join(tempfile.gettempdir(), f"rcs_{name}.wav")
    if not os.path.exists(path):
        frames = bytearray()
        for f, dur in TONES[name]:
            n = int(RATE * dur)
            for i in range(n):
                fade = min(1.0, i / 200, (n - i) / 200)     # no click at the edges
                frames += struct.pack("<h", int(12000 * fade * math.sin(2 * math.pi * f * i / RATE)))
        with wave.open(path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(RATE)
            w.writeframes(bytes(frames))
    return path


class Sounds:
    def __init__(self) -> None:
        self.enabled = True
        self._last = 0.0
        self._player = next((p for p in ("paplay", "aplay") if shutil.which(p)), None)

    def play(self, name: str) -> bool:
        """True if a sound was started."""
        now = time.monotonic()
        if not self.enabled or now - self._last < MIN_GAP_S:
            return False
        self._last = now
        try:
            path = _wav_path(name)
            if sys.platform == "win32":
                import winsound
                winsound.PlaySound(path, winsound.SND_FILENAME | winsound.SND_ASYNC)
            elif self._player:
                subprocess.Popen([self._player, path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            else:
                return False
        except (OSError, RuntimeError):
            return False
        return True
