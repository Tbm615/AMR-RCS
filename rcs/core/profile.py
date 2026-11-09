"""Robot connection + topic profile. Persisted as JSON under the user config dir."""
from __future__ import annotations

import json
import math
import os
import sys
import uuid
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import List, Optional


def _config_dir() -> Path:
    if sys.platform.startswith("win"):
        base = Path(os.environ.get("APPDATA", Path.home() / "AppData" / "Roaming"))
    elif sys.platform == "darwin":
        base = Path.home() / "Library" / "Application Support"
    else:
        base = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    d = base / "RCS"
    d.mkdir(parents=True, exist_ok=True)
    return d


CONFIG_DIR = _config_dir()
PROFILES_PATH = CONFIG_DIR / "robots.json"
MISSIONS_PATH = CONFIG_DIR / "missions.json"
SETTINGS_PATH = CONFIG_DIR / "settings.json"


def atomic_write(path: Path, text: str, private: bool = False) -> None:
    """Replace a file in one step: a crash leaves the old content or the new
    one, never half of it. private: readable by this user only (sign-in tokens)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    if private and os.name != "nt":
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text)
        os.chmod(tmp, 0o600)
    else:
        tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)


def load_settings() -> dict:
    """App-wide UI settings (window layout…); {} if missing or unreadable."""
    try:
        data = json.loads(SETTINGS_PATH.read_text())
        return data if isinstance(data, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def save_settings(updates: dict) -> None:
    data = load_settings()
    data.update(updates)
    try:
        atomic_write(SETTINGS_PATH, json.dumps(data, indent=2))
    except OSError:
        pass


def drop_setting(key: str) -> None:
    data = load_settings()
    if data.pop(key, None) is not None:
        try:
            atomic_write(SETTINGS_PATH, json.dumps(data, indent=2))
        except OSError:
            pass


@dataclass
class Topics:
    """Topic/frame mapping for one robot. Defaults match this workspace's bringup."""
    cmd_vel: str = "/cmd_vel_safe"
    odom: str = "/odom"
    scan: str = "/scan_filtered"
    map: str = "/map"
    global_costmap: str = "/global_costmap/costmap"
    local_costmap: str = "/local_costmap/costmap"
    plan: str = "/plan"
    camera_image: str = "/camera/color/image_raw/compressed"
    # 3D map sources: depth + color images (light, default) or a PointCloud2
    camera_depth: str = "/camera/depth/image_raw/compressedDepth"
    camera_info: str = "/camera/depth/camera_info"
    camera_cloud: str = "/camera/depth_registered/points"
    rosout: str = "/rosout"
    estop_topic: str = "/amr/estop"
    goal_pose: str = "/goal_pose"
    nav_action: str = "/navigate_to_pose"
    cancel_nav: str = "/navigate_to_pose/_action/cancel_goal"
    initial_pose: str = "/initialpose"
    base_frame: str = "base_link"
    odom_frame: str = "odom"
    map_frame: str = "map"
    laser_frame: str = "laser_frame"


@dataclass
class Limits:
    max_linear: float = 0.45          # m/s — matches amr_control_center safety
    max_angular: float = 0.8          # rad/s
    teleop_timeout: float = 0.5       # s — stale command cutoff
    footprint_half_length: float = 0.45
    footprint_half_width: float = 0.30
    laser_offset_x: float = 0.40      # m, used only if TF for laser_frame is absent
    cloud_throttle_ms: int = 500      # min interval between depth / PointCloud2 frames
    image_throttle_ms: int = 100      # min interval between camera frames


# What a profile from a file may set; the speeds are the Connection page's own
# bounds. A shared file must not be able to raise a limit past what an operator
# could type, or switch the stale-command cutoff off.
LIMIT_RANGES = {
    "max_linear": (0.05, 2.0),
    "max_angular": (0.1, 3.0),
    "teleop_timeout": (0.1, 2.0),
    "footprint_half_length": (0.05, 5.0),
    "footprint_half_width": (0.05, 5.0),
    "laser_offset_x": (-5.0, 5.0),
    "cloud_throttle_ms": (0, 10000),
    "image_throttle_ms": (0, 10000),
}


def checked_limits(d: dict) -> Limits:
    """Limits from an untrusted dict: bad values fall back, others are clamped."""
    out = Limits()
    for key, (lo, hi) in LIMIT_RANGES.items():
        v = d.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            continue
        setattr(out, key, type(getattr(out, key))(min(hi, max(lo, v))))
    return out


MODEL_KINDS = ("auto", "hinson", "box", "imported")


@dataclass
class RobotProfile:
    id: str
    name: str
    host: str = "192.168.10.10"
    port: int = 9090
    use_ssl: bool = False
    topics: Topics = field(default_factory=Topics)
    limits: Limits = field(default_factory=Limits)
    note: str = ""
    # how it is drawn: "auto" (the Hinson if the footprint is its size, else a
    # box), "hinson", "box", or "imported" (robots/<id>/model.npz)
    model: str = "auto"
    # sign-in with the robot's gateway (robot/rcs_gateway.py): who, the token it
    # gave (30 days), the role it said; tls_pin: SHA-256 of its certificate
    user: str = ""
    token: str = ""
    role: str = ""
    tls_pin: str = ""

    @property
    def ws_url(self) -> str:
        scheme = "wss" if self.use_ssl else "ws"
        return f"{scheme}://{self.host}:{self.port}"

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "RobotProfile":
        # ignore keys this version doesn't know (profile saved by a newer RCS)
        topics = Topics(**{k: v for k, v in d.get("topics", {}).items()
                           if k in Topics.__dataclass_fields__})
        limits = Limits(**{k: v for k, v in d.get("limits", {}).items()
                           if k in Limits.__dataclass_fields__})
        return cls(
            id=d.get("id") or str(uuid.uuid4()),
            name=d.get("name", "robot"),
            host=d.get("host", "192.168.10.10"),
            port=int(d.get("port", 9090)),
            use_ssl=bool(d.get("use_ssl", False)),
            topics=topics,
            limits=limits,
            note=d.get("note", ""),
            model=d.get("model") if d.get("model") in MODEL_KINDS else "auto",
            user=str(d.get("user") or "")[:40],
            token=str(d.get("token") or ""),
            role=d.get("role") if d.get("role") in ("viewer", "operator", "admin") else "",
            tls_pin=str(d.get("tls_pin") or "").lower()[:64],
        )

    def shared(self) -> dict:
        """What may leave this PC (packs): not the sign-in."""
        d = self.to_dict()
        d.update(user="", token="", role="")
        return d

    @classmethod
    def new(cls, name: str = "Hinson robot", host: str = "192.168.10.10") -> "RobotProfile":
        return cls(id=str(uuid.uuid4()), name=name, host=host)


class ProfileStore:
    def __init__(self, path: Path = PROFILES_PATH):
        self.path = path
        self.profiles: List[RobotProfile] = []
        self.active_id: Optional[str] = None
        self.load()

    def load(self) -> None:
        if not self.path.exists():
            # seed with a default profile derived from this workspace's robot
            p = RobotProfile.new()
            self.profiles = [p]
            self.active_id = p.id
            self.save()
            return
        try:
            data = json.loads(self.path.read_text())
        except (json.JSONDecodeError, OSError):
            data = {}
        self.profiles = [RobotProfile.from_dict(d) for d in data.get("profiles", [])]
        self.active_id = data.get("active_id")
        if not self.profiles:
            p = RobotProfile.new()
            self.profiles = [p]
            self.active_id = p.id
            self.save()

    def save(self) -> None:
        payload = {
            "profiles": [p.to_dict() for p in self.profiles],
            "active_id": self.active_id,
        }
        atomic_write(self.path, json.dumps(payload, indent=2), private=True)

    def active(self) -> Optional[RobotProfile]:
        for p in self.profiles:
            if p.id == self.active_id:
                return p
        return self.profiles[0] if self.profiles else None

    def set_active(self, pid: str) -> None:
        self.active_id = pid
        self.save()

    def upsert(self, profile: RobotProfile) -> None:
        for i, p in enumerate(self.profiles):
            if p.id == profile.id:
                self.profiles[i] = profile
                self.save()
                return
        self.profiles.append(profile)
        self.save()

    def remove(self, pid: str) -> None:
        self.profiles = [p for p in self.profiles if p.id != pid]
        if self.active_id == pid:
            self.active_id = self.profiles[0].id if self.profiles else None
        self.save()
