"""Robot connection + topic profile. Persisted as JSON under the user config dir."""
from __future__ import annotations

import json
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
        )

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
        self.path.write_text(json.dumps(payload, indent=2))

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
