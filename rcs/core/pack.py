""".rcspack files: one robot's data in a file, to back it up or to give to
someone else. A zip holding some of:

  manifest.json   format, version, app, date, robot name
  profile.json    robot profile
  missions.json   places.json   layouts.json   state.json   events.csv
  maps/<name>.yaml + maps/<name>.pgm
  model.npz       the robot's imported model (robot_model.py; no pickles)

A pack may come from anyone, so it is read as data: only the members above are
looked at, nothing is ever extracted to a path taken from the zip, and every
value is checked. Importing only adds. It never replaces a mission, place, map
or robot that is already here.
"""
from __future__ import annotations

import copy
import json
import math
import os
import re
import time
import uuid
import zipfile
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

from .. import __version__
from .map_files import MAX_BLOB, canonical_yaml, parse_yaml, pgm_size
from .maps import MapLibrary
from .mission import Mission, MissionStore
from .places import Place, PlaceStore
from . import robot_model
from .profile import ProfileStore, RobotProfile, checked_limits
from .robot_data import RobotState, robot_dir

FORMAT = "rcs-pack"
VERSION = 1
SUFFIX = ".rcspack"
MAX_TOTAL = 512 * 1024 * 1024   # unpacked, whole file
_MAP_MEMBER = re.compile(r"maps/([A-Za-z0-9_-]{1,40})\.yaml")
_BAD = (TypeError, ValueError, AttributeError, KeyError)


def export_pack(path, *, robot_name: str, profile: Optional[RobotProfile] = None,
                missions: Optional[List[Mission]] = None, places: Optional[List[Place]] = None,
                library: Optional[MapLibrary] = None, maps: Tuple[str, ...] = (),
                layouts: Optional[dict] = None, state: Optional[dict] = None,
                events_path: Optional[Path] = None, model: Optional["robot_model.Model"] = None) -> Path:
    path = Path(path)
    if path.suffix.lower() != SUFFIX:
        path = path.with_name(path.name + SUFFIX)
    tmp = path.with_name(path.name + ".tmp")
    manifest = {"format": FORMAT, "version": VERSION, "app": __version__,
                "created": time.strftime("%Y-%m-%d %H:%M"), "robot": robot_name}
    try:
        with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
            z.writestr("manifest.json", json.dumps(manifest, indent=2))
            if profile is not None:
                z.writestr("profile.json", json.dumps(profile.shared(), indent=2))
            if missions is not None:
                z.writestr("missions.json", json.dumps({"missions": [m.to_dict() for m in missions]}, indent=2))
            if places is not None:
                z.writestr("places.json", json.dumps({"places": [asdict(p) for p in places]}, indent=2))
            if layouts:
                z.writestr("layouts.json", json.dumps(layouts, indent=2))
            if state is not None:
                z.writestr("state.json", json.dumps(state, indent=2))
            if events_path is not None and Path(events_path).exists():
                z.write(events_path, "events.csv")
            if model is not None:
                z.writestr("model.npz", robot_model.to_bytes(model))
            for name in maps:
                meta, pgm = library.read(name)
                z.writestr(f"maps/{name}.yaml", canonical_yaml(name, meta))
                z.writestr(f"maps/{name}.pgm", pgm)
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)
    return path


def _clean_mission(d: dict) -> Mission:
    m = Mission.from_dict(d)
    m.id, m.name = str(m.id), str(m.name)[:100]
    m.laps = max(0, m.laps)
    for w in m.waypoints:
        w.id, w.name = str(w.id), str(w.name)[:100]
        w.x, w.y, w.yaw = float(w.x), float(w.y), float(w.yaw)
        w.tolerance, w.dwell = float(w.tolerance), float(w.dwell)
        if not all(map(math.isfinite, (w.x, w.y, w.yaw, w.tolerance, w.dwell))):
            raise ValueError("a waypoint is not a number")
        w.tolerance = min(5.0, max(0.05, w.tolerance))
        w.dwell = min(3600.0, max(0.0, w.dwell))
    return m


def _clean_place(d: dict) -> Place:
    p = Place(**{k: d[k] for k in Place.__dataclass_fields__ if k in d})
    p.id, p.name, p.map = str(p.id), str(p.name)[:100], str(p.map)
    p.x, p.y, p.yaw = float(p.x), float(p.y), float(p.yaw)
    if not all(map(math.isfinite, (p.x, p.y, p.yaw))):
        raise ValueError("a place is not a number")
    return p


def _clean_profile(d: dict) -> RobotProfile:
    topics = {k: v for k, v in d.get("topics", {}).items() if isinstance(v, str)}
    p = RobotProfile.from_dict({**d, "id": "", "topics": topics, "limits": {}})
    p.id = str(uuid.uuid4())
    p.name, p.host, p.note = str(p.name)[:100], str(p.host), str(p.note)
    p.port = min(65535, max(1, p.port))
    p.limits = checked_limits(d.get("limits", {}))
    p.user, p.token, p.role = "", "", ""         # a sign-in never comes from a file
    return p


def _clean_state(d: dict, renamed: Dict[str, str]) -> dict:
    out: dict = {}
    if isinstance(d.get("layout"), dict):
        out["layout"] = d["layout"]
    if isinstance(d.get("mission"), str):
        out["mission"] = d["mission"]
    nav = d.get("nav")
    if isinstance(nav, dict) and nav.get("mode") in ("map", "slam"):
        m = str(nav.get("map", ""))
        out["nav"] = {"mode": nav["mode"], "map": renamed.get(m, m)}
    poses = {}
    src = d.get("last_pose")
    for name, pose in (src.items() if isinstance(src, dict) else ()):
        if (isinstance(pose, list) and len(pose) == 3
                and all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose)):
            poses[renamed.get(name, name)] = pose
    if poses:
        out["last_pose"] = poses
    return out                  # not "run": that mission run was on the other PC


@dataclass
class Pack:
    manifest: dict
    profile: Optional[RobotProfile] = None
    missions: List[Mission] = field(default_factory=list)
    places: List[Place] = field(default_factory=list)
    maps: Dict[str, Tuple[dict, bytes]] = field(default_factory=dict)
    layouts: dict = field(default_factory=dict)
    state: Optional[dict] = None
    events: Optional[bytes] = None
    model: Optional["robot_model.Model"] = None


def read_pack(path) -> Pack:
    """Read and check a pack; raises ValueError with a reason to show."""
    try:
        with zipfile.ZipFile(path) as z:
            return _read(z)
    except zipfile.BadZipFile as e:
        raise ValueError("this is not an RCS pack") from e


def _read(z: zipfile.ZipFile) -> Pack:
    infos = {i.filename: i for i in z.infolist()}
    if sum(i.file_size for i in infos.values()) > MAX_TOTAL:
        raise ValueError("this pack is too large")

    def member(name: str, limit: int = MAX_BLOB) -> Optional[bytes]:
        if name not in infos:
            return None
        if infos[name].file_size > limit:
            raise ValueError(f"{name} is too large")
        return z.read(name)

    def doc(name: str):
        raw = member(name)
        if raw is None:
            return None
        try:
            return json.loads(raw.decode("utf-8"))
        except ValueError as e:
            raise ValueError(f"{name} is damaged") from e

    manifest = doc("manifest.json")
    if not isinstance(manifest, dict) or manifest.get("format") != FORMAT:
        raise ValueError("this is not an RCS pack")
    if not isinstance(manifest.get("version"), int) or manifest["version"] > VERSION:
        raise ValueError(f"this pack was made by a newer RCS ({manifest.get('app', '?')}): update the app")
    pack = Pack(manifest)
    try:
        d = doc("profile.json")
        if d is not None:
            pack.profile = _clean_profile(d)
    except _BAD as e:
        raise ValueError(f"the robot profile is damaged: {e}") from e
    try:
        pack.missions = [_clean_mission(m) for m in (doc("missions.json") or {}).get("missions", [])]
    except _BAD as e:
        raise ValueError(f"a mission is damaged: {e}") from e
    try:
        pack.places = [_clean_place(p) for p in (doc("places.json") or {}).get("places", [])]
    except _BAD as e:
        raise ValueError(f"a place is damaged: {e}") from e
    layouts = doc("layouts.json")
    if isinstance(layouts, dict):
        pack.layouts = {str(k)[:60]: v for k, v in layouts.items() if isinstance(v, dict)}
    state = doc("state.json")
    if isinstance(state, dict):
        pack.state = state
    pack.events = member("events.csv")
    raw = member("model.npz", robot_model.MAX_FILE)
    if raw is not None:
        try:
            pack.model = robot_model.from_bytes(raw)
        except ValueError as e:
            raise ValueError(f"the robot model is damaged: {e}") from e
    for filename in infos:
        m = _MAP_MEMBER.fullmatch(filename)
        if not m:
            continue
        name = m.group(1)
        try:
            pgm = member(f"maps/{name}.pgm")
            if pgm is None:
                raise ValueError("its image is missing")
            meta = parse_yaml(member(filename).decode("utf-8"))
            pgm_size(pgm)
        except (ValueError, UnicodeDecodeError) as e:
            raise ValueError(f"map {name} is damaged: {e}") from e
        pack.maps[name] = (meta, pgm)
    return pack


@dataclass
class ImportResult:
    maps_added: List[str] = field(default_factory=list)
    maps_same: int = 0
    renamed: Dict[str, str] = field(default_factory=dict)
    missions_added: int = 0
    missions_same: int = 0
    places_added: int = 0
    places_same: int = 0
    layouts_added: int = 0
    map_refs: Set[str] = field(default_factory=set)     # maps the imported items are on
    robot: Optional[RobotProfile] = None
    model: str = ""                     # "added" | "kept" (the robot has one) | ""

    def text(self) -> str:
        parts = []
        if self.robot is not None:
            parts.append(f"new robot {self.robot.name}")
        for n, same, what in ((self.missions_added, self.missions_same, "missions"),
                              (self.places_added, self.places_same, "places"),
                              (len(self.maps_added), self.maps_same, "maps")):
            if n or same:
                parts.append(f"{n} {what}" + (f" ({same} already here)" if same else ""))
        if self.layouts_added:
            parts.append(f"{self.layouts_added} layouts")
        if self.model == "added":
            parts.append("its robot model")
        elif self.model == "kept":
            parts.append("robot model not taken: this robot has an imported one already")
        for old, new in self.renamed.items():
            parts.append(f"map {old} kept as {new}: another map has that name")
        return "; ".join(parts) or "nothing new"


def _unique(name: str, taken: Set[str]) -> str:
    out, n = name, 2
    while out in taken:
        out, n = f"{name} ({n})", n + 1
    return out


def _import_maps(pack: Pack, library: MapLibrary, res: ImportResult) -> None:
    for name, (meta, pgm) in pack.maps.items():
        final, new = library.add(name, meta, pgm)
        if new:
            res.maps_added.append(final)
        else:
            res.maps_same += 1
        if final != name:
            res.renamed[name] = final


def _mission_key(m: Mission) -> tuple:
    return (m.name, m.map, m.loop, m.laps,
            tuple((w.x, w.y, w.yaw, w.tolerance, w.dwell) for w in m.waypoints))


def _merge_missions(pack: Pack, store: MissionStore, res: ImportResult) -> None:
    have = {_mission_key(m) for m in store.missions}
    for src in pack.missions:
        m = copy.deepcopy(src)
        m.map = res.renamed.get(m.map, m.map)
        if m.map:
            res.map_refs.add(m.map)
        if _mission_key(m) in have:
            res.missions_same += 1
            continue
        if store.get(m.id) is not None:
            m.id = str(uuid.uuid4())[:8]
        m.name = _unique(m.name, {x.name for x in store.missions})
        store.missions.append(m)
        res.missions_added += 1
    store.save()


def _place_key(p: Place) -> tuple:
    return (p.name, p.x, p.y, p.yaw, p.map)


def _merge_places(pack: Pack, store: PlaceStore, res: ImportResult) -> None:
    have = {_place_key(p) for p in store.places}
    for src in pack.places:
        p = copy.deepcopy(src)
        p.map = res.renamed.get(p.map, p.map)
        if p.map:
            res.map_refs.add(p.map)
        if _place_key(p) in have:
            res.places_same += 1
            continue
        if store.get(p.id) is not None:
            p.id = str(uuid.uuid4())[:8]
        p.name = _unique(p.name, {x.name for x in store.places})
        store.places.append(p)
        res.places_added += 1
    store.save()


def merge_layouts(have: dict, pack: Pack, res: ImportResult) -> None:
    for name, data in pack.layouts.items():
        if name not in have:
            have[name] = data
            res.layouts_added += 1


def import_into(pack: Pack, want: Set[str], library: MapLibrary,
                missions: MissionStore, places: PlaceStore) -> ImportResult:
    """Add to a robot already here."""
    res = ImportResult()
    if "maps" in want:
        _import_maps(pack, library, res)
    if "missions" in want:
        _merge_missions(pack, missions, res)
    if "places" in want:
        _merge_places(pack, places, res)
    return res


def install_model(pack: Pack, profile: RobotProfile, profiles: ProfileStore, res: ImportResult) -> None:
    """Give a robot the pack's model, unless it has an imported one already."""
    if pack.model is None:
        return
    f = robot_dir(profile.id) / robot_model.MODEL_FILE
    if f.exists():
        res.model = "kept"
        return
    robot_model.save(pack.model, f)
    profile.model = "imported"
    if any(p.id == profile.id for p in profiles.profiles):
        profiles.save()
    res.model = "added"


def import_as_new_robot(pack: Pack, want: Set[str], library: MapLibrary,
                        profiles: ProfileStore) -> ImportResult:
    """Make a robot from the pack's profile, with its own id: what is here stays."""
    prof = copy.deepcopy(pack.profile)
    prof.name = _unique(prof.name, {p.name for p in profiles.profiles})
    d = robot_dir(prof.id)
    res = import_into(pack, want, library, MissionStore(d / "missions.json"), PlaceStore(d / "places.json"))
    if "state" in want and pack.state is not None:
        RobotState(d / "state.json").update(**_clean_state(pack.state, res.renamed))
    if "events" in want and pack.events is not None:
        (d / "events.csv").write_bytes(pack.events)
    if "model" in want and pack.model is not None:
        install_model(pack, prof, profiles, res)
    elif prof.model == "imported":
        prof.model = "auto"             # its model did not come along
    profiles.profiles.append(prof)
    profiles.save()
    res.robot = prof
    return res
