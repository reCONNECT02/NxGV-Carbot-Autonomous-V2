"""Mission data: mission.yaml (v1 or v2) + its rules, as one object (no ROS).

version 1 (V4 reference layout, config/data/v4_reference/mission.yaml)
  Legs = V4 checkpoints; block 07 plans the road with hybrid A*. The rules
  (speed zones, gate checks, traffic light, transitions, roundabout visits)
  are inside the same file.

version 2 (tools/map/mission_planner.py, the stack default)
  Poses P0..Pn and, per leg, the computed pieces: `road` (lane driving) and
  `manoeuvre` (into / out of a bay; a PREVIEW - block 11 replans it from the
  observed bay). The file carries the sha1 of the track_map.yaml it was
  planned on. The rules come from data/mission_rules.yaml.

Pieces are what mission logic executes in order:
  road       -> ROAD (lane planner + path tracker)
  manoeuvre  -> PARKING (block 11 plans, path tracker follows)
A v1 leg that ends with parking_handoff gets an empty manoeuvre piece (block 11
has no preview then).
"""
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

Pose = Tuple[float, float, float]

RULE_KEYS = ('roundabout_visits', 'gate_route_check', 'challenge4_gate', 'traffic_light',
             'transitions', 'speed_zones', 'gate_association')
END_BEHAVIOURS = ('traffic_light_stop', 'parking_handoff', 'finish', 'none')


@dataclass
class Piece:
    leg: int                     # index into Mission.legs
    kind: str                    # road | manoeuvre
    points: np.ndarray           # N x 4: x, y, yaw, dir (+1 fwd, -1 reverse); may be empty
    bay: str = ''
    index: int = 0               # position in Mission.pieces


@dataclass
class Leg:
    index: int
    id: str
    start: Optional[Pose]
    end: Optional[Pose]
    end_behaviour: str
    parking_bay: str = ''
    enabled: bool = True
    clockwise: bool = True
    checkpoints: List[Dict] = field(default_factory=list)   # v1: {name, x, y, a} (or ref)
    declared_exits: List[str] = field(default_factory=list)  # v2: roundabout_exits in the file
    description: str = ''


@dataclass
class Mission:
    version: int
    path: str
    legs: List[Leg]
    pieces: List[Piece]
    rules: Dict
    poses: Dict[str, Pose]                 # v2: P0..Pn ; v1: {}
    map_sha1: str = ''                     # v2: sha1 of the map it was planned on
    planned: bool = False                  # v2: routes present in the file
    rules_path: str = ''

    def leg_by_id(self, leg_id: str) -> Optional[Leg]:
        return next((lg for lg in self.legs if lg.id == leg_id), None)

    def exit_direction(self, label: str) -> str:
        """Roundabout exit label -> west | north | east (v1 labels map via roundabout_visits)."""
        for v in self.rules.get('roundabout_visits', []):
            if v.get('exit') == label:
                return str(v.get('direction', label))
        return label

    def visits(self) -> List[Dict]:
        """Planned roundabout visits with leg index and exit direction, in visit order."""
        out = []
        for v in sorted(self.rules.get('roundabout_visits', []), key=lambda d: int(d['visit'])):
            lg = self.leg_by_id(str(v['leg']))
            out.append({'visit': int(v['visit']), 'leg_id': str(v['leg']),
                        'leg': lg.index if lg else -1, 'exit': str(v['exit']),
                        'direction': str(v.get('direction', v['exit']))})
        return out


def _rad(deg) -> float:
    return math.radians(float(deg))


def load_yaml(path: str) -> Dict:
    with open(os.path.expanduser(path), 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f) or {}
    if not isinstance(d, dict):
        raise ValueError(f'{path}: top level must be a mapping')
    return d


def load_mission(path: str, rules_path: Optional[str] = None) -> Mission:
    """mission.yaml (v1 or v2) + rules. v2 rules: rules_path, else the sibling
    mission_rules.yaml."""
    d = load_yaml(path)
    ver = int(d.get('version', 1))
    if ver == 1:
        return _load_v1(path, d)
    if ver == 2:
        rp = rules_path or os.path.join(os.path.dirname(os.path.abspath(path)), 'mission_rules.yaml')
        return _load_v2(path, d, load_yaml(rp), rp)
    raise ValueError(f'{path}: unknown mission.yaml version {ver}')


def _check_rules(rules: Dict, where: str) -> None:
    missing = [k for k in RULE_KEYS if k not in rules]
    if missing:
        raise ValueError(f'{where}: missing mission rule keys {missing}')


def _load_v1(path: str, d: Dict) -> Mission:
    _check_rules(d, path)
    legs, pieces = [], []
    for i, ld in enumerate(d.get('legs', [])):
        cps = []
        for c in ld.get('checkpoints') or []:
            cps.append(dict(c))
        lg = Leg(index=i, id=str(ld['id']), start=None, end=None,
                 end_behaviour=str(ld.get('end_behaviour', 'none')),
                 parking_bay=str(ld.get('parking_bay', '')), enabled=bool(ld.get('enabled', True)),
                 clockwise=bool(ld.get('clockwise', True)), checkpoints=cps,
                 description=str(ld.get('description', '')))
        legs.append(lg)
        if not lg.enabled or not cps:
            continue
        pieces.append(Piece(leg=i, kind='road', points=np.zeros((0, 4))))
        if lg.end_behaviour == 'parking_handoff':
            pieces.append(Piece(leg=i, kind='manoeuvre', points=np.zeros((0, 4)), bay=lg.parking_bay))
    for k, p in enumerate(pieces):
        p.index = k
    return Mission(version=1, path=path, legs=legs, pieces=pieces,
                   rules={k: d[k] for k in RULE_KEYS}, poses={}, planned=False)


def _load_v2(path: str, d: Dict, rules: Dict, rules_path: str) -> Mission:
    _check_rules(rules, rules_path)
    poses = {p['id']: (float(p['x']), float(p['y']), _rad(p['yaw_deg'])) for p in d.get('poses', [])}
    behaviour = rules.get('legs') or {}
    legs, pieces = [], []
    planned = True
    for i, ld in enumerate(d.get('legs', [])):
        lid = str(ld['id'])
        b = behaviour.get(lid)
        if b is None:
            raise ValueError(f'{rules_path}: legs.{lid} missing (end_behaviour of every mission.yaml leg)')
        eb = str(b['end_behaviour'])
        if eb not in END_BEHAVIOURS:
            raise ValueError(f'{rules_path}: legs.{lid}.end_behaviour {eb} not in {END_BEHAVIOURS}')
        lg = Leg(index=i, id=lid, start=poses.get(str(ld.get('start'))), end=poses.get(str(ld.get('end'))),
                 end_behaviour=eb, parking_bay=str(b.get('parking_bay', '')),
                 enabled=bool(b.get('enabled', True)), declared_exits=list(ld.get('roundabout_exits') or []),
                 checkpoints=[dict(c) for c in ld.get('checkpoints') or []])
        legs.append(lg)
        if not ld.get('ok') or not ld.get('pieces'):
            planned = False
            continue
        if not lg.enabled:
            continue
        for pc in ld['pieces']:
            pts = np.asarray(pc['points'], float).reshape(-1, 4)
            pieces.append(Piece(leg=i, kind=str(pc['kind']), points=pts, bay=str(pc.get('bay', ''))))
    for k, p in enumerate(pieces):
        p.index = k
    return Mission(version=2, path=path, legs=legs, pieces=pieces,
                   rules={k: rules[k] for k in RULE_KEYS}, poses=poses,
                   map_sha1=str((d.get('map') or {}).get('sha1', '')), planned=planned,
                   rules_path=rules_path)


def mission_from_params(p) -> Mission:
    """Mission for a CarbotNode (p = node.p): data.mission + data.mission_rules."""
    return load_mission(str(p('data.mission')), str(p('data.mission_rules')))


def named_poses(config_dir: str) -> Dict[str, Pose]:
    """start_pose, light_goal_pose and every mission checkpoint / pose with x/y/a
    (calibration step 11 'points' mode). Works for v1 and v2 data folders."""
    from .course import load_course
    data = os.path.join(config_dir, 'data')
    course = load_course(os.path.join(data, 'track_map.yaml'))
    mi = load_mission(os.path.join(data, 'mission.yaml'))
    out: Dict[str, Pose] = {}
    for k in ('start_pose', 'light_goal_pose'):
        if course.has_feature(k):
            out[k] = course.pose(k)
    for pid, p in mi.poses.items():
        out.setdefault(pid, p)
    for lg in mi.legs:
        for c in lg.checkpoints:
            if 'x' in c and 'name' in c and 'a' in c:
                out.setdefault(str(c['name']), (float(c['x']), float(c['y']), float(c['a'])))
    return out
