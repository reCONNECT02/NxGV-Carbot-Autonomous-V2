"""BLOCK 14 core (no ROS): decide whether motion is allowed (V4 app.js step).

Checks, in V4 order (the first failing one is the veto shown in the GUI):
  e_stop            GUI / hardware e-stop (counts as manual intervention = 0 marks)
  motion_fresh      /odom younger than 0.2 s          (V4 'Motion sensor stale')
  local_sigma       local sigma <= 3.5 cm             (V4 'Local motion uncertainty exceeds clearance budget')
  camera_fresh      road grid younger than 0.45 s     (V4 'Local camera guidance stale')
  route_identity    corridor branch hold, except the recoverable reason
                    (that one is a PROBLEM for block 12, V4 recoverableBranch). Not in parking.
  road_mask         >= 60 connected road cells, low for > 0.5 s -> veto ('No usable camera road mask')
  tunnel_clearance  in the tunnel: LiDAR fresh (0.35 s) and no return within
                    +-0.14 rad ahead closer than 0.24 m
A valid path never overrides a veto; recovery cannot bypass it. UWB is NOT an
input: a WiFi dropout must never stop the car.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class SafetyCfg:
    camera_max_age_s: float = 0.45
    motion_max_age_s: float = 0.2
    local_sigma_max_m: float = 0.035
    road_min_connected_cells: int = 60
    road_low_dwell_s: float = 0.5
    tunnel_front_half_angle_rad: float = 0.14
    tunnel_min_clearance_m: float = 0.24
    lidar_max_age_s: float = 0.35
    hold_on_branch_conflict: bool = True
    recoverable_branch_reason: str = 'Intended branch has insufficient local road evidence'
    localization_max_age_s: float = 0.5

    @classmethod
    def from_params(cls, p) -> 'SafetyCfg':
        return cls(float(p('camera_max_age_s')), float(p('motion_max_age_s')), float(p('local_sigma_max_m')),
                   int(p('road_min_connected_cells')), float(p('road_low_dwell_s')),
                   float(p('tunnel_front_half_angle_rad')), float(p('tunnel_min_clearance_m')),
                   float(p('lidar_max_age_s')), bool(p('hold_on_branch_conflict')),
                   str(p('recoverable_branch_reason')), float(p('localization_max_age_s')))


@dataclass
class SafetyInputs:
    t: float
    estop: bool = False
    odom_t: float = -1e9
    local_sigma: Optional[float] = None
    local_sigma_t: float = -1e9
    grid_t: float = -1e9
    connected: int = 0
    branch_hold: bool = False
    branch_reason: str = ''
    parking: bool = False
    in_tunnel: bool = False
    scan_t: float = -1e9
    scan_ranges: Optional[np.ndarray] = None     # forward-referenced ranges (m)
    scan_angles: Optional[np.ndarray] = None     # angles in base_link (rad, 0 = ahead)


@dataclass
class Check:
    name: str
    ok: bool
    value: float
    limit: float
    detail: str = ''


@dataclass
class SafetyResult:
    motion_allowed: bool
    veto_check: str
    veto_reason: str
    checks: List[Check] = field(default_factory=list)


class SafetyCore:

    def __init__(self, cfg: SafetyCfg):
        self.cfg = cfg
        self.low_road_since = None

    def evaluate(self, i: SafetyInputs) -> SafetyResult:
        c = self.cfg
        t = i.t
        checks: List[Check] = []
        checks.append(Check('e_stop', not i.estop, float(i.estop), 0.0,
                            'Emergency stop (manual intervention = 0 marks)' if i.estop else ''))
        age = t - i.odom_t
        checks.append(Check('motion_fresh', age <= c.motion_max_age_s, age, c.motion_max_age_s,
                            '' if age <= c.motion_max_age_s else 'Motion sensor stale'))
        sig_age = t - i.local_sigma_t
        if i.local_sigma is None or sig_age > c.localization_max_age_s:
            checks.append(Check('local_sigma', False, -1.0, c.local_sigma_max_m, 'Local estimate unavailable'))
        else:
            ok = i.local_sigma <= c.local_sigma_max_m
            checks.append(Check('local_sigma', ok, i.local_sigma, c.local_sigma_max_m,
                                '' if ok else 'Local motion uncertainty exceeds clearance budget'))
        cam = t - i.grid_t
        checks.append(Check('camera_fresh', cam <= c.camera_max_age_s, cam, c.camera_max_age_s,
                            '' if cam <= c.camera_max_age_s else 'Local camera guidance stale'))
        branch_veto = c.hold_on_branch_conflict and not i.parking and i.branch_hold and \
            i.branch_reason != c.recoverable_branch_reason
        checks.append(Check('route_identity', not branch_veto, float(i.branch_hold), 0.0,
                            i.branch_reason if branch_veto else
                            ('recoverable: handled by recovery' if i.branch_hold else '')))
        if i.connected < c.road_min_connected_cells:
            if self.low_road_since is None:
                self.low_road_since = t
            low = t - self.low_road_since
        else:
            self.low_road_since, low = None, 0.0
        road_ok = low <= c.road_low_dwell_s
        checks.append(Check('road_mask', road_ok, float(i.connected), float(c.road_min_connected_cells),
                            '' if road_ok else f'No usable camera road mask for {low:.1f} s'))
        if i.in_tunnel:
            lid = t - i.scan_t
            if lid > c.lidar_max_age_s or i.scan_ranges is None:
                checks.append(Check('tunnel_clearance', False, -1.0, c.tunnel_min_clearance_m,
                                    'Tunnel forward LiDAR clearance unavailable'))
            else:
                a = np.arctan2(np.sin(i.scan_angles), np.cos(i.scan_angles))
                r = i.scan_ranges
                front = (np.abs(a) < c.tunnel_front_half_angle_rad) & np.isfinite(r) & (r > 0)
                near = float(r[front].min()) if front.any() else float('inf')
                ok = near >= c.tunnel_min_clearance_m
                checks.append(Check('tunnel_clearance', ok, near if math.isfinite(near) else -1.0,
                                    c.tunnel_min_clearance_m,
                                    '' if ok else 'Tunnel forward LiDAR clearance below 24 cm'))
        else:
            checks.append(Check('tunnel_clearance', True, -1.0, c.tunnel_min_clearance_m, 'not in the tunnel'))
        veto = next((k for k in checks if not k.ok), None)
        return SafetyResult(veto is None, veto.name if veto else '', veto.detail if veto else '', checks)
