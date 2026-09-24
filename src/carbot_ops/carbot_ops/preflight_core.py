"""race_supervisor pure logic (no rclpy, unit tested): preflight checks and the state machine.

evaluate() turns one Snapshot of the latest inputs into a list of Check rows (the GUI shows each
row with value / expected / detail). Machine walks LOADING -> CAL_MISSING / NOT_READY / PREFLIGHT ->
READY -> RUNNING -> FINISHED / ESTOPPED. START is only accepted at READY, exactly once.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# PreflightReport.STATE_* (duplicated so this module needs no ROS messages)
LOADING, CAL_MISSING, PREFLIGHT, NOT_READY, READY, RUNNING, FINISHED, ESTOPPED = range(8)

NODE_ERROR, NODE_STUB = 2, 3     # NodeStatus.ERROR / NodeStatus.STUB


@dataclass(frozen=True)
class Cfg:
    camera_min_rate_ratio: float
    camera_max_age_s: float
    lidar_min_hz: float
    lidar_max_age_s: float
    uwb_anchor_max_age_s: float
    battery_min_v: float
    start_position_tolerance_m: float
    start_heading_tolerance_deg: float
    ready_hold_s: float
    status_max_age_s: float       # battery / system health / safety / node heartbeat freshness
    pose_max_age_s: float
    startup_grace_s: float
    required_nodes: Tuple[str, ...]
    warn_only_nodes: Tuple[str, ...]


@dataclass
class Check:
    name: str
    ok: bool
    value: str = ''
    expected: str = ''
    detail: str = ''


@dataclass
class Snapshot:
    now: float
    calibration_missing: Sequence[str] = ()
    unconfirmed_roles: Sequence[str] = ()
    # camera: (rate_hz, expected_hz, age_s, report age) from system_monitor, None = never seen
    camera_name: str = ''
    camera: Optional[Tuple[float, float, float]] = None
    camera_report_age_s: float = -1.0
    lidar_rate_hz: float = 0.0
    lidar_age_s: float = -1.0
    uwb_link_ok: bool = False
    uwb_report_age_s: float = -1.0
    uwb_anchor_ids: Sequence[str] = ()
    uwb_anchor_age_s: Sequence[float] = ()
    uwb_anchor_seen: Sequence[bool] = ()
    uwb_anchors_surveyed: bool = False
    uwb_offsets_calibrated: bool = False
    battery_v: Optional[float] = None
    battery_age_s: float = -1.0
    pose: Optional[Tuple[float, float, float]] = None      # x, y, yaw_rad (global pose, track frame)
    pose_age_s: float = -1.0
    start_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw_deg (mission.yaml P0)
    safety_allowed: Optional[bool] = None
    safety_veto: str = ''
    safety_age_s: float = -1.0
    node_levels: Dict[str, Tuple[int, float]] = field(default_factory=dict)   # name -> (level, age_s)
    takeover: bool = False
    estop: bool = False


def _ok(name, value='', expected='', detail=''):
    return Check(name, True, str(value), str(expected), detail)


def _bad(name, value, expected, detail):
    return Check(name, False, str(value), str(expected), detail)


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def evaluate(cfg: Cfg, s: Snapshot) -> List[Check]:
    out: List[Check] = []

    if s.estop:
        out.append(_bad('e_stop', 'pressed', 'released', 'Release the e-stop in the GUI'))
    else:
        out.append(_ok('e_stop', 'released'))
    if s.takeover:
        out.append(_bad('manual_takeover', 'on', 'off', 'Manual control is on: press Hand back in the GUI'))
    else:
        out.append(_ok('manual_takeover', 'off'))

    if s.unconfirmed_roles:
        out.append(_bad('camera_roles', ','.join(s.unconfirmed_roles), 'confirmed (step 2)',
                        'Calibration step 2 has not confirmed this camera'))
    else:
        out.append(_ok('camera_roles', 'confirmed'))

    # ---- required nodes alive, not ERROR, not still a stub
    for n in cfg.required_nodes:
        lv = s.node_levels.get(n)
        if lv is None or lv[1] > cfg.status_max_age_s:
            out.append(_bad('node_' + n, 'no heartbeat', 'running', f'{n} is not publishing /carbot/status'))
        elif lv[0] == NODE_ERROR:
            out.append(_bad('node_' + n, 'ERROR', 'OK/WARN', f'{n} reports an ERROR: see System health'))
        elif lv[0] == NODE_STUB:
            out.append(_bad('node_' + n, 'STUB', 'implemented', f'{n} is still a stub in this build'))
        else:
            out.append(_ok('node_' + n, 'ok'))
    for n in cfg.warn_only_nodes:
        lv = s.node_levels.get(n)
        if lv is not None and lv[1] <= cfg.status_max_age_s and lv[0] == NODE_ERROR:
            out.append(_ok('node_' + n, 'ERROR', 'OK', 'warning only: detector holds are disabled'))

    # ---- sensors
    if s.camera is None or s.camera_report_age_s < 0 or s.camera_report_age_s > cfg.status_max_age_s:
        out.append(_bad('camera_front', 'no data', f'>= {cfg.camera_min_rate_ratio:.0%} of expected',
                        f'system_monitor has no fresh report for {s.camera_name or "the camera"}'))
    else:
        rate, exp, age = s.camera
        need = cfg.camera_min_rate_ratio * exp
        good = rate >= need and 0 <= age <= cfg.camera_max_age_s
        out.append(Check('camera_front', good, f'{rate:.1f} Hz, age {age:.2f} s',
                         f'>= {need:.1f} Hz, age <= {cfg.camera_max_age_s} s',
                         '' if good else 'Front camera too slow or stale: Restart camera drivers'))
    lid_ok = s.lidar_rate_hz >= cfg.lidar_min_hz and 0 <= s.lidar_age_s <= cfg.lidar_max_age_s
    out.append(Check('lidar', lid_ok, f'{s.lidar_rate_hz:.1f} Hz, age {s.lidar_age_s:.2f} s',
                     f'>= {cfg.lidar_min_hz} Hz, age <= {cfg.lidar_max_age_s} s',
                     '' if lid_ok else 'LiDAR scan missing or too slow'))

    # ---- UWB
    if s.uwb_report_age_s < 0 or s.uwb_report_age_s > cfg.status_max_age_s:
        out.append(_bad('uwb_link', 'no status', 'link ok', 'uwb_ranges is not reporting'))
    elif not s.uwb_link_ok:
        out.append(_bad('uwb_link', 'down', 'link ok', 'The tag is not sending: check tag power and WiFi'))
    else:
        out.append(_ok('uwb_link', 'ok'))
    for i, aid in enumerate(s.uwb_anchor_ids):
        seen = i < len(s.uwb_anchor_seen) and s.uwb_anchor_seen[i]
        age = s.uwb_anchor_age_s[i] if i < len(s.uwb_anchor_age_s) else -1.0
        good = bool(seen) and 0 <= age <= cfg.uwb_anchor_max_age_s
        out.append(Check('uwb_anchor_' + str(aid), good, f'age {age:.2f} s',
                         f'seen, age <= {cfg.uwb_anchor_max_age_s} s',
                         '' if good else f'Anchor {aid} not heard: check anchor power'))
    if not s.uwb_anchor_ids:
        out.append(_bad('uwb_anchors', 'none', '>= 1', 'No anchor list from uwb_ranges'))
    ready_uwb = s.uwb_anchors_surveyed and s.uwb_offsets_calibrated
    out.append(Check('uwb_calibrated', ready_uwb,
                     f'surveyed {s.uwb_anchors_surveyed}, offsets {s.uwb_offsets_calibrated}',
                     'both true (step 10)', '' if ready_uwb else 'Redo calibration step 10'))

    # ---- battery
    if s.battery_v is None or s.battery_age_s < 0 or s.battery_age_s > cfg.status_max_age_s * 3:
        out.append(_bad('battery', 'no data', f'>= {cfg.battery_min_v} V', 'No battery voltage from the motor board'))
    else:
        good = s.battery_v >= cfg.battery_min_v
        out.append(Check('battery', good, f'{s.battery_v:.2f} V', f'>= {cfg.battery_min_v} V',
                         '' if good else 'Battery too low: charge or swap the pack'))

    # ---- start pose = map start (mission.yaml P0)
    if s.pose is None or s.pose_age_s < 0 or s.pose_age_s > cfg.pose_max_age_s or s.start_pose is None:
        out.append(_bad('start_pose', 'no pose', 'at P0', 'No fresh global pose yet'))
    else:
        px, py, pyaw = s.pose
        sx, sy, syaw_deg = s.start_pose
        dpos = math.hypot(px - sx, py - sy)
        dyaw = abs(wrap_deg(math.degrees(pyaw) - syaw_deg))
        good = dpos <= cfg.start_position_tolerance_m and dyaw <= cfg.start_heading_tolerance_deg
        out.append(Check('start_pose', good,
                         f'{dpos * 100:.0f} cm, {dyaw:.0f} deg off P0',
                         f'<= {cfg.start_position_tolerance_m * 100:.0f} cm, '
                         f'<= {cfg.start_heading_tolerance_deg:.0f} deg',
                         '' if good else 'Place the car on the start mark (P0) facing the start heading'))

    # ---- safety veto (would stop the car the moment it is armed)
    if s.safety_allowed is None or s.safety_age_s < 0 or s.safety_age_s > cfg.status_max_age_s:
        out.append(_bad('safety_monitor', 'no data', 'motion allowed', 'safety_monitor is not reporting'))
    elif not s.safety_allowed:
        out.append(_bad('safety_monitor', 'veto: ' + s.safety_veto, 'motion allowed',
                        'The safety monitor would stop the car: ' + s.safety_veto))
    else:
        out.append(_ok('safety_monitor', 'motion allowed'))
    return out


class Machine:
    """PreflightReport state. START latches RUNNING; nothing but the e-stop / mission end changes it."""

    def __init__(self, cfg: Cfg, t0: float):
        self.cfg, self.t0 = cfg, t0
        self.state = LOADING
        self.green_since: Optional[float] = None
        self.started = False

    def step(self, now: float, calibration_missing: Sequence[str], checks: Sequence[Check]) -> int:
        if self.started:
            return self.state
        if calibration_missing:
            self.green_since, self.state = None, CAL_MISSING
        elif now - self.t0 < self.cfg.startup_grace_s:
            self.green_since, self.state = None, LOADING
        elif all(c.ok for c in checks):
            if self.green_since is None:
                self.green_since = now
            self.state = READY if now - self.green_since >= self.cfg.ready_hold_s else PREFLIGHT
        else:
            self.green_since, self.state = None, NOT_READY
        return self.state

    def start(self) -> Tuple[bool, str]:
        if self.started:
            return False, 'START refused: the run has already started'
        if self.state != READY:
            return False, f'START refused: preflight is not READY (state {self.state})'
        self.started, self.state = True, RUNNING
        return True, 'START accepted: race armed'

    def estop(self) -> None:
        if self.started and self.state == RUNNING:
            self.state = ESTOPPED

    def mission_complete(self) -> None:
        if self.started and self.state == RUNNING:
            self.state = FINISHED
