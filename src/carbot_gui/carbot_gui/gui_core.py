"""GUI core (no ROS): everything the GUI server computes, unit tested.

* TABS / TAB_GROUPS: which topic groups each tab needs. Topic groups are
  subscribed ONLY while a browser polls them (LazyGroups), so a closed tab
  costs nothing. 'core' is what the header needs on every tab.
* running_now(): the header's "code running now" lines, built from the command
  owner, safety, mission and node heartbeats (no new messages).
* leg_progress(): current leg, piece and % along it, from route_info + the
  global route + the local pose.
* encode_grid(): LocalGrid -> one byte per cell for the browser canvas.
* EventLog: ring buffer for the Events + log tab (mission events, mode and
  winner changes, safety vetoes, WARN+ /rosout, manual / e-stop).
"""
import base64
import math
import time
from collections import deque
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# ----------------------------------------------------------------------------- tabs
RACE_TABS = ['drive', 'map', 'percplan', 'memory', 'loc', 'det', 'control', 'health', 'events']
CALIBRATE_TABS = ['calibration', 'tuning', 'map', 'percplan', 'memory', 'loc', 'det', 'control',
                  'health', 'events']
TAB_TITLES = {
    'drive': 'Drive', 'map': 'Global map', 'percplan': 'Perception + planner',
    'memory': 'Memory + LiDAR', 'loc': 'Localization', 'det': 'Detections',
    'control': 'Control + safety', 'health': 'System health', 'events': 'Events + log',
    'calibration': 'Calibration', 'tuning': 'Tuning',
}
# topic groups per tab ('core' is always added while any page is open)
TAB_GROUPS = {
    'drive': ['drive'], 'map': ['map'], 'percplan': ['percplan'], 'memory': ['memory'],
    'loc': ['loc', 'map'], 'det': ['det'], 'control': ['control'], 'health': ['health'],
    'events': [], 'calibration': ['calibration'], 'tuning': [],
}
# `tab:` ids used in challenges.yaml / calibration_steps.yaml (phase 1 names, kept) -> GUI tab ids
YAML_TAB_ALIASES = {'main': 'drive', 'map': 'map', 'perception': 'percplan', 'planner': 'percplan',
                    'memory_lidar': 'memory', 'localization': 'loc', 'detections': 'det', 'control': 'control',
                    'health': 'health', 'tuning': 'tuning', 'scoreboard': 'events', 'recording': 'events',
                    'events': 'events'}
IMAGE_KEYS = ('cam_front', 'ov_front', 'stitched', 'mask', 'warped', 'det')   # front camera only (side cameras removed 2026-09-24)

# blocks named in the header
BLOCK_OF_WINNER = {'ROAD': ('09-13', 'Corridor + local planner', 'percplan'),
                   'TUNNEL': ('13', 'Tunnel follower (base, wrapped)', 'memory'),
                   'PARKING': ('11', 'Parking planner', 'percplan'),
                   'RECOVERY': ('12', 'Recovery planner', 'percplan')}


# ----------------------------------------------------------------------------- lazy subscriptions
class LazyGroups:
    """Tracks which topic groups a browser asked for recently.

    touch(group) on every poll; wanted() = groups polled within idle_s.
    The server subscribes wanted groups and destroys the rest (diff()).
    """

    def __init__(self, idle_s: float, enabled: bool = True, clock=time.monotonic):
        self.idle_s = float(idle_s)
        self.enabled = enabled
        self.clock = clock
        self.last: Dict[str, float] = {}
        self.active: set = set()

    def touch(self, group: str) -> None:
        self.last[group] = self.clock()

    def wanted(self, all_groups: Iterable[str]) -> set:
        if not self.enabled:
            return set(all_groups)
        now = self.clock()
        return {g for g in all_groups if now - self.last.get(g, -1e9) <= self.idle_s}

    def diff(self, all_groups: Iterable[str]) -> Tuple[set, set]:
        """(to_add, to_remove) and records the new active set."""
        want = self.wanted(all_groups)
        add, rem = want - self.active, self.active - want
        self.active = want
        return add, rem


# ----------------------------------------------------------------------------- header: running now
def _item(block: str, text: str, kind: str = '', tab: str = '') -> Dict:
    return {'id': block, 'text': text, 'kind': kind, 'tab': tab}


def running_now(mode: str, owner: Optional[Dict], safety: Optional[Dict], mission: Optional[Dict],
                nodes: Sequence[Dict], manual: bool, leg: Optional[Dict] = None,
                calib: Optional[Dict] = None) -> List[Dict]:
    """Header lines [{label, items:[{id,text,kind,tab}]}].

    owner   = {winner, reason, active_source, armed, stale}
    safety  = {motion_allowed, veto_check, veto_reason, value, limit}
    mission = {mode, hold_reason, speed_zone, zone_max, challenge_id, challenge_name, stale}
    nodes   = [{node, block, level(0..3), state, detail, stale}]
    """
    lines: List[Dict] = []
    if manual:
        items = [_item('base', 'servo_controller · controller drives', 'warn', 'control'),
                 _item('15', 'Command owner · autonomy paused', '', 'control')]
        lines.append({'label': 'Manual', 'items': items})
        return lines + _faults(nodes)
    if mode == 'calibrate' and calib:
        lines.append({'label': 'Running', 'items': [_item(f"Step {calib.get('index', '?')}",
                                                          calib.get('text', ''), 'go', 'calibration')]})
    if owner is None or owner.get('stale'):
        lines.append({'label': 'Stopped by', 'items': [_item('15', 'Command owner silent: no /carbot/owner/state',
                                                             'bad', 'health')]})
        return lines + _faults(nodes)
    w = owner.get('winner', '')
    m = mission or {}
    if w in BLOCK_OF_WINNER:
        blk, name, tab = BLOCK_OF_WINNER[w]
        leg_txt = f" · leg {leg['leg'] + 1} of {leg['legs']}" if leg and leg.get('legs') else ''
        items = [_item('08', f"Mission · {m.get('mode', '?')}{leg_txt}", 'go', 'map'),
                 _item(blk, name, 'go', tab),
                 _item('15', f'Command owner · {w}', 'go', 'control')]
        lines.append({'label': 'Driving', 'items': items, 'chain': True})
        if m.get('zone_max') and m.get('speed_zone'):
            lines.append({'label': 'Limiting', 'items': [
                _item('08', f"Speed zone {m['speed_zone']} · {m['zone_max']:.2f} m/s", 'warn', 'map')]})
    else:
        stop: List[Dict] = []
        if w == 'SAFETY_STOP':
            s = safety or {}
            if s.get('veto_check'):
                txt = f"Safety · {s['veto_check']}"
                if s.get('value') is not None and s.get('limit') is not None:
                    txt += f" {s['value']:.2f} vs limit {s['limit']:.2f}"
                elif s.get('veto_reason'):
                    txt += f" · {s['veto_reason']}"
                stop.append(_item('14', txt, 'bad', 'control'))
            else:
                stop.append(_item('15', owner.get('reason', 'Safety stop'), 'bad', 'control'))
        elif w == 'WATCHDOG':
            stop.append(_item('15', f"Watchdog · {owner.get('reason', '')}", 'bad', 'control'))
        elif w == 'DISARMED':
            stop.append(_item('15', owner.get('reason', 'Not armed'), '', 'control'))
        elif w == 'MANUAL':
            stop.append(_item('15', 'Manual control active', 'warn', 'control'))
        if m.get('hold_reason'):
            stop.append(_item('08', f"Mission · {m['hold_reason']}", 'bad', 'det' if 'TRAFFIC' in m['hold_reason']
                              or 'GATE' in m['hold_reason'] else 'map'))
        elif w == 'HOLD':
            stop.append(_item('08', f"Mission · {owner.get('reason', 'HOLD')}", 'bad', 'map'))
        if m.get('stale'):
            stop.append(_item('08', 'Mission state stale', 'bad', 'health'))
        lines.append({'label': 'Stopped by' if w != 'DISARMED' else 'Waiting', 'items': stop})
    return lines + _faults(nodes)


def _faults(nodes: Sequence[Dict]) -> List[Dict]:
    """Nodes in ERROR or silent: listed so a stop can be traced to its block."""
    bad = [n for n in nodes if n.get('level') == 2 or n.get('stale')]
    if not bad:
        return []
    items = [_item(n.get('block') or '--', f"{n['node']} · " + ('silent' if n.get('stale') else
                                                                  (n.get('detail') or n.get('state', ''))),
                   'bad', 'health') for n in bad[:4]]
    if len(bad) > 4:
        items.append(_item('+', f'{len(bad) - 4} more', 'bad', 'health'))
    return [{'label': 'Faults', 'items': items}]


# ----------------------------------------------------------------------------- legs
def leg_progress(info: Optional[Dict], route_xy: Optional[Sequence[Tuple[float, float]]],
                 pose_xy: Optional[Tuple[float, float]], mission_leg: int) -> Optional[Dict]:
    """Current leg/piece and progress.

    info = route_info JSON (pieces with start/end indices into route_xy, legs).
    mission_leg = MissionState.route_leg (0-based index into legs).
    Progress = nearest route index inside the leg's pieces, as a length fraction.
    """
    if not info or not info.get('legs'):
        return None
    legs = info['legs']
    n_legs = len(legs)
    leg = max(0, min(int(mission_leg), n_legs - 1))
    pieces = [p for p in info.get('pieces', []) if p.get('leg') == leg and p.get('start', -1) >= 0]
    out = {'leg': leg, 'legs': n_legs, 'pct': 0.0, 'piece': None, 'piece_kind': '',
           'piece_no': 0, 'pieces': len(pieces), 'lengths': [], 'route_index': -1}
    out['lengths'] = [round(sum(p.get('length_m', 0.0) for p in info.get('pieces', []) if p.get('leg') == k), 2)
                      for k in range(n_legs)]
    if not pieces or route_xy is None or pose_xy is None or len(route_xy) == 0:
        return out
    lo, hi = pieces[0]['start'], pieces[-1]['end']
    best, bi = 1e18, lo
    px, py = pose_xy
    for i in range(lo, min(hi, len(route_xy) - 1) + 1):
        x, y = route_xy[i]
        d = (x - px) ** 2 + (y - py) ** 2
        if d < best:
            best, bi = d, i
    cum = 0.0
    tot = 0.0
    for i in range(lo + 1, min(hi, len(route_xy) - 1) + 1):
        seg = math.hypot(route_xy[i][0] - route_xy[i - 1][0], route_xy[i][1] - route_xy[i - 1][1])
        tot += seg
        if i <= bi:
            cum += seg
    out['pct'] = round(100.0 * cum / tot, 1) if tot > 0 else 0.0
    out['route_index'] = bi
    for k, p in enumerate(pieces):
        if p['start'] <= bi <= p['end']:
            out['piece'], out['piece_kind'], out['piece_no'] = p['index'], p.get('kind', ''), k + 1
            break
    return out


# ----------------------------------------------------------------------------- grids
def encode_grid(rows: int, cols: int, kind: Sequence[int], grown: Sequence[int] = (),
                age: Sequence[float] = (), age_max_s: float = 3.0, stride: int = 1) -> Dict:
    """One byte per (decimated) cell: bits 0-1 kind, bit 2 grown, bits 4-7 age bucket (0 fresh..15 old).

    kind/grown are row-major rows*cols (row = +x, col = +y), as LocalGrid.
    """
    stride = max(1, int(stride))
    r2, c2 = (rows + stride - 1) // stride, (cols + stride - 1) // stride
    out = bytearray(r2 * c2)
    has_g, has_a = len(grown) == rows * cols, len(age) == rows * cols
    for r in range(0, rows, stride):
        base = r * cols
        orow = (r // stride) * c2
        for c in range(0, cols, stride):
            i = base + c
            k = int(kind[i]) & 3
            b = k
            if has_g and grown[i]:
                b |= 4
            if has_a:
                a = float(age[i])
                if a >= 0:
                    b |= min(15, int(15 * a / max(age_max_s, 1e-3))) << 4
                elif k == 0:
                    pass
            out[orow + c // stride] = b
    return {'rows': r2, 'cols': c2, 'b64': base64.b64encode(bytes(out)).decode('ascii')}


def decimate(points: Sequence, max_n: int) -> List:
    n = len(points)
    if n <= max_n or max_n <= 1:
        return list(points)
    step = (n - 1) / (max_n - 1)
    return [points[int(round(i * step))] for i in range(max_n)]


def to_base(px: float, py: float, pose: Tuple[float, float, float]) -> Tuple[float, float]:
    """Track-frame point -> base_link given the car pose (x, y, yaw) in the track frame."""
    x, y, a = pose
    dx, dy = px - x, py - y
    c, s = math.cos(a), math.sin(a)
    return (c * dx + s * dy, -s * dx + c * dy)


def venue_to_track(vx: float, vy: float, t2v: Dict) -> Tuple[float, float]:
    """Inverse of uwb.yaml track_to_venue (venue = R(yaw) * track + t)."""
    a = math.radians(float(t2v.get('yaw_deg', 0.0)))
    dx, dy = vx - float(t2v.get('x_m', 0.0)), vy - float(t2v.get('y_m', 0.0))
    c, s = math.cos(a), math.sin(a)
    return (c * dx + s * dy, -s * dx + c * dy)


def cov_ellipse(cxx: float, cxy: float, cyy: float, k: float = 2.0) -> Tuple[float, float, float]:
    """(semi-major, semi-minor, angle rad) of the k-sigma ellipse of a 2x2 covariance."""
    tr, det = cxx + cyy, cxx * cyy - cxy * cxy
    disc = math.sqrt(max(tr * tr / 4 - det, 0.0))
    l1, l2 = tr / 2 + disc, max(tr / 2 - disc, 0.0)
    ang = 0.5 * math.atan2(2 * cxy, cxx - cyy)
    return (k * math.sqrt(max(l1, 0.0)), k * math.sqrt(l2), ang)


# ----------------------------------------------------------------------------- events
class EventLog:
    """Ring buffer; each event {t (run s), wall, kind, level, source, text}."""

    def __init__(self, max_n: int = 300, clock=time.monotonic):
        self.q = deque(maxlen=int(max_n))
        self.clock = clock
        self.t0 = clock()
        self.seq = 0
        self._last = {}

    def reset_clock(self) -> None:
        self.t0 = self.clock()

    def add(self, kind: str, level: str, source: str, text: str) -> None:
        self.seq += 1
        self.q.append({'seq': self.seq, 't': round(self.clock() - self.t0, 1), 'wall': time.time(),
                       'kind': kind, 'level': level, 'source': source, 'text': text})

    def change(self, key: str, value, kind: str, level: str, source: str, text: str) -> None:
        """Add only when `value` changed since the last call with this key."""
        if self._last.get(key, object()) != value:
            self._last[key] = value
            self.add(kind, level, source, text)

    def since(self, seq: int = 0) -> List[Dict]:
        return [e for e in self.q if e['seq'] > seq]


# ----------------------------------------------------------------------------- health
def node_rows(nodes: Dict[str, Tuple[float, object]], now: float, stale_s: float) -> List[Dict]:
    """nodes = {name: (receive_monotonic, NodeStatus-like)} -> rows for the header and health tab."""
    rows = []
    for name, (t, m) in sorted(nodes.items(), key=lambda kv: (getattr(kv[1][1], 'block', '') or 'zz', kv[0])):
        ages = list(getattr(m, 'input_age_s', []))
        topics = list(getattr(m, 'input_topics', []))
        oldest = max([a for a in ages if a >= 0], default=-1.0)
        never = [tp for tp, a in zip(topics, ages) if a < 0]
        rows.append({'node': name, 'block': getattr(m, 'block', ''), 'level': int(getattr(m, 'level', 0)),
                     'state': getattr(m, 'state', ''), 'detail': getattr(m, 'detail', ''),
                     'oldest_input_s': round(oldest, 2), 'never': never, 'stale': now - t > stale_s})
    return rows
