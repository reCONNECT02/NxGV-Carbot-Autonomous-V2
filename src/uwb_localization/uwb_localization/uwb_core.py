"""UWB range processing, pure Python (no ROS). Used by the uwb_ranges node,
calibration step 10 (calib_uwb) and the sandbox. The position filter that sits
on top of the solvers here is positioning.py (Haffiz's method).

Follows docs/reference/UWB_Handoff.md:
  * tag JSON: {"tag","boot_id","seq","t_ms","last_unknown_id",
               "links":[{"A","R","age_ms","sample_seq"}]}, R in METRES (raw, 3-D)
  * sample_seq increments only on a new radio measurement -> skip repeats
  * range time = arrival - age_ms; WiFi adds 40-130 ms of variable latency
  * empty links = no measurement (NEVER (0, 0))
  * per-anchor offset (uwb_calib.py): corrected = R - offset, then flattened to
    the floor plane with the anchor/tag height difference
  * boot_id change = tag restarted -> forget sample_seq and clock history

Latency: the tag clock (t_ms) and the RDK clock are not synchronised. With
mode 'min_filter' the node tracks the smallest (arrival - t_ms) over a sliding
window; that sample is the one with the least WiFi delay, so
  encode_time = t_ms + min_offset + base_transit
removes most of the 40-130 ms jitter. Mode 'arrival' uses arrival time only.
"""
import json
import math
import statistics
from collections import deque
from dataclasses import dataclass, field
from itertools import combinations
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


# --------------------------------------------------------------------------- anchors
@dataclass
class Anchor:
    id: str
    x: float
    y: float
    z: float
    offset: float = 0.0          # range_offset_m (measured - true)


@dataclass
class AnchorSet:
    anchors: Dict[str, Anchor]
    tag_z: float
    surveyed: bool
    offsets_calibrated: bool

    @staticmethod
    def from_yaml(doc: Dict) -> 'AnchorSet':
        """doc = config/data/uwb.yaml (or a calibration session copy)."""
        anchors = {}
        for a in doc['anchors']:
            aid = str(a['id']).upper()
            x, y, z = (float(v) for v in a['xyz_m'])
            anchors[aid] = Anchor(aid, x, y, z, float(a.get('range_offset_m', 0.0)))
        if len(anchors) < 3:
            raise ValueError('uwb.yaml: at least 3 anchors are needed')
        return AnchorSet(anchors, float(doc['tag']['z_m']),
                         bool(doc.get('anchors_surveyed', False)),
                         bool(doc.get('offsets_calibrated', False)))

    @property
    def ids(self) -> List[str]:
        return sorted(self.anchors)

    def flatten(self, aid: str, range_3d: float) -> float:
        dz = self.anchors[aid].z - self.tag_z
        h2 = range_3d * range_3d - dz * dz
        return math.sqrt(h2) if h2 > 0.0 else 0.0

    def true_range_3d(self, aid: str, x: float, y: float, z: Optional[float] = None) -> float:
        a = self.anchors[aid]
        tz = self.tag_z if z is None else z
        return math.sqrt((x - a.x) ** 2 + (y - a.y) ** 2 + (tz - a.z) ** 2)


# --------------------------------------------------------------------------- parsing
@dataclass
class Link:
    anchor: str
    r: float
    age_ms: int
    sample_seq: int


@dataclass
class Report:
    tag: str
    boot_id: str
    seq: int
    t_ms: int
    last_unknown_id: str
    links: List[Link]


def _strip_echo_prefix(text: str) -> str:
    """Haffiz's parser: accept `data: '{...}'` (a pasted `ros2 topic echo` line) as well as plain JSON."""
    raw = (text or '').strip()
    if raw.startswith('data:'):
        raw = raw[len('data:'):].strip()
        if len(raw) >= 2 and raw[0] == raw[-1] and raw[0] in ('"', "'"):
            raw = raw[1:-1]
    return raw


def parse_report(text: str) -> Optional[Report]:
    """JSON string -> Report, or None if it is not a valid tag report."""
    try:
        d = json.loads(_strip_echo_prefix(text))
    except (ValueError, TypeError):
        return None
    if not isinstance(d, dict):
        return None
    links = []
    for ln in d.get('links') or []:
        try:
            links.append(Link(str(ln['A']).upper(), float(ln['R']),
                              int(ln.get('age_ms', 0)), int(ln.get('sample_seq', -1))))
        except (KeyError, TypeError, ValueError):
            continue
    try:
        return Report(str(d.get('tag', '')), str(d.get('boot_id', '')), int(d.get('seq', 0)),
                      int(d.get('t_ms', 0)), str(d.get('last_unknown_id', '0000')).upper(), links)
    except (TypeError, ValueError):
        return None


# --------------------------------------------------------------------------- processing
@dataclass
class ProcessedRange:
    anchor: str
    raw_m: float
    corrected_m: float           # offset applied, flattened to the floor plane
    age_ms: int
    sample_seq: int
    fresh: bool                  # new sample, in range, not too old
    reason: str                  # '' or why it is not fresh
    stamp: float                 # estimated measurement time (s, node clock)


@dataclass
class Processed:
    report: Report
    arrival: float
    ranges: List[ProcessedRange]
    rebooted: bool
    latency_ms: float            # estimated extra WiFi delay of this report
    unknown_ids: List[str] = field(default_factory=list)
    encode_time: float = 0.0     # estimated time the tag encoded the report (s, node clock)


class LatencyFilter:
    """Sliding-window minimum of (arrival - t_ms): the least-delayed report."""

    def __init__(self, window_s: float):
        self.window = float(window_s)
        self.buf: Deque[Tuple[float, float]] = deque()

    def reset(self) -> None:
        self.buf.clear()

    def update(self, arrival: float, t_tag: float) -> float:
        off = arrival - t_tag
        while self.buf and self.buf[-1][1] >= off:      # monotone deque for window min
            self.buf.pop()
        self.buf.append((arrival, off))
        while self.buf and self.buf[0][0] < arrival - self.window:
            self.buf.popleft()
        return self.buf[0][1]


class RangeProcessor:

    def __init__(self, anchors: AnchorSet, max_age_ms: float, min_range_m: float,
                 max_range_m: float, latency_mode: str = 'min_filter',
                 latency_window_s: float = 10.0, base_transit_ms: float = 5.0,
                 max_extra_latency_ms: float = 300.0):
        if latency_mode not in ('min_filter', 'arrival'):
            raise ValueError(f'latency.mode must be min_filter or arrival, not {latency_mode}')
        self.anchors = anchors
        self.max_age_ms = float(max_age_ms)
        self.min_range = float(min_range_m)
        self.max_range = float(max_range_m)
        self.mode = latency_mode
        self.lat = LatencyFilter(latency_window_s)
        self.base = float(base_transit_ms) / 1000.0
        self.max_extra = float(max_extra_latency_ms) / 1000.0
        self.boot_id: Optional[str] = None
        self.last_seq: Dict[str, int] = {}
        self.last_t_ms: Optional[int] = None
        self.reboots = 0

    def process(self, rep: Report, arrival: float) -> Processed:
        rebooted = False
        if rep.boot_id != self.boot_id or (self.last_t_ms is not None and rep.t_ms < self.last_t_ms):
            if self.boot_id is not None:
                rebooted = True
                self.reboots += 1
            self.boot_id = rep.boot_id
            self.last_seq.clear()
            self.lat.reset()
        self.last_t_ms = rep.t_ms

        t_tag = rep.t_ms / 1000.0
        if self.mode == 'min_filter':
            off = self.lat.update(arrival, t_tag)
            extra = (arrival - t_tag) - off
            encode = t_tag + off + self.base if extra <= self.max_extra else arrival - self.base
        else:
            extra = 0.0
            encode = arrival
        out, unknown = [], []
        for ln in rep.links:
            if ln.anchor not in self.anchors.anchors:
                unknown.append(ln.anchor)
                continue
            a = self.anchors.anchors[ln.anchor]
            corr = self.anchors.flatten(ln.anchor, ln.r - a.offset)
            reason = ''
            if ln.sample_seq >= 0 and self.last_seq.get(ln.anchor) == ln.sample_seq:
                reason = 'repeat'
            elif ln.age_ms > self.max_age_ms:
                reason = 'old'
            elif not (self.min_range <= corr <= self.max_range):
                reason = 'out_of_range'
            if reason != 'repeat' and ln.sample_seq >= 0:
                self.last_seq[ln.anchor] = ln.sample_seq
            out.append(ProcessedRange(ln.anchor, ln.r, corr, ln.age_ms, ln.sample_seq,
                                      reason == '', reason, encode - ln.age_ms / 1000.0))
        return Processed(rep, arrival, out, rebooted, extra * 1000.0, unknown, encode)


# --------------------------------------------------------------------------- trilateration
# Three solvers, chosen with common.yaml uwb_positioning.solver (default: linear).
#   linear    Haffiz's method (tools/uwb/haffiz/turtle_uwb_visualizer.py solve_trilateration):
#             subtract the first anchor's circle equation from the others -> linear system
#             A p = B. With exactly 3 anchors it is his 2x2 solve, identical for any choice of
#             reference anchor; with 4+ anchors it is the least-squares solution.
#   nlls      Haffiz's noEKF method (turtle_uwb_visualizer_noEKF.py RobustTrilateration):
#             non-linear least squares, soft_l1 loss, started from the previous solution.
#   pairwise  the old tools/uwb/uwb_xy.py method (average of the pairwise circle intersections).
SOLVERS = ('linear', 'nlls', 'pairwise')


def solve_linear(anchors: 'AnchorSet', ranges: Dict[str, float]) -> Optional[Tuple[float, float]]:
    """Haffiz closed-form trilateration. ranges = flattened, offset-corrected metres per anchor id.
    None for < 3 anchors or a singular (collinear) geometry, never (0, 0)."""
    ids = sorted(i for i in ranges if i in anchors.anchors)
    if len(ids) < 3:
        return None
    x1, y1 = anchors.anchors[ids[0]].x, anchors.anchors[ids[0]].y
    r1 = float(ranges[ids[0]])
    A, B = [], []
    for i in ids[1:]:
        xi, yi = anchors.anchors[i].x, anchors.anchors[i].y
        ri = float(ranges[i])
        A.append([2.0 * (xi - x1), 2.0 * (yi - y1)])
        B.append(r1 * r1 - ri * ri - x1 * x1 + xi * xi - y1 * y1 + yi * yi)
    a = np.array(A)
    b = np.array(B)
    try:
        if len(ids) == 3:
            p = np.linalg.solve(a, b)
        else:
            if np.linalg.matrix_rank(a) < 2:
                return None
            p = np.linalg.lstsq(a, b, rcond=None)[0]
    except np.linalg.LinAlgError:
        return None
    if not np.all(np.isfinite(p)):
        return None
    return float(p[0]), float(p[1])


def _soft_l1_irls(anchor_xy: np.ndarray, r: np.ndarray, x0: Sequence[float], f_scale: float,
                  iters: int = 50) -> Optional[np.ndarray]:
    """Gauss-Newton with soft_l1 weights (scipy.optimize.least_squares loss='soft_l1'),
    used when scipy is not installed. rho(z) = 2((1+z)^0.5 - 1), z = (res/f)^2."""
    p = np.array(x0, float)
    for _ in range(iters):
        d = p[None, :] - anchor_xy
        h = np.maximum(np.hypot(d[:, 0], d[:, 1]), 1e-9)
        res = h - r
        J = d / h[:, None]
        w = 1.0 / np.sqrt(1.0 + (res / f_scale) ** 2)
        JW = J * w[:, None]
        H = JW.T @ J + np.eye(2) * 1e-9
        step = np.linalg.solve(H, -JW.T @ res)
        p = p + step
        if np.linalg.norm(step) < 1e-7:
            break
    return p if np.all(np.isfinite(p)) else None


def solve_nlls(anchors: 'AnchorSet', ranges: Dict[str, float], x0: Sequence[float],
               f_scale: float = 1.0) -> Optional[Tuple[float, float]]:
    """Haffiz RobustTrilateration.solve: least_squares(residuals, x0, loss='soft_l1')."""
    ids = sorted(i for i in ranges if i in anchors.anchors)
    if len(ids) < 3:
        return None
    axy = np.array([[anchors.anchors[i].x, anchors.anchors[i].y] for i in ids])
    r = np.array([float(ranges[i]) for i in ids])
    try:
        from scipy.optimize import least_squares
    except ImportError:                                   # pragma: no cover  (scipy on the RDK)
        p = _soft_l1_irls(axy, r, x0, f_scale)
        return None if p is None else (float(p[0]), float(p[1]))

    def residuals(pos):
        return np.hypot(pos[0] - axy[:, 0], pos[1] - axy[:, 1]) - r
    res = least_squares(residuals, np.array(x0, float), loss='soft_l1', f_scale=float(f_scale))
    if not res.success or not np.all(np.isfinite(res.x)):
        return None
    return float(res.x[0]), float(res.x[1])


def pair_estimate(a, ra, b, rb, others) -> Optional[Tuple[float, float]]:
    """tools/uwb/uwb_xy.py (pre-Haffiz) pair_estimate, generalised: `others` = [(xy, r), ...]
    picks the mirror candidate that fits the other anchors best."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return None
    ux, uy = dx / d, dy / d
    nx, ny = -uy, ux
    along = (ra * ra - rb * rb + d * d) / (2.0 * d)
    h2 = ra * ra - along * along
    h = math.sqrt(h2) if h2 > 0.0 else 0.0
    mx, my = a[0] + along * ux, a[1] + along * uy
    p1 = (mx + h * nx, my + h * ny)
    p2 = (mx - h * nx, my - h * ny)

    def err(p):
        return sum(abs(math.hypot(p[0] - c[0], p[1] - c[1]) - rc) for c, rc in others)
    return p1 if err(p1) <= err(p2) else p2


def trilaterate_pairwise(anchors: 'AnchorSet', ranges: Dict[str, float]) -> Optional[Tuple[float, float]]:
    """Average of all pairwise estimates (the old uwb_xy.py method). Needs >= 3 anchors."""
    ids = sorted(i for i in ranges if i in anchors.anchors)
    if len(ids) < 3:
        return None
    xy = {i: (anchors.anchors[i].x, anchors.anchors[i].y) for i in ids}
    est = []
    for ia, ib in combinations(ids, 2):
        others = [(xy[i], ranges[i]) for i in ids if i not in (ia, ib)]
        p = pair_estimate(xy[ia], ranges[ia], xy[ib], ranges[ib], others)
        if p is not None:
            est.append(p)
    if not est:
        return None
    return (sum(p[0] for p in est) / len(est), sum(p[1] for p in est) / len(est))


def trilaterate(anchors: 'AnchorSet', ranges: Dict[str, float], method: str = 'linear',
                x0: Optional[Sequence[float]] = None, f_scale: float = 1.0) -> Optional[Tuple[float, float]]:
    """One 2-D fix from one set of ranges. method: linear (Haffiz, default) | nlls | pairwise."""
    if method == 'linear':
        return solve_linear(anchors, ranges)
    if method == 'pairwise':
        return trilaterate_pairwise(anchors, ranges)
    if method == 'nlls':
        if x0 is None:
            x0 = solve_linear(anchors, ranges)
            if x0 is None:
                return None
        return solve_nlls(anchors, ranges, x0, f_scale)
    raise ValueError(f'unknown trilateration method {method!r} (allowed: {", ".join(SOLVERS)})')


# --------------------------------------------------------------------------- survey geometry
def hdop(anchors: AnchorSet, x: float, y: float, ids: Optional[Sequence[str]] = None) -> float:
    """Horizontal dilution of precision at (x, y): position error per metre of
    range error. < 1.5 good, > 3 poor, inf = degenerate (collinear / on anchor)."""
    ids = list(ids or anchors.ids)
    sxx = sxy = syy = 0.0
    for i in ids:
        a = anchors.anchors[i]
        dx, dy = x - a.x, y - a.y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return float('inf')
        ux, uy = dx / d, dy / d
        sxx += ux * ux
        sxy += ux * uy
        syy += uy * uy
    det = sxx * syy - sxy * sxy
    if det < 1e-9:
        return float('inf')
    return math.sqrt((sxx + syy) / det)


def layout_checks(anchors: AnchorSet, min_spacing_m: float, min_angle_deg: float) -> List[str]:
    """Problems with the anchor layout (empty list = OK). UWB_Handoff section 10:
    anchor spacing matters far more than the algorithm."""
    problems = []
    ids = anchors.ids
    for ia, ib in combinations(ids, 2):
        a, b = anchors.anchors[ia], anchors.anchors[ib]
        d = math.hypot(a.x - b.x, a.y - b.y)
        if d < min_spacing_m:
            problems.append(f'anchors {ia} and {ib} are only {d:.2f} m apart '
                            f'(need >= {min_spacing_m:.1f} m; positions must be in METRES)')
    best_angle = 0.0
    for tri in combinations(ids, 3):
        pts = [anchors.anchors[i] for i in tri]
        angles = []
        for k in range(3):
            p, q, r = pts[k], pts[(k + 1) % 3], pts[(k + 2) % 3]
            v1 = (q.x - p.x, q.y - p.y)
            v2 = (r.x - p.x, r.y - p.y)
            n1, n2 = math.hypot(*v1), math.hypot(*v2)
            if n1 < 1e-6 or n2 < 1e-6:
                angles.append(0.0)
                continue
            c = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
            angles.append(math.degrees(math.acos(c)))
        best_angle = max(best_angle, min(angles))
    if best_angle < min_angle_deg:
        problems.append(f'anchors are nearly in a line (best triangle min angle '
                        f'{best_angle:.0f} deg, need >= {min_angle_deg:.0f} deg)')
    heights = [anchors.anchors[i].z for i in ids]
    if max(heights) <= 0.05:
        problems.append('all anchors are on the floor (z = 0): ground reflection causes the '
                        '~25 cm flip-flop seen in testing. Raise them to 1.0-1.5 m if possible.')
    return problems


def hdop_coverage(anchors: AnchorSet, points: Iterable[Tuple[float, float]],
                  limit: float) -> Tuple[float, float]:
    """(fraction of points with hdop <= limit, worst hdop) over venue points."""
    vals = [hdop(anchors, x, y) for x, y in points]
    if not vals:
        return 0.0, float('inf')
    return sum(v <= limit for v in vals) / len(vals), max(vals)


# --------------------------------------------------------------------------- offset calibration
@dataclass
class OffsetResult:
    anchor: str
    samples: int
    measured_m: float
    true_m: float
    offset_m: float
    spread_m: float


def compute_offsets(anchors: AnchorSet, raw: Dict[str, List[float]], x: float, y: float,
                    min_samples: int = 20) -> Dict[str, OffsetResult]:
    """uwb_calib.py: offset = median(raw 3-D range) - true 3-D distance, per anchor.
    raw[aid] = raw R values in METRES (one per NEW sample_seq)."""
    out = {}
    for aid in anchors.ids:
        s = raw.get(aid, [])
        if len(s) < min_samples:
            continue
        meas = statistics.median(s)
        true = anchors.true_range_3d(aid, x, y)
        out[aid] = OffsetResult(aid, len(s), meas, true, meas - true, statistics.pstdev(s))
    return out


def fix_clusters(points: List[Tuple[float, float]], split_m: float = 0.10) -> Tuple[float, float]:
    """(typical jitter, largest cluster separation) of stationary fixes. A large
    separation is the multipath flip-flop described in UWB_Handoff section 10."""
    if len(points) < 5:
        return 0.0, 0.0
    mx = statistics.median(p[0] for p in points)
    my = statistics.median(p[1] for p in points)
    d = sorted(math.hypot(p[0] - mx, p[1] - my) for p in points)
    jitter = d[len(d) // 2]
    split_m = max(split_m, 3.0 * jitter)      # ordinary noise is not a second cluster
    far = [p for p in points if math.hypot(p[0] - mx, p[1] - my) > split_m]
    if len(far) < max(3, len(points) // 10):
        return jitter, 0.0
    fx = statistics.median(p[0] for p in far)
    fy = statistics.median(p[1] for p in far)
    near = [p for p in points if math.hypot(p[0] - fx, p[1] - fy) > split_m]
    nx = statistics.median(p[0] for p in near) if near else mx
    ny = statistics.median(p[1] for p in near) if near else my
    return jitter, math.hypot(fx - nx, fy - ny)
