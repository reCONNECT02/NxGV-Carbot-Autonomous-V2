"""ROS helpers shared by the phase-4 planning nodes (message <-> numpy, pose
history, and the camera/memory evidence inputs of blocks 09 and 10).

Path convention (all planning Paths, frame `track`): pose.position.z holds the
driving direction (+1 forward, -1 reverse), like Candidate.points.
"""
import bisect
import math
from collections import deque
from typing import Optional, Tuple

import numpy as np
from carbot_common import topics as T
from carbot_common.qos import SENSOR
from carbot_common.static_tf import StaticMount
from carbot_interfaces.msg import LocalGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

from .evidence import Evidence, EvidenceCfg, Grid, track_from_odom


def stamp_s(st) -> float:
    return st.sec + st.nanosec * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_of(msg) -> Tuple[float, float, float]:
    """(x, y, yaw) of an Odometry / PoseWithCovarianceStamped / Pose."""
    p = msg.pose.pose if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose') else msg
    return float(p.position.x), float(p.position.y), yaw_of(p.orientation)


def path_to_msg(points: np.ndarray, frame: str, stamp) -> Path:
    m = Path()
    m.header.frame_id = frame
    m.header.stamp = stamp
    for x, y, a, d in np.asarray(points, float).reshape(-1, 4):
        ps = PoseStamped()
        ps.header = m.header
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(d or 1.0)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(a / 2.0), math.cos(a / 2.0)
        m.poses.append(ps)
    return m


def path_from_msg(m: Path) -> np.ndarray:
    if not m.poses:
        return np.zeros((0, 4))
    return np.array([(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation),
                      1.0 if p.pose.position.z >= 0 else -1.0) for p in m.poses])


def path_key(m: Path):
    return (m.header.stamp.sec, m.header.stamp.nanosec, len(m.poses))


class PoseHistory:
    """Timestamped (x, y, a), interpolated lookup."""

    def __init__(self, keep_s: float = 2.0):
        self.keep = keep_s
        self.t = deque()
        self.p = deque()

    def add(self, t: float, pose) -> None:
        if self.t and t <= self.t[-1]:
            if t == self.t[-1]:
                self.p[-1] = pose
            return
        self.t.append(t)
        self.p.append(pose)
        while self.t and self.t[0] < t - self.keep:
            self.t.popleft()
            self.p.popleft()

    def latest(self):
        return (self.t[-1], self.p[-1]) if self.t else (None, None)

    def at(self, t: float, max_gap: float = 0.15) -> Optional[Tuple[float, float, float]]:
        if not self.t:
            return None
        ts = list(self.t)
        i = bisect.bisect_left(ts, t)
        if i == 0:
            return self.p[0] if ts[0] - t <= max_gap else None
        if i >= len(ts):
            return self.p[-1] if t - ts[-1] <= max_gap else None
        t0, t1 = ts[i - 1], ts[i]
        if t1 - t0 > 2 * max_gap:
            return None
        u = (t - t0) / max(t1 - t0, 1e-9)
        a, b = self.p[i - 1], self.p[i]
        da = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), a[2] + u * da


class EvidenceInputs:
    """Subscribes a CarbotNode to what V4 guidance.evidence reads:
    road grid (block 03), memory grid (block 04), local pose (block 05), /odom.
    evidence(now) returns an Evidence ready for queries."""

    def __init__(self, node, cfg: EvidenceCfg, max_gap_s: float):
        self.node = node
        self.cfg = cfg
        self.max_gap = max_gap_s
        self.local = PoseHistory(2.0)
        self.odom = {}                       # stamp -> odom pose (recent)
        self.pair = None                     # latest T_track_odom
        self.grid = None
        self.grid_pose = None
        self.mem = None
        self.speed = 0.0
        node.sub(Odometry, T.LOCAL_POSE, self._on_local, 10)
        node.sub(Odometry, T.ODOM, self._on_odom, SENSOR)
        node.sub(LocalGrid, T.ROAD_GRID, self._on_grid, 10)
        node.sub(LocalGrid, T.MEMORY_GRID, self._on_mem, 1)

    def _on_odom(self, m: Odometry) -> None:
        t = stamp_s(m.header.stamp)
        self.odom[t] = pose_of(m)
        self.speed = float(m.twist.twist.linear.x)
        if len(self.odom) > 80:
            for k in sorted(self.odom)[:-60]:
                del self.odom[k]

    def _on_local(self, m: Odometry) -> None:
        t = stamp_s(m.header.stamp)
        p = pose_of(m)
        self.local.add(t, p)
        o = self.odom.get(t)
        if o is None and self.odom:                # nearest /odom sample within 20 ms
            k = min(self.odom, key=lambda s: abs(s - t))
            o = self.odom[k] if abs(k - t) < 0.02 else None
        if o is not None:
            self.pair = track_from_odom(p, o)

    def _on_grid(self, g: LocalGrid) -> None:
        t = stamp_s(g.header.stamp)
        pose = self.local.at(t, self.max_gap)
        if pose is None:
            return
        self.grid = Grid.from_msg(g, t)
        self.grid_pose = pose

    def _on_mem(self, g: LocalGrid) -> None:
        self.mem = Grid.from_msg(g, stamp_s(g.header.stamp))

    def pose(self):
        return self.local.latest()

    def evidence(self, now: float) -> Evidence:
        ev = Evidence(self.cfg)
        ev.now = now
        if self.grid is not None:
            ev.set_live(self.grid, self.grid_pose)
        if self.mem is not None and self.pair is not None:
            ev.set_memory(self.mem, self.pair)
        return ev


def evidence_cfg(p) -> EvidenceCfg:
    return EvidenceCfg(live_max_age_s=float(p('live_max_age_s')),
                       unc_base_m=float(p('memory_evidence.uncertainty_base_m')),
                       unc_per_m=float(p('memory_evidence.uncertainty_per_m')),
                       unc_limit_m=float(p('memory_evidence.uncertainty_limit_m')))


def scan_hits(scan, pose, mount, max_range: float) -> Optional[np.ndarray]:
    """LaserScan returns (< max_range) as track-frame points (M x 2), V4
    obstaclesClear origin. mount = (x, y, yaw) of laser_frame in base_link.
    Same conversion as local_planner._hits."""
    if scan is None:
        return None
    r = np.asarray(scan.ranges, float)
    ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r > scan.range_min) & (r < max_range)
    mx, my, myaw = mount
    c, sn = math.cos(pose[2]), math.sin(pose[2])
    ox, oy = pose[0] + mx * c - my * sn, pose[1] + mx * sn + my * c
    a = pose[2] + myaw + ang[ok]
    return np.column_stack([ox + r[ok] * np.cos(a), oy + r[ok] * np.sin(a)])


class LaserMount:
    """base_link -> laser_frame from the static TF (latched /tf_static only, no /tf listener),
    else the fallback (vehicle.lidar_x_m, 0, 0) until it arrives."""

    def __init__(self, node, use_tf: bool, base: str, laser: str, fallback):
        self.mount = tuple(fallback)
        self._static = StaticMount(node, base, laser) if use_tf else None

    def get(self):
        if self._static is not None:
            got = self._static.get()
            if got is not None:
                self.mount = got
        return self.mount


def memory_paint_xy(mem, t_track_odom, now: float, max_age: float) -> np.ndarray:
    """Track-frame centres of memory PAINT cells younger than max_age (V4
    bayObservation input: q.kind === 2 && t - q.stamp <= 24)."""
    from .evidence import PAINT
    if mem is None or t_track_odom is None:
        return np.zeros((0, 2))
    k = mem.kind == PAINT
    if mem.age is not None:
        k &= (mem.age >= 0) & (mem.age + max(0.0, now - mem.stamp) <= max_age)
    r, c = np.nonzero(k)
    if not len(r):
        return np.zeros((0, 2))
    ox = mem.x0 + (r + 0.5) * mem.res
    oy = mem.y0 + (c + 0.5) * mem.res
    x, y, a = t_track_odom
    ca, sa = math.cos(a), math.sin(a)
    return np.column_stack([x + ox * ca - oy * sa, y + ox * sa + oy * ca])
