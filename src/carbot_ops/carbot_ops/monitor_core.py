"""Pure helpers for system_monitor (no rclpy, unit tested).

* RateMeter        per-topic arrival rate, age and header latency over a sliding window.
* cdr_stamp()      header.stamp straight from the serialized CDR bytes, so image
                   topics are never deserialised in Python (raw subscriptions).
* scan_processes() camera / agent processes by command-line pattern, with the
                   sudo / bash / `ros2 run` wrappers filtered out so one camera
                   counts as one process.
* read_number()    sysfs readers that never raise (a missing file = -1).
"""
import collections
import math
import os
import struct
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple


# ----------------------------------------------------------------------------- rates
class RateMeter:
    """Arrival times of one topic inside the last `window_s` seconds."""

    def __init__(self, window_s: float):
        if window_s <= 0:
            raise ValueError('window_s must be > 0')
        self.window_s = float(window_s)
        self.times: Deque[float] = collections.deque()
        self.last_t: Optional[float] = None
        self.last_latency_ms: float = -1.0
        self.count = 0

    def add(self, t: float, latency_ms: float = -1.0) -> None:
        self.times.append(t)
        self.last_t = t
        self.last_latency_ms = latency_ms
        self.count += 1
        self._prune(t)

    def _prune(self, now: float) -> None:
        while self.times and now - self.times[0] > self.window_s:
            self.times.popleft()

    def rate_hz(self, now: float) -> float:
        self._prune(now)
        n = len(self.times)
        if n < 2:
            return 0.0
        span = self.times[-1] - self.times[0]
        return (n - 1) / span if span > 1e-6 else 0.0

    def age_s(self, now: float) -> float:
        return -1.0 if self.last_t is None else max(0.0, now - self.last_t)


def cdr_stamp(raw: bytes) -> Optional[Tuple[int, int]]:
    """(sec, nanosec) of a message whose FIRST field is std_msgs/Header.

    CDR layout: 4-byte encapsulation (0x00 0x01 = little endian, 0x00 0x00 =
    big endian), then Header.stamp.sec (int32) and .nanosec (uint32).
    Returns None for anything that does not look like that."""
    if raw is None or len(raw) < 12:
        return None
    kind = raw[1]
    if raw[0] != 0 or kind not in (0, 1):
        return None
    fmt = '<iI' if kind == 1 else '>iI'
    sec, nsec = struct.unpack_from(fmt, raw, 4)
    if sec <= 0 or nsec >= 1_000_000_000:
        return None
    return sec, nsec


def latency_ms(stamp: Optional[Tuple[int, int]], now_s: float, max_abs_ms: float) -> float:
    """now - stamp in ms; -1 when unknown or implausible (driver not on wall clock)."""
    if stamp is None:
        return -1.0
    v = (now_s - (stamp[0] + stamp[1] * 1e-9)) * 1000.0
    if not math.isfinite(v) or abs(v) > max_abs_ms:
        return -1.0
    return v


# ----------------------------------------------------------------------------- processes
def _match(cmdline: str, pattern: str) -> bool:
    return bool(pattern) and pattern in cmdline


def ros_namespace(cmdline: str) -> str:
    """'/cam_ov5647' from '... -r __ns:=/cam_ov5647 ...' ('' if absent)."""
    key = '__ns:='
    i = cmdline.find(key)
    if i < 0:
        return ''
    return cmdline[i + len(key):].split(' ', 1)[0]


def scan_processes(procs: Iterable[Tuple[int, str, Sequence[str]]], patterns: Sequence[str],
                   wrappers: Sequence[str]) -> List[Tuple[str, int]]:
    """[(label, pid)] of processes matching any pattern.

    procs: (pid, name, cmdline list), e.g. from psutil. Wrappers (sudo, bash,
    `ros2 run`'s python) are skipped: they carry the same command line as the
    real driver. label = pattern + ' ' + ROS namespace when there is one, so
    'mipi_cam /cam_ov5647' and 'mipi_cam /cam_imx219' count separately."""
    wrap = set(wrappers)
    out: List[Tuple[str, int]] = []
    for pid, name, cmd in procs:
        if name in wrap:
            continue
        line = ' '.join(cmd or [])
        for pat in patterns:
            if _match(line, pat) or (name and pat == name):
                ns = ros_namespace(line)
                out.append((pat + (' ' + ns if ns else ''), int(pid)))
                break
    return sorted(out)


def psutil_procs():
    """(pid, name, cmdline) for every process psutil can read; never raises."""
    try:
        import psutil
    except ImportError:
        return []
    out = []
    for p in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            info = p.info
            out.append((info['pid'], info.get('name') or '', info.get('cmdline') or []))
        except Exception:  # noqa: BLE001  process vanished / no permission
            continue
    return out


# ----------------------------------------------------------------------------- sysfs
def read_number(path: str, scale: float = 1.0) -> float:
    """First number in a text file times scale, -1 if the file is missing or odd."""
    try:
        with open(path, 'r', encoding='utf-8') as f:
            txt = f.read().strip()
    except OSError:
        return -1.0
    for tok in txt.replace('%', ' ').replace(':', ' ').split():
        try:
            return float(tok) * scale
        except ValueError:
            continue
    return -1.0


def env_network(domain_want: int) -> Dict[str, object]:
    """ROS network settings of this process (UWB_Handoff.md section 8)."""
    dom = os.environ.get('ROS_DOMAIN_ID', '')
    loc = os.environ.get('ROS_LOCALHOST_ONLY', '0')
    problems = []
    if loc == '1':
        problems.append('ROS_LOCALHOST_ONLY=1 hides /uwb3/input_json (the agent ignores it)')
    if dom != str(domain_want):
        problems.append(f'ROS_DOMAIN_ID={dom or "unset (0)"} but the UWB tag uses {domain_want}')
    return {'domain_id': dom, 'localhost_only': loc, 'ok': not problems, 'problems': problems}
