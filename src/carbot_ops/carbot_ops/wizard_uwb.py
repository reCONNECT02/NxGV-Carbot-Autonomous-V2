"""Raw UWB tag reports for calibration wizard pages (step 10 uwb_survey, step 11
map_uwb_alignment). Pure except attach(); unit tested.

Why raw and not /carbot/uwb/ranges: the uwb_ranges node applies the offsets of
the uwb.yaml the LAUNCH loaded. Step 10 measures new offsets and step 11 must use
the ones step 10 just saved into the session, so wizard pages process the raw
tag JSON themselves with uwb_localization.uwb_core (same math as the node and
the calib_uwb CLI). UWB here is read-only: nothing in the wizard reaches the
servo or the local pose.

Node side (calibration_wizard, once):
    feed = attach(node, buffer_s, rate_window_s)     # subscribes T.UWB_INPUT_JSON, BEST_EFFORT
    inputs[INPUT_KEY] = feed                         # in every inputs() dict

Step side (inputs[INPUT_KEY] is a UwbFeed, or None when the node has none):
    c = feed.cursor()                         at start(): collection window begins now
    rows, c, lost = feed.rows_since(c)        each tick(): new rows [{'t': s, 'json': str}]
                                              (same shape as calib_uwb captures -> ct.write_jsonl,
                                              calib_uwb.raw_fresh_samples / fixes / --replay)
    feed.stats(ids, now=None)                 live view: per-anchor age / rate / last raw R,
                                              report rate, boot id, unknown ids

Helpers: session_uwb(session, base_doc) (this session's data/uwb.yaml, i.e. what
step 10 saved, else the launch's doc), anchor_set(doc, zero_offsets),
fresh_samples(rows), fixes(anchors, rows).
"""
import os
import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple

import yaml

from uwb_localization import calib_uwb as _cli
from uwb_localization.uwb_core import AnchorSet, parse_report

INPUT_KEY = 'uwb_raw'


class UwbFeed:
    """Bounded buffer of raw tag reports + per-anchor freshness (one per new sample_seq)."""

    def __init__(self, buffer_s: float, rate_window_s: float, clock: Callable[[], float] = time.monotonic):
        if buffer_s <= 0 or rate_window_s <= 0:
            raise ValueError('uwb_buffer_s and uwb_rate_window_s must be > 0')
        self.buffer_s, self.window = float(buffer_s), float(rate_window_s)
        self.clock = clock
        self.lock = threading.Lock()
        self.rows: Deque[Tuple[int, Dict]] = deque()
        self.n = 0                                   # rows ever added (cursor space)
        self.reports: Deque[float] = deque()
        self.bad_json = 0
        self.boot_id = ''
        self.last_t: Optional[float] = None
        self.unknown: Dict[str, float] = {}          # anchor id not configured -> last heard
        self.last_seq: Dict[str, int] = {}
        self.anchor: Dict[str, Dict] = {}            # id -> {'t', 'r', 'count', 'times': deque}

    # ------------------------------------------------------------------ input
    def add(self, text: str, t: Optional[float] = None) -> None:
        t = self.clock() if t is None else float(t)
        rep = parse_report(text)
        with self.lock:
            if rep is None:
                self.bad_json += 1
                return
            self.rows.append((self.n, {'t': t, 'json': text}))
            self.n += 1
            while self.rows and self.rows[0][1]['t'] < t - self.buffer_s:
                self.rows.popleft()
            self.reports.append(t)
            self.last_t = t
            if rep.boot_id != self.boot_id:
                self.boot_id, self.last_seq = rep.boot_id, {}
            if rep.last_unknown_id not in ('', '0000'):
                self.unknown[rep.last_unknown_id] = t
            for ln in rep.links:
                if ln.sample_seq >= 0 and self.last_seq.get(ln.anchor) == ln.sample_seq:
                    continue
                self.last_seq[ln.anchor] = ln.sample_seq
                a = self.anchor.setdefault(ln.anchor, {'t': None, 'r': None, 'count': 0, 'times': deque()})
                a['t'], a['r'] = t, ln.r
                a['count'] += 1
                a['times'].append(t)

    # ------------------------------------------------------------------ steps
    def cursor(self) -> int:
        with self.lock:
            return self.n

    def rows_since(self, cursor: int) -> Tuple[List[Dict], int, int]:
        """-> (rows added at/after cursor, new cursor, rows lost because the buffer dropped them)."""
        with self.lock:
            out = [r for i, r in self.rows if i >= cursor]
            first = self.rows[0][0] if self.rows else self.n
            return out, self.n, max(0, min(first, self.n) - cursor)

    def stats(self, ids: Sequence[str], now: Optional[float] = None) -> Dict:
        """Live view. ids = anchors to report (configured / surveyed); others heard are listed
        in 'unknown_ids' together with the tag's last_unknown_id."""
        now = self.clock() if now is None else float(now)
        with self.lock:
            lo = now - self.window
            while self.reports and self.reports[0] < lo:
                self.reports.popleft()
            anchors = {}
            for aid in ids:
                a = self.anchor.get(str(aid).upper())
                if a is None:
                    anchors[aid] = {'age_s': None, 'rate_hz': 0.0, 'last_raw_m': None, 'fresh_count': 0}
                    continue
                while a['times'] and a['times'][0] < lo:
                    a['times'].popleft()
                anchors[aid] = {'age_s': round(now - a['t'], 2), 'rate_hz': round(len(a['times']) / self.window, 1),
                                'last_raw_m': round(a['r'], 3), 'fresh_count': a['count']}
            idset = {str(i).upper() for i in ids}
            heard = {k for k, v in self.anchor.items() if v['t'] is not None and now - v['t'] < self.buffer_s}
            unknown = sorted((heard - idset) | {k for k, tt in self.unknown.items() if now - tt < self.buffer_s})
            return {'reports': self.n, 'hz': round(len(self.reports) / self.window, 1),
                    'age_s': None if self.last_t is None else round(now - self.last_t, 2),
                    'boot_id': self.boot_id, 'bad_json': self.bad_json, 'unknown_ids': unknown,
                    'anchors': anchors}


def attach(node, buffer_s: float, rate_window_s: float) -> UwbFeed:
    """Subscribe `node` to the tag's raw JSON (BEST_EFFORT: a RELIABLE subscriber gets nothing)."""
    from carbot_common import topics as T
    from carbot_common.qos import UWB
    from std_msgs.msg import String
    feed = UwbFeed(buffer_s, rate_window_s)
    node.create_subscription(String, T.UWB_INPUT_JSON, lambda m: feed.add(m.data), UWB)
    return feed


# ---------------------------------------------------------------------- pure helpers
def session_uwb(session: Optional[str], base_doc: Dict) -> Dict:
    """<session>/data/uwb.yaml if a step already saved it (step 10 anchors + offsets), else base_doc."""
    if session:
        p = os.path.join(session, 'data', 'uwb.yaml')
        if os.path.isfile(p):
            with open(p, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f) or {}
    return base_doc


def anchor_set(doc: Dict, zero_offsets: bool = False) -> AnchorSet:
    if zero_offsets:
        doc = dict(doc, anchors=[dict(a, range_offset_m=0.0) for a in doc['anchors']])
    return AnchorSet.from_yaml(doc)


fresh_samples = _cli.raw_fresh_samples      # rows -> ({anchor: [raw R m, one per new sample]}, unknown, reports)
fixes = _cli.fixes                          # (AnchorSet, rows) -> [(x, y)] pairwise trilateration per report
