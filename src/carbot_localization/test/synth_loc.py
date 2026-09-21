"""Synthetic localization data for tests and the sandbox (no ROS).

* road grid: rendered from the prior map like V4 scene.js (road where
  clearance >= 0, white paint in -0.10 <= clearance < 0, other beyond)
* tag JSON: ranges to the anchors of a TrackToVenue-transformed tag position,
  with per-anchor offsets (uncalibrated antenna delay), noise and multipath spikes
"""
import json
import math
import os
import random

import numpy as np
import yaml

SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIG = os.path.join(SRC, 'carbot_bringup', 'config')
ANCHORS = {'1782': (0.0, 0.0, 0.0), '1786': (7.5, 0.0, 0.0), '1783': (7.5, 4.83, 0.0)}
OFFSETS = {'1782': 0.98, '1786': 1.05, '1783': 0.92}


def track_map():
    with open(os.path.join(CONFIG, 'data', 'track_map.yaml')) as f:
        return yaml.safe_load(f)


def uwb_doc():
    with open(os.path.join(CONFIG, 'data', 'uwb.yaml')) as f:
        return yaml.safe_load(f)


def render_grid(course, pose, rows=100, res=0.018, x0=-0.65, y0=-0.9):
    lx = x0 + (np.arange(rows) + 0.5) * res
    ly = y0 + (np.arange(rows) + 0.5) * res
    X, Y = np.meshgrid(lx, ly, indexing='ij')
    c, s = math.cos(pose[2]), math.sin(pose[2])
    d = course.clearance(pose[0] + X * c - Y * s, pose[1] + X * s + Y * c)
    kind = np.where(d >= 0, 1, np.where(d >= -0.10, 2, 3)).astype(np.uint8)
    return kind, (kind == 1).astype(np.uint8), lx, ly


def tag_report(venue_xy, seq, t_ms, rng, noise=0.04, spike_p=0.0, offsets=OFFSETS,
               drop=(), boot='B00T'):
    links = []
    for k, (ax, ay, az) in ANCHORS.items():
        if k in drop:
            continue
        seq[k] = seq.get(k, 0) + 1
        r = math.hypot(venue_xy[0] - ax, venue_xy[1] - ay) + offsets[k] + rng.gauss(0, noise)
        if rng.random() < spike_p:
            r += rng.uniform(0.3, 0.8)
        links.append({'A': k, 'R': round(r, 4), 'age_ms': 40, 'sample_seq': seq[k]})
    return json.dumps({'tag': 'RISA01', 'boot_id': boot, 'seq': t_ms // 100, 't_ms': t_ms,
                       'last_unknown_id': '0000', 'links': links})


def outer_loop(course, n_paths=9):
    """Ordered points along the outer loop centrelines (for a synthetic lap)."""
    return np.concatenate(course.paths[:n_paths])


def rng(seed=0):
    return random.Random(seed)
