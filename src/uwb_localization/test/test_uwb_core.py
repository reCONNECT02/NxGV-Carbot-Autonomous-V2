"""UWB parsing / processing against UWB_Handoff.md rules (no ROS)."""
import json
import math
import os

import pytest
import yaml

from uwb_localization.uwb_core import (AnchorSet, LatencyFilter, RangeProcessor, compute_offsets,
                                       fix_clusters, hdop, layout_checks, parse_report, trilaterate)

UWB = os.path.join(os.path.dirname(__file__), '..', '..', 'carbot_bringup', 'config', 'data', 'uwb.yaml')
EXAMPLE = ('{"tag":"RISA01","boot_id":"8543AB52","seq":15772,"t_ms":1577491,"last_unknown_id":"0000",'
           '"links":[{"A":"1786","R":2.5800,"age_ms":51,"sample_seq":6567},'
           '{"A":"1782","R":2.1800,"age_ms":65,"sample_seq":6023},'
           '{"A":"1783","R":2.8900,"age_ms":79,"sample_seq":6040}]}')


@pytest.fixture
def anchors():
    with open(UWB) as f:
        return AnchorSet.from_yaml(yaml.safe_load(f))


def rep(links, boot='B', t_ms=1000, unknown='0000'):
    return parse_report(json.dumps({'tag': 'T', 'boot_id': boot, 'seq': 1, 't_ms': t_ms,
                                    'last_unknown_id': unknown, 'links': links}))


def test_handoff_example_parses():
    r = parse_report(EXAMPLE)
    assert r.boot_id == '8543AB52' and len(r.links) == 3
    assert r.links[0].anchor == '1786' and r.links[0].r == pytest.approx(2.58)


def test_garbage_is_none():
    assert parse_report('not json') is None and parse_report('[1,2]') is None


def test_yaml_anchors_in_metres(anchors):
    assert anchors.ids == ['1782', '1783', '1786']
    assert anchors.anchors['1783'].x == pytest.approx(7.5) and anchors.anchors['1783'].y == pytest.approx(4.83)


def test_empty_links_is_no_measurement(anchors):
    p = RangeProcessor(anchors, 400, 0.05, 30, 'arrival')
    out = p.process(rep([]), 10.0)
    assert out.ranges == []
    assert trilaterate(anchors, {}) is None             # never (0, 0)


def test_repeated_sample_seq_not_fresh(anchors):
    p = RangeProcessor(anchors, 400, 0.05, 30, 'arrival')
    ln = [{'A': '1782', 'R': 2.0, 'age_ms': 10, 'sample_seq': 5}]
    assert p.process(rep(ln), 1.0).ranges[0].fresh
    second = p.process(rep(ln, t_ms=1100), 1.1).ranges[0]
    assert not second.fresh and second.reason == 'repeat'


def test_reboot_resets_sequence(anchors):
    p = RangeProcessor(anchors, 400, 0.05, 30, 'arrival')
    ln = [{'A': '1782', 'R': 2.0, 'age_ms': 10, 'sample_seq': 5}]
    p.process(rep(ln, boot='A'), 1.0)
    out = p.process(rep(ln, boot='B'), 1.1)
    assert out.rebooted and out.ranges[0].fresh and p.reboots == 1


def test_old_and_out_of_range(anchors):
    p = RangeProcessor(anchors, 400, 0.05, 30, 'arrival')
    out = p.process(rep([{'A': '1782', 'R': 2.0, 'age_ms': 500, 'sample_seq': 1},
                         {'A': '1786', 'R': 40.0, 'age_ms': 5, 'sample_seq': 1}]), 1.0)
    assert [r.reason for r in out.ranges] == ['old', 'out_of_range']


def test_unknown_anchor_reported(anchors):
    p = RangeProcessor(anchors, 400, 0.05, 30, 'arrival')
    out = p.process(rep([{'A': 'ABCD', 'R': 2.0, 'age_ms': 5, 'sample_seq': 1}]), 1.0)
    assert out.unknown_ids == ['ABCD'] and out.ranges == []


def test_offset_and_flatten():
    doc = {'anchors': [{'id': '1', 'xyz_m': [0, 0, 1.2], 'range_offset_m': 1.0},
                       {'id': '2', 'xyz_m': [5, 0, 1.2]}, {'id': '3', 'xyz_m': [5, 4, 1.2]}],
           'tag': {'z_m': 0.2}}
    a = AnchorSet.from_yaml(doc)
    p = RangeProcessor(a, 400, 0.05, 30, 'arrival')
    r = p.process(rep([{'A': '1', 'R': 1.0 + math.hypot(3.0, 1.0), 'age_ms': 0, 'sample_seq': 1}]), 1.0)
    assert r.ranges[0].corrected_m == pytest.approx(3.0)


def test_stamp_is_arrival_minus_age(anchors):
    p = RangeProcessor(anchors, 400, 0.05, 30, 'arrival')
    r = p.process(rep([{'A': '1782', 'R': 2.0, 'age_ms': 80, 'sample_seq': 1}]), 10.0).ranges[0]
    assert r.stamp == pytest.approx(9.92)


def test_latency_min_filter_removes_jitter():
    f = LatencyFilter(10.0)
    offs = [f.update(100.0 + i * 0.1 + d, i * 0.1) for i, d in enumerate([0.12, 0.05, 0.09, 0.13, 0.06])]
    assert offs[-1] == pytest.approx(100.05)            # least-delayed packet wins


def test_trilateration_matches_truth(anchors):
    x, y = 4.0, 1.5
    r = {k: math.hypot(x - a.x, y - a.y) for k, a in anchors.anchors.items()}
    assert trilaterate(anchors, r) == pytest.approx((x, y), abs=1e-6)


def test_trilateration_needs_three(anchors):
    assert trilaterate(anchors, {'1782': 2.0, '1786': 5.0}) is None


def test_hdop_and_layout(anchors):
    assert hdop(anchors, 5.0, 1.5) < 2.0
    probs = layout_checks(anchors, 2.0, 25.0)
    assert len(probs) == 1 and 'floor' in probs[0]     # only the z = 0 warning
    close = AnchorSet.from_yaml({'anchors': [{'id': 'a', 'xyz_m': [0, 0, 1]}, {'id': 'b', 'xyz_m': [0.2, 0, 1]},
                                             {'id': 'c', 'xyz_m': [0.2, 0.18, 1]}], 'tag': {'z_m': 0}})
    assert any('apart' in p for p in layout_checks(close, 2.0, 25.0))
    line = AnchorSet.from_yaml({'anchors': [{'id': 'a', 'xyz_m': [0, 0, 1]}, {'id': 'b', 'xyz_m': [3, 0, 1]},
                                            {'id': 'c', 'xyz_m': [6, 0.1, 1]}], 'tag': {'z_m': 0}})
    assert any('line' in p for p in layout_checks(line, 2.0, 25.0))


def test_offsets_like_uwb_calib(anchors):
    raw = {k: [anchors.true_range_3d(k, 5.0, 1.5) + 1.0 + e for e in (-0.01, 0.0, 0.01)] * 10
           for k in anchors.ids}
    res = compute_offsets(anchors, raw, 5.0, 1.5)
    assert all(v.offset_m == pytest.approx(1.0, abs=1e-6) for v in res.values())
    assert compute_offsets(anchors, {'1782': [1.0] * 5}, 5, 1.5) == {}   # too few samples


def test_flip_flop_detection():
    import random
    rng = random.Random(1)
    noise = [(rng.gauss(0, .03), rng.gauss(0, .03)) for _ in range(200)]
    assert fix_clusters(noise)[1] == 0.0
    flip = [(p[0] + (0.25 if i % 3 == 0 else 0), p[1]) for i, p in enumerate(noise)]
    assert fix_clusters(flip)[1] == pytest.approx(0.25, abs=0.05)
