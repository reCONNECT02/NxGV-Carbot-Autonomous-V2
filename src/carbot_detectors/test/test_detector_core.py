"""bpu_detector core: YOLO11 decode on synthetic BPU outputs, class mapping,
debounce, bearing/range (no ROS, no BPU)."""
import math
import os
import random

import numpy as np
import pytest
import yaml
from carbot_detectors import detector_core as dc

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, '..', '..', 'carbot_bringup', 'config', 'params', 'detectors.yaml')
NC, REG, N = 14, 16, 640


def yaml_spec():
    p = yaml.safe_load(open(PARAMS))['bpu_detector']['ros__parameters']
    return dc.ModelSpec.from_yaml(p['class_names'], p['class_map'], p['thresholds'], p['info_threshold'],
                                  p['input_width'], p['reg_max'], p['nms_iou']), p


def fake_outputs(objects, nchw=False, shuffle=False):
    """objects: (class_id, stride, gx, gy, l, t, r, b) in cells. Returns 6 tensors."""
    outs = []
    for s in (8, 16, 32):
        g = N // s
        cls = np.full((1, g, g, NC), -12.0, np.float32)
        box = np.zeros((1, g, g, 4 * REG), np.float32)
        for c, st, gx, gy, *ltrb in objects:
            if st != s:
                continue
            cls[0, gy, gx, c] = 6.0                                     # sigmoid ~ 0.998
            for e, d in enumerate(ltrb):                                # one-hot DFL at bin d
                box[0, gy, gx, e * REG + int(d)] = 20.0
        if nchw:
            cls, box = np.transpose(cls, (0, 3, 1, 2)), np.transpose(box, (0, 3, 1, 2))
        outs += [cls, box]
    if shuffle:
        random.Random(3).shuffle(outs)
    return outs


def test_yaml_lists_match_the_team_model():
    spec, p = yaml_spec()
    assert len(spec.class_names) == NC == len(p['class_map'])
    assert spec.class_names[9:14] == ['traffic_red', 'traffic_yellow', 'traffic_green', 'boom_closed', 'boom_open']
    assert p['class_map'][9:14] == ['traffic_light_red', 'traffic_light_yellow', 'traffic_light_green',
                                    'boom_gate_closed', 'boom_gate_open']
    assert p['class_map'][6] == ''                                       # bump sign not trained yet
    assert 'tunnel_detected' not in open(PARAMS).read()


@pytest.mark.parametrize('nchw,shuffle', [(False, False), (True, False), (False, True), (True, True)])
def test_decode_recovers_the_box_any_layout_any_order(nchw, shuffle):
    spec, _ = yaml_spec()
    # boom_closed at stride 16, cell (20, 10), l t r b = 3 1 4 2 cells
    outs = fake_outputs([(12, 16, 20, 10, 3, 1, 4, 2)], nchw=nchw, shuffle=shuffle)
    dets = dc.detect(outs, spec, 640, 480)
    assert len(dets) == 1 and dets[0].name == 'boom_gate_closed' and dets[0].score > 0.99
    x1, y1, x2, y2 = dets[0].box
    exp = ((20.5 - 3) * 16, (10.5 - 1) * 16, (20.5 + 4) * 16, (10.5 + 2) * 16)
    sx, sy = 640 / 640, 480 / 640
    assert math.isclose(x1, exp[0] * sx, abs_tol=0.5) and math.isclose(y1, exp[1] * sy, abs_tol=0.5)
    assert math.isclose(x2, exp[2] * sx, abs_tol=0.5) and math.isclose(y2, exp[3] * sy, abs_tol=0.5)


def test_unmapped_class_is_dropped_and_nms_merges_duplicates():
    spec, _ = yaml_spec()
    outs = fake_outputs([(6, 8, 10, 10, 2, 2, 2, 2),                    # speed bump: mapped to ""
                         (11, 8, 40, 30, 3, 3, 3, 3), (11, 8, 41, 30, 3, 3, 3, 3)])   # two greens, same lamp
    dets = dc.detect(outs, spec, 640, 640)
    assert [d.name for d in dets] == ['traffic_light_green']


def test_wrong_class_count_is_a_model_mismatch():
    spec, _ = yaml_spec()
    bad = dc.ModelSpec(spec.class_names[:10], spec.class_map[:10], spec.thresholds[:10])
    with pytest.raises(dc.ModelMismatch):
        dc.detect(fake_outputs([]), bad, 640, 480)
    with pytest.raises(dc.ModelMismatch):
        dc.detect(fake_outputs([])[:4], spec, 640, 480)


def test_debounce_needs_consecutive_frames_and_expires():
    d = dc.Debounced(3, 0.4)
    assert d.update('RED', 0.0) == 'UNKNOWN' and d.update('RED', 0.1) == 'UNKNOWN'
    assert d.update('', 0.2) == 'UNKNOWN'                # the team counter would have said RED here
    assert d.update('RED', 0.3) == 'RED'                 # 3rd sighting; one missed frame tolerated
    d = dc.Debounced(3, 0.4)
    assert d.update('RED', 0.0) == 'UNKNOWN' and d.update('RED', 0.1) == 'UNKNOWN'
    assert d.update('', 0.6) == 'UNKNOWN'                # gap > expiry: count reset
    assert d.update('RED', 0.7) == 'UNKNOWN'
    assert d.update('RED', 0.8) == 'UNKNOWN' and d.update('RED', 0.9) == 'RED'
    assert d.update('GREEN', 1.0) == 'RED' and d.update('GREEN', 1.1) == 'RED'   # RED held until GREEN confirmed
    assert d.update('GREEN', 1.2) == 'GREEN'
    assert d.update('', 1.5) == 'GREEN' and d.update('', 1.7) == 'UNKNOWN'


def test_frame_rules_are_conservative():
    D = lambda n: dc.Det(0, n, 0.9, (0, 0, 1, 1))      # noqa: E731
    assert dc.frame_light([D('traffic_light_green'), D('traffic_light_red')]) == 'RED'
    assert dc.frame_light([D('traffic_light_yellow')]) == 'RED'
    assert dc.frame_light([D('traffic_light_green')]) == 'GREEN'
    assert dc.frame_light([D('hill_sign')]) == ''
    assert dc.frame_gate([D('boom_gate_open'), D('boom_gate_closed')]) == 'CLOSED'
    assert dc.frame_gate([D('boom_gate_open')]) == 'OPEN'


def test_locate_bearing_and_range():
    cam = dc.CameraGeom.from_hfov(640, 60.0, mount_x=0.217, mount_y=0.0)
    sizes = {'boom_gate_closed': {'width_m': 0.375}}
    fx = 320 / math.tan(math.radians(30))
    px = fx * 0.375 / 0.6                                   # arm 0.6 m in front of the camera
    centre = dc.Det(12, 'boom_gate_closed', 0.9, (320 - px / 2, 200, 320 + px / 2, 210))
    x, y, r = dc.locate(centre, cam, sizes)
    assert math.isclose(r, 0.6, rel_tol=1e-6) and math.isclose(x, 0.817, abs_tol=1e-6) and abs(y) < 1e-9
    left = dc.Det(13, 'hill_sign', 0.9, (0, 0, 20, 20))    # no size -> unit bearing, to the LEFT (+y)
    x, y, r = dc.locate(left, cam, sizes)
    assert r == -1.0 and y > 0 and math.isclose(math.hypot(x, y), 1.0)


# --------------------------------------------------------------------------- faster decode (BACKLOG #52)
def _decode_level_reference(cls, box, stride, thresh, reg_max):
    """The original implementation (sigmoid over every class score first)."""
    scores = 1.0 / (1.0 + np.exp(-np.clip(cls, -30.0, 30.0)))
    cid = np.argmax(scores, axis=-1)
    best = np.take_along_axis(scores, cid[..., None], axis=-1)[..., 0]
    ys, xs = np.where(best >= thresh[cid])
    ltrb = box[ys, xs].reshape(-1, 4, reg_max)
    ltrb = ltrb - ltrb.max(axis=-1, keepdims=True)
    e = np.exp(ltrb)
    off = (e / e.sum(axis=-1, keepdims=True) * np.arange(reg_max, dtype=np.float32)).sum(axis=-1)
    gx, gy = xs.astype(np.float32) + 0.5, ys.astype(np.float32) + 0.5
    boxes = np.stack([(gx - off[:, 0]) * stride, (gy - off[:, 1]) * stride,
                      (gx + off[:, 2]) * stride, (gy + off[:, 3]) * stride], axis=1)
    return boxes.astype(np.float32), best[ys, xs].astype(np.float32), cid[ys, xs].astype(np.int32)


@pytest.mark.parametrize('seed', [0, 1, 2])
def test_decode_level_matches_the_original(seed):
    rng = np.random.default_rng(seed)
    nc, reg = 14, 16
    thresh = np.array([0.25] * 3 + [0.35, 0.5, 1.01, 0.25, 0.5, 0.25, 0.25, 0.25, 0.25, 0.35, 0.35], np.float32)
    for hw, stride in ((40, 16), (20, 32)):
        cls = rng.normal(-4.0, 3.0, (hw, hw, nc)).astype(np.float32)       # a few cells score high
        box = rng.normal(0.0, 2.0, (hw, hw, 4 * reg)).astype(np.float32)
        got = dc.decode_level(cls, box, stride, thresh, reg)
        want = _decode_level_reference(cls, box, stride, thresh, reg)
        assert len(want[1]) > 0 and len(got[1]) == len(want[1])
        for a, b in zip(got, want):
            np.testing.assert_allclose(a, b, rtol=1e-5, atol=1e-4)
        assert (got[2] != 5).all()                                          # class with threshold 1.01 never passes


def test_decode_level_nothing_passes():
    cls = np.full((8, 8, 14), -20.0, np.float32)
    b, s, c = dc.decode_level(cls, np.zeros((8, 8, 64), np.float32), 8, np.full(14, 0.5, np.float32), 16)
    assert b.shape == (0, 4) and s.shape == (0,) and c.shape == (0,)
