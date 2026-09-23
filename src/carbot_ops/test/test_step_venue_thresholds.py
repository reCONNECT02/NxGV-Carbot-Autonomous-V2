"""Step 9 (venue colour / lighting thresholds) page logic, no ROS.

A synthetic venue: the front camera sees the floor from x ~0.47 m (as the CAD
mount does), a lighter-than-V4 grey road, two cream tape lines at +-0.15 m and
a coloured floor outside the track. road_perception's grid and its stitched
debug image are simulated with the same V4 rule and a real JPEG round trip.
"""
import copy
import os
import sys

import numpy as np
import pytest
import yaml

from carbot_ops import step_venue_thresholds as svt
from carbot_ops import wizard_core as wc
from helpers import REPO_CAMERAS, STEPS

HERE = os.path.dirname(os.path.abspath(__file__))
STEP9 = next(s for s in STEPS['steps'] if s['id'] == 'venue_thresholds')
V4 = {'road_max_luma': 105, 'road_max_chroma': 50, 'paint_min_luma': 190}
N, RES, X0, Y0 = 100, 0.018, -0.65, -0.9


# ----------------------------------------------------------------------------- synthetic venue
def scene(road=125, tape=215, floor=(90, 150, 190), noise=4, dark=1.0, glare=0.0, seed=0):
    """(n, n, 3) BGR per cell + seen mask. dark scales every colour (tunnel)."""
    rng = np.random.RandomState(seed)
    xs = X0 + (np.arange(N) + 0.5) * RES
    ys = Y0 + (np.arange(N) + 0.5) * RES
    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    seen = (gx > 0.466) & (np.abs(gy) < 0.15 + 0.5 * (gx - 0.466))
    img = np.empty((N, N, 3), np.float64)
    img[:] = road
    tp = (np.abs(gy) > 0.14) & (np.abs(gy) < 0.175)
    img[tp] = tape
    img[np.abs(gy) >= 0.30] = floor
    img += rng.randn(N, N, 1) * noise + rng.randn(N, N, 3) * 1.5     # brightness + a little colour noise
    if glare:
        g = rng.rand(N, N) < glare
        img[g] = 235
    img = np.clip(img * dark, 0, 255).astype(np.uint8)
    img[~seen] = (48, 33, 24)       # road_mask 'unseen' colour
    return img, seen


def grid_of(cells, seen, th):
    luma, chroma = svt.luma_chroma(cells)
    kind = svt.classify(luma, chroma, th)
    kind[~seen] = svt.UNSEEN
    return {'rows': N, 'cols': N, 'res': RES, 'x0': X0, 'y0': Y0, 'kind': kind, 'age_s': 0.1}


def stitched(cells, scale=3, jpeg=True):
    """= road_mask.bev_view(cells, scale) (+ the JPEG road_perception publishes)."""
    img = np.repeat(np.repeat(cells[::-1, ::-1], scale, 0), scale, 1)
    if jpeg:
        import cv2
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return img


class Feed:
    """inputs['road_pair'] / ['road_grid'] of the node, from one scene and the LIVE thresholds."""

    def __init__(self, link, **kw):
        self.link, self.kw, self.seq = link, kw, 0

    def set(self, **kw):
        self.kw = kw

    def _frame(self):
        cells, seen = scene(seed=self.seq, **self.kw)
        return cells, seen, grid_of(cells, seen, self.link.live())

    def pair(self):
        self.seq += 1
        cells, seen, g = self._frame()
        return self.seq, g, svt.stitched_to_cells(stitched(cells), N, N)

    def grid(self):
        return self._frame()[2]


class Link:
    """Fake ParamLink on road_perception (answers synchronously)."""

    def __init__(self, vals=None, alive=True, accept=True):
        self.vals = {f'classify.{k}': v for k, v in (vals or V4).items()}
        self.alive, self.accept, self.sets = alive, accept, []

    def live(self):
        return {k.split('.', 1)[1]: v for k, v in self.vals.items()}

    def get(self, names, done):
        done({n: self.vals.get(n) for n in names} if self.alive else None)

    def set(self, values, done):
        self.sets.append(dict(values))
        if self.alive and self.accept:
            self.vals.update(values)
        done(self.alive and self.accept)


def front_only():
    return copy.deepcopy(REPO_CAMERAS)


def make(cfg=None, cams=None):
    return svt.VenueThresholdsStep(copy.deepcopy(cfg or STEP9), cams or front_only())


def run(step, link, feed, arg, t0=0.0, dt=0.25, limit=400):
    assert step.start(t0, {'argument': arg, 'road_params': link}) is None
    t = t0
    for _ in range(limit):
        t += dt
        res = step.tick(t, {'road_pair': feed.pair, 'road_grid': feed.grid, 'road_params': link})
        if res is not None:
            return res
    raise AssertionError('step never finished')


def full(step=None, link=None, **kw):
    step, link = step or make(), link or Link()
    feed = Feed(link, **kw)
    run(step, link, feed, 'road')
    feed.set(dark=0.35)
    run(step, link, feed, 'tunnel')
    res = run(step, link, feed, 'apply')
    return step, link, feed, res


def check(res, label):
    return next(c for c in res['checks'] if c['label'] == label)


# ----------------------------------------------------------------------------- pure helpers
def test_stitched_to_cells_inverts_bev_view():
    cells, _ = scene()
    assert np.array_equal(svt.stitched_to_cells(stitched(cells, 3, jpeg=False), N, N), cells)
    assert np.array_equal(svt.stitched_to_cells(stitched(cells, 1, jpeg=False), N, N), cells)
    with pytest.raises(ValueError):
        svt.stitched_to_cells(np.zeros((301, 300, 3), np.uint8), N, N)


def test_bev_view_and_classify_match_road_perception():
    """Same flip as road_mask.bev_view, same rule as road_mask.RoadMask.process."""
    cv2 = pytest.importorskip('cv2')
    sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', '..', 'carbot_perception')))
    try:
        rm = pytest.importorskip('carbot_perception.road_mask')
    finally:
        sys.path.pop(0)
    cells, _ = scene(noise=30)
    assert np.array_equal(svt.stitched_to_cells(rm.bev_view(cells, 3), N, N), cells)
    mask = rm.RoadMask(rm.GridSpec(N, RES, X0, Y0), rm.Seed())
    ij = np.meshgrid(np.arange(N, dtype=np.float32), np.arange(N, dtype=np.float32), indexing='ij')
    mask.maps['front'] = rm.CameraMap('front', N, N, ij[1].copy(), ij[0].copy(), np.ones((N, N), np.float32), None, None)
    for th in (V4, {'road_max_luma': 140, 'road_max_chroma': 30, 'paint_min_luma': 170}):
        res = mask.process({'front': cells}, rm.Classify(*(float(th[k]) for k in svt.PARAMS)))
        luma, chroma = svt.luma_chroma(cells)
        assert np.array_equal(res.kind, svt.classify(luma, chroma, th))
    assert cv2 is not None


def test_roi_inside_front_camera_view():
    g = grid_of(*scene(), V4)
    for key in ('roi', 'tunnel_roi'):
        box = svt.roi_mask(g, STEP9['procedure'][key])
        assert box.sum() >= STEP9['procedure']['min_roi_cells']
        assert (g['kind'][box] != svt.UNSEEN).all(), key


# ----------------------------------------------------------------------------- the procedure
def test_lighter_venue_road_passes_with_raised_thresholds():
    step, link, feed, res = full()
    assert res['passed'], res['summary']
    p = res['proposal']
    assert p['road_max_luma'] > 125 > V4['road_max_luma']          # venue road is lighter than V4
    assert p['road_max_luma'] < p['paint_min_luma'] < 215           # between road and tape
    assert all(isinstance(v, int) for v in p.values())
    road = res['samples']['road']
    assert road['before']['road'] < 0.1 and road['after']['road'] > 0.95     # V4 misses it, proposal fixes it
    assert road['after']['paint'] == 0.0
    assert road['decode_agreement'] >= 0.9                          # JPEG round trip still reproduces kinds
    assert link.sets[-1] == {f'classify.{k}': v for k, v in p.items()}
    assert res['params_overlay'] == {'road_perception': {f'classify.{k}': v for k, v in p.items()}}
    assert res['before'] == V4
    assert 'bpu_detector' in res
    assert p['paint_min_luma'] > 165                                # dark tunnel tape ignored ...
    assert any('tunnel tape' in n for n in res['notes'])            # ... and reported
    # live: road_perception now gives the new result in the box
    lv = step.live({'road_grid': feed.grid})
    assert lv['applied_ok'] and lv['now']['road']['road'] is not None
    assert lv['map']['rows'] > 0 and len(lv['map']['kinds']) == lv['map']['rows'] * lv['map']['cols']
    assert lv['map']['boxes']['road'] and lv['map']['boxes']['tunnel']


def test_not_passed_until_every_phase_and_apply():
    step, link = make(), Link()
    feed = Feed(link)
    res = run(step, link, feed, 'road')
    assert not res['passed'] and 'tunnel' in res['summary']
    assert step.live({})['next'] == 'tunnel'
    feed.set(dark=0.35)
    res = run(step, link, feed, '')                 # no argument = next phase (tunnel)
    assert 'tunnel' in res['samples'] and not res['passed']
    assert check(res, 'Applied live on road_perception')['passed'] is False
    assert 'Apply' in res['summary']
    res = run(step, link, feed, '')                 # next = apply
    assert res['passed']


def test_resample_after_apply_needs_apply_again():
    step, link, feed, res = full()
    old = dict(res['proposal'])
    feed.set(road=100)                               # lights changed
    res = run(step, link, feed, 'road')
    assert res['proposal'] != old and step.applied is None
    assert not res['passed']
    assert not check(res, 'Applied live on road_perception')['passed']


def test_tunnel_check_off_skips_tunnel():
    cfg = copy.deepcopy(STEP9)
    cfg['pass']['tunnel_dark_check'] = False
    step, link = make(cfg), Link()
    feed = Feed(link)
    run(step, link, feed, 'road')
    assert step.start(0, {'argument': 'tunnel', 'road_params': link})
    res = run(step, link, feed, 'apply')
    assert res['passed'] and 'tunnel' not in res['samples']


def test_black_tunnel_fails():
    step, link = make(), Link()
    feed = Feed(link)
    run(step, link, feed, 'road')
    feed.set(dark=0.03)
    run(step, link, feed, 'tunnel')
    res = run(step, link, feed, 'apply')
    c = check(res, 'Tunnel: camera sees the floor')
    assert not res['passed'] and not c['passed'] and c['fix']


def test_glare_hides_the_tape_and_fails():
    step, link = make(), Link()
    res = run(step, link, Feed(link, glare=0.08), 'road')
    assert not check(res, 'Lane tape seen beside the box')['passed']
    assert not res['passed']


def test_no_tape_in_view_fails():
    step, link = make(), Link()
    res = run(step, link, Feed(link, tape=125), 'road')
    c = check(res, 'Lane tape seen beside the box')
    assert not c['passed'] and 'lane' in c['fix']
    assert any('no usable lane tape' in n for n in res['notes'])


def test_box_outside_camera_view_fails():
    cfg = copy.deepcopy(STEP9)
    cfg['procedure']['roi'] = {'x_min_m': 0.0, 'x_max_m': 0.3, 'half_width_m': 0.08}   # under the car
    step, link = make(cfg), Link()
    res = run(step, link, Feed(link), 'road')
    assert not check(res, 'Open road sample')['passed']
    assert not res['passed']


def test_road_perception_not_running():
    step, link = make(), Link(alive=False)
    res = run(step, link, Feed(Link()), 'road')
    assert not res['passed'] and 'road_perception' in res['summary']
    assert check(res, 'Last action')['why']


def test_apply_refused_by_node():
    step, link = make(), Link()
    feed = Feed(link)
    run(step, link, feed, 'road')
    link.accept = False
    res = run(step, link, feed, 'apply')
    assert not res['passed'] and 'refused' in res['summary']


def test_no_frames_times_out():
    step, link = make(), Link()

    class Silent(Feed):
        def pair(self):
            return None
    res = run(step, link, Silent(link), 'road')
    assert not res['passed'] and 'stitched frames' in res['summary']


def test_revert_puts_back_original():
    step, link, feed, res = full()
    assert link.live() != V4
    res = run(step, link, feed, 'revert')
    assert link.live() == V4 and step.applied is None and not res['passed']


def test_refusals():
    step, link = make(), Link()
    assert 'Unknown phase' in step.start(0, {'argument': 'bogus', 'road_params': link})
    assert 'Nothing to apply' in step.start(0, {'argument': 'apply', 'road_params': link})
    assert 'Nothing to put back' in step.start(0, {'argument': 'revert', 'road_params': link})
    assert 'parameter link' in step.start(0, {'argument': 'road'})


# ----------------------------------------------------------------------------- config
def test_front_camera_switched_off_is_a_config_error():
    cams = front_only()
    cams['sensors'][cams['roles']['front']]['enabled'] = False
    with pytest.raises(svt.ConfigError, match='switched off'):
        make(cams=cams)


def test_missing_key_is_loud():
    cfg = copy.deepcopy(STEP9)
    del cfg['procedure']['paint_gap']
    with pytest.raises(svt.ConfigError, match='paint_gap'):
        make(cfg)
    cfg = copy.deepcopy(STEP9)
    del cfg['procedure']['roi']['half_width_m']
    with pytest.raises(svt.ConfigError, match='roi'):
        make(cfg)


def test_repo_yaml_keeps_contract():
    assert STEP9['index'] == 9 and STEP9['pass'] == {'min_road_coverage': 0.5, 'max_false_paint_ratio': 0.05,
                                                     'tunnel_dark_check': True}
    assert STEP9['writes'] == ['params_overlay road_perception.classify.*', 'bpu_detector.thresholds.*']
    assert STEP9['instructions'] and set(svt.PROC_KEYS) <= set(STEP9['procedure'])


def test_classify_keys_exist_in_perception_yaml():
    with open(os.path.join(HERE, '..', '..', 'carbot_bringup', 'config', 'params', 'perception.yaml'), encoding='utf-8') as f:
        rp = yaml.safe_load(f)['road_perception']['ros__parameters']['classify']
    assert set(rp) == set(svt.PARAMS) and all(isinstance(v, int) for v in rp.values())


# ----------------------------------------------------------------------------- save / keep through the wizard
def wizard(root, impl):
    doc = {'steps': [dict(copy.deepcopy(STEP9), index=1)]}
    return wc.Wizard(doc, str(root), {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True,
                                      'resume_max_age_h': 0}, {'venue_thresholds': impl})


def test_save_writes_params_overlay_and_keep_copies(tmp_path):
    step, link, feed, res = full()
    wiz = wizard(tmp_path, step)
    s = wiz.slot('venue_thresholds')
    s.result, s.unsaved, s.status = res, True, 'PASS'
    r = wiz.action('venue_thresholds', 'SAVE', '', {})
    assert r['ok'], r['message']
    ov = yaml.safe_load(open(os.path.join(wiz.session, 'params_overlay.yaml')))
    assert ov['road_perception']['ros__parameters']['classify'] == res['proposal']
    assert 'params_overlay.yaml' in r['message']

    # a newer session keeps the values
    step2 = make()
    wiz2 = wizard(tmp_path, step2)
    wiz2.session = None
    wiz2._find_previous()
    assert wiz2.slot('venue_thresholds').previous
    r = wiz2.action('venue_thresholds', 'KEEP_PREVIOUS', '', {})
    assert r['ok'], r['message']
    ov2 = yaml.safe_load(open(os.path.join(wiz2.session, 'params_overlay.yaml')))
    assert ov2['road_perception']['ros__parameters']['classify'] == res['proposal']


def test_keep_refused_without_overlay(tmp_path):
    old, new = tmp_path / 'old', tmp_path / 'new'
    old.mkdir()
    new.mkdir()
    with pytest.raises(wc.StepRefused, match='must be measured now'):
        make().keep_data(str(old), str(new))


def test_save_refused_without_proposal(tmp_path):
    with pytest.raises(wc.StepRefused):
        make().save_data(str(tmp_path), {'passed': True})
