"""Step 12 (mission planner) page logic, no ROS.

The team's tools/map/mission_planner.py is imported for real (map, road, fit
check, mission.yaml format); its slow parts (lane graph, plan_leg: ~1 min per
leg) are replaced by fakes except in test_real_planner_one_leg. The race-time
check (carbot_planning.route_core) runs for real on the repo mission.
"""
import copy
import json
import math
import os
import shutil
import sys
import threading
import time
import types

import pytest
import yaml

from carbot_common import calibration_store as cs
from carbot_ops import step_mission_planner as smp
from carbot_ops import wizard_core as wc
from helpers import DATA, STEPS

pytest.importorskip('cv2')          # map_builder.road_field (the planner's road raster)

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
CONFIG = os.path.dirname(DATA)
STEP12 = next(s for s in STEPS['steps'] if s['id'] == 'mission_planner')
MAP = os.path.join(DATA, 'track_map.yaml')
with open(os.path.join(DATA, 'mission.yaml'), encoding='utf-8') as _f:
    REPO_MISSION = yaml.safe_load(_f)
POSES = [[p['x'], p['y'], p['yaw_deg']] for p in REPO_MISSION['poses']]
ARG = json.dumps({'poses': POSES})
REAL = smp.load_planner(STEP12['procedure']['planner_dir'], CONFIG)


class Planner(types.SimpleNamespace):
    """The real planner module with a fake lane graph + plan_leg (straight line, records calls)."""

    def __init__(self, fail_leg=None, block=None):
        super().__init__(calls=[], fail_leg=fail_leg, block=block)

    def __getattr__(self, k):
        return getattr(REAL, k)

    def LaneGraph(self, road):  # noqa: N802
        return 'graph'

    def plan_leg(self, road, graph, a, b, cancel=None):
        self.calls.append((a, b))
        if self.block is not None:
            while not self.block.is_set():
                if cancel and cancel():
                    raise REAL.Cancelled()
                time.sleep(0.01)
        leg = len(self.calls) - 1
        if self.fail_leg is not None and (a, b) == self.fail_leg:
            return {'ok': False, 'reason': 'no feasible path (fake)', 'seconds': 0.1}
        path = [(a[0] + (b[0] - a[0]) * t / 10, a[1] + (b[1] - a[1]) * t / 10, a[2], 1) for t in range(11)]
        return {'ok': True, 'pieces': [{'kind': 'road', 'path': path, 'nodes': 1}],
                'length_m': math.hypot(b[0] - a[0], b[1] - a[1]), 'sections': ['start_lane'],
                'roundabout_exits': ['west'] if leg == 0 else [], 'reason': 'ok', 'seconds': 0.1}


class Race:
    def __init__(self, ok=True, reason='ok'):
        self.ok, self.reason, self.docs = ok, reason, []

    def __call__(self, map_path, doc):
        self.docs.append(doc)
        return {'ok': self.ok, 'reason': self.reason, 'warnings': [], 'visits': []}


def make(session=None, planner=None, race=None, cfg=STEP12):
    return smp.MissionPlannerStep(cfg, CONFIG, lambda: session, planner or Planner(), race or Race())


def finish(step, arg=ARG, limit=30.0):
    err = step.start(0.0, {'argument': arg})
    assert err is None, err
    t0 = time.time()
    while time.time() - t0 < limit:
        res = step.tick(1.0, {})
        if res is not None:
            return res
        time.sleep(0.01)
    raise AssertionError('planning did not finish')


def check(res, label):
    return next(c for c in res['checks'] if c['label'].startswith(label))


# ------------------------------------------------------------------ config + arguments
def test_missing_key_is_loud():
    cfg = copy.deepcopy(STEP12)
    del cfg['procedure']['planner_dir']
    with pytest.raises(smp.ConfigError, match='planner_dir'):
        make(cfg=cfg)
    cfg = copy.deepcopy(STEP12)
    del cfg['pass']['block07_check']
    with pytest.raises(smp.ConfigError, match='block07_check'):
        make(cfg=cfg)


def test_step12_is_required_and_configured():
    assert STEP12['index'] == 12 and STEP12['required'] is True
    assert smp.find_planner_dir(STEP12['procedure']['planner_dir'], CONFIG)


def test_parse_poses_forms_and_errors():
    st = make()
    ps, err = st.parse_poses('')
    assert not err and ps == POSES                                     # repo mission.yaml
    ps, err = st.parse_poses(json.dumps({'poses': [{'x': 1, 'y': 2, 'yaw_deg': 90}] * 4}))
    assert not err and ps == [[1.0, 2.0, 90.0]] * 4
    assert 'JSON' in st.parse_poses('P0 here')[1]
    assert 'Need 4 poses' in st.parse_poses(json.dumps({'poses': POSES[:3]}))[1]
    assert 'P1' in st.parse_poses(json.dumps({'poses': [POSES[0], ['a', 1, 2]] + POSES[2:]}))[1]
    assert st.start(0.0, {'argument': '{"poses": 3}'}) is not None


# ------------------------------------------------------------------ planning
def test_pass_with_repo_poses_and_mission_records_map(tmp_path):
    race = Race()
    st = make(planner=Planner(), race=race)
    res = finish(st)
    assert res['passed'], res['summary']
    assert res['map']['source'] == 'repo' and res['map']['sha1'] == smp.fingerprints(MAP)['raw']
    assert [lg['ok'] for lg in res['legs']] == [True] * 3
    assert res['legs'][0]['roundabout_exits'] == ['west']
    assert 'exits leg1: west' in res['summary']
    assert check(res, 'P2 in parallel_bay')['passed'] and check(res, 'P3 in perpendicular_bay')['passed']
    assert check(res, 'Race route check')['passed']
    doc = race.docs[0]                                   # what block 07 checked = what Save writes
    assert doc['map'] == {'file': 'track_map.yaml', 'sha1': smp.fingerprints(MAP)['raw']}
    lv = st.live({})
    assert lv['plan']['from'] == 'this run' and len(lv['plan']['routes']) == 3
    assert lv['track']['sections'] and lv['track']['areas']['parallel_bay']['kind'] == 'bay'
    # Save
    sess = str(tmp_path / 'S1')
    paths = st.save_data(sess, res)
    assert paths == [os.path.join(sess, 'data', 'mission.yaml')]
    with open(paths[0], encoding='utf-8') as f:
        saved = yaml.safe_load(f)
    assert saved == doc and saved['version'] == 2 and len(saved['poses']) == 4
    with pytest.raises(wc.StepRefused, match='Nothing planned'):
        st.save_data(sess, res)                          # pending is consumed


def test_off_road_pose_fails_and_its_legs_are_not_planned():
    pl = Planner()
    st = make(planner=pl)
    poses = copy.deepcopy(POSES)
    poses[1] = [0.2, 0.2, 0.0]                            # far outside the track
    res = finish(st, json.dumps({'poses': poses}))
    assert not res['passed']
    c = check(res, 'P1 fits')
    assert not c['passed'] and 'off the road' in c['why'] and c['fix']
    assert len(pl.calls) == 1                             # only leg3 (P2 -> P3) planned
    assert [lg['ok'] for lg in res['legs']] == [False, False, True]
    assert 'not planned: P1' in res['legs'][0]['reason']
    assert not check(res, 'Race route check')['passed']   # not run: a leg has no route
    assert st.pending is None
    with pytest.raises(wc.StepRefused):
        st.save_data('/nowhere', res)


def test_parking_pose_must_be_in_its_bay():
    st = make()
    poses = copy.deepcopy(POSES)
    poses[2] = POSES[1]                                   # P2 on the road, not in parallel_bay
    res = finish(st, json.dumps({'poses': poses}))
    c = check(res, 'P2 in parallel_bay')
    assert not res['passed'] and not c['passed'] and 'mission_rules.yaml' in c['why']


def test_leg_without_route_fails():
    a = tuple([POSES[1][0], POSES[1][1], math.radians(POSES[1][2])])
    b = tuple([POSES[2][0], POSES[2][1], math.radians(POSES[2][2])])
    st = make(planner=Planner(fail_leg=(a, b)))
    res = finish(st)
    c = check(res, 'leg2 route')
    assert not res['passed'] and not c['passed'] and 'no feasible path' in c['why']


def test_race_check_refusal_fails_the_step():
    st = make(race=Race(False, 'roundabout visit 1: route leaves by north on leg1, mission rules plan west on leg1'))
    res = finish(st)
    c = check(res, 'Race route check')
    assert not res['passed'] and not c['passed'] and 'mission rules plan west' in c['why']


def test_unchanged_legs_are_reused():
    pl = Planner()
    st = make(planner=pl)
    finish(st)
    assert len(pl.calls) == 3
    finish(st)
    assert len(pl.calls) == 3                             # same map, same poses: nothing replanned
    poses = copy.deepcopy(POSES)
    poses[3][0] += 0.02                                   # move P3 only -> leg3 only
    finish(st, json.dumps({'poses': poses}))
    assert len(pl.calls) == 4


def test_cancel_stops_planning():
    ev = threading.Event()
    pl = Planner(block=ev)
    st = make(planner=pl)
    assert st.start(0.0, {'argument': ARG}) is None
    t0 = time.time()
    while not pl.calls and time.time() - t0 < 10:
        time.sleep(0.01)
    assert st.progress(1.0)['summary'].startswith('planning: leg 1')
    th = st.run['thread']
    st.cancel()
    th.join(5)
    assert not th.is_alive() and st.tick(2.0, {}) is None and st.pending is None


# ------------------------------------------------------------------ session map, save + keep
def _session_with_map(tmp_path, name, edit=False, crlf=False):
    sess = tmp_path / name
    (sess / 'data').mkdir(parents=True)
    raw = open(MAP, 'rb').read()
    if edit:
        raw = raw.replace(b'version: 2', b'version: 2\n# edited by step 11', 1)
    if crlf:
        raw = raw.replace(b'\r\n', b'\n').replace(b'\n', b'\r\n')
    (sess / 'data' / 'track_map.yaml').write_bytes(raw)
    return str(sess)


def test_session_map_is_used_and_recorded(tmp_path):
    sess = _session_with_map(tmp_path, 'S1', edit=True)
    race = Race()
    st = make(session=sess, race=race)
    res = finish(st)
    own = os.path.join(sess, 'data', 'track_map.yaml')
    assert res['passed'] and res['map']['source'] == 'session' and res['map']['path'] == os.path.abspath(own)
    assert race.docs[0]['map']['sha1'] == smp.fingerprints(own)['raw'] != smp.fingerprints(MAP)['raw']
    assert st.live({})['map']['source'] == 'session'


def test_save_refused_when_map_changed_after_run(tmp_path):
    sess = _session_with_map(tmp_path, 'S1')
    st = make(session=sess)
    res = finish(st)
    with open(os.path.join(sess, 'data', 'track_map.yaml'), 'ab') as f:
        f.write(b'# step 11 redone\n')
    with pytest.raises(wc.StepRefused, match='different track_map'):
        st.save_data(sess, res)


def test_keep_previous_copies_or_refuses(tmp_path):
    old = str(tmp_path / 'OLD')
    st = make(session=old)
    res = finish(st)
    st.save_data(old, res)
    # same (repo) map, even stored with CRLF in the new session: kept
    new = _session_with_map(tmp_path, 'NEW', crlf=True)
    assert st.keep_data(old, new) == [os.path.join(new, 'data', 'mission.yaml')]
    assert open(os.path.join(new, 'data', 'mission.yaml'), 'rb').read() == \
        open(os.path.join(old, 'data', 'mission.yaml'), 'rb').read()
    # the new session races on a different map: refused, nothing copied
    other = _session_with_map(tmp_path, 'OTHER', edit=True)
    with pytest.raises(wc.StepRefused, match='different track_map'):
        st.keep_data(old, other)
    assert not os.path.exists(os.path.join(other, 'data', 'mission.yaml'))
    # nothing to keep
    empty = tmp_path / 'EMPTY'
    empty.mkdir()
    with pytest.raises(wc.StepRefused, match='no data/mission.yaml'):
        st.keep_data(str(empty), new)


def test_wizard_run_save_keep(tmp_path):
    steps = copy.deepcopy(STEPS)
    for s in steps['steps']:
        s['required'] = s['id'] == 'mission_planner'
    holder = {}
    impl = make(planner=Planner())
    impl.session = lambda: holder['w'].session
    cfg = {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True, 'resume_max_age_h': 0.0, 'page_watch_s': 8.0}
    w = wc.Wizard(steps, str(tmp_path), cfg, {'mission_planner': impl})
    holder['w'] = w
    assert w.action('mission_planner', 'RUN', ARG, {})['ok']
    t0 = time.time()
    while w.tick({}) is None and time.time() - t0 < 30:
        time.sleep(0.01)
    s = w.slot('mission_planner')
    assert s.status == 'PASS', s.result['summary']
    r = w.action('mission_planner', 'SAVE', '', {})
    assert r['ok'] and 'data/mission.yaml' in r['message'] and 'ACTIVE' in r['message']
    first = w.session
    assert os.path.isfile(os.path.join(first, 'data', 'mission.yaml'))
    assert cs.load_summary(first)['steps']['mission_planner']['status'] == 'PASS'
    live = w.live({})['step']['live']
    assert live['plan']['from'] == 'this run'
    # a new wizard (new session): Keep previous copies mission.yaml (same repo map)
    impl2 = make(planner=Planner())
    w2 = wc.Wizard(steps, str(tmp_path), cfg, {'mission_planner': impl2})
    impl2.session = lambda: w2.session
    r = w2.action('mission_planner', 'KEEP_PREVIOUS', '', {})
    assert r['ok'], r['message']
    assert os.path.isfile(os.path.join(w2.session, 'data', 'mission.yaml')) and w2.session != first
    assert impl2.live({})['plan']['from'] == 'saved mission.yaml'


# ------------------------------------------------------------------ the real checks
def _with_planning_path():
    p = os.path.join(SRC, 'carbot_planning')
    if p not in sys.path:
        sys.path.insert(0, p)


def test_block07_check_accepts_repo_mission_and_refuses_wrong_exit():
    _with_planning_path()
    r = smp.block07_check(CONFIG, None, MAP, os.path.join(DATA, 'track_features.yaml'),
                          os.path.join(DATA, 'mission_rules.yaml'), REPO_MISSION)
    assert r['ok'], r['reason']
    assert [(v['leg'], v['exit']) for v in r['visits']] == [('leg1', 'west'), ('leg2', 'north')]
    bad = copy.deepcopy(REPO_MISSION)
    bad['legs'][0]['pieces'] = bad['legs'][1]['pieces']
    r = smp.block07_check(CONFIG, None, MAP, os.path.join(DATA, 'track_features.yaml'),
                          os.path.join(DATA, 'mission_rules.yaml'), bad)
    assert not r['ok'] and 'mission rules plan west' in r['reason']


def test_real_planner_one_leg():
    """P0 -> P1 with the real lane graph + hybrid search (about 10 s on a laptop)."""
    _with_planning_path()
    cfg = copy.deepcopy(STEP12)
    cfg['procedure']['legs'] = 1
    st = smp.MissionPlannerStep(cfg, CONFIG, lambda: None, REAL)
    res = finish(st, json.dumps({'poses': POSES[:2]}), limit=300)
    leg = res['legs'][0]
    assert leg['ok'] and leg['roundabout_exits'] == ['west'], leg
    assert check(res, 'P0 fits')['passed'] and check(res, 'P1 fits')['passed']
    # block 07: a one-leg mission visits the roundabout once, mission_rules plans two visits
    c = check(res, 'Race route check')
    assert not c['passed'] and 'visits the roundabout 1 times' in c['why']
