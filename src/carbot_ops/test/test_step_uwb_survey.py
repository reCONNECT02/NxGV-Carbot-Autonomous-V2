"""Step 10 (UWB anchor survey + offsets) and the shared wizard_uwb feed (no ROS)."""
import copy
import json
import os

import pytest
import yaml

from carbot_common import calibration_store as cs
from carbot_ops import wizard_core as wc
from carbot_ops import wizard_uwb as wu
from carbot_ops.step_uwb_survey import WRITES, ConfigError, UwbSurveyStep, parse_argument
from helpers import STEPS, UWB

CFG = {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True, 'resume_max_age_h': 12.0, 'page_watch_s': 8.0}
_STEP10_YAML = next(s for s in STEPS['steps'] if s['id'] == 'uwb_survey')
STEP10 = dict(_STEP10_YAML, procedure=dict(_STEP10_YAML['procedure'], verify_spot=True))   # tests cover Verify
# the true layout on the "venue": anchors raised to 1.2 m, tag antenna 0.15 m
TRUE = {'1782': (0.0, 0.0, 1.2), '1786': (7.5, 0.0, 1.2), '1783': (7.5, 4.83, 1.2)}
BIAS = {'1782': 0.95, '1786': 1.02, '1783': 0.88}
TAG = {'z_m': 0.15, 'mount_xy_m': [0.12, 0.0]}


def survey_arg(true=TRUE, tag=TAG, drop=()):
    return json.dumps({'stage': 'survey', 'tag': tag,
                       'anchors': [{'id': a, 'xyz_m': list(v)} for a, v in true.items() if a not in drop]})


def arg(stage, spot=None):
    return json.dumps({'stage': stage, **({'spot': list(spot)} if spot else {})})


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Rig:
    """Wizard with step 10 only (index 1), a synthetic tag at 10 Hz and the real feed."""

    def __init__(self, root, uwb=None, clock=None, resume=12.0):
        self.clock = clock or Clock()
        self.uwb = copy.deepcopy(uwb or UWB)
        self.step = UwbSurveyStep(dict(STEP10, index=1), self.uwb)
        self.wiz = wc.Wizard({'steps': [dict(STEP10, index=1)]}, str(root), dict(CFG, resume_max_age_h=resume),
                             {'uwb_survey': self.step}, now=self.clock)
        self.feed = wu.UwbFeed(30.0, 3.0, clock=self.clock)
        self.tag = wu.SyntheticTag(TRUE, TAG['z_m'], BIAS, noise_m=0.01, xy=(5.0, 1.5))
        self.silent = False

    def inputs(self):
        return {wu.INPUT_KEY: self.feed}

    def do(self, action, argument=''):
        return self.wiz.action('uwb_survey', action, argument, self.inputs())

    def run(self, argument, seconds=25.0):
        r = self.do('RUN', argument)
        assert r['ok'], r
        done = None
        for k in range(int(seconds * 10)):
            self.clock.t += 0.1
            if not self.silent:
                self.feed.add(self.tag.report())
            if k % 2 == 0:
                done = self.wiz.tick(self.inputs()) or done
            if done:
                break
        return done

    def full(self, offset_spot=(5.0, 1.5), verify_spot=(3.5, 2.5)):
        assert self.run(arg('link')).result['stages']['link']['status'] == 'PASS'
        assert self.run(survey_arg()).result['stages']['survey']['status'] == 'PASS'
        self.tag.xy = list(offset_spot)
        assert self.run(arg('offsets', offset_spot)).result['stages']['offsets']['status'] == 'PASS'
        self.tag.xy = list(verify_spot)
        return self.run(arg('verify', verify_spot))


def session_uwb(w):
    with open(os.path.join(w.session, 'data', 'uwb.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------- config
def test_repo_yaml_builds_and_writes_list_matches():
    step = UwbSurveyStep(STEP10, UWB)
    assert step.configured == sorted(str(a['id']) for a in UWB['anchors'])
    assert [w.replace('uwb.yaml ', '') for w in STEP10['writes']] == list(WRITES)
    assert 'track_to_venue' not in ' '.join(STEP10['writes'])


@pytest.mark.parametrize('where,key', [('procedure', 'seconds'), ('procedure', 'min_verify_separation_m'),
                                       ('procedure', 'max_extent_m'), ('pass', 'max_verify_error_m')])
def test_missing_step_key_is_config_error(where, key):
    cfg = copy.deepcopy(STEP10)
    del cfg[where][key]
    with pytest.raises(ConfigError, match=key):
        UwbSurveyStep(cfg, UWB)


def test_missing_uwb_key_is_config_error():
    uwb = copy.deepcopy(UWB)
    del uwb['tag']['mount_xy_m']
    with pytest.raises(ConfigError, match='tag.mount_xy_m'):
        UwbSurveyStep(STEP10, uwb)


def test_parse_argument():
    assert parse_argument(arg('link')) == ({'stage': 'link'}, '')
    a, err = parse_argument(survey_arg())
    assert not err and a['tag'] == {'z_m': 0.15, 'mount_xy_m': [0.12, 0.0]} and len(a['anchors']) == 3
    assert parse_argument(json.dumps({'stage': 'offsets', 'spot': '5, 1.5'}))[0]['spot'] == [5.0, 1.5]
    for bad in ('', 'x', '{}', '{"stage": "nope"}', arg('offsets'), json.dumps({'stage': 'verify', 'spot': [1, 'a']}),
                json.dumps({'stage': 'survey', 'anchors': [{'id': '1782', 'xyz_m': [0, 0]}], 'tag': TAG}),
                json.dumps({'stage': 'survey', 'anchors': [{'id': '1', 'xyz_m': [0, 0, 1]}] * 2, 'tag': TAG}),
                json.dumps({'stage': 'survey', 'anchors': [{'id': '1', 'xyz_m': [0, 0, 1]}], 'tag': {'z_m': 'x'}})):
        assert parse_argument(bad)[0] is None, bad


# ---------------------------------------------------------------- happy path + save
def test_full_flow_passes_and_saves_only_step10_keys(tmp_path):
    rig = Rig(tmp_path)
    done = rig.full()
    assert done.status == 'PASS', done.result
    res = done.result
    assert res['passed'] and not res['next']
    off = {a['id']: a['range_offset_m'] for a in res['uwb']['anchors']}
    for aid, b in BIAS.items():
        assert off[aid] == pytest.approx(b, abs=0.01)
    assert res['stages']['verify']['error_m'] <= 0.05
    assert done.message == 'All stages passed: press Save.'
    # a step-11 alignment already in this session's uwb.yaml must survive
    session = rig.wiz._ensure_session()
    prior = copy.deepcopy(UWB)
    prior['track_to_venue'] = {'x_m': 1.5, 'y_m': -0.2, 'yaw_deg': 3.0, 'aligned': True}
    prior['tag']['name'] = 'KEEPME'
    cs.write_yaml(os.path.join(session, 'data', 'uwb.yaml'), prior)
    r = rig.do('SAVE')
    assert r['ok'] and 'data/uwb.yaml' in r['message'], r
    saved = session_uwb(rig.wiz)
    assert saved['track_to_venue'] == prior['track_to_venue']
    assert saved['tag']['name'] == 'KEEPME' and saved['tag']['input_topic'] == UWB['tag']['input_topic']
    assert saved['tag']['z_m'] == 0.15 and saved['tag']['mount_xy_m'] == [0.12, 0.0]
    assert saved['anchors_surveyed'] is True and saved['offsets_calibrated'] is True
    assert [a['id'] for a in saved['anchors']] == ['1782', '1783', '1786']
    assert saved['anchors'][0]['xyz_m'] == [0.0, 0.0, 1.2]
    assert saved['agent'] == UWB['agent']
    # the saved file is what the uwb_ranges node and page 11 load
    aset = wu.anchor_set(wu.session_uwb(rig.wiz.session, UWB))
    assert aset.surveyed and aset.offsets_calibrated and aset.anchors['1786'].offset == pytest.approx(1.02, abs=0.01)
    assert cs.load_summary(rig.wiz.session)['steps']['uwb_survey']['status'] == 'PASS'


def test_intermediate_stage_says_what_is_next(tmp_path):
    rig = Rig(tmp_path)
    done = rig.run(arg('link'))
    assert done.status == 'FAIL' and '1 of 3 stages passed' in done.result['summary']   # offsets optional
    assert done.message == 'Link check passed. Next: Anchor + tag survey.'
    assert not rig.do('SAVE')['ok']


# ---------------------------------------------------------------- order + refusals
def test_order_refusals(tmp_path):
    rig = Rig(tmp_path)
    r = rig.do('RUN', arg('offsets', (5.0, 1.5)))
    assert not r['ok'] and 'pass Link check and Anchor + tag survey first' in r['message']
    assert not rig.do('RUN', arg('verify', (3.5, 2.5)))['ok']
    rig.run(arg('link'))
    low = {k: (v[0], v[1], 0.3) for k, v in TRUE.items()}      # 3-D distance, as calib_uwb.spot_ok
    rig.run(survey_arg(true=low))
    r = rig.do('REDO', arg('offsets', (0.5, 0.3)))
    assert not r['ok'] and 'closer than 1 m' in r['message'] and '1782' in r['message']
    rig.run(survey_arg())
    rig.run(arg('offsets', (5.0, 1.5)))
    r = rig.do('REDO', arg('verify', (5.2, 1.6)))
    assert not r['ok'] and 'from the offset spot' in r['message']
    r = rig.do('REDO', survey_arg(drop=('1783',)))
    assert not r['ok'] and '1783' in r['message'] and 'enter every anchor' in r['message']
    assert not rig.do('REDO', '')['ok']


def test_no_feed_refused(tmp_path):
    rig = Rig(tmp_path)
    r = rig.wiz.action('uwb_survey', 'RUN', arg('link'), {})
    assert not r['ok'] and 'No UWB feed' in r['message']


def test_changing_survey_clears_offsets_and_verify(tmp_path):
    rig = Rig(tmp_path)
    assert rig.full().status == 'PASS'
    moved = dict(TRUE, **{'1783': (7.4, 4.83, 1.2)})
    done = rig.run(survey_arg(true=moved))
    assert done.status == 'FAIL' and done.result['next'] == 'verify'      # offsets optional (Haffiz)
    assert done.result['stages']['offsets'] is None and done.result['stages']['verify'] is None
    # the same survey again keeps them
    rig2 = Rig(tmp_path / 'b')
    rig2.full()
    assert rig2.run(survey_arg()).status == 'PASS'


# ---------------------------------------------------------------- failures
def test_link_fails_on_missing_anchor_and_lists_unknown(tmp_path):
    rig = Rig(tmp_path)
    rig.tag.drop = {'1783'}
    rig.tag.extra = {'17A0': (0.0, 4.83, 1.2)}
    done = rig.run(arg('link'))
    lk = done.result['stages']['link']
    assert lk['status'] == 'FAIL' and '1783' in lk['why'] and lk['unknown_ids'] == ['17A0']
    assert 'RangeProtocol.h' in lk['fix']
    chk = next(c for c in done.result['checks'] if c['key'] == 'link')
    assert not chk['passed'] and chk['why'] and chk['fix']


def test_nothing_received_fails_early(tmp_path):
    rig = Rig(tmp_path)
    rig.run(arg('link'))
    rig.run(survey_arg())
    rig.silent = True
    t0 = rig.clock.t
    done = rig.run(arg('offsets', (5.0, 1.5)))
    assert done.result['stages']['offsets']['status'] == 'FAIL'
    assert 'no UWB tag report' in done.result['stages']['offsets']['why']
    assert rig.clock.t - t0 < STEP10['procedure']['seconds']


def test_survey_geometry_errors(tmp_path):
    rig = Rig(tmp_path)
    line = {'1782': (0.0, 0.0, 1.2), '1786': (7.5, 0.0, 1.2), '1783': (3.75, 0.1, 1.2)}
    sv = rig.run(survey_arg(true=line)).result['stages']['survey']
    assert sv['status'] == 'FAIL' and 'nearly in a line' in sv['why']
    cm = {k: (v[0] * 100, v[1] * 100, v[2]) for k, v in TRUE.items()}
    sv = rig.run(survey_arg(true=cm)).result['stages']['survey']
    assert sv['status'] == 'FAIL' and 'METRES' in sv['why']
    floor = {k: (v[0], v[1], 0.0) for k, v in TRUE.items()}
    sv = rig.run(survey_arg(true=floor)).result['stages']['survey']
    assert sv['status'] == 'PASS' and any('on the floor' in w for w in sv['geometry']['warnings'])


def test_verify_fails_when_the_tag_is_not_where_you_said(tmp_path):
    rig = Rig(tmp_path)
    rig.run(arg('link'))
    rig.run(survey_arg())
    rig.tag.xy = [5.0, 1.5]
    rig.run(arg('offsets', (5.0, 1.5)))
    rig.tag.xy = [3.8, 2.5]                                  # tape says 3.5: 30 cm off
    done = rig.run(arg('verify', (3.5, 2.5)))
    vf = done.result['stages']['verify']
    assert done.status == 'FAIL' and vf['status'] == 'FAIL' and vf['error_m'] > 0.15
    assert 'tape' in vf['why'] and vf['fix']
    assert not rig.do('SAVE')['ok']


def test_new_anchor_in_survey_needs_a_new_link_check(tmp_path):
    rig = Rig(tmp_path)
    rig.run(arg('link'))
    four = dict(TRUE, **{'17A0': (0.0, 4.83, 1.2)})
    sv = rig.run(survey_arg(true=four)).result['stages']['survey']
    assert sv['status'] == 'PASS' and sv['added'] == ['17A0']
    r = rig.do('REDO', arg('offsets', (5.0, 1.5)))
    assert not r['ok'] and 'Link check again' in r['message']
    rig.tag.anchors['17A0'] = four['17A0']
    rig.tag.bias['17A0'] = 0.9
    assert rig.run(arg('link')).result['stages']['link']['status'] == 'PASS'
    rig.tag.xy = [5.0, 1.5]
    assert rig.run(arg('offsets', (5.0, 1.5))).result['stages']['offsets']['status'] == 'PASS'
    rig.tag.xy = [3.5, 2.5]
    done = rig.run(arg('verify', (3.5, 2.5)))
    assert done.status == 'PASS'
    assert [a['id'] for a in done.result['uwb']['anchors']] == ['1782', '1783', '1786', '17A0']


# ---------------------------------------------------------------- keep previous
def _passed_session(root, clock):
    rig = Rig(root, clock=clock)
    assert rig.full().status == 'PASS'
    assert rig.do('SAVE')['ok']
    return os.path.basename(rig.wiz.session)


def test_keep_previous_copies_survey_keys_only(tmp_path):
    clock = Clock()
    first = _passed_session(tmp_path, clock)
    rig = Rig(tmp_path, clock=clock, resume=0.0)
    assert rig.wiz.slot('uwb_survey').previous == first
    session = rig.wiz._ensure_session()
    prior = copy.deepcopy(UWB)
    prior['track_to_venue']['aligned'] = True
    cs.write_yaml(os.path.join(session, 'data', 'uwb.yaml'), prior)
    r = rig.do('KEEP_PREVIOUS')
    assert r['ok'], r
    saved = session_uwb(rig.wiz)
    assert saved['anchors_surveyed'] and saved['offsets_calibrated']
    assert saved['tag']['z_m'] == 0.15 and saved['track_to_venue']['aligned'] is True
    assert {a['id']: a['range_offset_m'] for a in saved['anchors']}['1782'] == pytest.approx(0.95, abs=0.01)
    assert cs.load_summary(rig.wiz.session)['steps']['uwb_survey']['status'] == 'KEPT_PREVIOUS'


def test_keep_previous_refused_when_a_new_anchor_is_configured(tmp_path):
    clock = Clock()
    _passed_session(tmp_path, clock)
    uwb = copy.deepcopy(UWB)
    uwb['anchors'].append({'id': '17A0', 'xyz_m': [0.0, 4.83, 0.0], 'range_offset_m': 0.0})
    rig = Rig(tmp_path, uwb=uwb, clock=clock, resume=0.0)
    r = rig.do('KEEP_PREVIOUS')
    assert not r['ok'] and '17A0' in r['message'] and 'run this step again' in r['message']
    assert 'uwb_survey' not in (cs.load_summary(rig.wiz.session).get('steps') or {})
    assert not os.path.isfile(os.path.join(rig.wiz.session, 'data', 'uwb.yaml'))


def test_keep_previous_refused_without_data_file(tmp_path):
    step = UwbSurveyStep(STEP10, UWB)
    src, dst = tmp_path / 'old', tmp_path / 'new'
    src.mkdir()
    dst.mkdir()
    with pytest.raises(wc.StepRefused, match='no data/uwb.yaml'):
        step.keep_data(str(src), str(dst))
    (src / 'data').mkdir()
    cs.write_yaml(str(src / 'data' / 'uwb.yaml'), UWB)          # offsets_calibrated false
    with pytest.raises(wc.StepRefused, match='not complete'):
        step.keep_data(str(src), str(dst))


# ---------------------------------------------------------------- live view + feed
def test_live_view_while_measuring_is_json(tmp_path):
    rig = Rig(tmp_path)
    rig.run(arg('link'))
    rig.run(survey_arg())
    assert rig.do('RUN', arg('offsets', (5.0, 1.5)))['ok']
    for _ in range(30):
        rig.clock.t += 0.1
        rig.feed.add(rig.tag.report())
        rig.wiz.tick(rig.inputs())
    lv = rig.wiz.live(rig.inputs())
    json.dumps(lv)
    st = lv['step']
    assert st['status'] == 'RUNNING' and st['run']['stage'] == 'offsets' and st['run']['remaining_s'] > 0
    view = st['live']
    rows = {a['id']: a for a in view['anchors']}
    assert set(rows) == set(TRUE)
    assert rows['1782']['samples'] >= 20 and rows['1782']['spread_m'] is not None
    assert rows['1782']['rate_hz'] == pytest.approx(10.0, abs=1.0) and rows['1782']['age_s'] < 0.5
    assert view['feed']['hz'] == pytest.approx(10.0, abs=1.0)
    assert [s['state'] for s in view['stages']] == ['pass', 'pass', 'running', 'todo']
    assert view['entered'] and view['form']['tag']['z_m'] == 0.15
    assert view['limits']['max_verify_error_m'] == STEP10['pass']['max_verify_error_m']


def test_live_view_without_feed(tmp_path):
    step = UwbSurveyStep(STEP10, UWB)
    lv = step.live({})
    assert lv['feed'] is None and [a['id'] for a in lv['anchors']] == sorted(TRUE)
    assert lv['geometry']['warnings'] and lv['next'] == 'link'
    json.dumps(lv)


def test_feed_cursor_lost_and_stats():
    clock = Clock()
    feed = wu.UwbFeed(2.0, 1.0, clock=clock)
    tag = wu.SyntheticTag(TRUE, 0.15, BIAS)
    c = feed.cursor()
    feed.add('not json')
    for _ in range(10):
        clock.t += 0.1
        feed.add(tag.report())
    rows, c2, lost = feed.rows_since(c)
    assert len(rows) == 10 and c2 == 10 and lost == 0 and feed.bad_json == 1
    assert set(rows[0]) == {'t', 'json'}
    samples, unknown, n = wu.fresh_samples(rows)
    assert n == 10 and all(len(v) == 10 for v in samples.values())
    for _ in range(40):                                   # 4 s > buffer_s: early rows dropped
        clock.t += 0.1
        feed.add(tag.report())
    rows, c3, lost = feed.rows_since(c2)
    assert lost > 0 and len(rows) + lost == 40 and c3 == 50
    tag.extra = {'17A0': (0, 4.83, 1.2)}
    clock.t += 0.1
    feed.add(tag.report())
    s = feed.stats(['1782', '1786', '1783', '9999'])
    assert s['anchors']['1782']['rate_hz'] == pytest.approx(10.0, abs=1.1)
    assert s['anchors']['9999']['age_s'] is None and s['unknown_ids'] == ['17A0']
    # a repeated sample_seq is not a new sample
    rep = json.loads(tag.report())
    clock.t += 0.1
    feed.add(json.dumps(rep))
    before = feed.stats(['1782'])['anchors']['1782']['fresh_count']
    feed.add(json.dumps(rep))
    assert feed.stats(['1782'])['anchors']['1782']['fresh_count'] == before


def test_session_uwb_falls_back_to_base(tmp_path):
    assert wu.session_uwb(str(tmp_path), UWB) is UWB
    assert wu.session_uwb(None, UWB) is UWB
    assert wu.anchor_set(UWB, zero_offsets=True).ids == sorted(TRUE)


# ---------------------------------------------------------------- Haffiz switch: offsets optional
def test_haffiz_no_offsets_verify_passes_directly(tmp_path):
    """Calibrated tag (no range bias): link -> survey -> verify passes, offsets saved as 0."""
    rig = Rig(tmp_path)
    rig.tag.bias = {}
    rig.run(arg('link'))
    rig.run(survey_arg())
    rig.tag.xy = [3.5, 2.5]
    done = rig.run(arg('verify', (3.5, 2.5)))
    assert done.status == 'PASS', done.result
    vf = done.result['stages']['verify']
    assert vf['with_offsets'] is False and vf['error_m'] <= 0.05 and vf['method'] == 'linear + cv_kf'
    assert done.result['stages']['offsets'] is None
    assert [a['range_offset_m'] for a in done.result['uwb']['anchors']] == [0.0, 0.0, 0.0]
    assert rig.do('SAVE')['ok']
    saved = session_uwb(rig.wiz)
    assert saved['offsets_calibrated'] is True
    live = rig.step.live(rig.inputs())
    assert live['offsets_mode'] == 'optional'
    assert next(x for x in live['stages'] if x['key'] == 'offsets')['state'] == 'optional'


def test_haffiz_biased_ranges_fail_verify_then_offsets_fix_it(tmp_path):
    rig = Rig(tmp_path)                                   # BIAS ~1 m on every range
    rig.run(arg('link'))
    rig.run(survey_arg())
    rig.tag.xy = [3.5, 2.5]
    done = rig.run(arg('verify', (3.5, 2.5)))
    assert done.status == 'FAIL' and done.result['next'] == 'offsets'
    assert 'Measure offsets' in done.result['stages']['verify']['fix']
    rig.tag.xy = [5.0, 1.5]
    assert rig.run(arg('offsets', (5.0, 1.5))).result['stages']['offsets']['status'] == 'PASS'
    rig.tag.xy = [3.5, 2.5]
    done = rig.run(arg('verify', (3.5, 2.5)))
    assert done.status == 'PASS' and done.result['stages']['verify']['with_offsets'] is True


def test_offsets_required_mode_keeps_the_old_order(tmp_path):
    step_cfg = copy.deepcopy(dict(STEP10, index=1))
    step_cfg['procedure']['offsets_mode'] = 'required'
    rig = Rig(tmp_path)
    rig.step = UwbSurveyStep(step_cfg, rig.uwb)
    rig.wiz = wc.Wizard({'steps': [step_cfg]}, str(tmp_path), CFG, {'uwb_survey': rig.step}, now=rig.clock)
    rig.run(arg('link'))
    rig.run(survey_arg())
    r = rig.do('RUN', arg('verify', (3.5, 2.5)))
    assert not r['ok'] and 'Offsets' in r['message']


def test_bad_offsets_mode_is_config_error():
    bad = copy.deepcopy(STEP10)
    bad['procedure']['offsets_mode'] = 'sometimes'
    with pytest.raises(ConfigError):
        UwbSurveyStep(bad, UWB)


def test_verify_spot_false_link_and_survey_pass_the_step(tmp_path):
    """procedure.verify_spot: false -> Verify is optional; link + survey alone pass and save (offsets 0)."""
    rig = Rig(tmp_path)
    rig.step.need_verify = False
    rig.tag.bias = {}
    rig.run(arg('link'))
    done = rig.run(survey_arg())
    assert done.status == 'PASS', done.result
    assert done.result['next'] == '' and done.result['stages']['verify'] is None
    assert next(c for c in done.result['checks'] if c['key'] == 'verify')['passed']
    assert [a['range_offset_m'] for a in done.result['uwb']['anchors']] == [0.0, 0.0, 0.0]
    assert next(x for x in rig.step.live(rig.inputs())['stages'] if x['key'] == 'verify')['state'] == 'optional'
    assert rig.do('SAVE')['ok']


def test_verify_spot_must_be_a_bool():
    bad = dict(STEP10, procedure=dict(STEP10['procedure'], verify_spot='no'))
    with pytest.raises(ConfigError):
        UwbSurveyStep(dict(bad, index=1), copy.deepcopy(UWB))
