"""Step 13 (practice runs): attempts from mission topics, grading, done/pass, save/keep (no ROS)."""
import copy
import json
import os

import pytest
import yaml

from carbot_ops import wizard_core as wc
from carbot_ops.sensor_checks import ConfigError
from carbot_ops.step_practice_runs import PracticeRunsStep, parse_argument, slug
from helpers import STEPS, load

CFG = {'session_format': '%Y%m%d_%H%M%S', 'allow_keep_previous': True, 'resume_max_age_h': 0.0, 'page_watch_s': 8.0}
STEP13 = next(s for s in STEPS['steps'] if s['id'] == 'practice_runs')
CHALLENGES = load('challenges.yaml')


def only_step13():
    s = copy.deepcopy(STEP13)
    s['index'] = 1
    return {'version': 1, 'steps': [s]}


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Mission:
    """Synthetic /carbot/mission/state + events + armed/manual."""

    def __init__(self):
        self.state = {'mode': 'IDLE', 'challenge_id': 0, 'challenge_name': '', 'hold_reason': '', 'banner': '',
                      'age_s': 0.1}
        self.events, self.seq, self.manual, self.armed = [], 0, False, True

    def event(self, name, detail='', cid=0):
        self.seq += 1
        self.events.append({'seq': self.seq, 'name': name, 'detail': detail, 'challenge_id': cid})

    def inputs(self):
        return {'mission': dict(self.state), 'mission_events': list(self.events), 'armed': self.armed,
                'manual': self.manual}


def make(root, doc=None):
    clock = Clock()
    impl = PracticeRunsStep(STEP13, CHALLENGES)
    w = wc.Wizard(doc or only_step13(), str(root), dict(CFG), {'practice_runs': impl}, now=clock)
    return w, impl, clock


def op(**kw):
    return json.dumps(kw)


def tick(w, clock, m, dt=0.2, n=1):
    done = None
    for _ in range(n):
        clock.t += dt
        done = w.tick(m.inputs()) or done
    return done


# ---------------------------------------------------------------- config
def test_repo_yaml_builds_and_contract_unchanged():
    impl = PracticeRunsStep(STEP13, CHALLENGES)
    assert STEP13['index'] == 13 and STEP13['required'] is False and STEP13['tab'] == 'scoreboard'
    assert STEP13['writes'] == ['practice/<challenge>.yaml']
    assert len(impl.challenges) == 13
    assert impl.challenges[2]['file'] == 'practice/02_roundabout_1st_exit.yaml'
    assert impl.challenges[12]['whole_run'] and not impl.challenges[3]['whole_run']


def test_missing_key_is_loud():
    for k in ('max_attempt_s', 'end_on_challenge_exit', 'max_events', 'max_transitions', 'recent_attempts'):
        bad = copy.deepcopy(STEP13)
        del bad['procedure'][k]
        with pytest.raises(ConfigError, match=k):
            PracticeRunsStep(bad, CHALLENGES)
    bad = copy.deepcopy(STEP13)
    bad['pass'] = {}
    with pytest.raises(ConfigError, match='user_marked_done'):
        PracticeRunsStep(bad, CHALLENGES)
    ch = copy.deepcopy(CHALLENGES)
    del ch['challenges'][0]['marks']
    with pytest.raises(ConfigError, match='marks'):
        PracticeRunsStep(STEP13, ch)


def test_parse_argument_and_slug():
    assert parse_argument('')[0] is None
    assert parse_argument('nope')[0] is None
    assert parse_argument(op(op='jump'))[0] is None
    assert parse_argument(op(op='done'))[0] == {'op': 'done'}
    assert slug('Roundabout (2nd exit)') == 'roundabout_2nd_exit'


def test_blocked_until_required_steps_pass(tmp_path):
    impl = PracticeRunsStep(STEP13, CHALLENGES)
    w = wc.Wizard(STEPS, str(tmp_path), dict(CFG), {'practice_runs': impl}, now=Clock())
    r = w.action('practice_runs', 'RUN', op(op='start', challenge=1), Mission().inputs())
    assert not r['ok'] and 'Finish step 1' in r['message']


# ---------------------------------------------------------------- attempts
def test_attempt_ends_when_mission_leaves_challenge(tmp_path):
    w, impl, clock = make(tmp_path)
    m = Mission()
    m.state.update(mode='ROAD')
    assert w.action('practice_runs', 'RUN', op(op='start', challenge=3), m.inputs())['ok']
    assert w.slot('practice_runs').status == 'RUNNING'
    assert tick(w, clock, m, n=5) is None
    m.state.update(mode='TUNNEL', challenge_id=3, challenge_name='Tunnel')
    m.event('TUNNEL', 'LiDAR trigger', 3)
    assert tick(w, clock, m, n=10) is None           # 2 s inside
    live = w.live(m.inputs())['step']['live']
    assert live['current']['challenge'] == 3 and live['current']['entered_s'] is not None
    m.state.update(mode='ROAD', challenge_id=0, challenge_name='')
    s = tick(w, clock, m)
    assert s is not None and s.status == 'FAIL' and s.result['in_progress'] is True
    a = impl.attempts[0]
    assert a['outcome'] == 'LEFT_CHALLENGE' and a['entered_challenge']
    assert a['challenge_time_s'] == pytest.approx(2.0, abs=0.01)
    assert [e['name'] for e in a['events']] == ['TUNNEL']
    assert [t['mode'] for t in a['transitions']] == ['ROAD', 'TUNNEL', 'ROAD']
    assert a['level'] is None                          # user grades
    row = next(r for r in impl.table() if r['id'] == 3)
    assert row['attempts'] == 1 and row['graded'] == 0


def test_old_events_before_start_are_ignored(tmp_path):
    w, impl, clock = make(tmp_path)
    m = Mission()
    m.event('MANUAL INTERVENTION', 'old one')
    assert w.action('practice_runs', 'RUN', op(op='start', challenge=1), m.inputs())['ok']
    assert tick(w, clock, m, n=3) is None
    assert impl.cur['events'] == []


def test_manual_intervention_ends_attempt_as_fail(tmp_path):
    w, impl, clock = make(tmp_path)
    m = Mission()
    m.state.update(mode='ROAD', challenge_id=7)
    assert w.action('practice_runs', 'RUN', op(op='start', challenge=7), m.inputs())['ok']
    tick(w, clock, m, n=2)
    m.event('MANUAL INTERVENTION', 'E-stop pressed', 7)
    assert tick(w, clock, m) is not None
    a = impl.attempts[0]
    assert a['outcome'] == 'MANUAL_INTERVENTION' and a['level'] == 'FAIL' and a['marks'] == 0
    # manual takeover topic does the same
    m.manual = True
    assert w.action('practice_runs', 'RUN', op(op='start', challenge=7), m.inputs())['ok']
    assert tick(w, clock, m) is not None and impl.attempts[1]['outcome'] == 'MANUAL_INTERVENTION'


def test_stop_cancel_and_time_limit(tmp_path):
    w, impl, clock = make(tmp_path)
    m = Mission()                                     # mission IDLE (calibrate mode, not armed)
    m.armed = None
    assert w.action('practice_runs', 'RUN', op(op='start', challenge=10), m.inputs())['ok']
    assert any('IDLE' in x for x in w.live(m.inputs())['step']['live']['warnings'])
    tick(w, clock, m, dt=1.0, n=4)
    assert w.action('practice_runs', 'CANCEL', '', {})['ok']
    assert impl.attempts[0]['outcome'] == 'STOPPED' and impl.attempts[0]['duration_s'] == pytest.approx(4.0)
    assert impl.attempts[0]['challenge_time_s'] is None
    assert w.slot('practice_runs').status == 'SKIPPED_OPTIONAL'
    assert w.action('practice_runs', 'RUN', op(op='start', challenge=10), m.inputs())['ok']
    s = tick(w, clock, m, dt=10.0, n=int(STEP13['procedure']['max_attempt_s'] / 10) + 1)
    assert s is not None and impl.attempts[1]['outcome'] == 'TIME_LIMIT'


def test_mission_complete_ends_whole_run_challenge(tmp_path):
    w, impl, clock = make(tmp_path)
    m = Mission()
    assert w.action('practice_runs', 'RUN', op(op='start', challenge=13), m.inputs())['ok']
    tick(w, clock, m, n=2)
    m.state.update(mode='ROAD', challenge_id=1)
    tick(w, clock, m, dt=1.0, n=30)                   # leaving challenge 1 does not end a whole-run attempt
    assert impl.cur is not None
    m.state.update(mode='COMPLETE', challenge_id=0)
    assert tick(w, clock, m) is not None
    assert impl.attempts[0]['outcome'] == 'MISSION_COMPLETE'
    assert impl.attempts[0]['challenge_time_s'] == pytest.approx(29.2, abs=0.01)   # entered on the first non-IDLE tick


def test_refusals(tmp_path):
    w, impl, clock = make(tmp_path)
    m = Mission()
    assert not w.action('practice_runs', 'RUN', '', m.inputs())['ok']
    assert not w.action('practice_runs', 'RUN', op(op='start', challenge=99), m.inputs())['ok']
    r = w.action('practice_runs', 'RUN', op(op='done'), m.inputs())
    assert not r['ok'] and 'No attempt' in r['message']
    assert not w.action('practice_runs', 'RUN', op(op='grade', attempt=1, level='COMPLETED'), m.inputs())['ok']
    assert w.action('practice_runs', 'RUN', op(op='start', challenge=1), m.inputs())['ok']
    assert not w.action('practice_runs', 'RUN', op(op='done'), m.inputs())['ok']      # still running
    w.action('practice_runs', 'CANCEL', '', {})
    assert not w.action('practice_runs', 'RUN', op(op='grade', attempt=1, level='GREAT'), m.inputs())['ok']
    assert not w.action('practice_runs', 'SAVE', '', {})['ok']


# ---------------------------------------------------------------- grade, done, save, keep
def practise(w, clock, m, cid, level):
    assert w.action('practice_runs', 'RUN', op(op='start', challenge=cid), m.inputs())['ok']
    tick(w, clock, m, dt=1.0, n=3)
    assert w.action('practice_runs', 'CANCEL', '', {})['ok']
    n = len(w.impls['practice_runs'].attempts)
    assert w.action('practice_runs', 'REDO', op(op='grade', attempt=n, level=level, note='ok run'), m.inputs())['ok']
    s = tick(w, clock, m)
    assert s is not None and s.status == 'FAIL' and 'graded' in s.result['summary']
    return n


def test_grade_done_save_and_resave_merges(tmp_path):
    w, impl, clock = make(tmp_path)
    m = Mission()
    practise(w, clock, m, 1, 'COMPLETED')
    practise(w, clock, m, 1, 'FAIL')
    practise(w, clock, m, 4, 'EXCELLENT')
    assert impl.attempts[0]['marks'] == 8 and impl.attempts[2]['marks'] == 5
    assert not w.action('practice_runs', 'SAVE', '', {})['ok']            # not done yet
    assert w.action('practice_runs', 'REDO', op(op='done'), m.inputs())['ok']
    s = tick(w, clock, m)
    assert s.status == 'PASS' and s.result['done'] and s.result['n_attempts'] == 3
    r = w.action('practice_runs', 'SAVE', '', {})
    assert r['ok'], r
    f1 = os.path.join(w.session, 'practice', '01_lane_change.yaml')
    with open(f1, encoding='utf-8') as f:
        d = yaml.safe_load(f)
    assert d['attempts_total'] == 2 and d['failed'] == 1 and d['best_marks'] == 8
    assert [a['level'] for a in d['attempts']] == ['COMPLETED', 'FAIL']
    assert os.path.isfile(os.path.join(w.session, 'practice', '04_boom_gate.yaml'))
    assert not os.path.isfile(os.path.join(w.session, 'practice', '03_tunnel.yaml'))
    with open(os.path.join(w.session, 'practice', 'summary.yaml'), encoding='utf-8') as f:
        summ = yaml.safe_load(f)
    assert {c['id'] for c in summ['challenges']} == {1, 4}
    assert 'practice/summary.yaml' in s.result['data_files']
    # a new wizard run (attempts in memory are gone) practises more and saves again: merged
    w2, impl2, clock2 = make(tmp_path)
    w2.session = w.session
    practise(w2, clock2, m, 1, 'PARTIAL')
    w2.action('practice_runs', 'RUN', op(op='done'), m.inputs())
    tick(w2, clock2, m)
    assert w2.action('practice_runs', 'SAVE', '', {})['ok']
    with open(f1, encoding='utf-8') as f:
        d = yaml.safe_load(f)
    assert d['attempts_total'] == 3 and [a['level'] for a in d['attempts']][-1] == 'PARTIAL'


def test_new_attempt_after_done_needs_done_again(tmp_path):
    w, impl, clock = make(tmp_path)
    m = Mission()
    practise(w, clock, m, 2, 'COMPLETED')
    w.action('practice_runs', 'RUN', op(op='done'), m.inputs())
    assert tick(w, clock, m).status == 'PASS'
    practise(w, clock, m, 2, 'PARTIAL')
    assert w.slot('practice_runs').status == 'FAIL' and not impl.done


def test_keep_previous_copies_practice_folder(tmp_path):
    w, impl, clock = make(tmp_path)
    m = Mission()
    practise(w, clock, m, 5, 'COMPLETED')
    w.action('practice_runs', 'RUN', op(op='done'), m.inputs())
    tick(w, clock, m)
    assert w.action('practice_runs', 'SAVE', '', {})['ok']
    old = w.session_name()
    w2, _, _ = make(tmp_path)
    assert w2.slot('practice_runs').previous == old
    r = w2.action('practice_runs', 'KEEP_PREVIOUS', '', {})
    assert r['ok'], r
    assert w2.session != w.session
    assert os.path.isfile(os.path.join(w2.session, 'practice', '05_hill.yaml'))
    assert os.path.isfile(os.path.join(w2.session, 'practice', 'summary.yaml'))


def test_keep_refused_without_practice_folder(tmp_path):
    impl = PracticeRunsStep(STEP13, CHALLENGES)
    src = tmp_path / 'old'
    src.mkdir()
    from carbot_ops.wizard_core import StepRefused
    with pytest.raises(StepRefused):
        impl.keep_data(str(src), str(tmp_path / 'new'))


def test_live_without_mission(tmp_path):
    w, impl, clock = make(tmp_path)
    lv = w.live({})['step']['live']
    assert lv['mission'] is None and lv['warnings'] and len(lv['challenges']) == 13
    assert lv['levels'] == CHALLENGES['levels']
