"""Calibration step 13 -- per-challenge practice runs (optional; pure, unit tested).

The page NEVER commands motion. The car drives the challenge through the normal
stack (mission_logic -> planners -> command_owner, the only motion writer); this
step only watches and records.

RUN / REDO argument JSON, one operation per press:
  {"op": "start", "challenge": N}   start an attempt at challenge N (challenges.yaml id).
                                    The step is RUNNING while the car drives.
  {"op": "grade", "attempt": k, "level": "COMPLETED", "note": "..."}
                                    grade attempt k (levels from challenges.yaml).
  {"op": "done"}                    practice finished: the step PASSES (failed attempts
                                    included) and Save becomes possible.

An attempt records, from topics that are actually published:
  * /carbot/mission/state   mode / challenge_id / hold_reason transitions, when the
                            mission entered and left the challenge (challenge time);
  * /carbot/mission/events  every event during the attempt;
  * /carbot/manual/takeover and a MANUAL INTERVENTION event -> suggested FAIL (rulebook:
                            manual intervention scores 0);
  * /carbot/race/armed      shown: mission_logic only starts once armed.
It ends by itself when the mission leaves the challenge (procedure.end_on_challenge_exit),
reaches COMPLETE, or a manual intervention is seen; else on Stop (CANCEL) / STOP MOTORS,
or after procedure.max_attempt_s. The scoreboard node is a stub and not launched, so the
result of each attempt is the user's grade; nothing is judged automatically except
manual intervention.

Between operations the step shows FAIL with result.in_progress = true (wizard_core has
no "in progress" result; the step is optional, so nothing is blocked).

Save writes <session>/practice/<NN>_<name>.yaml per practised challenge (attempts merged
with what an earlier Save of this session wrote) and practice/summary.yaml. Attempts
live in memory until Save. Keep previous copies an older session's practice/ folder.
"""
import datetime
import json
import os
import re
import shutil
import uuid
from typing import Dict, List, Optional, Tuple

from carbot_common import calibration_store as cs

from .sensor_checks import ConfigError
from .wizard_core import StepImpl, StepRefused

PROC_KEYS = ('max_attempt_s', 'end_on_challenge_exit', 'max_events', 'max_transitions', 'recent_attempts')
PASS_KEYS = ('user_marked_done',)
OPS = ('start', 'grade', 'done')
PRACTICE_DIR = 'practice'
# why an attempt ended
OUTCOMES = ('LEFT_CHALLENGE', 'MISSION_COMPLETE', 'MANUAL_INTERVENTION', 'STOPPED', 'TIME_LIMIT')
OUTCOME_TEXT = {'LEFT_CHALLENGE': 'mission left the challenge', 'MISSION_COMPLETE': 'mission complete',
                'MANUAL_INTERVENTION': 'manual intervention (scores 0)', 'STOPPED': 'stopped (Stop / STOP MOTORS)',
                'TIME_LIMIT': 'time limit reached'}
MAX_NOTE = 300


def _need(d: Dict, keys, where: str) -> None:
    miss = [k for k in keys if k not in (d or {})]
    if miss:
        raise ConfigError(f'{where}: missing {", ".join(miss)}')


def slug(name: str) -> str:
    return re.sub(r'[^a-z0-9]+', '_', str(name).lower()).strip('_') or 'challenge'


def parse_argument(arg: str) -> Tuple[Optional[Dict], str]:
    """RUN argument -> (op dict, error). op None = refused."""
    try:
        a = json.loads(arg) if arg else None
    except ValueError:
        a = None
    if not isinstance(a, dict) or a.get('op') not in OPS:
        return None, ('Pick a challenge and press Start attempt (or grade an attempt, or press Practice done): '
                      'this step needs to know which.')
    return a, ''


class PracticeRunsStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, challenges: Dict):
        super().__init__(cfg)
        self.proc = cfg.get('procedure') or {}
        self.pass_cfg = cfg.get('pass') or {}
        _need(self.proc, PROC_KEYS, 'calibration_steps.yaml practice_runs.procedure')
        _need(self.pass_cfg, PASS_KEYS, 'calibration_steps.yaml practice_runs.pass')
        if self.pass_cfg['user_marked_done'] is not True:
            raise ConfigError('calibration_steps.yaml practice_runs.pass.user_marked_done must be true '
                              '(only the user can say practice is over)')
        _need(challenges, ('levels', 'challenges', 'rules'), 'challenges.yaml')
        self.levels = [str(x) for x in challenges['levels']]
        if 'FAIL' not in self.levels:
            raise ConfigError('challenges.yaml levels must include FAIL')
        self.manual_zero = bool((challenges['rules'] or {}).get('manual_intervention_scores_zero', True))
        self.challenges: Dict[int, Dict] = {}
        for c in challenges['challenges'] or []:
            _need(c, ('id', 'name', 'marks', 'mandatory', 'zone'), f'challenges.yaml challenge {c.get("id", "?")}')
            cid = int(c['id'])
            self.challenges[cid] = {
                'id': cid, 'name': str(c['name']), 'marks': [int(m) for m in c['marks']],
                'mandatory': bool(c['mandatory']), 'whole_run': (c['zone'] or {}).get('type') == 'whole_run',
                'file': f'{PRACTICE_DIR}/{cid:02d}_{slug(c["name"])}.yaml'}
        if not self.challenges:
            raise ConfigError('challenges.yaml: no challenges')
        self.max_s = float(self.proc['max_attempt_s'])
        self.end_on_exit = bool(self.proc['end_on_challenge_exit'])
        self.max_events = int(self.proc['max_events'])
        self.max_trans = int(self.proc['max_transitions'])
        self.recent = int(self.proc['recent_attempts'])
        self.attempts: List[Dict] = []       # every attempt of this wizard run (1-based 'n')
        self.cur: Optional[Dict] = None      # attempt in progress
        self.pending: Optional[str] = None   # quick op waiting for the next tick
        self.done = False

    # ------------------------------------------------------------------ helpers
    def marks_for(self, cid: int, level: Optional[str]) -> Optional[int]:
        """Rubric marks for a level; None when ungraded or the challenge is ranked (#13)."""
        c = self.challenges[cid]
        if level is None or len(c['marks']) != len(self.levels):
            return None
        return c['marks'][self.levels.index(level)]

    def attempt(self, n) -> Optional[Dict]:
        try:
            n = int(n)
        except (TypeError, ValueError):
            return None
        return self.attempts[n - 1] if 1 <= n <= len(self.attempts) else None

    @staticmethod
    def _mission(inputs: Dict) -> Optional[Dict]:
        m = inputs.get('mission')
        return m if isinstance(m, dict) else None

    @staticmethod
    def _events(inputs: Dict) -> List[Dict]:
        return [e for e in (inputs.get('mission_events') or []) if isinstance(e, dict) and 'seq' in e]

    # ------------------------------------------------------------------ run
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        op, err = parse_argument(inputs.get('argument', ''))
        if op is None:
            return err
        if self.cur is not None:
            return 'An attempt is still being recorded: stop it first.'
        kind = op['op']
        if kind == 'start':
            try:
                cid = int(op.get('challenge'))
            except (TypeError, ValueError):
                cid = None
            if cid not in self.challenges:
                return f'No challenge {op.get("challenge")!r} in challenges.yaml (ids {min(self.challenges)}-{max(self.challenges)}).'
            m = self._mission(inputs) or {}
            ev = self._events(inputs)
            c = self.challenges[cid]
            n = len(self.attempts) + 1
            started = datetime.datetime.now()
            self.cur = {
                'n': n, 'uid': f'{started.strftime("%Y%m%d_%H%M%S")}_{cid:02d}_{uuid.uuid4().hex[:6]}', 'challenge': cid,
                'name': c['name'], 'started': started.isoformat(timespec='seconds'), 't0': now, 'elapsed_s': 0.0,
                'entered_s': 0.0 if self._in_challenge(c, m) else None, 'left_s': None,
                'manual': False, 'manual_why': '', 'last_seq': max((int(e['seq']) for e in ev), default=0),
                'last_key': None, 'transitions': [], 'dropped_transitions': 0, 'events': [], 'dropped_events': 0,
                'armed_at_start': inputs.get('armed'), 'mode_at_start': m.get('mode', '')}
            self._observe(now, inputs)
            self.done = False
            return None
        if kind == 'grade':
            a = self.attempt(op.get('attempt'))
            if a is None:
                return f'No attempt {op.get("attempt")!r} to grade.'
            level = str(op.get('level', '')).upper()
            if level not in self.levels:
                return f'Grade must be one of {", ".join(self.levels)}.'
            a['level'] = level
            a['marks'] = self.marks_for(a['challenge'], level)
            a['note'] = str(op.get('note', '') or '')[:MAX_NOTE]
            a['graded_by'] = 'user'
            self.pending = f'graded attempt {a["n"]} ({a["name"]}): {level}'
            self.done = False
            return None
        # done
        if not self.attempts:
            return ('No attempt recorded yet. Practise at least one challenge, or simply leave this optional '
                    'step: it does not block race mode.')
        self.done = True
        self.pending = 'practice marked done'
        return None

    def _in_challenge(self, c: Dict, m: Dict) -> bool:
        if not m:
            return False
        if c['whole_run']:
            return str(m.get('mode', '')) not in ('', 'IDLE')
        return int(m.get('challenge_id', 0) or 0) == c['id']

    def _observe(self, now: float, inputs: Dict) -> None:
        a = self.cur
        el = round(now - a['t0'], 2)
        a['elapsed_s'] = el
        c = self.challenges[a['challenge']]
        m = self._mission(inputs)
        if m:
            key = (str(m.get('mode', '')), int(m.get('challenge_id', 0) or 0), str(m.get('hold_reason', '')))
            if key != a['last_key']:
                a['last_key'] = key
                tr = {'t_s': el, 'mode': key[0], 'challenge_id': key[1],
                      'challenge_name': str(m.get('challenge_name', '')), 'hold_reason': key[2]}
                if len(a['transitions']) < self.max_trans:
                    a['transitions'].append(tr)
                else:
                    a['dropped_transitions'] += 1
            inside = self._in_challenge(c, m)
            if inside and a['entered_s'] is None:
                a['entered_s'] = el
            elif not inside and a['entered_s'] is not None and a['left_s'] is None and not c['whole_run']:
                a['left_s'] = el
        for e in self._events(inputs):
            if int(e['seq']) <= a['last_seq']:
                continue
            a['last_seq'] = int(e['seq'])
            ev = {'t_s': el, 'name': str(e.get('name', '')), 'detail': str(e.get('detail', '')),
                  'challenge_id': int(e.get('challenge_id', 0) or 0)}
            if len(a['events']) < self.max_events:
                a['events'].append(ev)
            else:
                a['dropped_events'] += 1
            if ev['name'] == 'MANUAL INTERVENTION' and not a['manual']:
                a['manual'], a['manual_why'] = True, ev['detail'] or 'MANUAL INTERVENTION event'
        if inputs.get('manual') and not a['manual']:
            a['manual'], a['manual_why'] = True, 'Manual control switched on (/carbot/manual/takeover)'

    def _end_reason(self, m: Optional[Dict]) -> Optional[str]:
        a = self.cur
        if a['manual']:
            return 'MANUAL_INTERVENTION'
        if m and str(m.get('mode', '')) == 'COMPLETE':
            return 'MISSION_COMPLETE'
        if self.end_on_exit and a['left_s'] is not None:
            return 'LEFT_CHALLENGE'
        if a['elapsed_s'] >= self.max_s:
            return 'TIME_LIMIT'
        return None

    def _finish(self, outcome: str) -> Dict:
        a = self.cur
        self.cur = None
        c = self.challenges[a['challenge']]
        ct = None
        if a['entered_s'] is not None:
            end = a['left_s'] if a['left_s'] is not None else a['elapsed_s']
            ct = round(end - a['entered_s'], 2)
        suggested = 'FAIL' if outcome == 'MANUAL_INTERVENTION' and self.manual_zero else None
        rec = {'n': a['n'], 'uid': a['uid'], 'challenge': a['challenge'], 'name': c['name'],
               'started': a['started'], 'duration_s': a['elapsed_s'], 'challenge_time_s': ct,
               'entered_challenge': a['entered_s'] is not None, 'outcome': outcome,
               'outcome_text': OUTCOME_TEXT[outcome] + (f': {a["manual_why"]}' if a['manual'] else ''),
               'suggested_level': suggested, 'level': suggested,
               'marks': self.marks_for(a['challenge'], suggested), 'note': '',
               'graded_by': 'rule: manual intervention' if suggested else '',
               'armed_at_start': a['armed_at_start'], 'mode_at_start': a['mode_at_start'],
               'transitions': a['transitions'], 'events': a['events'],
               'dropped_transitions': a['dropped_transitions'], 'dropped_events': a['dropped_events']}
        self.attempts.append(rec)
        return rec

    def cancel(self) -> None:
        if self.cur is not None:
            self._finish('STOPPED')
        self.pending = None

    def progress(self, now: float) -> Dict:
        if self.cur is None:
            return {}
        el = now - self.cur['t0']
        return {'elapsed_s': round(el, 1), 'remaining_s': round(max(0.0, self.max_s - el), 1),
                'fraction': round(min(1.0, el / self.max_s), 2) if self.max_s > 0 else 0.0,
                'samples': len(self.cur['transitions']), 'attempt': self.cur['n'],
                'challenge': self.cur['challenge']}

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        if self.cur is not None:
            self._observe(now, inputs)
            why = self._end_reason(self._mission(inputs))
            if why is None:
                return None
            rec = self._finish(why)
            return self._result(f'attempt {rec["n"]} ({rec["name"]}) ended: {rec["outcome_text"]}')
        if self.pending is not None:
            what, self.pending = self.pending, None
            return self._result(what)
        return None

    # ------------------------------------------------------------------ views
    def table(self) -> List[Dict]:
        """One row per challenge: attempts, graded, last result, times."""
        rows = []
        for cid, c in sorted(self.challenges.items()):
            att = [a for a in self.attempts if a['challenge'] == cid]
            last = att[-1] if att else None
            times = [a['challenge_time_s'] for a in att if a['challenge_time_s'] is not None
                     and a.get('level') not in (None, 'FAIL')]
            rows.append({
                'id': cid, 'name': c['name'], 'mandatory': c['mandatory'], 'max_marks': c['marks'][0],
                'file': c['file'], 'attempts': len(att), 'graded': sum(1 for a in att if a.get('level')),
                'failed': sum(1 for a in att if a.get('level') == 'FAIL'),
                'last_level': last.get('level') if last else None,
                'last_outcome': last['outcome'] if last else None,
                'last_time_s': (last['challenge_time_s'] if last['challenge_time_s'] is not None
                                else last['duration_s']) if last else None,
                'best_time_s': min(times) if times else None,
                'best_marks': max((a['marks'] for a in att if a.get('marks') is not None), default=None)})
        return rows

    @staticmethod
    def _brief(a: Dict) -> Dict:
        return {k: a[k] for k in ('n', 'challenge', 'name', 'started', 'duration_s', 'challenge_time_s',
                                  'entered_challenge', 'outcome', 'outcome_text', 'suggested_level', 'level',
                                  'marks', 'note', 'armed_at_start')}

    def live(self, inputs: Dict) -> Dict:
        m = self._mission(inputs)
        cur = None
        if self.cur is not None:
            a = self.cur
            cur = {'n': a['n'], 'challenge': a['challenge'], 'name': a['name'], 'elapsed_s': a['elapsed_s'],
                   'entered_s': a['entered_s'], 'left_s': a['left_s'], 'manual': a['manual'],
                   'transitions': a['transitions'][-8:], 'events': a['events'][-8:]}
        warnings = []
        if m is None:
            warnings.append('No /carbot/mission/state: mission_logic is not running, so only your grade and '
                            'the attempt time are recorded.')
        elif inputs.get('armed') is not True and str(m.get('mode', '')) in ('', 'IDLE'):
            warnings.append('mission_logic is IDLE: it only starts driving once /carbot/race/armed is true, '
                            'and nothing arms it in calibrate mode. Attempts then record your grade and the time only.')
        if inputs.get('manual'):
            warnings.append('Manual control is ON: an attempt started now ends at once as a manual intervention.')
        return {'challenges': self.table(), 'levels': self.levels, 'current': cur, 'done': self.done,
                'attempts': [self._brief(a) for a in self.attempts[-self.recent:]][::-1],
                'n_attempts': len(self.attempts),
                'ungraded': [a['n'] for a in self.attempts if not a.get('level')],
                'mission': None if m is None else {k: m.get(k) for k in ('mode', 'challenge_id', 'challenge_name',
                                                                         'hold_reason', 'banner', 'age_s')},
                'armed': inputs.get('armed'), 'manual': inputs.get('manual'), 'warnings': warnings,
                'max_attempt_s': self.max_s}

    def _result(self, what: str) -> Dict:
        n = len(self.attempts)
        practised = [r for r in self.table() if r['attempts']]
        ungraded = [a['n'] for a in self.attempts if not a.get('level')]
        if self.done:
            summary = (f'practice done: {n} attempt{"s" if n != 1 else ""} on {len(practised)} '
                       f'challenge{"s" if len(practised) != 1 else ""}')
        else:
            summary = f'practice in progress: {what}; {n} attempt{"s" if n != 1 else ""} so far'
        checks = [{'key': 'user_marked_done', 'label': 'You marked practice done',
                   'measured': 'done' if self.done else 'not yet', 'limit': 'Practice done pressed',
                   'passed': self.done, 'why': '' if self.done else 'Practice is still going on.',
                   'fix': '' if self.done else 'Record and grade attempts, then press Practice done.'},
                  {'key': 'attempts', 'label': 'Attempts recorded', 'measured': str(n), 'limit': '>= 1',
                   'passed': n > 0, 'why': '', 'fix': ''}]
        return {'passed': self.done, 'in_progress': not self.done, 'summary': summary, 'last_action': what,
                'checks': checks, 'done': self.done, 'n_attempts': n, 'ungraded_attempts': ungraded,
                'challenges': practised,
                'source': 'user grades + /carbot/mission/state and /carbot/mission/events (scoreboard node '
                          'not launched: no automatic grading except manual intervention = FAIL)'}

    # ------------------------------------------------------------------ save / keep
    def save_data(self, session: str, res: Dict) -> List[str]:
        if not res.get('done') or not self.done:
            raise StepRefused('Press Practice done first.')
        if not self.attempts:
            raise StepRefused('No attempt recorded: nothing to save.')
        written = []
        now = datetime.datetime.now().isoformat(timespec='seconds')
        for cid, c in sorted(self.challenges.items()):
            mine = [a for a in self.attempts if a['challenge'] == cid]
            path = os.path.join(session, *c['file'].split('/'))
            old = []
            if os.path.isfile(path):
                old = _load(path).get('attempts') or []
            if not mine and not old:
                continue
            by_uid = {str(a.get('uid')): a for a in old if isinstance(a, dict)}
            for a in mine:
                by_uid[a['uid']] = {k: v for k, v in a.items() if k != 'n'}
            att = sorted(by_uid.values(), key=lambda a: str(a.get('started', '')))
            graded = [a for a in att if a.get('level')]
            ok_times = [a['challenge_time_s'] for a in graded
                        if a.get('level') != 'FAIL' and a.get('challenge_time_s') is not None]
            cs.write_yaml(path, {
                'challenge': cid, 'name': c['name'], 'mandatory': c['mandatory'], 'max_marks': c['marks'][0],
                'updated': now, 'attempts_total': len(att), 'graded': len(graded),
                'failed': sum(1 for a in graded if a['level'] == 'FAIL'),
                'last_level': att[-1].get('level') if att else None,
                'best_marks': max((a['marks'] for a in att if a.get('marks') is not None), default=None),
                'best_time_s': min(ok_times) if ok_times else None,
                'source': res.get('source', ''), 'attempts': att})
            written.append(path)
        summ_path = os.path.join(session, PRACTICE_DIR, 'summary.yaml')
        cs.write_yaml(summ_path, {'updated': now, 'done': True, 'levels': self.levels,
                                  'challenges': [{k: r[k] for k in ('id', 'name', 'attempts', 'graded', 'failed',
                                                                    'last_level', 'best_time_s', 'best_marks', 'file')}
                                                 for r in self._merged_rows(session)]})
        written.append(summ_path)
        return written

    def _merged_rows(self, session: str) -> List[Dict]:
        """Per-challenge rows read back from the files just written (earlier Saves included)."""
        rows = []
        for cid, c in sorted(self.challenges.items()):
            path = os.path.join(session, *c['file'].split('/'))
            if not os.path.isfile(path):
                continue
            d = _load(path)
            rows.append({'id': cid, 'name': c['name'], 'attempts': d.get('attempts_total', 0),
                         'graded': d.get('graded', 0), 'failed': d.get('failed', 0),
                         'last_level': d.get('last_level'), 'best_time_s': d.get('best_time_s'),
                         'best_marks': d.get('best_marks'), 'file': c['file']})
        return rows

    def keep_data(self, src_session: str, session: str) -> List[str]:
        src = os.path.join(src_session, PRACTICE_DIR)
        if not os.path.isdir(src) or not os.listdir(src):
            raise StepRefused(f'{os.path.basename(src_session)} has no {PRACTICE_DIR}/ folder: nothing to keep.')
        dst = os.path.join(session, PRACTICE_DIR)
        out = []
        for root, _dirs, files in os.walk(src):
            rel = os.path.relpath(root, src)
            tgt = os.path.normpath(os.path.join(dst, rel))
            os.makedirs(tgt, exist_ok=True)
            for f in files:
                shutil.copy2(os.path.join(root, f), os.path.join(tgt, f))
                out.append(os.path.join(tgt, f))
        return out


def _load(path: str) -> Dict:
    import yaml
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f)
    return d if isinstance(d, dict) else {}
