"""Calibration wizard logic -- pure, no rclpy, unit tested.

Rules (calibration_steps.yaml header, project brief):
* RUN of step N is refused while a REQUIRED step in its `depends_on` (calibration_steps.yaml)
  has not passed (or been set to keep its previous value). Step numbers are not prerequisites.
* Each step: RUN -> result with pass/fail -> SAVE (only a PASS) or REDO (= run
  again). A failing result blocks Next even if an older PASS was saved.
* KEEP_PREVIOUS is offered only when an older session has a PASS for the step
  and the step's page supports it.
* Results go to ONE timestamped session folder (carbot_common.calibration_store),
  created on the first Save, never overwritten (a re-save moves the old file
  aside), older sessions untouched -> rollback.
* The session becomes ACTIVE (what race.launch.py loads) only once every
  required step has passed, so a half-finished calibration never replaces a
  complete one.
* A relaunch within resume_max_age_h resumes the newest unfinished session.
* Steps without a wizard page yet ("placeholders") show their instructions and
  the terminal tool; results that tool writes into this session's summary.yaml
  are picked up by refresh().

Every public method returns or records a human-readable message; nothing here
raises on user actions (ConfigError only at construction, for broken YAML).
"""
import datetime
import json
import os
import shutil
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

import yaml

from carbot_common import calibration_store as cs

PASSING = cs.PASSING
STATUSES = ('PENDING', 'RUNNING', 'PASS', 'FAIL', 'KEPT_PREVIOUS', 'SKIPPED_OPTIONAL')
ACTIONS = ('SELECT', 'RUN', 'REDO', 'SAVE', 'KEEP_PREVIOUS', 'CANCEL', 'ROLLBACK', 'RESTART_CAMERAS', 'STEP')
CFG_KEYS = ('session_format', 'allow_keep_previous', 'resume_max_age_h', 'page_watch_s')


class ConfigError(ValueError):
    pass


def result(ok: bool, message: str, passed: bool = False, doc: Optional[Dict] = None) -> Dict:
    return {'ok': bool(ok), 'message': message, 'passed': bool(passed),
            'result_yaml': yaml.safe_dump(doc, sort_keys=False) if doc else ''}


class StepRefused(Exception):
    """Raised by StepImpl.save_data / keep_data: the message is shown, nothing is saved."""


class StepImpl:
    """A step with a wizard page. Subclasses: SensorHealthStep (step 1),
    CameraIdentityStep (step 2), ..."""
    can_keep_previous = True     # False while a keep would need data files copied
    ops_while_running = False    # True: page operations (STEP) also reach handle() while RUNNING (step 7 Go)

    def __init__(self, cfg: Dict):
        self.cfg = cfg

    def start(self, now: float, inputs: Dict) -> Optional[str]:
        """Begin RUN. inputs['argument'] = the RUN/REDO argument. Return an error message to refuse."""
        return None

    def save_data(self, session: str, res: Dict) -> List[str]:
        """On Save of a PASS, before the result file: write <session>/data/* (calib_tools.merge_data).
        Returns the written paths. Raise StepRefused / OSError to abort the save."""
        return []

    def keep_data(self, src_session: str, session: str) -> List[str]:
        """On KEEP_PREVIOUS, before anything is recorded: copy this step's data from
        src_session. Raise StepRefused (e.g. the old value no longer fits) / OSError to abort."""
        return []

    def handle(self, op: str, args: Dict, inputs: Dict) -> Dict:
        """Page operation (action STEP, argument JSON {"op": ...}), e.g. step 5 CAPTURE.
        Returns {'ok', 'message'}. Not allowed while the step is RUNNING."""
        return {'ok': False, 'message': 'This step has no page operations'}

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        """While RUNNING. Return the finished result dict ({'passed': bool, ...}) or None."""
        return None

    def cancel(self) -> None:
        pass

    def background(self, session: Optional[str], inputs: Dict) -> None:
        """Called on EVERY wizard tick, also while another step is open or nothing is running. `session` is the
        resumed / current session folder (None until one exists). Used by step 6 to put the calibration saved in
        the session back on servo_controller after a relaunch (the launch itself loads no session overlay)."""

    def progress(self, now: float) -> Dict:
        return {}

    def live(self, inputs: Dict) -> Dict:
        return {}


@dataclass
class Slot:
    index: int
    id: str
    title: str
    required: bool
    cfg: Dict
    status: str = 'PENDING'           # what the page shows
    saved_status: str = ''            # what summary.yaml says ('' = nothing saved)
    saved_file: str = ''
    from_session: str = ''            # KEPT_PREVIOUS source
    result: Optional[Dict] = None     # latest result (unsaved or loaded from the saved file)
    unsaved: bool = False
    previous: str = ''                # newest older session with a PASS (keep-previous candidate)
    message: str = ''
    extra: Dict = field(default_factory=dict)


class Wizard:

    def __init__(self, steps_doc: Dict, root: str, cfg: Dict, impls: Dict[str, StepImpl],
                 now: Callable[[], float] = time.monotonic):
        miss = [k for k in CFG_KEYS if k not in cfg]
        if miss:
            raise ConfigError('ops.yaml calibration_wizard: missing ' + ', '.join(miss))
        self.cfg, self.root, self.impls, self.now = cfg, root, impls, now
        self.slots: List[Slot] = self._slots(steps_doc)
        self.session: Optional[str] = None
        self.current = 1
        self.running: Optional[Slot] = None
        self.sessions: List[Dict] = []
        self.notice = ''                 # one-line banner (session activated, resumed, ...)
        self.watched: Dict[int, float] = {}   # step index -> last SELECT (an open GUI page)
        self._resume()
        self._find_previous()
        self.refresh_sessions()

    # ------------------------------------------------------------------ construction
    @staticmethod
    def _slots(doc: Dict) -> List[Slot]:
        steps = (doc or {}).get('steps')
        if not isinstance(steps, list) or not steps:
            raise ConfigError('calibration_steps.yaml: no steps list')
        out, ids = [], set()
        for s in steps:
            for k in ('index', 'id', 'title', 'depends_on'):
                if k not in s:
                    raise ConfigError(f'calibration_steps.yaml: a step has no {k}: {s}')
            if s['id'] in ids:
                raise ConfigError(f'calibration_steps.yaml: duplicate id {s["id"]}')
            ids.add(s['id'])
            out.append(Slot(int(s['index']), str(s['id']), str(s['title']), bool(s.get('required', True)), s))
        out.sort(key=lambda x: x.index)
        if [x.index for x in out] != list(range(1, len(out) + 1)):
            raise ConfigError('calibration_steps.yaml: indices must be 1..N without gaps, got '
                              + str([x.index for x in out]))
        for x in out:
            deps = x.cfg['depends_on']
            if not isinstance(deps, list) or any(not isinstance(d, int) or not 1 <= d < x.index for d in deps):
                raise ConfigError(f'calibration_steps.yaml: step {x.index} depends_on must be a list of '
                                  f'earlier step indices, got {deps!r}')
        for x in out:
            if not x.required:
                x.status = 'SKIPPED_OPTIONAL'
        return out

    def _resume(self) -> None:
        hours = float(self.cfg['resume_max_age_h'])
        names = cs.list_sessions(self.root)
        if hours <= 0 or not names:
            return
        path = os.path.join(cs.calibration_dir(self.root), names[-1])
        summ = cs.load_summary(path)
        created = summ.get('created')
        try:
            age_h = (datetime.datetime.now() - datetime.datetime.fromisoformat(str(created))).total_seconds() / 3600
        except (TypeError, ValueError):
            return
        if age_h > hours or summ.get('all_required_passed'):
            return
        self.session = path
        self._load_summary(summ)
        self.notice = f'Resumed unfinished session {names[-1]} (started {age_h:.1f} h ago).'
        first = next((s for s in self.slots if s.required and not self.can_advance(s)), None)
        self.current = first.index if first else 1

    def _load_summary(self, summ: Dict) -> bool:
        changed = False
        steps = (summ or {}).get('steps') or {}
        for s in self.slots:
            e = steps.get(s.id) or {}
            st = e.get('status', '')
            if s is self.running or (s.unsaved and st == s.saved_status):
                continue
            if st != s.saved_status or e.get('file', '') != s.saved_file:
                s.saved_status, s.saved_file = st, e.get('file', '')
                s.from_session = e.get('from_session', '')
                s.status = st if st in STATUSES else ('PENDING' if s.required else 'SKIPPED_OPTIONAL')
                s.result, s.unsaved = None, False
                changed = True
        return changed

    def _find_previous(self) -> None:
        own = os.path.basename(self.session) if self.session else None
        for s in self.slots:
            s.previous = ''
        for name in reversed(cs.list_sessions(self.root)):
            if name == own:
                continue
            steps = cs.load_summary(os.path.join(cs.calibration_dir(self.root), name)).get('steps') or {}
            for s in self.slots:
                if s.previous:
                    continue
                e = steps.get(s.id) or {}
                if e.get('status') == 'PASS':
                    s.previous = name
                elif e.get('status') == 'KEPT_PREVIOUS' and e.get('from_session'):
                    s.previous = str(e['from_session'])

    # ------------------------------------------------------------------ queries
    def slot(self, key: Any) -> Optional[Slot]:
        for s in self.slots:
            if str(s.index) == str(key) or s.id == str(key):
                return s
        return None

    def can_advance(self, s: Slot) -> bool:
        if s.status in ('RUNNING', 'FAIL') and s.required:
            return False
        if not s.required:
            return s.status != 'RUNNING'
        return s.saved_status in PASSING

    def blocker(self, s: Slot) -> Optional[Slot]:
        deps = s.cfg['depends_on']       # the step number itself is not a prerequisite
        return next((x for x in self.slots if x.index in deps and x.required and not self.can_advance(x)), None)

    @staticmethod
    def blocked_text(blk: Slot) -> str:
        """Why an earlier step still blocks: a PASS that was never saved is the common case."""
        if blk.status == 'PASS' and blk.unsaved:
            return f'Step {blk.index} ({blk.title}) passed but is not saved yet: open it and press Save.'
        return (f'Finish step {blk.index} ({blk.title}) first: it has to pass and be saved, '
                'or be set to keep its previous value.')

    def all_required_passed(self) -> bool:
        return all(self.can_advance(s) for s in self.slots if s.required)

    def session_name(self) -> str:
        return os.path.basename(self.session) if self.session else ''

    def summary_text(self, s: Slot) -> str:
        if s is self.running:
            p = self.impls[s.id].progress(self.now()) if s.id in self.impls else {}
            if p.get('summary'):                       # a step's own running text (step 12: planning)
                return str(p['summary'])
            return f'measuring, {p.get("remaining_s", 0):.0f} s left' if p else 'running'
        if s.status == 'KEPT_PREVIOUS':
            return f'kept from {s.from_session or s.previous}'
        if s.result:
            txt = str(s.result.get('summary', ''))
            return txt + (' (not saved)' if s.unsaved else '')
        if s.saved_status:
            return f'{s.saved_status} saved in {s.saved_file or "summary.yaml"}'
        if s.id not in self.impls:
            return 'page not built yet' + (f'; previous PASS in {s.previous}' if s.previous else '')
        return f'previous PASS in {s.previous}' if s.previous else 'no previous result'

    def state(self) -> Dict:
        return {'session': self.session_name(), 'current': self.current,
                'steps': [{'index': s.index, 'id': s.id, 'title': s.title, 'status': s.status,
                           'required': s.required, 'can_advance': self.can_advance(s),
                           'summary': self.summary_text(s), 'file': s.saved_file,
                           'previous': s.from_session if s.status == 'KEPT_PREVIOUS' else s.previous}
                          for s in self.slots]}

    def saved_result(self, s: Slot) -> Optional[Dict]:
        if s.result is None and s.saved_file and self.session:
            try:
                with open(os.path.join(self.session, s.saved_file), 'r', encoding='utf-8') as f:
                    s.result = yaml.safe_load(f) or {}
            except (OSError, yaml.YAMLError) as e:
                s.result = {'summary': f'cannot read {s.saved_file}: {e}'}
        return s.result

    def live(self, inputs: Dict, task: Optional[Dict] = None) -> Dict:
        """'step' = the current step; 'pages' = a view per step whose GUI page SELECTed within
        page_watch_s, so two open pages (laptop + phone, two tabs) do not fight over one view."""
        s = self.slot(self.current) or self.slots[0]
        now = self.now()
        watch = float(self.cfg['page_watch_s'])
        self.watched = {i: t for i, t in self.watched.items() if now - t <= watch}
        views = {i: self._view(self.slot(i), inputs) for i in sorted(set(self.watched) | {s.index})}
        return {
            'session': self.session_name(), 'active': self.active_name(), 'notice': self.notice,
            'all_required_passed': self.all_required_passed(), 'sessions': self.sessions,
            'current': s.index, 'n_steps': len(self.slots), 'page_watch_s': watch,
            'step': views[s.index], 'pages': {str(i): v for i, v in views.items()},
            'task': task}

    def _view(self, s: Slot, inputs: Dict) -> Dict:
        impl = self.impls.get(s.id)
        live = None
        if impl is not None:
            try:
                live = impl.live(inputs)
            except Exception as e:  # noqa: BLE001  a broken live view must not stop the wizard
                live = {'error': f'live view failed: {e!r}'}
        blk = self.blocker(s)
        tool = str(s.cfg.get('tool', '') or '')
        return {'index': s.index, 'id': s.id, 'title': s.title, 'required': s.required,
                     'status': s.status, 'built': impl is not None, 'can_advance': self.can_advance(s),
                     'blocked_by': {'index': blk.index, 'title': blk.title, 'unsaved_pass': blk.status == 'PASS' and blk.unsaved,
                                    'text': self.blocked_text(blk)} if blk else None,
                     'previous': s.previous, 'from_session': s.from_session,
                     'can_keep': bool(s.previous and impl is not None and impl.can_keep_previous
                                      and self.cfg['allow_keep_previous']),
                     'saved_file': s.saved_file, 'saved_status': s.saved_status, 'unsaved': s.unsaved,
                     'message': s.message, 'result': self.saved_result(s),
                     'run': impl.progress(self.now()) if impl is not None and s is self.running else None,
                     'live': live,
                     'meta': {'instructions': s.cfg.get('instructions'), 'need': s.cfg.get('need', ''),
                              'tool': tool, 'tool_cmd': self._tool_cmd(tool), 'tab': s.cfg.get('tab', ''),
                              'writes': s.cfg.get('writes', []), 'pass': s.cfg.get('pass', {}),
                              'procedure': s.cfg.get('procedure', {})}}

    def _tool_cmd(self, tool: str) -> str:
        if not tool.startswith('ros2 run') or '<' in tool:
            return tool
        return f'{tool} --session {self.session_name()}' if self.session else ''

    def active_name(self) -> str:
        a = cs.active_session(self.root)
        return os.path.basename(a) if a else ''

    # ------------------------------------------------------------------ disk
    def refresh(self) -> bool:
        """Pick up results a terminal tool wrote into this session. True if anything changed."""
        changed = False
        if self.session:
            changed = self._load_summary(cs.load_summary(self.session))
        self.refresh_sessions()
        return changed

    def refresh_sessions(self) -> None:
        req = [s.id for s in self.slots if s.required]
        active = self.active_name()
        out = []
        for name in reversed(cs.list_sessions(self.root)):
            summ = cs.load_summary(os.path.join(cs.calibration_dir(self.root), name))
            steps = summ.get('steps') or {}
            passed = sum(1 for i in req if (steps.get(i) or {}).get('status') in PASSING)
            out.append({'name': name, 'passed': passed, 'required': len(req), 'active': name == active,
                        'current': name == self.session_name(), 'created': str(summ.get('created', ''))})
        self.sessions = out

    def _ensure_session(self) -> str:
        if self.session:
            return self.session
        name = datetime.datetime.now().strftime(str(self.cfg['session_format']))
        base, k = name, 1
        while os.path.exists(os.path.join(cs.calibration_dir(self.root), name)):
            k += 1
            name = f'{base}_{k}'
        self.session = cs.open_session(self.root, name)
        self._find_previous()
        return self.session

    def _write_hint(self, e: Exception) -> str:
        return (f'Cannot write to {cs.calibration_dir(self.root)}: {e}. Check the folder exists and belongs '
                f'to the user running the launch (e.g. sudo chown -R sunrise: {self.root}).')

    def _after_save(self) -> str:
        if not self.all_required_passed():
            left = [str(s.index) for s in self.slots if s.required and not self.can_advance(s)]
            return f' Race mode keeps using {self.active_name() or "no session"} until steps {", ".join(left)} pass.'
        try:
            summ = cs.load_summary(self.session)
            summ['all_required_passed'] = True
            cs.write_yaml(os.path.join(self.session, 'summary.yaml'), summ)
            cs.set_active(self.root, self.session)
        except OSError as e:
            return ' ' + self._write_hint(e)
        self.notice = f'Session {self.session_name()} is now ACTIVE: race.launch.py will load it.'
        return ' ' + self.notice

    # ------------------------------------------------------------------ actions
    def action(self, step_key: str, action: str, argument: str, inputs: Dict) -> Dict:
        action = (action or '').upper()
        if action not in ACTIONS:
            return result(False, f'Unknown action {action!r}. Known: {", ".join(ACTIONS)}')
        if action == 'ROLLBACK':
            return self._rollback(argument)
        s = self.slot(step_key) if step_key else (self.slot(self.current) if action == 'CANCEL' else None)
        if s is None:
            return result(False, f'No calibration step {step_key!r}')
        fn = getattr(self, '_a_' + action.lower(), None)
        if fn is None:
            return result(False, f'{action} is handled by the node, not the wizard core')
        r = fn(s, argument, inputs)
        if action != 'SELECT':           # pages re-SELECT to stay watched: keep "Passed: press Save." etc.
            s.message = r['message']
        return r

    def _a_select(self, s: Slot, arg, inputs) -> Dict:
        self.current = s.index
        self.watched[s.index] = self.now()
        return result(True, f'Step {s.index} open')

    def _a_run(self, s: Slot, arg, inputs) -> Dict:
        impl = self.impls.get(s.id)
        if impl is None:
            how = self._tool_cmd(str(s.cfg.get('tool', '') or ''))
            msg = f'Step {s.index} ({s.title}) has no wizard page yet.'
            if how:
                msg += f' Meanwhile run it in a terminal: {how}'
            elif s.cfg.get('tool'):
                msg += ' Save step 1 first so a session exists, then use the terminal tool shown on this page.'
            return result(False, msg)
        if self.running is not None and self.running is not s:
            return result(False, f'Step {self.running.index} ({self.running.title}) is still running: '
                                 'wait for it or press Cancel there.')
        if self.running is s:
            return result(False, 'Already running')
        blk = self.blocker(s)
        if blk is not None:
            return result(False, self.blocked_text(blk))
        try:
            err = impl.start(self.now(), dict(inputs or {}, argument=arg or ''))
        except Exception as e:  # noqa: BLE001
            err = f'could not start: {e!r}'
        if err:
            return result(False, err)
        self.current = s.index
        self.running = s
        s.status, s.result, s.unsaved = 'RUNNING', None, False
        return result(True, f'Step {s.index} running')

    _a_redo = _a_run

    def _a_step(self, s: Slot, arg, inputs) -> Dict:
        impl = self.impls.get(s.id)
        if impl is None:
            return result(False, f'Step {s.index} has no wizard page yet')
        if self.running is s and not impl.ops_while_running:
            return result(False, 'Wait for the running measurement (or Cancel it) first')
        try:
            a = json.loads(arg or '{}')
        except ValueError:
            return result(False, f'STEP argument is not JSON: {arg!r}')
        if not isinstance(a, dict) or not a.get('op'):
            return result(False, 'STEP argument needs {"op": ...}')
        try:
            r = impl.handle(str(a['op']), a, inputs)
        except Exception as e:  # noqa: BLE001  a page operation must not stop the wizard
            return result(False, f'{a["op"]} failed: {e!r}')
        self.current = s.index
        if r.get('ok') and r.get('invalidate_result') and s.unsaved and s.status in ('PASS', 'FAIL'):
            # the op removed data an unsaved result was computed from (e.g. step 6 deleted a run)
            s.status, s.result, s.unsaved = s.saved_status or ('PENDING' if s.required else 'SKIPPED_OPTIONAL'), None, False
        return result(bool(r.get('ok')), str(r.get('message', '')))

    def _a_cancel(self, s: Slot, arg, inputs) -> Dict:
        if self.running is not s:
            return result(False, 'Nothing is running on this step')
        self.impls[s.id].cancel()
        self.running = None
        s.status = s.saved_status or ('PENDING' if s.required else 'SKIPPED_OPTIONAL')
        return result(True, f'Step {s.index} cancelled' + (f': {arg}' if arg else ''))

    def cancel_running(self, why: str) -> Optional[Dict]:
        if self.running is None:
            return None
        s = self.running
        r = self._a_cancel(s, why, {})
        s.message = r['message']
        return r

    def tick(self, inputs: Dict) -> Optional[Slot]:
        """Advance the running step. Returns the slot when it just finished."""
        for impl in self.impls.values():
            try:
                impl.background(self.session, inputs)
            except Exception:  # noqa: BLE001  a background chore must never stop the wizard
                pass
        s = self.running
        if s is None:
            return None
        try:
            res = self.impls[s.id].tick(self.now(), inputs)
        except Exception as e:  # noqa: BLE001
            res = {'passed': False, 'summary': f'step crashed: {e!r}', 'checks': []}
        if res is None:
            return None
        self.running = None
        res.setdefault('step', s.id)
        res.setdefault('time', datetime.datetime.now().isoformat(timespec='seconds'))
        s.result, s.unsaved = res, True
        s.status = 'PASS' if res.get('passed') else 'FAIL'
        # a multi-stage page may say what comes next (res['message']), e.g. step 10 after one stage
        s.message = str(res.get('message') or ('Passed: press Save.' if res.get('passed') else
                                               'Failed: fix the problems below and press Redo.'))
        return s

    def _a_save(self, s: Slot, arg, inputs) -> Dict:
        if s is self.running:
            return result(False, 'Still running: wait for the result')
        if not s.result or not s.unsaved:
            return result(False, 'Nothing new to save: press Run first')
        if not s.result.get('passed'):
            return result(False, 'Only a passing result can be saved. Fix the problems and press Redo.')
        impl = self.impls.get(s.id)
        try:
            session = self._ensure_session()
            written = impl.save_data(session, s.result) if impl is not None else []
            if written:
                s.result['data_files'] = [os.path.relpath(p, session).replace(os.sep, '/') for p in written]
            fname = f'{s.index:02d}_{s.id}.yaml'
            path = os.path.join(session, fname)
            if os.path.isfile(path):
                stamp = datetime.datetime.fromtimestamp(os.path.getmtime(path)).strftime('%H%M%S')
                shutil.move(path, os.path.join(session, f'{s.index:02d}_{s.id}.{stamp}.yaml'))
            doc = dict(s.result, session=os.path.basename(session))
            cs.write_yaml(path, doc)
            cs.update_step(session, s.id, 'PASS', fname, summary=str(s.result.get('summary', '')))
        except StepRefused as e:
            return result(False, str(e))
        except OSError as e:
            return result(False, self._write_hint(e))
        s.saved_status, s.saved_file, s.unsaved, s.status, s.from_session = 'PASS', fname, False, 'PASS', ''
        msg = f'Saved to {os.path.basename(session)}/{fname}.'
        if s.result.get('data_files'):
            msg += ' Wrote ' + ', '.join(s.result['data_files']) + '.'
        msg += self._after_save()
        self.refresh_sessions()
        return result(True, msg, True, s.result)

    def _a_keep_previous(self, s: Slot, arg, inputs) -> Dict:
        if not self.cfg['allow_keep_previous']:
            return result(False, 'Keep previous value is switched off (ops.yaml allow_keep_previous)')
        impl = self.impls.get(s.id)
        if impl is None or not impl.can_keep_previous:
            return result(False, 'Keep previous value is not available for this step yet (its page will copy the '
                                 'calibration files it needs).')
        if not s.previous:
            return result(False, 'No earlier session has a PASS for this step: it must pass now.')
        if s is self.running:
            return result(False, 'Cancel the running measurement first')
        blk = self.blocker(s)
        if blk is not None:
            return result(False, self.blocked_text(blk))
        try:
            session = self._ensure_session()
            src_dir = os.path.join(cs.calibration_dir(self.root), s.previous)
            impl.keep_data(src_dir, session)
            e = (cs.load_summary(src_dir).get('steps') or {}).get(s.id) or {}
            fname = ''
            if e.get('file') and os.path.isfile(os.path.join(src_dir, e['file'])):
                fname = e['file']
                shutil.copy2(os.path.join(src_dir, fname), os.path.join(session, fname))
            cs.update_step(session, s.id, 'KEPT_PREVIOUS', fname, from_session=s.previous)
        except StepRefused as e:
            return result(False, str(e))
        except OSError as e:
            return result(False, self._write_hint(e))
        s.saved_status, s.saved_file, s.status, s.from_session = 'KEPT_PREVIOUS', fname, 'KEPT_PREVIOUS', s.previous
        s.result, s.unsaved = None, False
        msg = f'Keeping the value from {s.previous}.' + self._after_save()
        self.refresh_sessions()
        return result(True, msg, True)

    def _rollback(self, name: str) -> Dict:
        name = (name or '').strip()
        if not name or name not in cs.list_sessions(self.root):
            return result(False, f'No session {name!r}. Available: {", ".join(cs.list_sessions(self.root)) or "none"}')
        summ = cs.load_summary(os.path.join(cs.calibration_dir(self.root), name))
        steps = summ.get('steps') or {}
        missing = [s.index for s in self.slots if s.required and (steps.get(s.id) or {}).get('status') not in PASSING]
        try:
            cs.set_active(self.root, os.path.join(cs.calibration_dir(self.root), name))
        except OSError as e:
            return result(False, self._write_hint(e))
        self.refresh_sessions()
        msg = f'{name} is now ACTIVE.'
        if missing:
            msg += f' Warning: steps {", ".join(map(str, missing))} are not passed in it, so race mode will refuse to arm.'
        return result(True, msg, not missing)
