"""Calibration step 10 -- UWB anchor survey + per-anchor range offsets (pure, unit tested).

The terminal tool (uwb_localization.calib_uwb) as a wizard page. Same math
(uwb_core: layout_checks, hdop_coverage, compute_offsets, trilaterate) on the raw
tag JSON (carbot_ops.wizard_uwb feed, inputs['uwb_raw']). One stage per Run;
RUN/REDO argument JSON:

  {"stage": "link"}                          procedure.link_check_s listen: every anchor
                                             (uwb.yaml + surveyed) heard, unknown ids listed
  {"stage": "survey", "anchors": [{"id": "1782", "xyz_m": [x, y, z]}, ...],
   "tag": {"z_m": z, "mount_xy_m": [fwd, left]}}
                                             tape-measured positions, METRES. Checks spacing,
                                             triangle angle, HDOP coverage, anchors on the floor.
                                             Every uwb.yaml anchor must be given; a new id may be
                                             added (4th anchor: firmware RangeProtocol.h too)
  {"stage": "offsets", "spot": [x, y]}       tag still at a measured spot (>= min_distance_from_
                                             anchor_m from every anchor), procedure.seconds:
                                             offset = median(raw 3-D R) - tape 3-D distance
  {"stage": "verify", "spot": [x, y]}        tag still at a measured spot: median of Haffiz's
                                             FILTERED position (solver + CV Kalman filter, the
                                             /carbot/uwb/position the car uses) within
                                             pass.max_verify_error_m. If offsets were measured,
                                             it must be a DIFFERENT spot (>= min_verify_separation_m).

Haffiz switch: procedure.offsets_mode
  optional  (default) Haffiz uses no offsets. Verify may run right after link +
            survey; if it passes, offsets are saved as 0.0. If it fails with ranges
            that read long / short, the page says: Measure offsets, then Verify again.
  required  the old order: offsets must pass before verify.
Order: offsets needs link + survey passed. Changing the survey clears offsets and
verify. The step PASSES when link, survey and verify passed (+ offsets when required)
and the link check covered every surveyed anchor.

Save merges ONLY these keys into <session>/data/uwb.yaml (calib_tools.merge_data):
anchors (id, xyz_m, range_offset_m), tag.z_m, tag.mount_xy_m, anchors_surveyed,
offsets_calibrated. Step 11 writes track_to_venue into the same file; it is never
touched here. Keep previous copies the same keys from the older session and
refuses when that survey does not cover every anchor configured now.

UWB is read-only here: nothing reaches the servo or the local pose.
"""
import json
import math
import os
import statistics
from typing import Dict, List, Optional, Tuple

from carbot_common import calib_tools as ct
from uwb_localization.calib_uwb import spot_ok, venue_points
from uwb_localization.positioning import PositioningCfg
from uwb_localization.uwb_core import AnchorSet, compute_offsets, fix_clusters, hdop_coverage, layout_checks

from . import wizard_uwb as wu
from .wizard_core import StepImpl, StepRefused

PROC_KEYS = ('seconds', 'min_distance_from_anchor_m', 'link_check_s', 'min_samples', 'min_spacing_m',
             'min_triangle_angle_deg', 'hdop_limit', 'max_spread_m', 'min_verify_separation_m', 'min_fixes',
             'max_extent_m', 'verify_settle_fixes')
OFFSETS_MODES = ('optional', 'required')
PASS_KEYS = ('max_verify_error_m', 'all_anchors_seen', 'min_hdop_coverage')
UWB_KEYS = ('anchors', 'tag', 'anchors_surveyed', 'offsets_calibrated')
STAGES = ('link', 'survey', 'offsets', 'verify')
LABEL = {'link': 'Link check', 'survey': 'Anchor + tag survey', 'offsets': 'Offsets (tag still)',
         'verify': 'Verify at a different spot'}
# uwb.yaml keys this step writes (= calibration_steps.yaml uwb_survey.writes)
WRITES = ('anchors', 'tag.z_m', 'tag.mount_xy_m', 'anchors_surveyed', 'offsets_calibrated')


class ConfigError(ValueError):
    pass


def _num(v, what: str) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ValueError(f'{what}: {v!r} is not a number')
    if not math.isfinite(f):
        raise ValueError(f'{what}: {v!r} is not a number')
    return f


def _vec(v, n: int, what: str) -> List[float]:
    if isinstance(v, str):
        v = v.replace(',', ' ').split()
    if not isinstance(v, (list, tuple)) or len(v) != n:
        raise ValueError(f'{what}: need {n} numbers, got {v!r}')
    return [_num(x, what) for x in v]


def parse_argument(arg: str) -> Tuple[Optional[Dict], str]:
    """RUN argument -> ({'stage', ...parsed values}, '') or (None, why refused)."""
    try:
        a = json.loads(arg) if arg else None
    except ValueError:
        a = None
    if not isinstance(a, dict) or a.get('stage') not in STAGES:
        return None, ('Choose what to run on this page: Link check, Save survey, Measure offsets or Verify '
                      f'(argument {{"stage": {" | ".join(STAGES)}}}).')
    out = {'stage': a['stage']}
    try:
        if a['stage'] == 'survey':
            anchors = a.get('anchors')
            if not isinstance(anchors, list) or not anchors:
                raise ValueError('anchors: give every anchor id with x, y, height')
            seen, rows = set(), []
            for x in anchors:
                if not isinstance(x, dict) or not str(x.get('id', '')).strip():
                    raise ValueError(f'anchor entry {x!r} has no id')
                aid = str(x['id']).strip().upper()
                if aid in seen:
                    raise ValueError(f'anchor {aid} is listed twice')
                seen.add(aid)
                rows.append({'id': aid, 'xyz_m': _vec(x.get('xyz_m'), 3, f'anchor {aid} x y height')})
            tag = a.get('tag') or {}
            out['anchors'] = rows
            out['tag'] = {'z_m': _num(tag.get('z_m'), 'tag antenna height'),
                          'mount_xy_m': _vec(tag.get('mount_xy_m'), 2, 'tag position on the car')}
        elif a['stage'] in ('offsets', 'verify'):
            out['spot'] = _vec(a.get('spot'), 2, f'{LABEL[a["stage"]]} spot x y')
    except ValueError as e:
        return None, f'{LABEL[a["stage"]]}: {e}.'
    return out, ''


def _check(key, label, measured, limit, passed, why='', fix='', **kw) -> Dict:
    return dict({'key': key, 'label': label, 'measured': measured, 'limit': limit, 'passed': bool(passed),
                 'why': '' if passed else why, 'fix': '' if passed else fix}, **kw)


class UwbSurveyStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, uwb: Dict, config_dir: str = '', pos_cfg: Optional[PositioningCfg] = None):
        super().__init__(cfg)
        self.pos_cfg = pos_cfg or PositioningCfg()
        self.proc, self.pas = cfg.get('procedure') or {}, cfg.get('pass') or {}
        for where, d, keys in (('procedure', self.proc, PROC_KEYS), ('pass', self.pas, PASS_KEYS)):
            miss = [k for k in keys if k not in d]
            if miss:
                raise ConfigError(f'calibration_steps.yaml uwb_survey.{where}: missing {", ".join(miss)}')
        miss = [k for k in UWB_KEYS if k not in (uwb or {})] + \
               [f'tag.{k}' for k in ('z_m', 'mount_xy_m') if k not in ((uwb or {}).get('tag') or {})]
        if miss:
            raise ConfigError(f'uwb.yaml: missing {", ".join(miss)}')
        self.base = uwb
        self.configured = wu.anchor_set(uwb, zero_offsets=True).ids      # ValueError if < 3 anchors
        self.config_dir = config_dir
        self.p = {k: float(self.proc[k]) for k in PROC_KEYS}
        self.offsets_mode = str(self.proc.get('offsets_mode', ''))
        if self.offsets_mode not in OFFSETS_MODES:
            raise ConfigError(f'calibration_steps.yaml uwb_survey.procedure.offsets_mode: '
                              f'{self.offsets_mode!r} (use {" | ".join(OFFSETS_MODES)})')
        self.max_err = float(self.pas['max_verify_error_m'])
        self.min_cov = float(self.pas['min_hdop_coverage'])
        self.need_all = bool(self.pas['all_anchors_seen'])
        self.stages: Dict[str, Optional[Dict]] = {k: None for k in STAGES}
        self.survey: Optional[Dict] = None          # {'anchors': [{id, xyz_m}], 'tag': {z_m, mount_xy_m}}
        self.offsets: Dict[str, float] = {}
        self.run: Optional[Dict] = None
        base_anchors = [{'id': str(a['id']).upper(), 'xyz_m': [float(v) for v in a['xyz_m']]} for a in uwb['anchors']]
        self.base_survey = {'anchors': base_anchors,
                            'tag': {'z_m': float(uwb['tag']['z_m']),
                                    'mount_xy_m': [float(v) for v in uwb['tag']['mount_xy_m']]}}
        self.base_geometry = self._geometry(self.base_survey)

    # ------------------------------------------------------------------ geometry
    def _doc(self, survey: Dict, offsets: Optional[Dict[str, float]] = None) -> Dict:
        return dict(self.base, tag=dict(self.base['tag'], **survey['tag']),
                    anchors=[{'id': a['id'], 'xyz_m': list(a['xyz_m']),
                              'range_offset_m': round(float((offsets or {}).get(a['id'], 0.0)), 4)}
                             for a in survey['anchors']])

    def anchors(self, offsets: bool = False) -> AnchorSet:
        return AnchorSet.from_yaml(self._doc(self.survey or self.base_survey, self.offsets if offsets else None))

    def ids(self) -> List[str]:
        return sorted(a['id'] for a in (self.survey or self.base_survey)['anchors'])

    def _geometry(self, survey: Dict) -> Dict:
        """Layout errors / warnings + HDOP coverage (calib_uwb.check_layout, without printing)."""
        errors, warnings = [], []
        try:
            aset = AnchorSet.from_yaml(self._doc(survey))
        except ValueError as e:
            return {'ok': False, 'errors': [str(e)], 'warnings': [], 'hdop_coverage': 0.0, 'hdop_worst': None,
                    'hdop_area': ''}
        for p in layout_checks(aset, self.p['min_spacing_m'], self.p['min_triangle_angle_deg']):
            (warnings if 'on the floor' in p else errors).append(p)
        xs = [a.x for a in aset.anchors.values()]
        ys = [a.y for a in aset.anchors.values()]
        ext = max(max(xs) - min(xs), max(ys) - min(ys))
        if ext > self.p['max_extent_m']:
            errors.append(f'anchors span {ext:.1f} m (limit {self.p["max_extent_m"]:g} m): '
                          'positions must be in METRES, not cm')
        for aid in aset.ids:
            if aset.anchors[aid].z < 0:
                errors.append(f'anchor {aid} height {aset.anchors[aid].z:.2f} m is below the floor')
        if aset.tag_z < 0:
            errors.append(f'tag antenna height {aset.tag_z:.2f} m is below the floor')
        pts, where = venue_points(self.base, aset, self.config_dir)
        cov, worst = hdop_coverage(aset, pts, self.p['hdop_limit'])
        ok = not errors and cov >= self.min_cov
        return {'ok': ok, 'errors': errors, 'warnings': warnings, 'hdop_coverage': round(cov, 3),
                'hdop_worst': round(worst, 2) if math.isfinite(worst) else None, 'hdop_area': where}

    # ------------------------------------------------------------------ StepImpl
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        a, err = parse_argument(inputs.get('argument', ''))
        if a is None:
            return err
        st = a['stage']
        if st == 'survey':
            missing = [i for i in self.configured if i not in {x['id'] for x in a['anchors']}]
            if missing:
                return (f'Survey: anchors {", ".join(missing)} are in uwb.yaml (and the tag firmware) but have no '
                        'position: enter every anchor.')
            if len(a['anchors']) < 3:
                return 'Survey: at least 3 anchors are needed.'
            self.run = {'stage': st, 't0': now, 'args': a}
            return None
        feed = inputs.get(wu.INPUT_KEY)
        if feed is None:
            return 'No UWB feed in the calibration wizard (node started without it): relaunch calibrate.launch.py.'
        if st in ('offsets', 'verify'):
            need = [k for k in (('link', 'survey') if st == 'offsets' or not self._offsets_required()
                                else ('offsets',))
                    if (self.stages[k] or {}).get('status') != 'PASS']
            if need:
                return f'{LABEL[st]}: pass {" and ".join(LABEL[k] for k in need)} first.'
            if not self._link_covers():
                return (f'{LABEL[st]}: the survey has anchors the link check did not listen for '
                        f'({", ".join(self._link_missing())}): run the Link check again.')
            x, y = a['spot']
            bad = spot_ok(self.anchors(), x, y, self.p['min_distance_from_anchor_m'])
            if bad:
                return (f'{LABEL[st]}: spot ({x:.2f}, {y:.2f}) is closer than '
                        f'{self.p["min_distance_from_anchor_m"]:g} m to anchor {", ".join(bad)}: pick a spot '
                        'inside the triangle, away from every anchor.')
            if st == 'verify' and self._offsets_done():
                ox, oy = self.stages['offsets']['spot_m']
                d = math.hypot(x - ox, y - oy)
                if d < self.p['min_verify_separation_m']:
                    return (f'Verify: spot is only {d:.2f} m from the offset spot ({ox:.2f}, {oy:.2f}); move it at '
                            f'least {self.p["min_verify_separation_m"]:g} m away, otherwise it checks nothing.')
        secs = self.p['link_check_s'] if st == 'link' else self.p['seconds']
        self.run = {'stage': st, 't0': now, 'args': a, 'secs': secs, 'cursor': feed.cursor(), 'rows': [],
                    'lost': 0, 'ids': self.ids()}
        return None

    def cancel(self) -> None:
        self.run = None

    def running_stage(self) -> str:
        return self.run['stage'] if self.run else ''

    def progress(self, now: float) -> Dict:
        r = self.run
        if r is None or 'secs' not in r:
            return {'stage': r['stage']} if r else {}
        el = now - r['t0']
        return {'stage': r['stage'], 'elapsed_s': round(el, 1), 'remaining_s': round(max(0.0, r['secs'] - el), 1),
                'fraction': round(min(1.0, el / r['secs']), 2), 'samples': len(r['rows'])}

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        r = self.run
        if r is None:
            return None
        st = r['stage']
        if st == 'survey':
            self._finish_survey(r['args'])
            self.run = None
            return self._result(st)
        feed = inputs.get(wu.INPUT_KEY)
        if feed is not None:
            rows, r['cursor'], lost = feed.rows_since(r['cursor'])
            r['rows'].extend(rows)
            r['lost'] += lost
        el = now - r['t0']
        if st != 'link' and el >= self.p['link_check_s'] and not r['rows']:
            self.stages[st] = {'status': 'FAIL', 'spot_m': r['args']['spot'], 'fixes': 0, 'reports': 0,
                               'why': f'no UWB tag report in {self.p["link_check_s"]:g} s',
                               'fix': 'Tag powered and on WiFi? micro-ROS agent running? Step 1 must show UWB green.'}
            if st == 'offsets':
                self.offsets, self.stages['verify'] = {}, None
            self.run = None
            return self._result(st)
        if el < r['secs']:
            return None
        self.run = None
        getattr(self, '_finish_' + st)(r)
        return self._result(st)

    # ------------------------------------------------------------------ stages
    def _finish_survey(self, a: Dict) -> None:
        new = {'anchors': sorted(a['anchors'], key=lambda x: x['id']), 'tag': a['tag']}
        geo = self._geometry(new)
        changed = new != self.survey
        self.survey = new
        if changed:
            self.offsets = {}
            self.stages['offsets'] = self.stages['verify'] = None
        added = [x['id'] for x in new['anchors'] if x['id'] not in self.configured]
        why, fix = [], []
        if geo['errors']:
            why += geo['errors']
            fix.append('Correct the numbers (metres, all in one frame of your choice) or move the anchors.')
        if geo['hdop_coverage'] < self.min_cov:
            why.append(f'position accuracy is good (HDOP <= {self.p["hdop_limit"]:g}) over only '
                       f'{geo["hdop_coverage"] * 100:.0f} % of the {geo["hdop_area"]}')
            fix.append('Move the anchors so they surround the track, or add a 4th anchor.')
        self.stages['survey'] = {'status': 'PASS' if geo['ok'] else 'FAIL', 'geometry': geo, 'added': added,
                                 'anchors': new['anchors'], 'tag': new['tag'],
                                 'why': '; '.join(why), 'fix': ' '.join(fix)}

    def _link_missing(self) -> List[str]:
        heard = (self.stages['link'] or {}).get('fresh_per_anchor') or {}
        return [i for i in self.ids() if heard.get(i, 0) < self._link_min()]

    def _link_covers(self) -> bool:
        return (self.stages['link'] or {}).get('status') == 'PASS' and (not self.need_all or not self._link_missing())

    def _link_min(self) -> int:
        return int(max(3, self.p['link_check_s']))            # calib_uwb.check_link: >= 1 fresh sample / s

    def _finish_link(self, r: Dict) -> None:
        samples, unknown, n = wu.fresh_samples(r['rows'])
        per = {aid: len(samples.get(aid, [])) for aid in sorted(set(r['ids']) | set(samples))}
        missing = [aid for aid in r['ids'] if per.get(aid, 0) < self._link_min()]
        unknown = sorted(set(unknown) | {a for a in samples if a not in r['ids']})
        why, fix = [], []
        if n == 0:
            why.append(f'nothing received in {r["secs"]:g} s')
            fix.append('Is the micro-ROS agent running and the tag powered? ROS_LOCALHOST_ONLY=0, ROS_DOMAIN_ID=1 '
                       '(step 1 checks both).')
        elif missing:
            why.append(f'anchor {", ".join(missing)} missing or weak (< {self._link_min()} new samples)')
            fix.append('Anchor powered? Same radio mode as the tag? Clear line of sight to the tag?')
        if unknown:
            fix.append(f'Heard anchor ids not surveyed: {", ".join(unknown)}: add them to RangeProtocol.h and to '
                       'the survey, or switch them off.')
        ok = n > 0 and (not missing or not self.need_all)
        self.stages['link'] = {'status': 'PASS' if ok else 'FAIL', 'reports': n,
                               'hz': round(n / max(r['secs'], 1e-6), 1), 'fresh_per_anchor': per,
                               'unknown_ids': unknown, 'why': '; '.join(why), 'fix': ' '.join(fix)}

    def _finish_offsets(self, r: Dict) -> None:
        x, y = r['args']['spot']
        zero = self.anchors()
        samples, _, n = wu.fresh_samples(r['rows'])
        res = compute_offsets(zero, samples, x, y, int(self.p['min_samples']))
        table, off, noisy = {}, {}, []
        for aid in zero.ids:
            o = res.get(aid)
            if o is None:
                table[aid] = {'samples': len(samples.get(aid, [])), 'offset_m': None}
                continue
            table[aid] = {'samples': o.samples, 'measured_m': round(o.measured_m, 4), 'true_m': round(o.true_m, 4),
                          'offset_m': round(o.offset_m, 4), 'spread_m': round(o.spread_m, 4)}
            off[aid] = o.offset_m
            if o.spread_m > self.p['max_spread_m']:
                noisy.append(aid)
        few = [aid for aid in zero.ids if aid not in off]
        ok = not few
        self.offsets = off if ok else {}
        self.stages['verify'] = None
        self.stages['offsets'] = {
            'status': 'PASS' if ok else 'FAIL', 'spot_m': [x, y], 'reports': n, 'lost': r['lost'],
            'anchors': table, 'noisy': noisy,
            'why': f'anchor {", ".join(few)}: fewer than {int(self.p["min_samples"])} samples in {r["secs"]:g} s'
                   if few else '',
            'fix': 'Is that anchor on and in line of sight? Then measure again.' if few else
                   ('Noisy anchors (multipath?): raise them / clear the line of sight.' if noisy else '')}

    def _finish_verify(self, r: Dict) -> None:
        """Median of Haffiz's FILTERED position (what the car uses) vs the tape spot. The first
        verify_settle_fixes are skipped while the Kalman filter converges."""
        x, y = r['args']['spot']
        with_offsets = self._offsets_done()
        fx_all = wu.positions(self.anchors(offsets=with_offsets), r['rows'], self.pos_cfg)
        settle = int(self.p['verify_settle_fixes'])
        fx = fx_all[settle:] if len(fx_all) > settle else []
        need = int(self.p['min_fixes'])
        if len(fx) < need:
            self.stages['verify'] = {'status': 'FAIL', 'spot_m': [x, y], 'fixes': len(fx), 'with_offsets': with_offsets,
                                     'why': f'only {len(fx)} position fixes after the first {settle} '
                                            f'(need {need}; every report needs >= {self.pos_cfg.min_anchors} anchors)',
                                     'fix': 'Check every anchor is seen (Link check), then Verify again.'}
            return
        filt = [f.xy for f in fx]
        raw = [f.raw for f in fx]
        mx = statistics.median(p[0] for p in filt)
        my = statistics.median(p[1] for p in filt)
        rx = statistics.median(p[0] for p in raw)
        ry = statistics.median(p[1] for p in raw)
        err = math.hypot(mx - x, my - y)
        jitter, flip = fix_clusters(filt)
        raw_jitter, raw_flip = fix_clusters(raw)
        gated = sum(1 for f in fx if not f.accepted)
        ok = err <= self.max_err
        if ok:
            fix = ''
        elif not with_offsets:
            fix = ('No range offsets are applied (Haffiz uses none). If the tape numbers are right, the ranges '
                   'read long / short (uncalibrated antenna delay, ~1 m before): put the tag at a measured spot and '
                   'press Measure offsets, then Verify at a different spot.')
        else:
            fix = ('Re-check the tape measurements (anchor x, y, height; tag height) and both spots, '
                   'then redo Offsets and Verify.')
        if not ok and raw_flip > 0:
            fix += (f' Raw fixes flip between two clusters {raw_flip * 100:.0f} cm apart (multipath): '
                    'raise the anchors / clear the line of sight.')
        self.stages['verify'] = {
            'status': 'PASS' if ok else 'FAIL', 'spot_m': [x, y], 'fixes': len(fx), 'with_offsets': with_offsets,
            'method': f'{self.pos_cfg.solver} + {self.pos_cfg.filter}',
            'median_fix_m': [round(mx, 4), round(my, 4)], 'error_m': round(err, 4),
            'raw_median_m': [round(rx, 4), round(ry, 4)], 'raw_error_m': round(math.hypot(rx - x, ry - y), 4),
            'jitter_m': round(jitter, 4), 'flip_flop_m': round(flip, 4),
            'raw_jitter_m': round(raw_jitter, 4), 'raw_flip_flop_m': round(raw_flip, 4), 'gated': gated,
            'why': '' if ok else f'median position ({mx:.2f}, {my:.2f}) is {err * 100:.1f} cm from the tape spot',
            'fix': fix}

    def _offsets_required(self) -> bool:
        return self.offsets_mode == 'required'

    def _offsets_done(self) -> bool:
        return (self.stages['offsets'] or {}).get('status') == 'PASS'

    # ------------------------------------------------------------------ result
    def _next(self) -> str:
        for k in STAGES:
            if k == 'offsets' and not self._offsets_required() and not self._offsets_done():
                v = self.stages['verify'] or {}
                if v.get('status') == 'PASS':
                    continue                      # verified without offsets: offsets not needed
                if v.get('status') == 'FAIL' and not v.get('with_offsets'):
                    return 'offsets'              # failed without offsets: measure them next
                continue
            if (self.stages[k] or {}).get('status') != 'PASS':
                return k
        return '' if self._link_covers() else 'link'

    def _result(self, just: str) -> Dict:
        s = self.stages
        checks = []
        lk = s['link']
        if lk is None:
            checks.append(_check('link', LABEL['link'], 'not run', 'every anchor heard', False,
                                 'not run yet', 'Press Link check.'))
        else:
            miss = self._link_missing()
            ok = lk['status'] == 'PASS' and (not miss or not self.need_all)
            checks.append(_check('link', LABEL['link'],
                                 f'{lk["reports"]} reports, {lk["hz"]:g} Hz; ' +
                                 ', '.join(f'{a} {k}' for a, k in lk['fresh_per_anchor'].items()),
                                 f'>= {self._link_min()} samples per anchor', ok,
                                 lk['why'] or (f'surveyed anchors {", ".join(miss)} were not heard' if miss else ''),
                                 lk['fix'] or 'Run the Link check again.'))
        sv = s['survey']
        if sv is None:
            checks.append(_check('survey', LABEL['survey'], 'not entered', 'metres, geometry ok', False,
                                 'not entered yet', 'Fill in the anchor and tag form and press Save survey.'))
        else:
            g = sv['geometry']
            checks.append(_check('layout', 'Anchor layout', f'{len(sv["anchors"])} anchors, '
                                 f'{len(g["errors"])} problems, {len(g["warnings"])} warnings',
                                 f'>= {self.p["min_spacing_m"]:g} m apart, angle >= '
                                 f'{self.p["min_triangle_angle_deg"]:g} deg', not g['errors'],
                                 '; '.join(g['errors']), 'Correct the numbers or move the anchors.'))
            checks.append(_check('hdop', f'Accuracy coverage ({g["hdop_area"]})',
                                 f'{g["hdop_coverage"] * 100:.0f} %', f'>= {self.min_cov * 100:.0f} % with HDOP <= '
                                 f'{self.p["hdop_limit"]:g}', g['hdop_coverage'] >= self.min_cov,
                                 'anchors do not surround the track well enough',
                                 'Move the anchors so they surround the track, or add a 4th anchor.'))
        of = s['offsets']
        if of is None and not self._offsets_required():
            checks.append(_check('offsets', LABEL['offsets'], 'not measured (optional: Haffiz uses none)',
                                 'only if Verify fails', True))
        elif of is None:
            checks.append(_check('offsets', LABEL['offsets'], 'not run', 'every anchor', False, 'not measured yet',
                                 'Put the tag still on a measured spot and press Measure offsets.'))
        else:
            meas = ', '.join(f'{a} {v["offset_m"]:+.3f} m' if v.get('offset_m') is not None else f'{a} -'
                             for a, v in (of.get('anchors') or {}).items()) or of.get('why', '')
            checks.append(_check('offsets', LABEL['offsets'], meas,
                                 f'>= {int(self.p["min_samples"])} samples per anchor',
                                 of['status'] == 'PASS', of['why'], of['fix']))
        vf = s['verify']
        if vf is None:
            checks.append(_check('verify', LABEL['verify'], 'not run', f'<= {self.max_err * 100:.0f} cm', False,
                                 'not verified yet', 'Move the tag to a different measured spot and press Verify.'))
        else:
            checks.append(_check('verify', LABEL['verify'],
                                 f'{vf["error_m"] * 100:.1f} cm ({vf["fixes"]} filtered fixes, '
                                 f'{"with" if vf.get("with_offsets") else "no"} offsets)' if 'error_m' in vf
                                 else f'{vf["fixes"]} fixes', f'<= {self.max_err * 100:.0f} cm',
                                 vf['status'] == 'PASS', vf['why'], vf['fix']))
        nxt = self._next()
        passed = not nxt
        done = sum(1 for k in STAGES if (s[k] or {}).get('status') == 'PASS')
        total = len(STAGES) if self._offsets_required() or self._offsets_done() else len(STAGES) - 1
        last = s.get(just) or {}
        if passed:
            summary = (f'verify error {vf["error_m"] * 100:.1f} cm ({vf.get("method", "")}); offsets ' +
                       (', '.join(f'{a} {o:+.3f}' for a, o in sorted(self.offsets.items())) + ' m'
                        if self.offsets else 'none (0.0)'))
            message = 'All stages passed: press Save.'
        else:
            summary = f'{done} of {total} stages passed; next: {LABEL[nxt]}'
            if last.get('status') == 'PASS':
                message = f'{LABEL[just]} passed. Next: {LABEL[nxt]}.'
            else:
                message = f'{LABEL[just]} failed: ' + (last.get('why') or 'see below') + '.'
        out = {'passed': passed, 'summary': summary, 'message': message, 'checks': checks, 'next': nxt,
               'stages': {k: v for k, v in s.items()}, 'last_stage': just}
        if self.survey:
            out['survey'] = self.survey
        if passed:
            out['uwb'] = self.updates()
        return out

    def updates(self) -> Dict:
        """The uwb.yaml keys Save writes (only these; track_to_venue etc. stay as they are)."""
        doc = self._doc(self.survey, self.offsets)
        return {'anchors': doc['anchors'],
                'tag': {'z_m': float(self.survey['tag']['z_m']),
                        'mount_xy_m': [float(v) for v in self.survey['tag']['mount_xy_m']]},
                'anchors_surveyed': True, 'offsets_calibrated': True}

    # ------------------------------------------------------------------ live view
    def live(self, inputs: Dict) -> Dict:
        feed = inputs.get(wu.INPUT_KEY)
        ids = self.ids()
        extra = [i for i in ids if i not in self.configured]
        st = feed.stats(ids) if feed is not None else None
        r = self.run
        run_samples = None
        if r is not None and r.get('rows'):
            smp, _, _ = wu.fresh_samples(r['rows'])
            run_samples = {a: {'samples': len(v), 'spread_m': round(statistics.pstdev(v), 4) if len(v) > 1 else None,
                               'median_m': round(statistics.median(v), 3)} for a, v in smp.items()}
        pos = {a['id']: a['xyz_m'] for a in (self.survey or self.base_survey)['anchors']}
        of = self.stages['offsets'] or {}
        rows = []
        for aid in ids:
            a = (st or {}).get('anchors', {}).get(aid) or {}
            rs = (run_samples or {}).get(aid) or {}
            spread = rs.get('spread_m') if rs else ((of.get('anchors') or {}).get(aid) or {}).get('spread_m')
            rows.append({'id': aid, 'xyz_m': pos.get(aid), 'configured': aid in self.configured,
                         'age_s': a.get('age_s'), 'rate_hz': a.get('rate_hz'), 'last_raw_m': a.get('last_raw_m'),
                         'fresh_count': a.get('fresh_count', 0),
                         'samples': rs.get('samples') if rs else ((of.get('anchors') or {}).get(aid) or {}).get('samples'),
                         'spread_m': spread, 'noisy': spread is not None and spread > self.p['max_spread_m'],
                         'offset_m': self.offsets.get(aid), 'saved_offset_m': next(
                             (float(x.get('range_offset_m', 0.0)) for x in self.base['anchors']
                              if str(x['id']).upper() == aid), None)})
        stages = []
        for k in STAGES:
            v = self.stages[k]
            state = ('running' if r and r['stage'] == k else 'pass' if (v or {}).get('status') == 'PASS'
                     else 'fail' if v else 'todo')
            if k == 'offsets' and state == 'todo' and not self._offsets_required():
                state = 'optional'
            stages.append({'key': k, 'label': LABEL[k], 'state': state,
                           'why': (v or {}).get('why', ''), 'fix': (v or {}).get('fix', '')})
        geo = (self.stages['survey'] or {}).get('geometry') or self.base_geometry
        return {
            'configured': list(self.configured), 'extra_anchors': extra, 'anchors': rows, 'feed': st,
            'form': self.survey or self.base_survey, 'entered': self.survey is not None,
            'geometry': geo, 'stages': stages, 'next': self._next(),
            'run': ({'stage': r['stage'], 'reports': len(r.get('rows') or []), 'lost': r.get('lost', 0),
                     'spot_m': (r['args'] or {}).get('spot')} if r else None),
            'link': self.stages['link'], 'offsets': self.stages['offsets'], 'verify': self.stages['verify'],
            'offset_spot_m': of.get('spot_m'),
            'saved_flags': {'anchors_surveyed': bool(self.base['anchors_surveyed']),
                            'offsets_calibrated': bool(self.base['offsets_calibrated'])},
            'offsets_mode': self.offsets_mode, 'method': f'{self.pos_cfg.solver} + {self.pos_cfg.filter}',
            'limits': {'min_distance_from_anchor_m': self.p['min_distance_from_anchor_m'],
                       'min_verify_separation_m': self.p['min_verify_separation_m'],
                       'max_verify_error_m': self.max_err, 'max_spread_m': self.p['max_spread_m'],
                       'seconds': self.p['seconds'], 'link_check_s': self.p['link_check_s'],
                       'min_samples': int(self.p['min_samples']), 'hdop_limit': self.p['hdop_limit'],
                       'min_hdop_coverage': self.min_cov}}

    # ------------------------------------------------------------------ save / keep
    def save_data(self, session: str, res: Dict) -> List[str]:
        upd = res.get('uwb')
        if not res.get('passed') or not upd:
            raise StepRefused('Nothing to save: link, survey and verify must pass first.')
        return [ct.merge_data(session, 'uwb.yaml', self.base, upd)]

    def keep_data(self, src_session: str, session: str) -> List[str]:
        name = os.path.basename(src_session)
        path = os.path.join(src_session, 'data', 'uwb.yaml')
        if not os.path.isfile(path):
            raise StepRefused(f'{name} has no data/uwb.yaml with the anchor survey: run this step again.')
        old = ct.load_yaml(path)
        if not old.get('anchors_surveyed') or not old.get('offsets_calibrated'):
            raise StepRefused(f'The UWB survey in {name} is not complete (anchors_surveyed / offsets_calibrated '
                              'false): run this step again.')
        try:
            old_ids = wu.anchor_set(old).ids
            tag = old['tag']
            upd = {'anchors': old['anchors'],
                   'tag': {'z_m': float(tag['z_m']), 'mount_xy_m': [float(v) for v in tag['mount_xy_m']]},
                   'anchors_surveyed': True, 'offsets_calibrated': True}
        except (KeyError, TypeError, ValueError) as e:
            raise StepRefused(f'data/uwb.yaml in {name} is broken ({e}): run this step again.')
        missing = [i for i in self.configured if i not in old_ids]
        if missing:
            raise StepRefused(f'{name} surveyed anchors {", ".join(old_ids)}, but uwb.yaml now also has '
                              f'{", ".join(missing)}: run this step again.')
        return [ct.merge_data(session, 'uwb.yaml', self.base, upd)]
