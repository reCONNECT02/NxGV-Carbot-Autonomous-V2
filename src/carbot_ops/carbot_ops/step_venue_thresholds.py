"""Calibration step 9 -- venue colour / lighting thresholds (pure, unit tested).

road_perception classifies every grid cell with the V4 rule (perception.js,
road_mask.RoadMask.process):

    luma = (r + g + b) / 3, chroma = max - min
    ROAD   if luma < road_max_luma and chroma < road_max_chroma
    PAINT  elif luma > paint_min_luma
    OTHER  otherwise

The V4 numbers (105 / 50 / 190) are simulator settings. This step measures the
real track under the venue lighting and proposes the three numbers.

Measured from what road_perception ALREADY publishes (no raw camera images in
the wizard, RDK CPU is tight, BACKLOG #23):
* /carbot/perception/road_grid (LocalGrid): the cell kinds road_perception
  produces right now -> "now" coverage / false paint, live, and which cells are
  seen at all.
* /carbot/perception/debug/stitched/compressed: the stitched colour of every
  cell (the exact colour the rule above is applied to, JPEG-compressed). Only
  subscribed while a sample is being taken (road_perception encodes it only
  while someone subscribes). Paired with the grid by header stamp.

Procedure (RUN argument = phase):
  road    car on a straight lane, the sample box ahead (procedure.roi) on bare road,
          lane tape visible beside it. Sample colours for sample_s.
  tunnel  (pass.tunnel_dark_check) car in the tunnel, box on the tunnel floor.
  apply   set the proposed values live on road_perception (ROS parameters), so
          the Perception tab shows the effect before saving.
  revert  put back the values road_perception had before this step changed them.

Proposal (every margin/percentile in calibration_steps.yaml procedure):
  road_max_luma   = P(road luma, road_percentile) + luma_margin      (all samples)
  road_max_chroma = P(road chroma, road_percentile) + chroma_margin
  paint_min_luma  = midpoint between the road's bright end and the tape's dark end
                    (tape = cells outside the box brighter than the road's bright
                    end + paint_gap; dark end = P(tape luma, paint_percentile)),
                    at least min_paint_margin above the road's bright end.

Pass (calibration_steps.yaml pass), with the PROPOSED values, per sample:
  road coverage   = share of the box cells classified ROAD   >= min_road_coverage
  false paint     = share of the box cells classified PAINT  <= max_false_paint_ratio
  tunnel_dark_check: the tunnel sample must pass the same two limits and its box
                  must not be black (mean luma >= tunnel_min_luma).
plus: lane tape seen in the open-road sample, decoded colours agree with
road_perception's own kinds (decode check), and the values are applied live and
read back from road_perception.

bpu_detector.thresholds.* are NOT changed here: those are detector confidences
that need labelled light / gate footage at the venue (BACKLOG #15, #16), not a
colour measurement. They stay in the Tuning / Detections tab.

Save writes params_overlay.yaml road_perception.classify.*; Keep previous copies
those three values from the older session's overlay (BACKLOG #21).
"""
import os
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from carbot_common import calib_tools as ct
from carbot_common.data import sensor_enabled

from .wizard_core import StepImpl, StepRefused

NODE = 'road_perception'
PARAMS = ('road_max_luma', 'road_max_chroma', 'paint_min_luma')
PARAM_NAMES = tuple(f'classify.{p}' for p in PARAMS)
PROC_KEYS = ('sample_s', 'min_frames', 'frame_timeout_s', 'roi', 'tunnel_roi', 'min_roi_cells', 'road_percentile',
             'luma_margin', 'chroma_margin', 'paint_gap', 'paint_percentile', 'min_paint_cells', 'min_paint_margin',
             'tunnel_min_luma', 'min_decode_agreement', 'param_timeout_s')
ROI_KEYS = ('x_min_m', 'x_max_m', 'half_width_m')
PASS_KEYS = ('min_road_coverage', 'max_false_paint_ratio', 'tunnel_dark_check')
UNSEEN, ROAD, PAINT, OTHER = 0, 1, 2, 3          # LocalGrid kinds (V4)
PHASES = ('road', 'tunnel', 'apply', 'revert')
LABEL = {'road': 'Open road', 'tunnel': 'Tunnel'}


class ConfigError(ValueError):
    pass


# --------------------------------------------------------------------------- pure helpers
def classify(luma: np.ndarray, chroma: np.ndarray, th: Dict[str, float]) -> np.ndarray:
    """The V4 rule, identical to road_mask.RoadMask.process (a test checks they agree)."""
    kind = np.full(np.shape(luma), OTHER, np.uint8)
    road = (luma < float(th['road_max_luma'])) & (chroma < float(th['road_max_chroma']))
    kind[road] = ROAD
    kind[~road & (luma > float(th['paint_min_luma']))] = PAINT
    return kind


def luma_chroma(bgr: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    f = bgr.astype(np.int16)
    return f.sum(axis=-1) / 3.0, (f.max(axis=-1) - f.min(axis=-1)).astype(np.float64)


def stitched_to_cells(img: np.ndarray, rows: int, cols: int) -> np.ndarray:
    """road_perception 'stitched' debug image -> (rows, cols, 3) BGR per grid cell.

    The image is road_mask.bev_view(bgr, scale): both axes flipped, then each cell
    blown up to scale x scale pixels (nearest). The centre pixel of each block is taken."""
    h, w = img.shape[:2]
    if h % rows or w % cols or h // rows != w // cols or h < rows:
        raise ValueError(f'stitched image {w}x{h} does not fit a {rows}x{cols} grid')
    s = h // rows
    c = s // 2
    cells = img[c::s, c::s][:rows, :cols]
    return cells[::-1, ::-1]


def roi_mask(grid: Dict, roi: Dict) -> np.ndarray:
    """Cells whose centre lies in the box x_min..x_max, |y| <= half_width (base_link)."""
    rows, cols, res = int(grid['rows']), int(grid['cols']), float(grid['res'])
    xs = float(grid['x0']) + (np.arange(rows) + 0.5) * res
    ys = float(grid['y0']) + (np.arange(cols) + 0.5) * res
    rx = (xs >= float(roi['x_min_m'])) & (xs <= float(roi['x_max_m']))
    cy = np.abs(ys) <= float(roi['half_width_m'])
    return rx[:, None] & cy[None, :]


def roi_box(grid: Dict, roi: Dict) -> Optional[List[int]]:
    m = roi_mask(grid, roi)
    r, c = np.where(m)
    if not len(r):
        return None
    return [int(r.min()), int(r.max()), int(c.min()), int(c.max())]


def pct(a: np.ndarray, q: float) -> Optional[float]:
    return float(np.percentile(a, q)) if len(a) else None


def ratios(kind: np.ndarray) -> Dict[str, Any]:
    n = int(kind.size)
    if not n:
        return {'cells': 0, 'road': None, 'paint': None}
    return {'cells': n, 'road': float((kind == ROAD).mean()), 'paint': float((kind == PAINT).mean())}


class Sample:
    """Colours collected for one phase (open road / tunnel)."""

    def __init__(self, phase: str, roi: Dict, before: Dict[str, float]):
        self.phase, self.roi, self.before = phase, roi, dict(before)
        self.frames = 0
        self.roi_l: List[np.ndarray] = []
        self.roi_c: List[np.ndarray] = []
        self.roi_live: List[np.ndarray] = []      # kinds road_perception itself gave the box cells
        self.out_l: List[np.ndarray] = []         # seen cells outside the box (tape candidates)
        self.agree = [0, 0]                       # decoded-colour kinds == grid kinds, seen cells

    def add(self, grid: Dict, cells_bgr: np.ndarray) -> None:
        kind = np.asarray(grid['kind'])
        seen = kind != UNSEEN
        box = roi_mask(grid, self.roi)
        luma, chroma = luma_chroma(cells_bgr)
        mine = classify(luma, chroma, self.before)
        self.agree[0] += int((mine[seen] == kind[seen]).sum())
        self.agree[1] += int(seen.sum())
        inb = box & seen
        self.roi_l.append(luma[inb])
        self.roi_c.append(chroma[inb])
        self.roi_live.append(kind[inb])
        self.out_l.append(luma[seen & ~box])
        self.frames += 1

    # aggregated arrays
    def cat(self, name: str) -> np.ndarray:
        parts = getattr(self, name)
        return np.concatenate(parts) if parts else np.zeros(0)

    def cells_per_frame(self) -> float:
        return float(np.mean([len(x) for x in self.roi_l])) if self.roi_l else 0.0

    def agreement(self) -> Optional[float]:
        return self.agree[0] / self.agree[1] if self.agree[1] else None


def road_stats(s: Sample, proc: Dict) -> Dict[str, Any]:
    q = float(proc['road_percentile'])
    luma, chroma = s.cat('roi_l'), s.cat('roi_c')
    hi = pct(luma, q)
    out = s.cat('out_l')
    tape = out[out > hi + float(proc['paint_gap'])] if hi is not None else np.zeros(0)
    return {'road_luma_hi': hi, 'road_chroma_hi': pct(chroma, q),
            'road_luma_mean': float(luma.mean()) if len(luma) else None,
            'tape_cells': int(len(tape)) // max(1, s.frames),
            'tape_luma_lo': pct(tape, float(proc['paint_percentile'])) if len(tape) else None}


def propose(samples: Dict[str, Sample], current: Dict[str, float], proc: Dict) -> Dict[str, Any]:
    """Proposed thresholds from every sample taken so far (see module doc)."""
    stats = {k: road_stats(s, proc) for k, s in samples.items()}
    luma = np.concatenate([s.cat('roi_l') for s in samples.values()])
    chroma = np.concatenate([s.cat('roi_c') for s in samples.values()])
    q = float(proc['road_percentile'])
    road_hi, chroma_hi = pct(luma, q), pct(chroma, q)
    notes = []
    if road_hi is None:
        return {'values': None, 'stats': stats, 'notes': ['no road cells sampled']}
    rml = int(round(min(255.0, road_hi + float(proc['luma_margin']))))
    rmc = int(round(min(255.0, chroma_hi + float(proc['chroma_margin']))))
    floor = road_hi + float(proc['min_paint_margin'])
    tape_lo = [st['tape_luma_lo'] for st in stats.values()
               if st['tape_luma_lo'] is not None and st['tape_cells'] >= int(proc['min_paint_cells'])]
    if tape_lo:
        lo = min(tape_lo)
        mid = (road_hi + lo) / 2.0
        pml = max(mid, floor)
        if lo <= floor:
            notes.append(f'the darkest tape (luma {lo:.0f}) is close to the brightest road (luma {road_hi:.0f}): '
                         'some tape will read as road')
    else:
        pml = max(float(current.get('paint_min_luma', floor)), floor)
        notes.append('no lane tape seen: paint_min_luma kept at the current value (or raised above the road)')
    pml = int(round(min(254.0, pml)))
    return {'values': {'road_max_luma': rml, 'road_max_chroma': rmc, 'paint_min_luma': pml},
            'road_luma_hi': round(road_hi, 1), 'road_chroma_hi': round(chroma_hi, 1),
            'tape_luma_lo': round(min(tape_lo), 1) if tape_lo else None, 'stats': stats, 'notes': notes}


def evaluate(s: Sample, th: Dict[str, float]) -> Dict[str, Any]:
    kind = classify(s.cat('roi_l'), s.cat('roi_c'), th)
    return ratios(kind)


# --------------------------------------------------------------------------- the step
class VenueThresholdsStep(StepImpl):
    can_keep_previous = True

    def __init__(self, cfg: Dict, cameras: Dict):
        super().__init__(cfg)
        proc, pas = cfg.get('procedure') or {}, cfg.get('pass') or {}
        for where, d, keys in (('procedure', proc, PROC_KEYS), ('pass', pas, PASS_KEYS),
                               ('procedure.roi', proc.get('roi') or {}, ROI_KEYS),
                               ('procedure.tunnel_roi', proc.get('tunnel_roi') or {}, ROI_KEYS)):
            miss = [k for k in keys if k not in d]
            if miss:
                raise ConfigError(f'calibration_steps.yaml venue_thresholds.{where}: missing {", ".join(miss)}')
        self.proc, self.pas = proc, pas
        self.tunnel = bool(pas['tunnel_dark_check'])
        front = (cameras.get('roles') or {}).get('front')
        if not front or front not in cameras.get('sensors', {}):
            raise ConfigError('cameras.yaml roles.front is missing: step 9 samples the road ahead of the car')
        if not sensor_enabled(cameras, front):
            raise ConfigError(f'the front camera ({front}) is switched off in cameras.yaml: step 9 samples the '
                              'road ahead of the car with it')
        self.roles_on = [r for r, s in (cameras.get('roles') or {}).items()
                         if s in cameras['sensors'] and sensor_enabled(cameras, s)]
        self.param_timeout = float(proc['param_timeout_s'])
        self.samples: Dict[str, Sample] = {}
        self.current: Optional[Dict[str, int]] = None    # live values last read from road_perception
        self.original: Optional[Dict[str, int]] = None   # before this step changed anything
        self.applied: Optional[Dict[str, int]] = None    # set by us and read back
        self.proposal: Optional[Dict] = None
        self.run: Optional[Dict] = None
        self.last_error = ''

    # ------------------------------------------------------------------ helpers for the node
    def sampling(self) -> bool:
        return bool(self.run and self.run['phase'] in ('road', 'tunnel') and self.run['state'] == 'sampling')

    def phases(self) -> List[str]:
        return ['road', 'tunnel'] if self.tunnel else ['road']

    def _roi(self, phase: str) -> Dict:
        return self.proc['tunnel_roi' if phase == 'tunnel' else 'roi']

    def _next_phase(self) -> str:
        for p in self.phases():
            if p not in self.samples:
                return p
        return 'apply'

    # ------------------------------------------------------------------ StepImpl
    def start(self, now: float, inputs: Dict) -> Optional[str]:
        arg = str(inputs.get('argument', '') or '').strip().lower() or self._next_phase()
        if arg not in PHASES:
            return f'Unknown phase {arg!r}. This step knows: {", ".join(PHASES)}'
        if arg == 'tunnel' and not self.tunnel:
            return 'The tunnel check is switched off (calibration_steps.yaml venue_thresholds.pass.tunnel_dark_check)'
        link = inputs.get('road_params')
        if link is None:
            return 'No parameter link to road_perception (calibration_wizard setup problem)'
        if arg == 'apply':
            vals = (self.proposal or {}).get('values')
            if not vals:
                return 'Nothing to apply yet: sample the open road first'
        if arg == 'revert' and not self.original:
            return 'Nothing to put back: this step has not changed road_perception'
        self.run = {'phase': arg, 't0': now, 'state': 'reading', 'reply': None, 'sample': None, 'seq': None,
                    'link': link, 'error': '', 'deadline': now + self.param_timeout + 1.0}
        self.last_error = ''
        if arg in ('road', 'tunnel'):
            link.get(list(PARAM_NAMES), lambda v, r=self.run: self._on_get(r, v))
        else:
            vals = self.proposal['values'] if arg == 'apply' else self.original
            self.run['want'] = dict(vals)
            self.run['state'] = 'setting'
            link.set({f'classify.{k}': int(v) for k, v in vals.items()}, lambda ok, r=self.run: self._on_set(r, ok))
        return None

    # link callbacks (called from the node executor, or synchronously by fakes)
    def _on_get(self, r: Dict, vals: Optional[Dict]) -> None:
        if r is not self.run:
            return
        if not vals or any(vals.get(n) is None for n in PARAM_NAMES):
            r['error'] = ('could not read road_perception classify.* parameters: is road_perception running? '
                          '(it is started by calibrate.launch.py; check the System health tab)')
            r['state'] = 'failed'
            return
        cur = {p: vals[f'classify.{p}'] for p in PARAMS}
        self.current = cur
        if r['state'] == 'reading':
            if self.original is None:
                self.original = dict(cur)
            r['sample'] = Sample(r['phase'], self._roi(r['phase']), cur)
            r['state'] = 'sampling'
            r['t_sample'] = None
        elif r['state'] == 'confirming':
            r['state'] = 'done'

    def _on_set(self, r: Dict, ok: bool) -> None:
        if r is not self.run:
            return
        if not ok:
            r['error'] = 'road_perception refused or did not answer the parameter change'
            r['state'] = 'failed'
            return
        r['state'] = 'confirming'
        r['link'].get(list(PARAM_NAMES), lambda v, rr=r: self._on_get(rr, v))

    def cancel(self) -> None:
        self.run = None

    def progress(self, now: float) -> Dict:
        r = self.run
        if r is None:
            return {}
        s = r.get('sample')
        need = float(self.proc['sample_s'])
        t_s = r.get('t_sample')
        waited = now - t_s if t_s is not None else 0.0
        frac = 0.0
        if s is not None:
            frac = min(1.0, waited / need, s.frames / max(1, int(self.proc['min_frames'])))
        return {'phase': r['phase'], 'state': r['state'], 'elapsed_s': round(now - r['t0'], 1),
                'remaining_s': round(max(0.0, need - waited), 1), 'fraction': round(frac, 2),
                'samples': s.frames if s else 0}

    def tick(self, now: float, inputs: Dict) -> Optional[Dict]:
        r = self.run
        if r is None:
            return None
        if r['state'] == 'failed':
            return self._end(r, r['error'])
        if r['phase'] in ('apply', 'revert'):
            if r['state'] == 'done':
                return self._end_set(r)
            if now > r['deadline']:
                return self._end(r, 'road_perception did not answer the parameter change in time')
            return None
        if r['state'] == 'reading':
            if now > r['deadline']:
                return self._end(r, 'road_perception did not answer: is it running? (System health tab)')
            return None
        # sampling
        s: Sample = r['sample']
        if r.get('t_sample') is None:
            r['t_sample'] = now
        get = inputs.get('road_pair')
        try:
            got = get() if callable(get) else None
            if got is not None and got[0] != r['seq']:
                r['seq'], grid, cells = got
                s.add(grid, cells)
        except (ValueError, KeyError) as e:
            return self._end(r, f'cannot read road_perception output: {e}')
        waited = now - r['t_sample']
        if waited >= float(self.proc['sample_s']) and s.frames >= int(self.proc['min_frames']):
            return self._end_sample(r)
        if waited >= float(self.proc['frame_timeout_s']):
            return self._end(r, f'only {s.frames} stitched frames from road_perception in {waited:.0f} s '
                                f'(need {self.proc["min_frames"]}). Check road_perception is running with '
                                'debug.enabled: true and the front camera is live (step 1).')
        return None

    def _end(self, r: Dict, error: str) -> Dict:
        self.run = None
        self.last_error = f'{r["phase"]}: {error}' if error else ''
        return self._result()

    def _end_set(self, r: Dict) -> Dict:
        self.run = None
        cur = self.current or {}
        want = r['want']
        if any(int(cur.get(k, -1)) != int(v) for k, v in want.items()):
            self.last_error = f'{r["phase"]}: road_perception reports {cur}, expected {want}'
            return self._result()
        self.applied = dict(want) if r['phase'] == 'apply' else None
        return self._result()

    def _end_sample(self, r: Dict) -> Dict:
        s: Sample = r['sample']
        self.run = None
        self.samples[s.phase] = s
        self.proposal = propose(self.samples, self.current or {}, self.proc)
        if self.applied and self.proposal.get('values') != self.applied:
            self.applied = None                  # the proposal moved: apply again
        return self._result()

    # ------------------------------------------------------------------ result
    def _sample_checks(self, phase: str, vals: Optional[Dict]) -> List[Dict]:
        lab = LABEL[phase]
        s = self.samples.get(phase)
        pas, proc = self.pas, self.proc
        if s is None:
            return [{'label': f'{lab} sample', 'measured': 'not run', 'limit': 'sampled', 'passed': False,
                     'why': 'not sampled yet', 'fix': f'Place the car as described and press "Sample {lab.lower()}".'}]
        out = []
        cells = s.cells_per_frame()
        agree = s.agreement()
        ok_cells = cells >= int(proc['min_roi_cells'])
        ok_agree = agree is not None and agree >= float(proc['min_decode_agreement'])
        out.append({'label': f'{lab} sample', 'measured': f'{s.frames} frames · {cells:.0f} box cells/frame',
                    'limit': f'≥ {proc["min_roi_cells"]} cells', 'passed': ok_cells,
                    'why': '' if ok_cells else f'only {cells:.0f} cells of the sample box are seen by the cameras',
                    'fix': '' if ok_cells else 'Check steps 3-4 (camera calibration) and that the front camera looks at '
                                               'the floor ahead (box on the map must be inside the camera view).'})
        out.append({'label': f'{lab}: colour decode check', 'measured': f'{(agree or 0) * 100:.1f} % agree',
                    'limit': f'≥ {float(proc["min_decode_agreement"]) * 100:.0f} %', 'passed': ok_agree,
                    'why': '' if ok_agree else ('the stitched debug colours do not reproduce road_perception\'s own '
                                                'cell kinds, so the proposal cannot be trusted'),
                    'fix': '' if ok_agree else 'Raise road_perception debug.jpeg_quality (Tuning tab) and sample again.'})
        if vals:
            ev = evaluate(s, vals)
            road, paint = ev['road'] or 0.0, ev['paint'] or 0.0
            ok_r = road >= float(pas['min_road_coverage'])
            ok_p = paint <= float(pas['max_false_paint_ratio'])
            out.append({'label': f'{lab}: road coverage', 'measured': f'{road * 100:.1f} %',
                        'limit': f'≥ {float(pas["min_road_coverage"]) * 100:.0f} %', 'passed': ok_r,
                        'why': '' if ok_r else 'much of the bare road in the box is not classified as road',
                        'fix': '' if ok_r else ('Make sure the box holds only bare road (no tape, no car parts), '
                                                'avoid strong reflections, then sample again.')})
            out.append({'label': f'{lab}: false paint', 'measured': f'{paint * 100:.1f} %',
                        'limit': f'≤ {float(pas["max_false_paint_ratio"]) * 100:.0f} %', 'passed': ok_p,
                        'why': '' if ok_p else 'bare road in the box reads as lane tape (glare / bright spots)',
                        'fix': '' if ok_p else ('Move lamps or shade the reflection, or check no tape lies in the box, '
                                                'then sample again.')})
        if phase == 'road':
            st = road_stats(s, proc)
            ok_t = st['tape_cells'] >= int(proc['min_paint_cells'])
            out.append({'label': 'Lane tape seen beside the box', 'measured': f'{st["tape_cells"]} cells/frame',
                        'limit': f'≥ {proc["min_paint_cells"]}', 'passed': ok_t,
                        'why': '' if ok_t else 'no bright lane tape in view: paint_min_luma cannot be measured',
                        'fix': '' if ok_t else 'Put the car in the middle of a lane so both lane lines are in the front '
                                               'camera view, then sample again.'})
        if phase == 'tunnel':
            mean = float(s.cat('roi_l').mean()) if s.frames and len(s.cat('roi_l')) else 0.0
            ok_d = mean >= float(proc['tunnel_min_luma'])
            out.append({'label': 'Tunnel: camera sees the floor', 'measured': f'mean luma {mean:.0f}',
                        'limit': f'≥ {proc["tunnel_min_luma"]}', 'passed': ok_d,
                        'why': '' if ok_d else 'the tunnel picture is (nearly) black: road and tape cannot be told apart',
                        'fix': '' if ok_d else 'Check the camera exposure (auto exposure on) and that the box is inside '
                                               'the tunnel, not the roof shadow edge only.'})
        return out

    def _result(self) -> Dict:
        vals = (self.proposal or {}).get('values')
        checks = []
        for ph in self.phases():
            checks += self._sample_checks(ph, vals)
        ok_apply = bool(vals) and self.applied == vals
        checks.append({'label': 'Applied live on road_perception',
                       'measured': 'yes' if ok_apply else ('not yet' if vals else 'nothing to apply'),
                       'limit': 'applied', 'passed': ok_apply,
                       'why': '' if ok_apply else 'the proposed values are not running on road_perception yet',
                       'fix': '' if ok_apply else 'Press "Apply live", check the Perception tab, then Save.'})
        if self.last_error:
            checks.insert(0, {'label': 'Last action', 'measured': 'failed', 'limit': 'ok', 'passed': False,
                              'why': self.last_error, 'fix': 'Fix the cause and press it again.'})
        passed = all(c['passed'] for c in checks)
        todo = [LABEL[p].lower() for p in self.phases() if p not in self.samples]
        if passed:
            summary = 'thresholds measured and applied: ' + ', '.join(f'{k} {v}' for k, v in vals.items())
        elif self.last_error:
            summary = self.last_error
        elif todo:
            summary = 'still to sample: ' + ', '.join(todo)
        elif all(c['passed'] for c in checks[:-1]):
            summary = 'proposal ready: press Apply live'
        else:
            summary = 'limits not met: ' + ', '.join(c['label'] for c in checks if not c['passed'])
        res = {'passed': passed, 'summary': summary, 'checks': checks,
               'before': dict(self.original or {}), 'proposal': dict(vals or {}),
               'notes': list((self.proposal or {}).get('notes') or []),
               'samples': {k: self._sample_row(s, vals) for k, s in self.samples.items()},
               'bpu_detector': 'not changed (detector confidences need labelled venue footage: Tuning tab)'}
        if vals:
            res['params_overlay'] = {NODE: {f'classify.{k}': int(v) for k, v in vals.items()}}
        return res

    def _sample_row(self, s: Sample, vals: Optional[Dict]) -> Dict:
        st = road_stats(s, self.proc)
        live = ratios(np.concatenate(s.roi_live) if s.roi_live else np.zeros(0, np.uint8))
        after = evaluate(s, vals) if vals else {'road': None, 'paint': None}
        r = lambda v, n=1: None if v is None else round(float(v), n)  # noqa: E731
        return {'phase': s.phase, 'label': LABEL[s.phase], 'frames': s.frames, 'box_cells': r(s.cells_per_frame(), 0),
                'thresholds_before': dict(s.before),
                'before': {'road': r(live['road'], 3), 'paint': r(live['paint'], 3)},
                'after': {'road': r(after['road'], 3), 'paint': r(after['paint'], 3)},
                'decode_agreement': r(s.agreement(), 3), 'road_luma_mean': r(st['road_luma_mean']),
                'road_luma_hi': r(st['road_luma_hi']), 'road_chroma_hi': r(st['road_chroma_hi']),
                'tape_cells': st['tape_cells'], 'tape_luma_lo': r(st['tape_luma_lo'])}

    # ------------------------------------------------------------------ live view
    def live(self, inputs: Dict) -> Dict:
        g = inputs.get('road_grid')
        grid = g() if callable(g) else g
        vals = (self.proposal or {}).get('values')
        now_m, gmap = {}, None
        if grid is not None:
            kind = np.asarray(grid['kind'])
            for ph in self.phases():
                box = roi_mask(grid, self._roi(ph))
                sel = kind[box & (kind != UNSEEN)]
                m = ratios(sel)
                now_m[ph] = {'cells': m['cells'], 'road': None if m['road'] is None else round(m['road'], 3),
                             'paint': None if m['paint'] is None else round(m['paint'], 3)}
            gmap = self._map(grid, kind)
        r = self.run
        return {'current': self.current, 'original': self.original, 'proposal': vals, 'applied': self.applied,
                'applied_ok': bool(vals) and self.applied == vals,
                'notes': list((self.proposal or {}).get('notes') or []),
                'now': now_m, 'grid_age_s': None if grid is None else grid.get('age_s'),
                'samples': {k: self._sample_row(s, vals) for k, s in self.samples.items()},
                'phases': self.phases(), 'next': self._next_phase(), 'tunnel_check': self.tunnel,
                'limits': {k: self.pas[k] for k in PASS_KEYS},
                'rois': {p: self._roi(p) for p in self.phases()},
                'run': None if r is None else {'phase': r['phase'], 'state': r['state'],
                                               'frames': r['sample'].frames if r.get('sample') else 0},
                'error': self.last_error, 'cameras': self.roles_on, 'map': gmap,
                'bpu_note': 'bpu_detector.thresholds are detector confidences, not colours: this step leaves them '
                            'as they are (tune them on the Detections / Tuning tab with the real light and gate).'}

    def _map(self, grid: Dict, kind: np.ndarray) -> Optional[Dict]:
        """Seen part of the grid (plus the sample boxes) as one digit per cell, for the page canvas."""
        seen = kind != UNSEEN
        boxes = {p: roi_box(grid, self._roi(p)) for p in self.phases()}
        rr, cc = np.where(seen)
        rows = list(rr) + [b[i] for b in boxes.values() if b for i in (0, 1)]
        cols = list(cc) + [b[i] for b in boxes.values() if b for i in (2, 3)]
        if not rows:
            return None
        r0, r1, c0, c1 = int(min(rows)), int(max(rows)), int(min(cols)), int(max(cols))
        sub = kind[r0:r1 + 1, c0:c1 + 1]
        return {'r0': r0, 'c0': c0, 'rows': int(sub.shape[0]), 'cols': int(sub.shape[1]),
                'kinds': ''.join(str(int(v)) for v in sub.ravel()),
                'boxes': {p: None if b is None else [b[0] - r0, b[1] - r0, b[2] - c0, b[3] - c0]
                          for p, b in boxes.items()}}

    # ------------------------------------------------------------------ save / keep
    def save_data(self, session: str, result: Dict) -> List[str]:
        vals = (result.get('params_overlay') or {}).get(NODE)
        if not vals:
            raise StepRefused('No proposed thresholds in this result: sample and apply first.')
        return [ct.merge_overlay(session, NODE, dict(vals))]

    def keep_data(self, src_session: str, session: str) -> List[str]:
        vals = {n: ct.overlay_value(src_session, NODE, n) for n in PARAM_NAMES}
        miss = [n for n, v in vals.items() if v is None]
        if miss:
            raise StepRefused(f'{os.path.basename(src_session)} has no {NODE} {", ".join(miss)} in its '
                              'params_overlay.yaml: this step must be measured now.')
        return [ct.merge_overlay(session, NODE, vals)]
