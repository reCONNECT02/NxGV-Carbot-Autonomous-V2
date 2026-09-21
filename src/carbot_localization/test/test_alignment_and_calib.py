"""Step 11 solver and the step 10/11 CLIs end to end on synthetic captures."""
import json
import math

import pytest
import yaml

import synth_loc as S
from carbot_common.course import Course
from carbot_localization.alignment import Sample, fit_track_to_venue, start_pose_error
from carbot_localization.estimator_core import TrackToVenue

TRUE = TrackToVenue(0.30, -0.12, math.radians(-3.0))
LEVER = (0.10, 0.0)


def _lap(course, rng, offsets=S.OFFSETS):
    pts = S.outer_loop(course)
    t, pose, uwb, seq = 0.0, [], [], {}
    for i in range(1, len(pts)):
        a = math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0])
        t += 0.025 / 0.12
        x, y = pts[i]
        pose.append({'t': t, 'x': x + rng.gauss(0, .015), 'y': y + rng.gauss(0, .015), 'a': a})
        v = TRUE.to_venue(x + LEVER[0] * math.cos(a), y + LEVER[0] * math.sin(a))
        uwb.append({'t': t + 0.06 + rng.uniform(0, .07),
                    'json': S.tag_report(v, seq, int(t * 1000), rng, spike_p=0.08, offsets=offsets)})
    return uwb, pose


def test_fit_recovers_transform():
    course = Course(S.track_map())
    rng = S.rng(3)
    samples = []
    for q in S.outer_loop(course)[::3]:
        v = TRUE.to_venue(q[0], q[1])
        for k, (ax, ay, _) in S.ANCHORS.items():
            r = math.hypot(v[0] - ax, v[1] - ay) + rng.gauss(0, .05)
            if rng.random() < 0.08:
                r += 0.5
            samples.append(Sample(q[0], q[1], k, r))
    f = fit_track_to_venue(samples, {k: v[:2] for k, v in S.ANCHORS.items()})
    assert (f.x, f.y) == pytest.approx((0.30, -0.12), abs=0.04)
    assert math.degrees(f.yaw) == pytest.approx(-3.0, abs=0.5)
    assert f.rms_m < 0.08 and f.inlier_frac > 0.85


def test_fit_needs_data():
    with pytest.raises(ValueError):
        fit_track_to_venue([Sample(0, 0, '1782', 1.0)], {'1782': (0, 0)})


def test_start_pose_error():
    start = (7.18, 1.30, math.pi)
    tx, ty = start[0] - LEVER[0], start[1]
    assert start_pose_error(TRUE.to_venue(tx, ty), TRUE, start, LEVER) == pytest.approx(0.0, abs=1e-9)


def _write(path, rows):
    with open(path, 'w') as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')


def test_step10_then_step11_offline(tmp_path):
    from carbot_localization import calib_map_uwb
    from uwb_localization import calib_uwb
    rng = S.rng(2)
    cap = tmp_path / 'cap10'
    cap.mkdir()
    for name, (x, y, n) in {'link': (4, 2, 50), 'spot': (5.0, 1.5, 200), 'verify': (3.5, 2.5, 200)}.items():
        seq = {}
        _write(cap / f'uwb_step10_{name}.jsonl',
               [{'t': 1000 + i * 0.1, 'json': S.tag_report((x, y), seq, i * 100, rng, noise=0.03)}
                for i in range(n)])
    root = tmp_path / 'data'
    common = ['--data-root', str(root), '--config-dir', S.CONFIG, '--yes', '--session', 'S1']
    assert calib_uwb.main(common + ['--replay', str(cap), '--spot', '5.0', '1.5',
                                    '--verify', '3.5', '2.5']) == 0
    ses = root / 'calibration' / 'S1'
    uwb = yaml.safe_load((ses / 'data' / 'uwb.yaml').read_text())
    offs = {a['id']: a['range_offset_m'] for a in uwb['anchors']}
    assert offs == pytest.approx(S.OFFSETS, abs=0.02)
    assert uwb['offsets_calibrated'] and uwb['anchors_surveyed']

    cap11 = tmp_path / 'cap11'
    cap11.mkdir()
    uwb_rows, pose_rows = _lap(Course(S.track_map()), S.rng(4))
    _write(cap11 / 'step11_lap_uwb.jsonl', uwb_rows)
    _write(cap11 / 'step11_lap_pose.jsonl', pose_rows)
    assert calib_map_uwb.main(common + ['--replay', str(cap11)]) == 0
    t = yaml.safe_load((ses / 'data' / 'uwb.yaml').read_text())['track_to_venue']
    assert t['aligned'] and (t['x_m'], t['y_m']) == pytest.approx((0.30, -0.12), abs=0.04)
    assert t['yaw_deg'] == pytest.approx(-3.0, abs=0.5)
    summary = yaml.safe_load((ses / 'summary.yaml').read_text())
    assert summary['steps']['uwb_survey']['status'] == 'PASS'
    assert summary['steps']['map_uwb_alignment']['status'] == 'PASS'


def test_step11_fails_without_offsets(tmp_path):
    """Uncalibrated ranges (~1 m long) must not pass the alignment."""
    from carbot_localization import calib_map_uwb
    cap = tmp_path / 'cap'
    cap.mkdir()
    uwb_rows, pose_rows = _lap(Course(S.track_map()), S.rng(5))
    _write(cap / 'step11_lap_uwb.jsonl', uwb_rows)
    _write(cap / 'step11_lap_pose.jsonl', pose_rows)
    rc = calib_map_uwb.main(['--data-root', str(tmp_path / 'd'), '--config-dir', S.CONFIG, '--yes',
                             '--session', 'S', '--replay', str(cap)])
    assert rc == 1
