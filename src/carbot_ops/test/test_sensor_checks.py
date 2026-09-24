"""Calibration step 1 checks on the repo's real YAML (no ROS)."""
import pytest

from carbot_ops import sensor_checks as sc
from helpers import CAMERAS, OLD_SESSION_CAMERAS, STEP1, UWB, good


def rows(snap, **pass_over):
    checks = sc.build_checks(CAMERAS, UWB, dict(STEP1['pass'], **pass_over))
    return {r['key']: r for r in sc.evaluate(checks, snap, STEP1['pass']['max_age_s'])}


def test_all_green():
    r = rows(good())
    assert len(r) == 9                                   # 1 camera + lidar/odom/imu/uwb tag + anchors/battery/processes/network
    assert all(x['state'] == 'ok' for x in r.values()), [x for x in r.values() if x['state'] != 'ok']


def test_camera_row_is_the_front_astra_only():
    r = rows(good())
    assert r['cam_astra']['label'] == 'Astra Pro (front)'
    assert [k for k in r if k.startswith('cam_')] == ['cam_astra']
    assert r['cam_astra']['topic'] == '/camera/color/image_raw'


def test_old_session_side_cameras_get_no_row():
    checks = sc.build_checks(OLD_SESSION_CAMERAS, UWB, STEP1['pass'])
    keys = [c.key for c in checks]
    assert 'cam_ov5647' not in keys and 'cam_imx219' not in keys and 'cam_astra' in keys


def test_camera_never_published_gives_usb_hint():
    s = good()
    s['topics']['/camera/color/image_raw'] = {'hz': 0.0, 'age': -1.0, 'latency': -1}
    x = rows(s)['cam_astra']
    assert x['state'] == 'bad' and x['measured'] == 'never'
    assert 'Restart camera drivers' in x['fix'] and 'USB' in x['fix']
    assert 'mipi' not in x['fix'].lower() and 'MIPI' not in x['fix']


def test_stale_and_slow():
    s = good()
    s['topics']['/scan'] = {'hz': 10.0, 'age': 4.0, 'latency': 5}
    s['topics']['/camera/color/image_raw'] = {'hz': 9.0, 'age': 0.05, 'latency': 5}
    r = rows(s, enforce_min_rates=True)
    assert r['lidar']['state'] == 'bad' and 'stopped' in r['lidar']['why']
    assert r['cam_astra']['state'] == 'bad' and 'below 12.0' in r['cam_astra']['why']


def test_rates_not_enforced_still_fails_stale_and_never():
    s = good()
    s['topics']['/scan'] = {'hz': 10.0, 'age': 4.0, 'latency': 5}
    s['topics']['/camera/color/image_raw'] = {'hz': 9.0, 'age': 0.05, 'latency': 5}
    s['topics']['/odom'] = {'hz': 0.0, 'age': -1.0, 'latency': -1}
    r = rows(s, enforce_min_rates=False)
    assert r['cam_astra']['state'] == 'ok' and 'not enforced' in r['cam_astra']['detail']
    assert r['cam_astra']['limit'] == 'publishing'
    assert r['lidar']['state'] == 'bad' and r['odom']['state'] == 'bad'


def test_topic_not_watched_tells_where_to_add_it():
    s = good()
    del s['topics']['/odom']
    x = rows(s)['odom']
    assert x['state'] == 'bad' and 'watch_topics' in x['fix']


def test_no_health_is_wait_not_crash():
    s = good()
    s['health_age_s'] = None
    r = rows(s)
    assert r['cam_astra']['state'] == 'wait' and r['processes']['state'] == 'wait'


def test_anchors_missing_and_unknown_id():
    s = good()
    s['uwb']['anchors']['1783']['seen'] = False
    s['uwb']['unknown'] = '1790'
    x = rows(s)['uwb_anchors']
    assert x['state'] == 'bad' and x['measured'] == '2 of 3' and '1783' in x['why'] and '1790' in x['fix']
    s['uwb'] = None
    assert rows(s)['uwb_anchors']['state'] == 'bad'


def test_uwb_tag_hint_depends_on_cause():
    s = good()
    s['topics']['/uwb3/input_json'] = {'hz': 0.0, 'age': -1.0, 'latency': -1}
    s['agent'] = False
    assert 'agent is not running' in rows(s)['uwb_tag']['fix']
    s['env'] = {'domain_id': '0', 'localhost_only': '1', 'ok': False, 'problems': ['x']}
    r = rows(s)
    assert 'ROS network' in r['uwb_tag']['fix'] and r['network']['state'] == 'bad'


def test_battery():
    s = good()
    s['battery_v'] = 10.2
    assert rows(s)['battery']['state'] == 'bad'
    s['battery_v'] = None
    assert 'carbot extension' in rows(s)['battery']['why']


def test_duplicates_and_old_viewer():
    s = good()
    s['procs'] += [('astra_camera /', 999), ('websocket', 555)]
    x = rows(s)['processes']
    assert x['state'] == 'bad' and x['measured'] == '2'
    assert '999' in x['why'] and 'websocket' in x['why']


def test_aggregate_tolerates_one_dropout():
    checks = sc.build_checks(CAMERAS, UWB, STEP1['pass'])
    ok = sc.evaluate(checks, good(), 1.0)
    s = good()
    s['topics']['/scan']['age'] = 3.0
    bad = sc.evaluate(checks, s, 1.0)
    res = sc.aggregate([ok, ok, ok, ok, bad], 0.8)
    assert res['passed'] and res['n_ok'] == res['n']
    res = sc.aggregate([ok, bad, bad, ok, ok], 0.8)
    assert not res['passed']
    lid = next(r for r in res['checks'] if r['key'] == 'lidar')
    assert not lid['passed'] and lid['state'] == 'bad' and lid['ok_fraction'] == 0.6
    assert not sc.aggregate([], 0.8)['passed']


def test_missing_yaml_key_is_a_clear_error():
    bad = dict(STEP1['pass'])
    del bad['processes']
    with pytest.raises(sc.ConfigError, match='processes'):
        sc.build_checks(CAMERAS, UWB, bad)
    cams = dict(CAMERAS, roles={'front': 'nope'})
    with pytest.raises(sc.ConfigError, match='front'):
        sc.build_checks(cams, UWB, STEP1['pass'])


def test_disabled_front_camera_has_no_row():
    cams = dict(CAMERAS, sensors={n: dict(s, enabled=False) for n, s in CAMERAS['sensors'].items()})
    s = good()
    del s['topics']['/camera/color/image_raw']
    s['procs'] = []
    checks = sc.build_checks(cams, UWB, STEP1['pass'])
    r = {x['key']: x for x in sc.evaluate(checks, s, STEP1['pass']['max_age_s'])}
    assert 'cam_astra' not in r and len(r) == 8
    assert all(x['state'] == 'ok' for x in r.values()), [x for x in r.values() if x['state'] != 'ok']


def test_missing_enabled_key_is_loud():
    cams = dict(CAMERAS, sensors={n: {k: v for k, v in s.items() if k != 'enabled'}
                                  for n, s in CAMERAS['sensors'].items()})
    with pytest.raises(KeyError):
        sc.build_checks(cams, UWB, STEP1['pass'])


def test_live_smoothing_ignores_one_late_report():
    from carbot_ops.step_sensor_health import SensorHealthStep
    step = SensorHealthStep(STEP1, CAMERAS, UWB)
    late = good()
    for t in late['topics'].values():
        t['age'] = 2.5                                   # one delayed system_monitor report
    states = []
    for seq, snap in enumerate([good(), good(), late, good(), late, late]):
        live = step.live({'snap': snap, 'health_seq': seq})
        states.append({r['key']: r['state'] for r in live['rows']}['lidar'])
    # one late report -> still ok; two late in the last three -> red
    assert states == ['ok', 'ok', 'ok', 'ok', 'bad', 'bad']


def test_live_smoothing_repeated_seq_does_not_count_twice():
    from carbot_ops.step_sensor_health import SensorHealthStep
    step = SensorHealthStep(STEP1, CAMERAS, UWB)
    bad = good()
    bad['topics']['/scan'] = {'hz': 0.0, 'age': -1.0, 'latency': -1}
    step.live({'snap': good(), 'health_seq': 1})
    for _ in range(5):                                   # wizard publishes live faster than health arrives
        live = step.live({'snap': bad, 'health_seq': 2})
    assert {r['key']: r['state'] for r in live['rows']}['lidar'] == 'ok'   # 1 of 2 reports bad
    live = step.live({'snap': bad, 'health_seq': 3})
    assert {r['key']: r['state'] for r in live['rows']}['lidar'] == 'bad'
