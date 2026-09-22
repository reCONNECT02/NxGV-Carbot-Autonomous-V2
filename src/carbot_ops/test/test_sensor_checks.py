"""Calibration step 1 checks on the repo's real YAML (no ROS)."""
import pytest

from carbot_ops import sensor_checks as sc
from helpers import CAMERAS, STEP1, UWB, good


def rows(snap):
    checks = sc.build_checks(CAMERAS, UWB, STEP1['pass'])
    return {r['key']: r for r in sc.evaluate(checks, snap, STEP1['pass']['max_age_s'])}


def test_all_green():
    r = rows(good())
    assert len(r) == 11
    assert all(x['state'] == 'ok' for x in r.values()), [x for x in r.values() if x['state'] != 'ok']


def test_camera_labels_follow_yaml_roles_and_show_unconfirmed():
    r = rows(good())
    assert r['cam_astra']['label'] == 'Astra Pro (front)'
    assert r['cam_ov5647']['label'] == 'OV5647 · MIPI ch 2 (right side?)'   # roles_confirmed false
    assert r['cam_imx219']['topic'] == '/cam_imx219/image_raw'


def test_mipi_never_published_gives_root_and_port_hints():
    s = good()
    s['topics']['/cam_ov5647/image_raw'] = {'hz': 0.0, 'age': -1.0, 'latency': -1}
    x = rows(s)['cam_ov5647']
    assert x['state'] == 'bad' and x['measured'] == 'never'
    assert 'Restart camera drivers' in x['fix'] and 'There are no available host' in x['fix']
    assert 'install_root_helpers' in x['fix']


def test_stale_and_slow():
    s = good()
    s['topics']['/scan'] = {'hz': 10.0, 'age': 4.0, 'latency': 5}
    s['topics']['/cam_imx219/image_raw'] = {'hz': 12.0, 'age': 0.05, 'latency': 5}
    r = rows(s)
    assert r['lidar']['state'] == 'bad' and 'stopped' in r['lidar']['why']
    assert r['cam_imx219']['state'] == 'bad' and 'below 24.0' in r['cam_imx219']['why']


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
    s['procs'] += [('mipi_cam /cam_ov5647', 999), ('websocket', 555)]
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
    cams = dict(CAMERAS, roles={'front': 'astra', 'left_rear': 'nope', 'right_rear': 'ov5647'})
    with pytest.raises(sc.ConfigError, match='left_rear'):
        sc.build_checks(cams, UWB, STEP1['pass'])
