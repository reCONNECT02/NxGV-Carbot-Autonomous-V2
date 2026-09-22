"""Phase 7: GUI core logic (no ROS)."""
import base64
import math

from carbot_gui.gui_core import (EventLog, LazyGroups, cov_ellipse, decimate, encode_grid, leg_progress,
                                 running_now, to_base, venue_to_track)


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_lazy_groups_subscribe_only_while_polled():
    c = Clock()
    lz = LazyGroups(5.0, clock=c)
    assert lz.diff(['core', 'map']) == (set(), set())
    lz.touch('map')
    assert lz.diff(['core', 'map']) == ({'map'}, set())
    c.t += 4.0
    assert lz.diff(['core', 'map']) == (set(), set())
    c.t += 2.0
    assert lz.diff(['core', 'map']) == (set(), {'map'})
    assert LazyGroups(5.0, enabled=False, clock=c).wanted(['a', 'b']) == {'a', 'b'}


def test_running_now_driving_chain_and_zone():
    owner = {'winner': 'ROAD', 'reason': 'ok'}
    mission = {'mode': 'ROAD', 'speed_zone': 'bump', 'zone_max': 0.35}
    lines = running_now('race', owner, None, mission, [], False, {'leg': 1, 'legs': 3})
    assert lines[0]['label'] == 'Driving'
    assert [i['id'] for i in lines[0]['items']] == ['08', '09-13', '15']
    assert 'leg 2 of 3' in lines[0]['items'][0]['text']
    assert lines[1]['label'] == 'Limiting' and '0.35' in lines[1]['items'][0]['text']


def test_running_now_lists_every_blocker():
    owner = {'winner': 'SAFETY_STOP', 'reason': 'front_clearance: obstacle'}
    safety = {'veto_check': 'front_clearance', 'veto_reason': 'obstacle', 'value': 0.12, 'limit': 0.15}
    mission = {'mode': 'HOLD', 'hold_reason': 'TRAFFIC HOLD'}
    nodes = [{'node': 'road_perception', 'block': '03', 'level': 2, 'detail': 'no camera'},
             {'node': 'local_pose', 'block': '05', 'level': 0, 'stale': True}]
    lines = running_now('race', owner, safety, mission, nodes, False)
    stop = lines[0]
    assert stop['label'] == 'Stopped by'
    assert [i['id'] for i in stop['items']] == ['14', '08']
    assert stop['items'][1]['tab'] == 'det'
    faults = lines[1]
    assert faults['label'] == 'Faults' and len(faults['items']) == 2


def test_running_now_manual_and_silent_owner():
    assert running_now('race', {'winner': 'ROAD'}, None, None, [], True)[0]['label'] == 'Manual'
    lines = running_now('race', None, None, None, [], False)
    assert lines[0]['label'] == 'Stopped by' and lines[0]['items'][0]['tab'] == 'health'


def test_leg_progress():
    route = [(i * 0.01, 0.0) for i in range(301)]   # 3 m straight
    info = {'legs': [{'id': 'leg1'}, {'id': 'leg2'}],
            'pieces': [{'index': 0, 'leg': 0, 'kind': 'road', 'start': 0, 'end': 100, 'length_m': 1.0},
                       {'index': 1, 'leg': 1, 'kind': 'road', 'start': 101, 'end': 200, 'length_m': 1.0},
                       {'index': 2, 'leg': 1, 'kind': 'manoeuvre', 'start': 201, 'end': 300, 'length_m': 1.0}]}
    p = leg_progress(info, route, (1.5, 0.02), 1)          # leg 2 spans 1.01..3.0 m
    assert p['leg'] == 1 and p['legs'] == 2 and p['pieces'] == 2
    assert p['piece'] == 1 and p['piece_no'] == 1
    assert abs(p['pct'] - 24.6) < 1.0
    assert p['lengths'] == [1.0, 2.0]
    assert leg_progress(info, route, (2.9, 0.0), 1)['piece_kind'] == 'manoeuvre'
    assert leg_progress(None, route, (0, 0), 0) is None


def test_encode_grid_bits_and_stride():
    kind = [1, 2, 3, 0]
    g = encode_grid(2, 2, kind, grown=[1, 0, 0, 0], age=[0.0, 1.5, 3.0, -1.0], age_max_s=3.0)
    b = base64.b64decode(g['b64'])
    assert b[0] & 3 == 1 and b[0] & 4 and b[0] >> 4 == 0
    assert b[1] & 3 == 2 and b[1] >> 4 == 7
    assert b[2] >> 4 == 15 and b[3] == 0
    g2 = encode_grid(4, 4, [1] * 16, stride=2)
    assert (g2['rows'], g2['cols']) == (2, 2)


def test_frames_and_ellipse():
    x, y = to_base(1.0, 1.0, (1.0, 0.0, math.pi / 2))
    assert abs(x - 1.0) < 1e-9 and abs(y) < 1e-9
    vx, vy = venue_to_track(3.0, 2.0, {'x_m': 1.0, 'y_m': 2.0, 'yaw_deg': 90.0})
    assert abs(vx) < 1e-9 and abs(vy + 2.0) < 1e-9
    a, b, ang = cov_ellipse(4e-4, 0.0, 1e-4, k=1.0)
    assert abs(a - 0.02) < 1e-9 and abs(b - 0.01) < 1e-9 and abs(ang) < 1e-9
    assert decimate(list(range(10)), 4) == [0, 3, 6, 9]


def test_event_log_changes_only():
    c = Clock()
    ev = EventLog(3, clock=c)
    ev.change('mode', 'ROAD', 'mission', 'info', '08', 'ROAD')
    ev.change('mode', 'ROAD', 'mission', 'info', '08', 'ROAD')
    ev.change('mode', 'HOLD', 'mission', 'warn', '08', 'HOLD')
    assert [e['text'] for e in ev.since(0)] == ['ROAD', 'HOLD']
    for i in range(5):
        ev.add('x', 'info', '', str(i))
    assert len(ev.since(0)) == 3 and ev.since(ev.seq - 1)[0]['text'] == '4'
