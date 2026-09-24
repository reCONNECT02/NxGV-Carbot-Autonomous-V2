"""Step 11 manual mode: the owner fits the map by hand and types x, y, yaw (no ROS)."""
import json

import pytest

from test_step_map_uwb_alignment import Rig, session_uwb


def test_manual_alignment_passes_and_save_writes_track_to_venue(tmp_path):
    rig = Rig(tmp_path)
    r = rig.do('RUN', json.dumps({'mode': 'manual', 'x_m': 0.05, 'y_m': -0.02, 'yaw_deg': 1.5}))
    assert r['ok'], r
    done = rig.advance(1.0)
    assert done.status == 'PASS', done.result
    assert done.result['mode'] == 'manual' and 'owner-entered' in done.result['summary']
    assert rig.do('SAVE')['ok']
    assert session_uwb(rig.wiz)['track_to_venue'] == {'x_m': 0.05, 'y_m': -0.02, 'yaw_deg': 1.5, 'aligned': True}


@pytest.mark.parametrize('arg', [{'mode': 'manual'}, {'mode': 'manual', 'x_m': 'a', 'y_m': 0, 'yaw_deg': 0},
                                 {'mode': 'manual', 'x_m': 500, 'y_m': 0, 'yaw_deg': 0},
                                 {'mode': 'manual', 'x_m': 0, 'y_m': 0, 'yaw_deg': 720},
                                 {'mode': 'manual', 'x_m': float('nan'), 'y_m': 0, 'yaw_deg': 0}])
def test_manual_alignment_rejects_bad_numbers(tmp_path, arg):
    rig = Rig(tmp_path)
    assert not rig.do('RUN', json.dumps(arg))['ok']


def test_manual_alignment_needs_step_10(tmp_path):
    rig = Rig(tmp_path, step10=False)
    r = rig.do('RUN', json.dumps({'mode': 'manual', 'x_m': 0, 'y_m': 0, 'yaw_deg': 0}))
    assert not r['ok'] and 'step 10' in r['message'].lower()
