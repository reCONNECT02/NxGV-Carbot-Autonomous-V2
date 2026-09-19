import os

import yaml

from carbot_common import calibration_store as cs

STEPS = {'steps': [{'id': 'a', 'required': True}, {'id': 'b', 'required': True},
                   {'id': 'c', 'required': False}]}


def test_missing_and_active(tmp_path):
    root = str(tmp_path)
    assert cs.active_session(root) is None
    sess = os.path.join(cs.calibration_dir(root), '20260925_080000')
    os.makedirs(sess)
    with open(os.path.join(sess, 'summary.yaml'), 'w') as f:
        yaml.safe_dump({'steps': {'a': {'status': 'PASS'}, 'b': {'status': 'FAIL'}}}, f)
    with open(os.path.join(cs.calibration_dir(root), 'ACTIVE'), 'w') as f:
        f.write('20260925_080000\n')
    assert cs.active_session(root) == sess
    assert cs.missing_required(cs.load_summary(sess), STEPS) == ['b']
    assert cs.missing_required({}, STEPS) == ['a', 'b']
    assert cs.list_sessions(root) == ['20260925_080000']
