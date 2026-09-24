"""calib_tools.floor_board_dicts: step-4 boards in base_link with the rear-axle offset applied (pure, no ROS)."""
import copy
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from carbot_common.calib_tools import floor_board_dicts  # noqa: E402

TARGET = {'type': 'floor_markers', 'axle_offset_m': 0.065,
          'boards': [{'name': 'front', 'roles': ['front'], 'inner_corners': [6, 4], 'square_m': 0.05,
                      'centre_m': [0.62, 0.0], 'yaw_deg': 90.0}]}


def test_boards_move_forward_by_the_axle_offset_only():
    (b,) = floor_board_dicts(TARGET)
    assert b['centre_m'] == [0.685, 0.0]                      # 0.62 + 0.065
    assert b['sheet_centre_m'] == [0.62, 0.0]
    assert b['yaw_deg'] == 90.0 and b['name'] == 'front' and b['inner_corners'] == [6, 4]
    assert TARGET['boards'][0]['centre_m'] == [0.62, 0.0]     # the YAML dict is not modified


def test_zero_offset_is_the_layout_as_written():
    t = copy.deepcopy(TARGET)
    t['axle_offset_m'] = 0.0
    assert floor_board_dicts(t)[0]['centre_m'] == [0.62, 0.0]


def test_missing_axle_offset_is_a_loud_error():
    t = copy.deepcopy(TARGET)
    del t['axle_offset_m']
    with pytest.raises(KeyError, match='axle_offset_m'):
        floor_board_dicts(t)


def test_shipped_yaml_has_the_offset_and_the_front_board():
    import yaml
    path = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'carbot_bringup', 'config', 'data',
                        'calibration_steps.yaml')
    steps = {s['id']: s for s in yaml.safe_load(open(path, encoding='utf-8'))['steps']}
    boards = floor_board_dicts(steps['extrinsics_ipm']['target'])
    assert [b['name'] for b in boards] == ['front']
    assert boards[0]['centre_m'] == [0.685, 0.0]
