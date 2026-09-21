"""Blocks 11/12 Reeds-Shepp + parkingPlan against the V4 JavaScript (Node).

tools/v4_harness/parking_v4.js runs reeds-shepp.js / core.js parkingPlan on
the V4 course. Skipped when `node` is not installed."""
import json
import math
import random
import shutil
import subprocess
import os

import numpy as np
import pytest
from carbot_common.course import load_course
from carbot_planning import reeds_shepp as rs
from carbot_planning.parking_core import ParkingCfg, parking_plan
from plan_fixtures import HARNESS, V4, geom

NODE = shutil.which('node')
pytestmark = pytest.mark.skipif(NODE is None, reason='node not installed')


def _js(arg):
    return json.loads(subprocess.check_output([NODE, os.path.join(HARNESS, 'parking_v4.js'), json.dumps(arg)]))


def _key(types, variant, lengths):
    return types, variant, tuple(round(x, 7) for x in lengths)


def test_reeds_shepp_candidates_equal_v4():
    random.seed(3)
    worst, n = 0.0, 0
    for _ in range(25):
        s = (random.uniform(0, 3), random.uniform(0, 3), random.uniform(-3, 3))
        g = (random.uniform(0, 3), random.uniform(0, 3), random.uniform(-3, 3))
        js = _js({'what': 'rs', 'start': dict(x=s[0], y=s[1], a=s[2]), 'goal': dict(x=g[0], y=g[1], a=g[2]),
                  'r': 0.4})
        py = rs.candidates(s, g, 0.4)
        # three families produce 'LRL': match by (types, variant, lengths); equal-length
        # mirror images may swap order by one ulp (libm vs V8), so compare as sets
        J = {_key(a['types'], a['variant'], a['lengths']): a for a in js}
        P = {_key(b.types, b.variant, b.lengths): b for b in py}
        assert J.keys() == P.keys()
        for k in J:
            A = np.array(J[k]['points'])
            worst = max(worst, float(np.abs(A - P[k].points).max()), abs(J[k]['cost'] - P[k].cost))
            n += 1
    assert n > 100 and worst < 1e-9


@pytest.mark.parametrize('start', [(2.4, 3.02, math.pi / 2), (2.38, 3.05, math.pi / 2 - 0.04),
                                   (2.4, 2.9, math.pi / 2)])
def test_parking_plan_equals_v4(start):
    js = _js({'what': 'park', 'start': dict(x=start[0], y=start[1], a=start[2])})
    course = load_course(os.path.join(V4, 'track_map.yaml'))
    goal = (js['goal']['x'], js['goal']['y'], js['goal']['a'])
    r = parking_plan(start, goal, course, geom(), ParkingCfg(plan_radius_factor=1.0), True, None)
    assert r.stage == js['stage']
    assert len(r.evaluated) == js['evaluated'] and sum(a.valid for a in r.evaluated) == js['valid']
    A = np.array(js['points'])[:, :4]
    assert A.shape == r.path.shape and np.abs(A - r.path).max() < 1e-7
