"""BLOCK 01 - Set the map, start and goals (V4 core.js DEFAULTS, Course).

Loads data/track_map.yaml (v1 or v2; v2 + data/track_features.yaml +
data/mission.yaml for the poses) through carbot_common.course and publishes it
latched as JSON for the GUI (map tab, main tab) and any late joiner:

  {version, frame, fingerprints, lane_width_m, ring, exits, sections {name: [[x, y]..]},
   areas {name: {kind, poly}}, paint [[xa, ya, xb, yb, style, width]], features {...},
   provisional [feature names still marked provisional], unfitted [map sections the
   map lap did not cover], vehicle {...}}

Car geometry is in common.yaml (vehicle.*). Status WARN while features are
provisional or map sections are unfitted (both are listed in docs/BACKLOG.md).
"""
import json

import numpy as np
import rclpy
from carbot_common import topics as T
from carbot_common.course import course_from_params, file_fingerprints
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED
from carbot_interfaces.msg import NodeStatus
from std_msgs.msg import String

REQUIRED = ['publish_period_s', 'centreline_step_m', 'data.track_map', 'data.track_features',
            'data.mission', 'vehicle.car_length_m', 'vehicle.wheelbase_m', 'vehicle.min_turning_radius_m']

FEATURE_KEYS = ('start_pose', 'light_goal_pose', 'traffic_light', 'boom_gates', 'speed_bump',
                'elevation', 'tunnel', 'sign_height_m')


def _r(v):
    return round(float(v), 4)


def _clean(v):
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, (float, np.floating)):
        return _r(v)
    if isinstance(v, np.integer):
        return int(v)
    return v


def map_json(course, fingerprints, vehicle, step: float) -> dict:
    stride = max(1, int(round(step / 0.025)))
    sections = {n: [[_r(x), _r(y)] for x, y in p[::stride]] + [[_r(p[-1, 0]), _r(p[-1, 1])]]
                for n, p in course.sections.items()}
    feats = {k: course.feat[k] for k in FEATURE_KEYS if k in course.feat}
    provisional = []
    for k, v in feats.items():
        if isinstance(v, dict):
            if v.get('provisional'):
                provisional.append(k)
            for sk, sv in v.items():
                if isinstance(sv, dict) and sv.get('provisional'):
                    provisional.append(f'{k}.{sk}')
    unfitted = [n for n, r in (course.tm.get('fit_report') or {}).items() if not r.get('ok', True)]
    return _clean({
        'version': course.version, 'frame': 'track', 'fingerprints': fingerprints,
        'lane_width_m': course.lane, 'ring': {'x': course.ring[0], 'y': course.ring[1], 'r': course.ring[2]},
        'exits': {k: list(v) for k, v in course.exits.items()}, 'sections': sections,
        'areas': {n: {'kind': course.area_kind.get(n, 'area'), 'poly': course.area_polys[n].tolist()}
                  for n in course.area_polys},
        'paint': [[a[0], a[1], b[0], b[1], s, w] for a, b, s, w in course.paint],
        'features': feats, 'provisional': provisional, 'unfitted': unfitted, 'vehicle': vehicle,
        'bounds': [course.x0, course.y0, course.x0 + course.w, course.y0 + course.h],
    })


class TrackMapServer(CarbotNode):

    def __init__(self):
        super().__init__('track_map_server', '01', REQUIRED)
        self.pub = self.create_publisher(String, T.TRACK_MAP_JSON, LATCHED)
        try:
            course = course_from_params(self.p)
            fp = file_fingerprints(str(self.p('data.track_map')))
            doc = map_json(course, fp, self.params_under('vehicle'), float(self.p('centreline_step_m')))
        except Exception as e:  # noqa: BLE001
            self.set_status(NodeStatus.ERROR, 'MAP_ERROR', str(e))
            self.get_logger().error(f'cannot load the map: {e}')
            return
        self.msg = String(data=json.dumps(doc, separators=(',', ':')))
        self.pub.publish(self.msg)
        warn = []
        if doc['provisional']:
            warn.append('provisional: ' + ', '.join(doc['provisional']))
        if doc['unfitted']:
            warn.append('not covered by the map lap: ' + ', '.join(doc['unfitted']))
        self.set_status(NodeStatus.WARN if warn else NodeStatus.OK, 'PUBLISHED',
                        f'track_map v{course.version} sha1 {fp["raw"][:10]}'
                        + ('; ' + '; '.join(warn) if warn else ''))
        self.get_logger().info(f'map v{course.version} published ({len(self.msg.data) // 1024} kB)')
        period = float(self.p('publish_period_s'))
        if period > 0:
            self.create_timer(period, lambda: self.pub.publish(self.msg))


def main(args=None):
    rclpy.init(args=args)
    node = TrackMapServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
