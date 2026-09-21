"""BLOCK 07 - Global planner: choose the road route (V4 core.js buildMission, hybridPlan).

At startup, once:
  v2 mission.yaml (default): CHECK the team's mission_planner.py route - map
     fingerprint, whole-body road check of every road piece, roundabout exits
     against mission_rules.yaml roundabout_visits. Nothing is replanned: any
     failure is a route FAIL with the reason (race refuses to arm).
  v1 mission.yaml (V4 reference): plan every leg through its checkpoints with
     the V4 hybrid A* (clockwise roundabout rule), cached by input sha1 in
     <data_root>/route_cache/.

The ROUNDABOUT EXITS ARE FIXED HERE; the boom gate and UWB never change them.
Road approach ends at the parking handoff; block 11 plans the manoeuvre.

Out (latched):
  /carbot/plan/global_route  nav_msgs/Path, track frame, every piece in order,
                             1 cm spacing, pose.position.z = direction (+1 / -1)
  /carbot/plan/route_info    JSON {ok, reason, source, warnings, fingerprints,
                             pieces [{index, leg, leg_id, kind, bay, start, end,
                             length_m, end_behaviour, sections [{section, start, end}]}],
                             visits [{visit, piece, enter, leave, entry, exit, label, leg_id}],
                             legs [{index, id, end_behaviour, parking_bay, exits}]}
                             (start/end = inclusive indices into global_route; -1 if empty)
"""
import hashlib
import json
import os
import time

import numpy as np
import rclpy
from carbot_common import topics as T
from carbot_common.course import course_from_params, file_fingerprints
from carbot_common.geometry import geometry
from carbot_common.mission import mission_from_params
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED
from carbot_interfaces.msg import NodeStatus
from nav_msgs.msg import Path
from std_msgs.msg import String

from .ros_util import path_to_msg
from .route_core import PlanCfg, RouteResult, path_length, plan_mission, section_spans

REQUIRED = ['step_m', 'reverse_step_m', 'xy_resolution_m', 'reverse_xy_resolution_m', 'heading_bins',
            'curvature_fractions', 'max_nodes', 'planning_margin_m', 'reverse_planning_margin_m',
            'goal_tolerance_m', 'goal_tolerance_rad', 'reverse_goal_tolerance_m',
            'reverse_goal_tolerance_rad', 'heuristic_heading_weight', 'heuristic_inflation',
            'cost_reverse_factor', 'cost_curvature', 'cost_direction_change', 'cost_curvature_change',
            'cost_clearance', 'clearance_floor_m', 'clockwise_ring_inner_m', 'clockwise_ring_outer_m',
            'clockwise_min_dot', 'cache_routes', 'route_check_tolerance_m', 'densify_step_m',
            'data.track_map', 'data.track_features', 'data.mission', 'data.mission_rules', 'data_root',
            'frames.track', 'vehicle.car_length_m', 'vehicle.wheelbase_m', 'vehicle.min_turning_radius_m']


def route_info(result: RouteResult, mission, fingerprints) -> dict:
    pieces, k = [], 0
    for p in result.pieces:
        n = len(p['points'])
        pieces.append({'index': p['index'], 'leg': p['leg'], 'leg_id': p['leg_id'], 'kind': p['kind'],
                       'bay': p['bay'], 'start': k if n else -1, 'end': k + n - 1 if n else -1,
                       'length_m': round(path_length(p['points']), 3) if n else 0.0,
                       'end_behaviour': p['end_behaviour'], 'sections': section_spans(p['sections'])})
        k += n
    legs = [{'index': lg.index, 'id': lg.id, 'end_behaviour': lg.end_behaviour, 'parking_bay': lg.parking_bay,
             'enabled': lg.enabled, 'exits': [v['exit'] for v in result.visits if v['leg_id'] == lg.id]}
            for lg in mission.legs]
    return {'ok': result.ok, 'reason': result.reason, 'source': result.source, 'warnings': result.warnings,
            'mission_version': mission.version, 'fingerprints': fingerprints, 'pieces': pieces,
            'visits': result.visits, 'legs': legs}


class GlobalPlanner(CarbotNode):

    def __init__(self):
        super().__init__('global_planner', '07', REQUIRED)
        self.pub_route = self.create_publisher(Path, T.GLOBAL_ROUTE, LATCHED)
        self.pub_info = self.create_publisher(String, T.ROUTE_INFO_JSON, LATCHED)
        self.set_status(NodeStatus.WARN, 'PLANNING', 'computing the route')
        self.create_timer(0.1, self._once)
        self._done = False

    def _once(self) -> None:
        if self._done:
            return
        self._done = True
        t0 = time.monotonic()
        try:
            course = course_from_params(self.p)
            mission = mission_from_params(self.p)
            g = geometry(self.params_under('vehicle'))
            fp = file_fingerprints(str(self.p('data.track_map')))
            result = self._cached(course, mission, g, fp)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'route FAILED: {e}')
            self.set_status(NodeStatus.ERROR, 'ROUTE_FAIL', str(e))
            self.pub_info.publish(String(data=json.dumps({'ok': False, 'reason': str(e), 'pieces': [],
                                                          'visits': [], 'legs': [], 'warnings': []})))
            return
        info = route_info(result, mission, fp)
        pts = np.vstack([p['points'] for p in result.pieces if len(p['points'])]) if result.ok else np.zeros((0, 4))
        self.pub_route.publish(path_to_msg(pts, str(self.p('frames.track')), self.get_clock().now().to_msg()))
        self.pub_info.publish(String(data=json.dumps(info, separators=(',', ':'))))
        dt = time.monotonic() - t0
        if not result.ok:
            self.get_logger().error(f'route FAIL: {result.reason}')
            self.set_status(NodeStatus.ERROR, 'ROUTE_FAIL', result.reason)
            return
        exits = ', '.join(f'visit {v["visit"]} {v["exit"]}' for v in result.visits)
        for w in result.warnings:
            self.get_logger().warn(w)
        self.get_logger().info(f'route OK ({result.source}): {len(result.pieces)} pieces, '
                               f'{path_length(pts):.2f} m, exits {exits}, {dt:.1f} s')
        self.set_status(NodeStatus.WARN if result.warnings else NodeStatus.OK, 'ROUTE_OK',
                        f'{len(result.pieces)} pieces, exits {exits}'
                        + (f'; {len(result.warnings)} warnings' if result.warnings else ''))

    def _cached(self, course, mission, g, fp) -> RouteResult:
        cfg = PlanCfg.from_params(self.p)
        tol = float(self.p('route_check_tolerance_m'))
        dens = float(self.p('densify_step_m'))
        log = lambda s: self.get_logger().info(s)  # noqa: E731
        if mission.version == 2 or not bool(self.p('cache_routes')):
            return plan_mission(course, mission, g, cfg, fp, tol, dens, log)
        h = hashlib.sha1()
        for k in ('data.track_map', 'data.mission', 'data.mission_rules'):
            with open(str(self.p(k)), 'rb') as f:
                h.update(f.read())
        h.update(repr((cfg, tol, dens, g)).encode())
        d = os.path.join(os.path.expanduser(str(self.p('data_root'))), 'route_cache')
        path = os.path.join(d, h.hexdigest()[:16] + '.npz')
        if os.path.isfile(path):
            try:
                z = np.load(path, allow_pickle=True)
                self.get_logger().info(f'route from cache {path}')
                return z['result'].item()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'route cache unreadable ({e}): replanning')
        r = plan_mission(course, mission, g, cfg, fp, tol, dens, log)
        if r.ok:
            try:
                os.makedirs(d, exist_ok=True)
                np.savez(path, result=np.array(r, dtype=object))
            except OSError as e:
                self.get_logger().warn(f'cannot write route cache: {e}')
        return r


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
