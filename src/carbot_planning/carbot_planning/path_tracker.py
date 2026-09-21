"""BLOCK 13 - Turn the path into a request (V4 vehicle.js Controller.update).

Pure pursuit on a nearby look-ahead point (9.5 cm road, 6.5 cm parking),
steering limited by wheelbase/turning radius, curvature-based speed, stops
before every gear change. See tracker_core.py.

ROAD (active piece = road): progress, speed and arrival come from the route
  piece (/carbot/mission/active_path); the STEERING aims at block 10's
  selected line (/carbot/plan/local_path), as V4 app.js step() does. An empty
  or stale local path = no feasible candidate -> speed 0 (V4: the problem
  becomes a safety hold unless recovery takes over). Published on
  /carbot/request/road.
PARKING (mission mode PARKING): follows block 11's /carbot/parking/path one gear
  section at a time (V4 splitGears + transition): stop, hold, switch gear.
  `arrived` only at the end of the last section. Published on /carbot/request/parking.

Speed is capped by MissionState.zone_max_speed_mps (lowest active speed zone).
Requests are published whether or not their source is active: the command
owner (block 15) only follows MissionState.active_source.
"""
import math

import rclpy
from carbot_common import topics as T
from carbot_common.geometry import geometry
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED, SENSOR
from carbot_interfaces.msg import MissionState, MotionRequest, NodeStatus
from nav_msgs.msg import Odometry, Path

from .ros_util import path_from_msg, path_key, pose_of
from .tracker_core import GearSequencer, Tracker, TrackerCfg

REQUIRED = ['rate_hz', 'lookahead_road_m', 'lookahead_parking_m', 'speed_base_mps', 'speed_curve_mps',
            'end_slowdown_gain', 'end_min_speed_mps', 'end_index_points', 'gear_window_points',
            'gear_window_speed_mps', 'gear_lookahead_points', 'gear_stop_speed_mps', 'gear_hold_s',
            'parking_gear_hold_s', 'arrive_road_m', 'arrive_parking_m', 'arrive_lateral_road_m',
            'arrive_lateral_parking_m', 'arrive_behind_m', 'arrived_speed_mps', 'local_path_max_age_s',
            'pose_max_age_s', 'limits.max_speed_mps', 'limits.parking_speed_mps', 'vehicle.wheelbase_m',
            'vehicle.min_turning_radius_m']

ROAD_MODES = (MissionState.MODE_ROAD, MissionState.MODE_HOLD, MissionState.MODE_RECOVERY,
              MissionState.MODE_SAFETY_STOP, MissionState.MODE_TUNNEL)


class PathTracker(CarbotNode):

    def __init__(self):
        super().__init__('path_tracker', '13', REQUIRED)
        self.cfg = TrackerCfg.from_params(self.p)
        self.g = geometry(self.params_under('vehicle'))
        self.road = Tracker(self.cfg, self.g)
        self.gears = GearSequencer(Tracker(self.cfg, self.g), float(self.p('parking_gear_hold_s')))
        self.pose, self.pose_t = None, -1e9
        self.speed = 0.0
        self.mission = None
        self.active, self.active_key = None, None
        self.local, self.local_t = None, -1e9
        self.parking, self.parking_key = None, None
        self.pub_road = self.create_publisher(MotionRequest, T.request_topic('ROAD'), 10)
        self.pub_park = self.create_publisher(MotionRequest, T.request_topic('PARKING'), 10)
        self.sub(Odometry, T.LOCAL_POSE, self._on_pose, 10)
        self.sub(Odometry, T.ODOM, self._on_odom, SENSOR)
        self.sub(MissionState, T.MISSION_STATE, lambda m: setattr(self, 'mission', m), LATCHED)
        self.sub(Path, T.ACTIVE_PATH, self._on_active, LATCHED)
        self.sub(Path, T.LOCAL_PATH, self._on_local, 10)
        self.sub(Path, T.PARKING_PATH, self._on_parking, LATCHED)
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 1.0), self._tick)
        self.set_status(NodeStatus.WARN, 'WAITING_INPUT', 'waiting for mission + pose')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, m):
        self.pose, self.pose_t = pose_of(m), self.now()

    def _on_odom(self, m):
        self.speed = float(m.twist.twist.linear.x)

    def _on_active(self, m: Path):
        if path_key(m) != self.active_key:
            self.active_key, self.active = path_key(m), path_from_msg(m)
            self.road.reset()

    def _on_local(self, m: Path):
        self.local, self.local_t = path_from_msg(m), self.now()

    def _on_parking(self, m: Path):
        self.parking_key, self.parking = path_key(m), path_from_msg(m)

    def _request(self, source: str, r: dict, cap: float) -> MotionRequest:
        m = MotionRequest()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source = source
        v = float(r['speed'])
        if cap > 0 and v != 0.0:
            v = math.copysign(min(abs(v), cap), v)
        m.speed_mps, m.steer_rad = v, float(r['steer'])
        m.arrived, m.reason = bool(r['arrived']), str(r['reason'])
        return m

    def _tick(self) -> None:
        now = self.now()
        ms = self.mission
        if ms is None or self.pose is None or now - self.pose_t > float(self.p('pose_max_age_s')):
            return
        cap = float(ms.zone_max_speed_mps)
        if ms.mode == MissionState.MODE_PARKING:
            if self.parking is None or len(self.parking) < 2:
                self.set_status(NodeStatus.WARN, 'PARKING', 'no parking path from block 11 yet')
                return
            self.gears.set_path(self.parking, self.parking_key)
            r = self.gears.update(self.pose, self.speed, now)
            self.pub_park.publish(self._request('PARKING', r, cap))
            self.set_status(NodeStatus.OK, 'PARKING', r['reason'])
            return
        if ms.mode not in ROAD_MODES or self.active is None or len(self.active) < 2 \
                or (self.active[:, 3] < 0).any():
            return
        r = self.road.update(self.active, self.pose, self.speed, now, False)
        if r['reason'] == 'TRACKING':
            fresh = now - self.local_t < float(self.p('local_path_max_age_s'))
            if not fresh or self.local is None or not len(self.local):
                r = {'speed': 0.0, 'steer': 0.0, 'arrived': False,
                     'reason': 'No feasible local candidate' if fresh else 'Local path stale'}
            else:
                s = self.road.road_steer(self.local, self.pose)
                if s is not None:
                    r['steer'] = s
        self.pub_road.publish(self._request('ROAD', r, cap))
        self.set_status(NodeStatus.OK, 'ROAD', f'{r["reason"]} v {r["speed"]:.3f} '
                        f'err {self.road.error * 100:.1f} cm idx {self.road.index}/{len(self.active)}')


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
