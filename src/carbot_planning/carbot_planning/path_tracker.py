"""BLOCK 13 - Turn the path into a request (V4 vehicle.js Controller.update).

Pure pursuit on a nearby look-ahead point (9.5 cm road, 6.5 cm parking),
steering limited by wheelbase/turning radius, curvature-based speed, stops
before every gear change. Road mode aims at the camera-conditioned corridor
target + selected local offset. Publishes MotionRequest on
/carbot/request/road or /carbot/request/parking.
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import Corridor, MissionState, MotionRequest
from nav_msgs.msg import Odometry, Path

SPEC = BlockSpec(
    node='path_tracker', block='13', title='Path -> Motion Request', phase=4,
    required=['rate_hz', 'lookahead_road_m', 'lookahead_parking_m', 'speed_base_mps',
              'speed_curve_mps', 'end_slowdown_gain', 'gear_stop_speed_mps', 'gear_hold_s',
              'arrive_road_m', 'arrive_parking_m', 'limits.max_speed_mps',
              'limits.parking_speed_mps', 'vehicle.wheelbase_m'],
    subs=[(Odometry, T.LOCAL_POSE, 10), (Path, T.LOCAL_PATH, 10), (Corridor, T.CORRIDOR, 10),
          (Path, T.PARKING_PATH, LATCHED), (Path, T.ACTIVE_PATH, LATCHED),
          (MissionState, T.MISSION_STATE, LATCHED), (Odometry, T.ODOM, 10)],
    pubs=[(MotionRequest, T.request_topic('ROAD'), 10),
          (MotionRequest, T.request_topic('PARKING'), 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
