"""BLOCK 05 - Keep a smooth local position (V4 vehicle.js Estimator).

Predict with wheel distance + IMU heading; correct translation by matching
camera road/paint edges to nearby prior-map boundaries (small bounded steps).
UWB NEVER writes this estimate, so it never jumps. Matching pauses on steep
pitch and during parking. Output frame: track.
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import LocalGrid, LocalizationStatus, MissionState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

SPEC = BlockSpec(
    node='local_pose', block='05', title='Smooth Local Position', phase=3,
    required=['rate_hz', 'motion_timeout_s', 'heading_blend', 'sigma.initial_m',
              'sigma.idle_growth_m_per_s', 'sigma.per_distance', 'sigma.per_sqrt_s',
              'sigma.floor_m', 'sigma.visual_decay', 'visual.enabled', 'visual.max_radius_m',
              'visual.max_edge_distance_m', 'visual.min_rows', 'visual.disable_pitch_rad',
              'data.track_map', 'vehicle.wheelbase_m'],
    subs=[(Odometry, T.ODOM, 10), (String, T.IMU_RPY, 10), (Float32, T.IMU_PITCH, 10),
          (LocalGrid, T.ROAD_GRID, 10), (MissionState, T.MISSION_STATE, LATCHED)],
    pubs=[(Odometry, T.LOCAL_POSE, 10), (LocalizationStatus, T.LOCAL_STATUS, 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
