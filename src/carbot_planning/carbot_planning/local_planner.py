"""BLOCK 10 - Local planner: choose the next movement (V4 local-planner.js).

At 5 Hz, rolls out nine lateral offsets along the corridor guide with a
bicycle model + servo lag, checks swept body clearance, road/paint evidence and
LiDAR obstacles, and picks the lowest-cost valid candidate (grey = tried,
blue = selected). If all strict candidates fail, retries nine more with the
paint allowance (limits.line_tolerance_m).
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED, SENSOR
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import CandidateArray, Corridor, LocalGrid, MissionState
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

SPEC = BlockSpec(
    node='local_planner', block='10', title='Local Path Planner', phase=4,
    required=['rate_hz', 'offsets_m', 'rollout_steps', 'rollout_ds_m', 'lookahead_m',
              'rollout_speed_min_mps', 'rollout_speed_max_mps', 'cost.tracking', 'cost.offset',
              'cost.paint', 'relaxed_retry', 'problem_tracking_error_m',
              'limits.line_tolerance_m', 'data.track_map'],
    subs=[(Corridor, T.CORRIDOR, 10), (Odometry, T.LOCAL_POSE, 10), (LocalGrid, T.ROAD_GRID, 10),
          (LocalGrid, T.MEMORY_GRID, 1), (LaserScan, T.SCAN, SENSOR),
          (MissionState, T.MISSION_STATE, LATCHED)],
    pubs=[(CandidateArray, T.LOCAL_CANDIDATES, 1), (Path, T.LOCAL_PATH, 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
