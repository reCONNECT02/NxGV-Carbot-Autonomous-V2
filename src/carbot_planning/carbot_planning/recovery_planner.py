"""BLOCK 12 - Recovery planner: make room, then rejoin (V4 recovery.js).

When no forward candidate is feasible for 0.45 s: brake, search Reeds-Shepp
reverse-then-forward connections to up to five future route poses (reverse
<= 20 cm, one gear change, rear road evidence, LiDAR-clear swept footprint),
then track it at 3 cm/s. Cannot bypass safety holds. Publishes its own
MotionRequest (source RECOVERY); mission logic decides when it is active.
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED, SENSOR
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import (CandidateArray, LocalGrid, MissionState, MotionRequest,
                                   SafetyStatus)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

SPEC = BlockSpec(
    node='recovery_planner', block='12', title='Recovery Planner', phase=5,
    required=['enabled', 'reverse_max_m', 'reverse_min_m', 'problem_dwell_s', 'max_attempts',
              'rescan_period_s', 'align_s', 'max_goals', 'max_cost', 'rear_evidence_min_ratio',
              'limits.recovery_speed_mps', 'limits.line_tolerance_m', 'data.track_map'],
    subs=[(Odometry, T.LOCAL_POSE, 10), (Path, T.ACTIVE_PATH, LATCHED),
          (LocalGrid, T.MEMORY_GRID, 1), (LocalGrid, T.ROAD_GRID, 10), (LaserScan, T.SCAN, SENSOR),
          (SafetyStatus, T.SAFETY_STATUS, 10), (CandidateArray, T.LOCAL_CANDIDATES, 1),
          (MissionState, T.MISSION_STATE, LATCHED), (Odometry, T.ODOM, 10)],
    pubs=[(CandidateArray, T.RECOVERY_CANDIDATES, 1),
          (MotionRequest, T.request_topic('RECOVERY'), 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
