"""BLOCK 04 - Remember what just disappeared (V4 vehicle.js LocalMemory).

Road/paint cells from block 03 are stored in the fixed odom frame with their
time and travelled distance. Driving uses cells up to 2-3 s old within a
motion-based uncertainty limit; parking may use 24 s. LiDAR is NOT stored here.
"""
from carbot_common import topics as T
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import LocalGrid, MissionState
from carbot_common.qos import LATCHED
from nav_msgs.msg import Odometry

SPEC = BlockSpec(
    node='local_memory', block='04', title='Recent Local Memory', phase=2,
    required=['resolution_m', 'drive_max_age_s', 'corridor_max_age_s', 'parking_max_age_s',
              'display_max_age_s', 'max_cells', 'uncertainty_base_m', 'uncertainty_per_m',
              'uncertainty_limit_m', 'publish_rate_hz'],
    subs=[(LocalGrid, T.ROAD_GRID, 10), (Odometry, T.LOCAL_POSE, 10),
          (MissionState, T.MISSION_STATE, LATCHED)],
    pubs=[(LocalGrid, T.MEMORY_GRID, 1)],
)


def main(args=None):
    run_stub(SPEC, args=args)
