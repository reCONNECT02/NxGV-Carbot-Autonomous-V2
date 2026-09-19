"""BLOCK 09 - Find the lane we should follow (V4 guidance.js corridor, branchCheck).

Combines the chosen route branch with the road actually seen: finds the
middle between paired road/paint edges (live camera, else remembered cells),
and applies a small, clamped common offset to the route guide. Branch check
holds when the intended opening lacks road evidence or course location
conflicts (route identity), which may trigger checked recovery.
"""
from carbot_common import topics as T
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import Corridor, LocalGrid, LocalizationStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from carbot_common.qos import LATCHED
from nav_msgs.msg import Odometry, Path

SPEC = BlockSpec(
    node='corridor', block='09', title='Corridor + Branch Check', phase=4,
    required=['rate_hz', 'horizon_m', 'edge_search_min_m', 'edge_search_max_m',
              'lane_width_min_m', 'lane_width_max_m', 'shift_clamp_m', 'shift_gain',
              'guide_points', 'live_max_age_s', 'lane_locked_min_observed',
              'branch.near_radius_m', 'branch.min_seen_ratio', 'data.track_map'],
    subs=[(Path, T.ACTIVE_PATH, LATCHED), (Odometry, T.LOCAL_POSE, 10),
          (PoseWithCovarianceStamped, T.GLOBAL_POSE, 10), (LocalGrid, T.ROAD_GRID, 10),
          (LocalGrid, T.MEMORY_GRID, 1), (LocalizationStatus, T.LOCALIZATION_STATUS, 10)],
    pubs=[(Corridor, T.CORRIDOR, 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
