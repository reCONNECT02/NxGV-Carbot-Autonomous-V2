"""BLOCK 07 - Global planner: choose the road route (V4 core.js buildMission, hybridPlan).

Plans each mission.yaml leg through its clockwise checkpoints with a car-like
hybrid A* that checks the whole nominal body and the turning radius. The
ROUNDABOUT EXITS ARE FIXED HERE from mission.yaml roundabout_visits; they are
never chosen from the boom gate or from UWB alone. Road approach ends at the
parking handoff; local parking (block 11) starts there.
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED
from carbot_common.stub import BlockSpec, run_stub
from nav_msgs.msg import Path
from std_msgs.msg import String

SPEC = BlockSpec(
    node='global_planner', block='07', title='Global Route Planner', phase=4,
    required=['step_m', 'xy_resolution_m', 'heading_bins', 'curvature_fractions', 'max_nodes',
              'planning_margin_m', 'goal_tolerance_m', 'goal_tolerance_rad',
              'heuristic_heading_weight', 'heuristic_inflation', 'clockwise_ring_inner_m',
              'clockwise_ring_outer_m', 'cache_routes', 'data.track_map', 'data.mission'],
    pubs=[(Path, T.GLOBAL_ROUTE, LATCHED), (String, T.ROUTE_INFO_JSON, LATCHED)],
)


def main(args=None):
    run_stub(SPEC, args=args)
