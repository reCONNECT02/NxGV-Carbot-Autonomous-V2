"""BLOCK 11 - Parking planner: compute the manoeuvre (V4 core.js parkingPlan, reeds-shepp.js).

From the handoff pose, observes the bay from camera/memory tape edges, then
searches Reeds-Shepp forward/reverse connections (preferring a 15/12/8 cm
docking straight), handoff extensions, and a bounded hybrid fallback. Replans
at gear cusps and for terminal heading error. Computed, never replayed.
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED, SENSOR
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import CandidateArray, LocalGrid, MissionState
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

SPEC = BlockSpec(
    node='parking_planner', block='11', title='Parking Planner', phase=5,
    required=['docking_tails_m', 'handoff_extensions_m', 'clearance_pad_m', 'fallback_max_nodes',
              'bay_observation.edge_tolerance_m', 'bay_observation.min_samples',
              'max_cusp_replans', 'heading_tolerance_rad', 'max_corrections', 'data.track_map'],
    subs=[(Odometry, T.LOCAL_POSE, 10), (LocalGrid, T.MEMORY_GRID, 1),
          (MissionState, T.MISSION_STATE, LATCHED), (LaserScan, T.SCAN, SENSOR)],
    pubs=[(CandidateArray, T.PARKING_CANDIDATES, 1), (Path, T.PARKING_PATH, LATCHED),
          (String, T.PARKING_BAY_JSON, 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
