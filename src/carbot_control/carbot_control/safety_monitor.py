"""BLOCK 14 - Safety: decide whether motion is allowed (V4 app.js safety checks).

Checks: e-stop, camera (road grid) freshness <= 0.45 s, motion sensor
freshness <= 0.2 s, local uncertainty <= 3.5 cm, usable road mask, tunnel
forward LiDAR clearance >= 24 cm, unresolved route identity. A valid path
never overrides a safety stop; recovery cannot bypass these. UWB is NOT an
input (a WiFi dropout must never stop the car).
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED, SENSOR
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import Corridor, LocalGrid, LocalizationStatus, MissionState, SafetyStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

SPEC = BlockSpec(
    node='safety_monitor', block='14', title='Safety Checks', phase=5,
    required=['rate_hz', 'camera_max_age_s', 'motion_max_age_s', 'local_sigma_max_m',
              'road_min_connected_cells', 'road_low_dwell_s', 'tunnel_front_half_angle_rad',
              'tunnel_min_clearance_m', 'lidar_max_age_s', 'hold_on_branch_conflict'],
    subs=[(Bool, T.E_STOP, 10), (LocalGrid, T.ROAD_GRID, 10), (Odometry, T.ODOM, 10),
          (LocalizationStatus, T.LOCAL_STATUS, 10), (LaserScan, T.SCAN, SENSOR),
          (MissionState, T.MISSION_STATE, LATCHED), (Corridor, T.CORRIDOR, 10)],
    pubs=[(SafetyStatus, T.SAFETY_STATUS, 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
