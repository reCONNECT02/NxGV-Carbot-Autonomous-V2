"""Tunnel wrapper (Challenge 3). Base tunnel code is used UNCHANGED.

The base risabot_automode/tunnel_wall_follower node runs as-is and keeps
publishing its LiDAR trigger (/tunnel_detected) and LiDAR lane-tracking
command (/tunnel_cmd_vel). This bridge only:
  * forwards /tunnel_cmd_vel as MotionRequest(source=TUNNEL) on
    /carbot/request/tunnel, and only while mission logic has TUNNEL active;
  * converts the base normalized angular.z to steer_rad with the exact
    inverse of command_owner.steering, so the servo receives the same value
    it would in the base stack;
  * sets speed from the tunnel speed zone (or passes base duty, see YAML).
Mission logic uses /tunnel_detected (LiDAR trigger) to activate TUNNEL.
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import MissionState, MotionRequest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

SPEC = BlockSpec(
    node='tunnel_bridge', block='13', title='Tunnel bridge (wraps base tunnel follower)', phase=5,
    required=['cmd_max_age_s', 'speed_source', 'zone_speed_mps'],
    subs=[(Twist, T.TUNNEL_CMD, 10), (Bool, T.TUNNEL_DETECTED, 10),
          (MissionState, T.MISSION_STATE, LATCHED)],
    pubs=[(MotionRequest, T.request_topic('TUNNEL'), 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
