"""BLOCK 08 - Mission logic: choose what happens now (V4 app.js step, transition).

State machine ROAD / TUNNEL / PARKING / RECOVERY / SAFETY_STOP (+ HOLD/IDLE/
COMPLETE). Chooses the active route leg, behaviour and the motion source the
command owner must follow. Holds at an observed red light / closed gate
(detector only, never a timer). Tunnel is activated from the base LiDAR
trigger (/tunnel_detected) + route position. If the detected roundabout gate
state disagrees with the planned exit it publishes a GateRouteMismatch and a
GUI warning banner, and does NOT change the route.
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import (Corridor, DetectionArray, GateRouteMismatch, MissionEvent,
                                   MissionState, MotionRequest, SafetyStatus)
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String

SPEC = BlockSpec(
    node='mission_logic', block='08', title='Mission Logic', phase=4,
    required=['rate_hz', 'tunnel_requires_route_zone', 'tunnel_zone_margin_m',
              'detection_max_age_s', 'announce_banners', 'data.mission', 'data.track_map',
              'data.challenges'],
    subs=[(Odometry, T.LOCAL_POSE, 10), (PoseWithCovarianceStamped, T.GLOBAL_POSE, 10),
          (DetectionArray, T.DETECTIONS, 10), (Bool, T.TUNNEL_DETECTED, 10),
          (Corridor, T.CORRIDOR, 10), (SafetyStatus, T.SAFETY_STATUS, 10),
          (Bool, T.RACE_ARMED, LATCHED), (Path, T.GLOBAL_ROUTE, LATCHED),
          (String, T.ROUTE_INFO_JSON, LATCHED), (MotionRequest, T.request_topic('ROAD'), 10),
          (MotionRequest, T.request_topic('PARKING'), 10), (Path, T.PARKING_PATH, 10),
          (Bool, T.E_STOP, 10)],
    pubs=[(MissionState, T.MISSION_STATE, LATCHED), (MissionEvent, T.MISSION_EVENTS, 10),
          (GateRouteMismatch, T.GATE_ROUTE_MISMATCH, 10), (Path, T.ACTIVE_PATH, LATCHED)],
)


def _idle(node):
    """Stub: announce IDLE so the GUI and command owner have a defined state."""
    msg = MissionState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.mode = MissionState.MODE_IDLE
    msg.active_source = 'HOLD'
    msg.banner = 'Phase 1 skeleton: mission logic not implemented'
    msg.banner_level = 1
    node.publishers_by_topic[T.MISSION_STATE].publish(msg)


def main(args=None):
    run_stub(SPEC, extra=_idle, args=args)
