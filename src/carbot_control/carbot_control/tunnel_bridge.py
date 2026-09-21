"""Tunnel wrapper (Challenge 3). Base tunnel code is used UNCHANGED.

The base risabot_automode/tunnel_wall_follower node runs as-is and keeps
publishing its LiDAR trigger (/tunnel_detected) and LiDAR lane-tracking
command (/tunnel_cmd_vel). This bridge only:
  * forwards /tunnel_cmd_vel as MotionRequest(source=TUNNEL) on
    /carbot/request/tunnel, and only while mission logic has TUNNEL active
    (MissionState.active_source == TUNNEL);
  * converts the base normalized angular.z to steer_rad with the exact
    inverse of command_owner.steering (owner_core.SteeringMap.to_steer), so the
    servo receives the same value it would in the base stack;
  * sets speed from the tunnel speed zone (speed_source: zone, through the
    owner's speed PID) or converts the base duty back to m/s with the inverse
    feedforward (speed_source: base_duty, approximate). A zero base speed
    (no centreline) is passed on as a stop.
Mission logic uses /tunnel_detected (LiDAR trigger) to activate TUNNEL; the
bridge itself never decides the mode.
"""
import time

import rclpy
from carbot_common import topics as T
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED
from carbot_interfaces.msg import MissionState, MotionRequest, NodeStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from .owner_core import SteeringMap, base_duty_to_mps

REQUIRED = ['cmd_max_age_s', 'speed_source', 'zone_speed_mps', 'rate_hz', 'use_zone_cap',
            'command_owner_steering.steer_sign', 'command_owner_steering.left_max_rad',
            'command_owner_steering.right_max_rad', 'command_owner_steering.trim_rad',
            'command_owner_steering.angular_limit', 'command_owner_feedforward.duty_per_mps',
            'command_owner_feedforward.static_duty']


class TunnelBridge(CarbotNode):

    def __init__(self):
        super().__init__('tunnel_bridge', '13', REQUIRED)
        p = lambda k: self.p('command_owner_steering.' + k)  # noqa: E731
        self.steering = SteeringMap(float(p('steer_sign')), float(p('left_max_rad')), float(p('right_max_rad')),
                                    float(p('trim_rad')), float(p('angular_limit')))
        self.cmd, self.cmd_t = None, -1e9
        self.detected = False
        self.mission = None
        self.pub = self.create_publisher(MotionRequest, T.request_topic('TUNNEL'), 10)
        self.sub(Twist, T.TUNNEL_CMD, self._on_cmd, 10)
        self.sub(Bool, T.TUNNEL_DETECTED, lambda m: setattr(self, 'detected', bool(m.data)), 10)
        self.sub(MissionState, T.MISSION_STATE, lambda m: setattr(self, 'mission', m), LATCHED)
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 1.0), self._tick)
        self.set_status(NodeStatus.OK, 'IDLE', 'waiting for mission TUNNEL')

    def _on_cmd(self, m: Twist):
        self.cmd, self.cmd_t = m, time.monotonic()

    def request_for(self, cmd: Twist, zone_cap: float) -> MotionRequest:
        m = MotionRequest()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source = 'TUNNEL'
        m.steer_rad = float(self.steering.to_steer(cmd.angular.z))
        if cmd.linear.x <= 0.0:
            m.speed_mps, m.reason = 0.0, 'base tunnel follower: no centreline (speed 0)'
        elif str(self.p('speed_source')) == 'base_duty':
            m.speed_mps = base_duty_to_mps(cmd.linear.x, float(self.p('command_owner_feedforward.duty_per_mps')),
                                           float(self.p('command_owner_feedforward.static_duty')))
            m.reason = f'base tunnel follower (duty {cmd.linear.x:.2f})'
        else:
            v = float(self.p('zone_speed_mps'))
            if bool(self.p('use_zone_cap')) and zone_cap > 0:
                v = min(v, zone_cap)
            m.speed_mps, m.reason = v, 'base tunnel follower (zone speed)'
        return m

    def _tick(self) -> None:
        ms = self.mission
        active = ms is not None and ms.active_source == 'TUNNEL'
        if not active:
            self.set_status(NodeStatus.OK, 'IDLE', f'trigger {self.detected}')
            return
        age = time.monotonic() - self.cmd_t
        if self.cmd is None or age > float(self.p('cmd_max_age_s')):
            self.set_status(NodeStatus.WARN, 'STALE', f'/tunnel_cmd_vel {age:.2f} s old: not forwarded')
            return                      # no request -> the owner's watchdog stops the car
        m = self.request_for(self.cmd, float(ms.zone_max_speed_mps))
        self.pub.publish(m)
        self.set_status(NodeStatus.OK, 'FORWARDING', f'v {m.speed_mps:.3f} steer {m.steer_rad:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = TunnelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
