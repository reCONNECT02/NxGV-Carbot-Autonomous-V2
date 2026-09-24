#!/usr/bin/env python3
"""UWB (x, y) debug viewer -- Haffiz's method (solver + CV Kalman filter).

Same code as the car (uwb_localization.positioning), anchors from uwb.yaml (the
ACTIVE calibration session's copy if step 10 saved one), settings from
common.yaml uwb_positioning. Prints raw solve and filtered position in METRES
and publishes the filtered one on /uwb/position (PointStamped, frame venue).

The stack already publishes the same thing on /carbot/uwb/position while
calibrate/race.launch.py runs; use this viewer only when the stack is NOT running.

Run (on the RDK):
  source /opt/ros/humble/setup.bash && source ~/carbot_ws/install/setup.bash
  export ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=1
  python3 tools/uwb/uwb_xy.py [--uwb-yaml path]

The pre-Haffiz pairwise viewer: set uwb_positioning.solver: pairwise, filter: none.
"""
import argparse
import os
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common  # noqa: E402
from uwb_localization.positioning import Positioner, report_ranges  # noqa: E402
from uwb_localization.uwb_core import RangeProcessor, parse_report  # noqa: E402


class UwbXY(Node):
    def __init__(self, override: str):
        super().__init__('uwb_xy_viewer')
        _, self.anchors, self.cfg, path = _common.load(override)
        self.proc = RangeProcessor(self.anchors, 400, 0.05, 30.0, 'arrival')
        self.pos = Positioner(self.anchors, self.cfg)
        print(f'anchors {path}: ' + ', '.join(f'{a} ({v.x:.2f}, {v.y:.2f})' for a, v in self.anchors.anchors.items()))
        print(f'method: {self.cfg.solver} + {self.cfg.filter}')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)          # BEST_EFFORT: the tag's QoS
        self.create_subscription(String, '/uwb3/input_json', self.on_msg, qos)
        self.pub = self.create_publisher(PointStamped, '/uwb/position', 10)

    def on_msg(self, msg):
        rep = parse_report(msg.data)
        if rep is None:
            return
        res = self.proc.process(rep, self.get_clock().now().nanoseconds * 1e-9)
        if res.rebooted:
            self.pos.reset()
        rng, t, _ = report_ranges(res, self.cfg.skip_repeat_reports)
        f = self.pos.update(rng, t) if rng else None
        if f is None:
            return
        gate = '' if f.accepted else '  (gated)'
        print(f'filtered ({f.xy[0]:6.2f}, {f.xy[1]:6.2f}) m   raw ({f.raw[0]:6.2f}, {f.raw[1]:6.2f}){gate}', flush=True)
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'venue'
        out.point.x, out.point.y = float(f.xy[0]), float(f.xy[1])
        self.pub.publish(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--uwb-yaml', default='')
    a = ap.parse_args()
    rclpy.init()
    node = UwbXY(a.uwb_yaml)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
