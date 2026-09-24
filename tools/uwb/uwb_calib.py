#!/usr/bin/env python3
"""Per-anchor range offset check at a known spot (debug; the wizard's step 10 does
this and saves it). With Haffiz's method offsets are OPTIONAL: run this only when
the position is off by a similar amount everywhere.

  python3 tools/uwb/uwb_calib.py 2.0 3.0        # true tag x y in METRES (uwb.yaml frame)
  python3 tools/uwb/uwb_calib.py 2.0 3.0 30     # collect 30 s (default 20)

Prints median measured vs true 3-D distance per anchor and the range_offset_m
values for uwb.yaml (or step 10 Measure offsets). Anchors / heights from uwb.yaml.
"""
import os
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common  # noqa: E402
from uwb_localization.uwb_core import parse_report  # noqa: E402


class Calib(Node):
    def __init__(self, anchors):
        super().__init__('uwb_calib')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(String, '/uwb3/input_json', self.on_msg, qos)
        self.samples = {aid: [] for aid in anchors.ids}
        self.last = {}

    def on_msg(self, msg):
        rep = parse_report(msg.data)
        if rep is None:
            return
        for ln in rep.links:
            if ln.anchor in self.samples and self.last.get(ln.anchor) != ln.sample_seq:
                self.last[ln.anchor] = ln.sample_seq
                self.samples[ln.anchor].append(ln.r)            # raw, metres, new samples only


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    tx, ty = float(sys.argv[1]), float(sys.argv[2])
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    _, anchors, _, path = _common.load()
    rclpy.init()
    node = Calib(anchors)
    print(f'anchors from {path}. Tag must stay still at ({tx:.2f}, {ty:.2f}) m. Collecting {seconds:.0f} s...')
    end = time.time() + seconds
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    print(f"\n{'anchor':>6} {'samples':>7} {'measured':>9} {'true':>7} {'offset':>7} {'spread':>7}   (metres)")
    for aid in anchors.ids:
        s = node.samples[aid]
        true = anchors.true_range_3d(aid, tx, ty)
        if len(s) < 20:
            print(f'{aid:>6} {len(s):>7}   too few samples -- is this anchor on?')
            continue
        meas = statistics.median(s)
        print(f'{aid:>6} {len(s):>7} {meas:9.3f} {true:7.3f} {meas - true:+7.3f} {statistics.pstdev(s):7.3f}')
    print('\noffset = range_offset_m in uwb.yaml (step 10 writes it). |offset| < ~0.05 m: leave them at 0.')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
