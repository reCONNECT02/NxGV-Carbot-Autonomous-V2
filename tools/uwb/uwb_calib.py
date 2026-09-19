#!/usr/bin/env python3
"""Per-anchor range offset calibration for uwb_xy.py.

Put the tag at a KNOWN spot, then run:
  python3 ~/uwb_calib.py 500 150        # true x y of the tag in cm
  python3 ~/uwb_calib.py 500 150 30     # optional: collect for 30 s (default 20)

It averages the raw distance to each anchor, compares it with the true
distance from the tape-measured positions, and prints a RANGE_OFFSET_CM
block to paste into uwb_xy.py.

Needs uwb_xy.py in the same folder (it reads ANCHORS and TAG_Z_CM from it).
"""
import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

from uwb_xy import ANCHORS, TAG_Z_CM


class Calib(Node):
    def __init__(self):
        super().__init__("uwb_calib")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(String, "/uwb3/input_json", self.on_msg, qos)
        self.samples = {aid: [] for aid in ANCHORS}

    def on_msg(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        for link in data.get("links", []):
            aid = link.get("A")
            if aid in self.samples:
                self.samples[aid].append(float(link["R"]) * 100.0)  # raw, cm


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    tx, ty = float(sys.argv[1]), float(sys.argv[2])
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

    rclpy.init()
    node = Calib()
    print(f"Tag must stay still at ({tx:.0f}, {ty:.0f}) cm. Collecting for {seconds:.0f} s...")
    end = time.time() + seconds
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    print()
    print(f"{'anchor':>6} {'samples':>7} {'measured':>9} {'true':>7} {'offset':>7} {'spread':>7}")
    offsets = {}
    for aid, (ax, ay, az) in ANCHORS.items():
        s = node.samples[aid]
        true = math.sqrt((tx - ax) ** 2 + (ty - ay) ** 2 + (TAG_Z_CM - az) ** 2)
        if len(s) < 20:
            print(f"{aid:>6} {len(s):>7}   too few samples -- is this anchor on?")
            continue
        meas = statistics.median(s)
        spread = statistics.pstdev(s)
        offsets[aid] = meas - true
        print(f"{aid:>6} {len(s):>7} {meas:9.1f} {true:7.1f} {offsets[aid]:7.1f} {spread:7.1f}")

    if len(offsets) == len(ANCHORS):
        print("\nPaste this into uwb_xy.py, replacing the RANGE_OFFSET_CM block:\n")
        print("RANGE_OFFSET_CM = {")
        for aid in ANCHORS:
            print(f'    "{aid}": {offsets[aid]:.1f},')
        print("}")

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
