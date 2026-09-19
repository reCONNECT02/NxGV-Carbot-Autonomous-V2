#!/usr/bin/env python3
"""UWB (x, y) viewer -- pairwise method.

For each PAIR of anchors, the tag's two distances give two circles. Their
intersection is the tag position (two mirror-image candidates; the third
anchor's distance picks the right one). 3 anchors -> 3 pairs -> 3 estimates,
averaged into one (x, y). Prints only "(x, y)" in cm.

Frame: origin = anchor 1782, +x towards 1786, +y from 1786 towards 1783.

Run:
  export ROS_LOCALHOST_ONLY=0
  export ROS_DOMAIN_ID=1
  source /opt/ros/humble/setup.bash
  python3 ~/uwb_xy.py
"""
import json
import math
from itertools import combinations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

# ---------------------------------------------------------------- [TUNE] ----
# Anchor antenna positions in CENTIMETRES: (x, y, z). z = antenna height.
ANCHORS = {
    "1786": (750.0, 0.0, 0.0),
    "1782": (0.0, 0.0, 0.0),
    "1783": (750.0, 483.0, 0.0),
}
TAG_Z_CM = 0.0          # tag antenna height (only the difference to anchor z matters)
# Per-anchor distance bias in cm (measured - true), subtracted from every reading.
# Get these values from uwb_calib.py.
RANGE_OFFSET_CM = {
    "1786": 0.0,
    "1782": 0.0,
    "1783": 0.0,
}
# ----------------------------------------------------------------------------


def pair_estimate(a, ra, b, rb, c, rc):
    """Intersect circle(a, ra) with circle(b, rb); use anchor c to pick the side.

    a, b, c are (x, y); ra, rb, rc are floor-plane distances in cm.
    Returns (x, y) or None if a and b are at the same spot.
    """
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return None
    ux, uy = dx / d, dy / d           # unit vector a -> b
    nx, ny = -uy, ux                  # unit vector perpendicular to a -> b

    # Distance from a, along a -> b, to the line joining the two intersections.
    along = (ra * ra - rb * rb + d * d) / (2.0 * d)
    h2 = ra * ra - along * along
    # h2 < 0 means the circles don't meet (measurement noise): use the
    # closest point on the a-b line instead of giving up.
    h = math.sqrt(h2) if h2 > 0.0 else 0.0

    mx, my = a[0] + along * ux, a[1] + along * uy
    p1 = (mx + h * nx, my + h * ny)
    p2 = (mx - h * nx, my - h * ny)

    # Keep the candidate whose distance to the third anchor matches best.
    e1 = abs(math.hypot(p1[0] - c[0], p1[1] - c[1]) - rc)
    e2 = abs(math.hypot(p2[0] - c[0], p2[1] - c[1]) - rc)
    return p1 if e1 <= e2 else p2


class UwbXY(Node):
    def __init__(self):
        super().__init__("uwb_xy_viewer")
        # BEST_EFFORT to match the tag's publisher.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(String, "/uwb3/input_json", self.on_msg, qos)
        self.pub = self.create_publisher(PointStamped, "/uwb/position", 10)

    def on_msg(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return

        # Tag-to-anchor distances, metres -> cm, flattened to the floor plane.
        r = {}
        for link in data.get("links", []):
            aid = link.get("A")
            if aid not in ANCHORS:
                continue
            dist = float(link["R"]) * 100.0 - RANGE_OFFSET_CM.get(aid, 0.0)
            dz = ANCHORS[aid][2] - TAG_Z_CM
            h2 = dist * dist - dz * dz
            r[aid] = math.sqrt(h2) if h2 > 0.0 else 0.0
        if len(r) < 3:
            return                      # need all 3 anchors in this message

        ids = sorted(r)[:3]
        estimates = []
        for ia, ib in combinations(ids, 2):
            ic = next(i for i in ids if i not in (ia, ib))
            p = pair_estimate(ANCHORS[ia][:2], r[ia], ANCHORS[ib][:2], r[ib],
                              ANCHORS[ic][:2], r[ic])
            if p is not None:
                estimates.append(p)
        if not estimates:
            return

        x = sum(p[0] for p in estimates) / len(estimates)
        y = sum(p[1] for p in estimates) / len(estimates)
        print(f"({x:.1f}, {y:.1f})", flush=True)

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "venue"
        out.point.x = x / 100.0
        out.point.y = y / 100.0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = UwbXY()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
