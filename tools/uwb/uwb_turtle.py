#!/usr/bin/env python3
"""Haffiz's turtle viewer (tools/uwb/haffiz/turtle_uwb_visualizer.py), wired to the stack:
anchors from uwb.yaml (ACTIVE session copy first), solver + filter from common.yaml
uwb_positioning -- the same position the car publishes on /carbot/uwb/position.
Needs a display (run on a laptop on the same WiFi / ROS domain, not over plain SSH).

  export ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=1
  python3 tools/uwb/uwb_turtle.py [--uwb-yaml path] [--raw]    # --raw also draws the unfiltered solve
"""
import argparse
import os
import queue
import sys
import threading
import turtle

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common  # noqa: E402
from uwb_localization.positioning import Positioner, report_ranges  # noqa: E402
from uwb_localization.uwb_core import RangeProcessor, parse_report  # noqa: E402

q: 'queue.Queue' = queue.Queue()


class TurtleNode(Node):
    def __init__(self, anchors, cfg):
        super().__init__('turtle_uwb_subscriber')
        self.proc = RangeProcessor(anchors, 400, 0.05, 30.0, 'arrival')
        self.pos, self.cfg = Positioner(anchors, cfg), cfg
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(String, '/uwb3/input_json', self.cb, qos)

    def cb(self, msg):
        rep = parse_report(msg.data)
        if rep is None:
            return
        res = self.proc.process(rep, self.get_clock().now().nanoseconds * 1e-9)
        if res.rebooted:
            self.pos.reset()
        rng, t, _ = report_ranges(res, self.cfg.skip_repeat_reports)
        f = self.pos.update(rng, t) if rng else None
        if f is not None:
            q.put(f)
            self.get_logger().info(f'Pose -> X: {f.xy[0]:.2f} m, Y: {f.xy[1]:.2f} m | Raw -> X: {f.raw[0]:.2f}, '
                                   f'Y: {f.raw[1]:.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uwb-yaml', default='')
    ap.add_argument('--raw', action='store_true')
    a = ap.parse_args()
    _, anchors, cfg, path = _common.load(a.uwb_yaml)
    rclpy.init()
    node = TurtleNode(anchors, cfg)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    screen = turtle.Screen()
    screen.setup(width=900, height=900)
    screen.title(f'UWB {cfg.solver} + {cfg.filter}  ({path})')
    screen.bgcolor('black')
    xs = [v.x for v in anchors.anchors.values()]
    ys = [v.y for v in anchors.anchors.values()]
    m = 2.5
    screen.setworldcoordinates(min(xs) - m, min(ys) - m, max(xs) + m, max(ys) + m)
    dr = turtle.Turtle()
    dr.hideturtle()
    dr.speed(0)
    dr.color('#333333')
    for p0, p1 in (((min(xs) - m, 0), (max(xs) + m, 0)), ((0, min(ys) - m), (0, max(ys) + m))):
        dr.penup()
        dr.goto(*p0)
        dr.pendown()
        dr.goto(*p1)
    for aid, v in anchors.anchors.items():
        dr.penup()
        dr.goto(v.x, v.y)
        dr.dot(14, 'red')
        dr.color('white')
        dr.goto(v.x, v.y + 0.3)
        dr.write(f' Anchor {aid} ({v.x:.1f}, {v.y:.1f}m)', font=('Arial', 10, 'bold'))
    tag = turtle.Turtle()
    tag.shape('circle')
    tag.color('cyan')
    tag.shapesize(0.6, 0.6)
    tag.penup()
    tag.speed(0)
    tag.pensize(2)
    raw = turtle.Turtle()
    raw.hideturtle()
    raw.penup()
    raw.speed(0)

    def tick():
        while not q.empty():
            f = q.get()
            tag.pendown()
            tag.goto(*f.xy)
            if a.raw:
                raw.goto(*f.raw)
                raw.dot(3, 'orange')
        screen.ontimer(tick, 30)
    tick()
    turtle.mainloop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
