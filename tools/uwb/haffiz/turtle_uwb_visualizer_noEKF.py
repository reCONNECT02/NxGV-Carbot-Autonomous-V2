#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import numpy as np
from scipy.optimize import least_squares
import turtle
import threading
import queue
from collections import deque

# ================================
# ANCHOR POSITIONS (In Meters)
# ================================
ANCHORS = {
    "1786": (-2.5, 0.0),   # Anchor 1786
    "1782": (6.0, -0.6),   # Anchor 1782
    "1783": (2.5, 8.0)     # Anchor 1783
}

position_queue = queue.Queue()


class RobustTrilateration:
    """ Solves 2D location using Non-Linear Least Squares to eliminate geometric jumps """
    def __init__(self):
        self.last_pos = [1.5, 2.5]  # Default initial guess

    def residuals(self, pos, anchor_coords, measured_ranges):
        # Calculate Euclidean distance error for each anchor
        x, y = pos
        res = []
        for (ax, ay), r_meas in zip(anchor_coords, measured_ranges):
            r_calc = np.hypot(x - ax, y - ay)
            res.append(r_calc - r_meas)
        return res

    def solve(self, ranges):
        anchor_coords = []
        measured_ranges = []

        for a_id, (ax, ay) in ANCHORS.items():
            if a_id in ranges:
                anchor_coords.append((ax, ay))
                measured_ranges.append(ranges[a_id])

        if len(anchor_coords) < 3:
            return None, None

        # Solve optimization problem
        res = least_squares(
            self.residuals, 
            self.last_pos, 
            args=(anchor_coords, measured_ranges), 
            loss='soft_l1'  # Robust loss function to suppress outlier ranges
        )

        if res.success:
            self.last_pos = res.x
            return float(res.x[0]), float(res.x[1])
        return None, None


class MovingAverageFilter:
    def __init__(self, window_size=10):
        self.buf_x = deque(maxlen=window_size)
        self.buf_y = deque(maxlen=window_size)

    def filter(self, x, y):
        self.buf_x.append(x)
        self.buf_y.append(y)
        return float(np.mean(self.buf_x)), float(np.mean(self.buf_y))


class UWBROSNode(Node):
    def __init__(self):
        super().__init__('turtle_uwb_subscriber')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            String,
            '/uwb3/input_json',
            self.json_callback,
            qos_profile
        )

        self.solver = RobustTrilateration()
        self.filter = MovingAverageFilter(window_size=10)

    def json_callback(self, msg: String):
        try:
            raw_data = msg.data.strip()
            if raw_data.startswith("data:"):
                raw_data = raw_data.replace("data:", "").strip()
                if raw_data.startswith("'") and raw_data.endswith("'"):
                    raw_data = raw_data[1:-1]

            payload = json.loads(raw_data)
            ranges = {}
            if "links" in payload:
                for link in payload["links"]:
                    ranges[str(link.get("A"))] = float(link.get("R", 0.0))

            if "1786" in ranges and "1782" in ranges and "1783" in ranges:
                raw_x, raw_y = self.solver.solve(ranges)
                
                if raw_x is not None and raw_y is not None:
                    smooth_x, smooth_y = self.filter.filter(raw_x, raw_y)
                    position_queue.put((smooth_x, smooth_y))
                    self.get_logger().info(f"Robust Pose -> X: {smooth_x:.2f} m, Y: {smooth_y:.2f} m")

        except Exception:
            pass


def setup_world_bounds(screen, margin=2.5):
    x_coords = [pos[0] for pos in ANCHORS.values()]
    y_coords = [pos[1] for pos in ANCHORS.values()]

    min_x = min(x_coords) - margin
    max_x = max(x_coords) + margin
    min_y = min(y_coords) - margin
    max_y = max(y_coords) + margin

    screen.setworldcoordinates(min_x, min_y, max_x, max_y)
    return min_x, max_x, min_y, max_y


def draw_anchors(min_x, max_x, min_y, max_y):
    drawer = turtle.Turtle()
    drawer.hideturtle()
    drawer.speed(0)

    drawer.color("#333333")
    drawer.pensize(1)

    drawer.penup()
    drawer.goto(min_x, 0)
    drawer.pendown()
    drawer.goto(max_x, 0)

    drawer.penup()
    drawer.goto(0, min_y)
    drawer.pendown()
    drawer.goto(0, max_y)

    for a_id, (ax, ay) in ANCHORS.items():
        drawer.penup()
        drawer.goto(ax, ay)
        drawer.dot(14, "red")
        drawer.color("white")
        drawer.goto(ax, ay + 0.3)
        drawer.write(f" Anchor {a_id} ({ax:.1f}, {ay:.1f}m)", font=("Arial", 10, "bold"))


def update_turtle_position(screen, tag):
    while not position_queue.empty():
        x, y = position_queue.get()
        tag.pendown()
        tag.goto(x, y)
    
    screen.ontimer(lambda: update_turtle_position(screen, tag), 30)


def main(args=None):
    rclpy.init(args=args)
    node = UWBROSNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    screen = turtle.Screen()
    screen.setup(width=900, height=900)
    screen.title("Robust Least-Squares Filtered UWB Tracking")
    screen.bgcolor("black")

    min_x, max_x, min_y, max_y = setup_world_bounds(screen, margin=2.5)
    draw_anchors(min_x, max_x, min_y, max_y)

    tag = turtle.Turtle()
    tag.shape("circle")
    tag.color("cyan")
    tag.shapesize(stretch_wid=0.6, stretch_len=0.6)
    tag.penup()
    tag.speed(0)
    tag.pensize(2)

    update_turtle_position(screen, tag)
    turtle.mainloop()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()