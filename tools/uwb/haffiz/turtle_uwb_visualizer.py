#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import numpy as np
import turtle
import threading
import queue
import time

# ================================
# ANCHOR POSITIONS (In Meters)
# ================================
ANCHORS = {
    "1786": (-2.5, 0.0),   # Anchor 1786
    "1782": (6.0, -0.6),   # Anchor 1782
    "1783": (2.5, 8.0)     # Anchor 1783
}

position_queue = queue.Queue()


class ExtendedKalmanFilter2D:
    """ 
    2D Extended Kalman Filter with Constant Velocity Motion Model.
    State Vector: [x, y, vx, vy]^T
    """
    def __init__(self, process_noise=0.1, measurement_noise=0.3):
        # Initial State: [x, y, vx, vy]
        self.x = np.zeros((4, 1))
        self.is_initialized = False
        self.last_time = None

        # Covariance Matrix P
        self.P = np.eye(4) * 1.0

        # Process Noise Covariance Q
        self.q_var = process_noise

        # Measurement Noise Covariance R (UWB range accuracy variance)
        self.R = np.eye(2) * (measurement_noise ** 2)

        # Measurement Matrix H (maps state [x,y,vx,vy] to measurement [x,y])
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

    def initialize(self, init_x, init_y):
        self.x = np.array([[init_x], [init_y], [0.0], [0.0]])
        self.last_time = time.time()
        self.is_initialized = True

    def predict(self, dt):
        """ Prediction step using kinematic equations x = x + v*dt """
        if dt <= 0:
            return

        # State Transition Matrix F
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1]
        ])

        # Discrete Process Noise Q
        Q = np.array([
            [0.25*dt**4, 0,           0.5*dt**3,  0],
            [0,          0.25*dt**4,  0,          0.5*dt**3],
            [0.5*dt**3,  0,           dt**2,      0],
            [0,          0.5*dt**3,   0,          dt**2]
        ]) * self.q_var

        # Predict State and Covariance
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z_x, z_y):
        """ Update step with incoming UWB measurement and Innovation Gating """
        z = np.array([[z_x], [z_y]])
        
        # Innovation (Residual) y = z - H*x
        y = z - (self.H @ self.x)

        # Innovation Covariance S = H*P*H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # Mahalanobis Distance Gating for Outlier Rejection
        mahalanobis_dist = float(y.T @ np.linalg.inv(S) @ y)
        if mahalanobis_dist > 16.0:  # Gating threshold (~4 sigma rejection)
            # Reject anomalous measurement, trust prediction
            return float(self.x[0, 0]), float(self.x[1, 0])

        # Kalman Gain K = P*H^T*S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update State and Covariance
        self.x = self.x + (K @ y)
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

        return float(self.x[0, 0]), float(self.x[1, 0])

    def process(self, raw_x, raw_y):
        current_time = time.time()

        if not self.is_initialized:
            self.initialize(raw_x, raw_y)
            return raw_x, raw_y

        dt = current_time - self.last_time
        self.last_time = current_time

        # Execute EKF predict-update cycle
        self.predict(dt)
        filtered_x, filtered_y = self.update(raw_x, raw_y)

        return filtered_x, filtered_y


class UWBROSNode(Node):
    def __init__(self):
        super().__init__('turtle_uwb_ekf_subscriber')

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

        # EKF instance: process_noise=0.2 (smoothness), measurement_noise=0.25 (sensor confidence)
        self.ekf = ExtendedKalmanFilter2D(process_noise=0.2, measurement_noise=0.25)

    def solve_trilateration(self, ranges):
        try:
            x1, y1 = ANCHORS["1786"]
            r1 = ranges["1786"]
            x2, y2 = ANCHORS["1782"]
            r2 = ranges["1782"]
            x3, y3 = ANCHORS["1783"]
            r3 = ranges["1783"]

            A = np.array([
                [2 * (x2 - x1), 2 * (y2 - y1)],
                [2 * (x3 - x1), 2 * (y3 - y1)]
            ])
            B = np.array([
                r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2,
                r1**2 - r3**2 - x1**2 + x3**2 - y1**2 + y3**2
            ])
            pos = np.linalg.solve(A, B)
            return float(pos[0]), float(pos[1])
        except Exception:
            return None, None

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
                raw_x, raw_y = self.solve_trilateration(ranges)
                
                if raw_x is not None and raw_y is not None:
                    # Pass through Extended Kalman Filter
                    ekf_x, ekf_y = self.ekf.process(raw_x, raw_y)

                    position_queue.put((ekf_x, ekf_y))
                    self.get_logger().info(f"EKF Pose -> X: {ekf_x:.2f} m, Y: {ekf_y:.2f} m | Raw -> X: {raw_x:.2f}, Y: {raw_y:.2f}")

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

    # Grid axes
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

    # Anchors
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
    screen.title("EKF Filtered UWB 2D Tracking")
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