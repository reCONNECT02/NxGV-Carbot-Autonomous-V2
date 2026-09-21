"""ROS helper for calibration steps 7/8: drive THROUGH the command owner and record.

Requests go to /carbot/calibration/request (MotionRequest, source CALIBRATION_RAW
= base duty + angular.z, or CALIBRATION = m/s + rad through the owner's PID and
steering map). The owner accepts them only in calibrate mode with the race not
armed, arms the base servo_controller while they are fresh, and disarms it
(joystick back) when they stop. The e-stop always wins.
"""
import json
import math
import threading
import time
from typing import Callable, Dict, List

from carbot_common import topics as T


class Driver:

    def __init__(self, rclpy, name: str):
        from carbot_interfaces.msg import MotionRequest
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Bool, String
        self.MotionRequest = MotionRequest
        self.node = rclpy.create_node(name)
        # parameter services get their own node: RemoteParams spins it itself, while the
        # recorder thread below spins self.node (one node must not be spun by two threads)
        self.param_node = rclpy.create_node(name + '_params')
        self.pub = self.node.create_publisher(MotionRequest, T.CALIBRATION_REQUEST, 10)
        self.lock = threading.Lock()
        self.cmd = None                 # (source, speed, steer, reason)
        self.prev_odom = None
        self.dist = 0.0
        self.speed = 0.0
        self.yaw_prev = None
        self.yaw = 0.0
        self.estop = False
        self.rows: List[Dict] = []
        self.recording = False
        self.t0 = time.monotonic()
        self.node.create_subscription(Odometry, T.ODOM, self._odom, qos_profile_sensor_data)
        self.node.create_subscription(String, T.IMU_RPY, self._imu, qos_profile_sensor_data)
        self.node.create_subscription(Bool, T.E_STOP, self._estop, 10)
        self.node.create_timer(0.05, self._publish)
        self._stop = False
        self.th = threading.Thread(target=self._spin, args=(rclpy,), daemon=True)
        self.th.start()

    def _spin(self, rclpy):
        while not self._stop and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _odom(self, m):
        p = m.pose.pose.position
        with self.lock:
            if self.prev_odom is not None:
                ds = math.hypot(p.x - self.prev_odom[0], p.y - self.prev_odom[1])
                self.dist += ds if m.twist.twist.linear.x >= 0 else -ds
            self.prev_odom = (p.x, p.y)
            self.speed = float(m.twist.twist.linear.x)
            if self.recording:
                self.rows.append({'t': time.monotonic() - self.t0, 'dist': self.dist, 'yaw': self.yaw,
                                  'v': self.speed, 'cmd': list(self.cmd or ('', 0.0, 0.0, ''))[:3]})

    def _imu(self, m):
        try:
            y = math.radians(float(json.loads(m.data)['yaw']))
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            if self.yaw_prev is not None:
                self.yaw += math.atan2(math.sin(y - self.yaw_prev), math.cos(y - self.yaw_prev))
            self.yaw_prev = y

    def _estop(self, m):
        if m.data:
            self.estop = True

    def _publish(self):
        c = self.cmd
        if c is None:
            return
        r = self.MotionRequest()
        r.header.stamp = self.node.get_clock().now().to_msg()
        r.source, r.speed_mps, r.steer_rad, r.reason = c[0], float(c[1]), float(c[2]), c[3]
        self.pub.publish(r)

    def read(self):
        with self.lock:
            return self.dist, self.yaw, self.speed

    def run(self, source: str, speed: float, steer: float, reason: str,
            until: Callable[[float, float, float], bool], timeout_s: float, settle_s: float = 0.8) -> Dict:
        """Drive until until(distance, yaw, elapsed) is true (or timeout / e-stop), then
        stop and settle. -> {distance, yaw, rows, aborted}."""
        d0, y0, _ = self.read()
        with self.lock:
            self.rows, self.recording = [], True
        t0 = time.monotonic()
        self.cmd = (source, speed, steer, reason)
        aborted = ''
        while True:
            time.sleep(0.02)
            d, y, _ = self.read()
            el = time.monotonic() - t0
            if self.estop:
                aborted = 'e-stop'
                break
            if until(d - d0, y - y0, el):
                break
            if el > timeout_s:
                aborted = 'timeout'
                break
        self.cmd = (source, 0.0, steer if source == 'CALIBRATION_RAW' else 0.0, 'stop')
        t1 = time.monotonic()
        while time.monotonic() - t1 < settle_s:
            time.sleep(0.02)
        self.cmd = None
        d, y, _ = self.read()
        with self.lock:
            self.recording = False
            rows = list(self.rows)
        return {'distance': d - d0, 'yaw': y - y0, 'rows': rows, 'aborted': aborted}

    def close(self):
        self.cmd = None
        time.sleep(0.1)
        self._stop = True
        self.th.join(timeout=1.0)
        self.node.destroy_node()
        self.param_node.destroy_node()
