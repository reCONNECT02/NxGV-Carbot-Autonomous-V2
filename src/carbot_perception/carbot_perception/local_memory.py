"""BLOCK 04 - Remember what just disappeared (V4 vehicle.js LocalMemory).

Road/paint cells from block 03 are stored in the fixed ODOM frame (wheel + IMU
dead reckoning, base servo_controller /odom) with their time and the travelled
distance at observation. The pose used for each grid is the odom pose
interpolated at the grid's camera timestamp (V4 capture(): cameraOdom).

Why /odom and not the block-05 local pose: memory must live in a frame that
never gets corrected. The local pose receives camera-to-map corrections; a
cell stored in that frame would shift every time a correction lands (V4 uses
estimate.odom for exactly this reason).

Out: LocalGrid /carbot/memory/grid, frame odom, an axis-aligned window of
window_m around the car, cells up to display_max_age_s old, with
  age_s[i]          seconds since the cell was last seen (-1 = empty)
  travel_since_m[i] distance driven since then (for the V4 uncertainty rule
                    base + per_m * travel <= limit, applied by the consumer)
Consumers apply their own age limit (drive 3 s, corridor 2 s, parking 24 s).
LiDAR is NOT stored here.
"""
import math
import time

import numpy as np
import rclpy
from carbot_common import topics as T
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED, SENSOR
from carbot_interfaces.msg import LocalGrid, MissionState, NodeStatus
from nav_msgs.msg import Odometry

from .memory_core import LocalMemory, OdomHistory
from .road_mask import GridSpec
from .ros_image import f32, u8

REQUIRED = ['resolution_m', 'drive_max_age_s', 'corridor_max_age_s', 'parking_max_age_s',
            'display_max_age_s', 'max_cells', 'uncertainty_base_m', 'uncertainty_per_m',
            'uncertainty_limit_m', 'publish_rate_hz', 'window_m', 'ring_cells',
            'odom_history_s', 'max_stamp_gap_s', 'frames.odom']


def stamp_s(st) -> float:
    return st.sec + st.nanosec * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocalMemoryNode(CarbotNode):

    def __init__(self):
        super().__init__('local_memory', '04', REQUIRED)
        self.mem = LocalMemory(float(self.p('resolution_m')), int(self.p('ring_cells')),
                               int(self.p('max_cells')), float(self.p('display_max_age_s')))
        self.odom = OdomHistory(float(self.p('odom_history_s')))
        self.odom_frame = str(self.p('frames.odom'))
        self.grid_cache = None            # (rows, cols, res, x0, y0) -> centre arrays
        self.skipped = 0
        self.integrated = 0
        self.last_int_ms = 0.0
        self.sub(Odometry, T.ODOM, self._on_odom, SENSOR)
        self.sub(LocalGrid, T.ROAD_GRID, self._on_grid, 10)
        self.sub(MissionState, T.MISSION_STATE, None, LATCHED)   # kept for later phases
        self.pub = self.create_publisher(LocalGrid, T.MEMORY_GRID, 1)
        self.create_timer(1.0 / max(float(self.p('publish_rate_hz')), 0.5), self._publish)
        self.set_status(NodeStatus.WARN, 'WAITING_INPUT', 'waiting for /odom and road grid')

    def _on_odom(self, msg: Odometry) -> None:
        if msg.header.frame_id:
            self.odom_frame = msg.header.frame_id
        p = msg.pose.pose
        self.odom.add(stamp_s(msg.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation))

    def _centres(self, g: LocalGrid):
        key = (g.rows, g.cols, round(g.resolution_m, 6), round(g.origin_x_m, 6), round(g.origin_y_m, 6))
        if self.grid_cache is None or self.grid_cache[0] != key:
            if g.rows != g.cols:
                raise ValueError('road grid must be square')
            gx, gy = GridSpec(g.rows, g.resolution_m, g.origin_x_m, g.origin_y_m).centres()
            self.grid_cache = (key, gx.ravel(), gy.ravel())
        return self.grid_cache[1], self.grid_cache[2]

    def _on_grid(self, g: LocalGrid) -> None:
        t = stamp_s(g.header.stamp)
        pose = self.odom.at(t, float(self.p('max_stamp_gap_s')))
        if pose is None:
            self.skipped += 1
            return
        t0 = time.perf_counter()
        lx, ly = self._centres(g)
        self.mem.integrate(np.frombuffer(bytes(g.kind), np.uint8), lx, ly, pose, t)
        self.integrated += 1
        self.last_int_ms = (time.perf_counter() - t0) * 1000.0

    def _publish(self) -> None:
        t_last, pose = self.odom.latest()
        if pose is None:
            self.set_status(NodeStatus.WARN, 'WAITING_INPUT', 'no /odom yet')
            return
        t = max(t_last, self.mem.last)
        x0, y0, n, kind, age, dist = self.mem.window(
            pose.x, pose.y, float(self.p('window_m')) / 2.0, t, float(self.p('display_max_age_s')))
        m = LocalGrid()
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = int((t - int(t)) * 1e9)
        m.header.frame_id = self.odom_frame
        m.resolution_m = float(self.mem.res)
        m.origin_x_m, m.origin_y_m = float(x0), float(y0)
        m.rows = m.cols = int(n)
        m.kind = u8(kind)
        m.grown = u8(np.zeros_like(kind))
        m.age_s = f32(age)
        m.travel_since_m = f32(np.where(age >= 0, pose.distance - dist, -1.0))
        m.coverage = float((kind > 0).mean())
        m.connected = int((kind == 1).sum())
        self.pub.publish(m)
        level = NodeStatus.OK if self.integrated else NodeStatus.WARN
        self.set_status(level, 'RUNNING',
                        f'cells {self.mem.size(t)} fresh {self.mem.fresh} '
                        f'integrated {self.integrated} skipped(no odom at stamp) {self.skipped} '
                        f'{self.last_int_ms:.1f}ms travelled {pose.distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = LocalMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
