"""A static base_link -> child transform read from /tf_static ONLY (BACKLOG #49).

tf2_ros.TransformListener also subscribes to /tf (31 Hz on the RDK) and turns every message into
Python objects; safety_monitor, local_planner and recovery_planner only ever wanted the fixed
base_link -> laser_frame mount, so each of them paid for the whole /tf stream forever. /tf_static is
latched (one message per static publisher, then silence), so this costs nothing after start-up.
Direct edges only: the mount is published as base_link -> laser_frame by carbot_bringup/stack.py.
"""
import math
from typing import Iterable, Optional, Tuple


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def find_static(transforms: Iterable, base: str, child: str) -> Optional[Tuple[float, float, float]]:
    """(x, y, yaw) of the transform base -> child among TransformStamped messages, else None."""
    for t in transforms:
        if t.header.frame_id.lstrip('/') == base and t.child_frame_id.lstrip('/') == child:
            return (float(t.transform.translation.x), float(t.transform.translation.y), yaw_of(t.transform.rotation))
    return None


class StaticMount:
    """Subscribes /tf_static (latched) and remembers base -> child. get() is None until it arrives."""

    def __init__(self, node, base: str, child: str):
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from tf2_msgs.msg import TFMessage
        self.base, self.child = base, child
        self._mount = None
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=100,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)     # what tf2_ros uses for /tf_static
        self._sub = node.create_subscription(TFMessage, '/tf_static', self._on, qos)

    def _on(self, msg) -> None:
        found = find_static(msg.transforms, self.base, self.child)
        if found is not None:
            self._mount = found

    def get(self) -> Optional[Tuple[float, float, float]]:
        return self._mount
