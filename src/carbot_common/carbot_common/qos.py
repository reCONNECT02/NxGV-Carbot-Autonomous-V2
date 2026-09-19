"""QoS profiles used across the stack."""
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)

SENSOR = qos_profile_sensor_data

# micro-ROS tag publishes BEST_EFFORT: a RELIABLE subscriber receives NOTHING.
UWB = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=1,
                 durability=DurabilityPolicy.VOLATILE)

# Map, route and mission state: late joiners (GUI, rosbag) get the last value.
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Control-loop messages: newest wins.
CONTROL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

DEFAULT = 10
