import time, rclpy
from std_msgs.msg import Float32, Bool
from carbot_interfaces.msg import CommandOwnerState
from carbot_common import topics as T
rclpy.init(); n = rclpy.create_node('batt5'); box = {}
n.create_subscription(Float32, T.VEHICLE_BATTERY, lambda m: box.__setitem__('v', m.data), 10)
n.create_subscription(CommandOwnerState, T.OWNER_STATE, lambda m: box.__setitem__('o', (m.winner, m.reason[:70])), 10)
t0 = time.time()
while time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.2)
print('battery V:', box.get('v', 'no message'), '| owner:', box.get('o'))
