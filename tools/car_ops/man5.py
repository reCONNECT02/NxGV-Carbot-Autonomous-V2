import sys, time, rclpy
from std_msgs.msg import Bool
from carbot_common import topics as T
from carbot_common.qos import LATCHED
rclpy.init(); n = rclpy.create_node('man5'); got = []
n.create_subscription(Bool, T.MANUAL_TAKEOVER, lambda m: got.append(m.data), LATCHED)
t0 = time.time()
while not got and time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.2)
print('MANUAL_TAKEOVER topic', T.MANUAL_TAKEOVER, '=', got[-1] if got else 'no message')
if len(sys.argv) > 1 and sys.argv[1] == 'release':
    pub = n.create_publisher(Bool, T.MANUAL_TAKEOVER, LATCHED)
    time.sleep(1.0); pub.publish(Bool(data=False)); time.sleep(1.0)
    print('published False (same as the GUI Manual control button off)')
