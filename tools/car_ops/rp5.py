import sys, time, rclpy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node('rp5'); got = []
n.create_subscription(String, '/record_playback_state', lambda m: got.append(m.data), 10)
pub = n.create_publisher(String, '/record_playback_cmd', 10)
time.sleep(1.5)
def ask(cmd):
    got.clear(); pub.publish(String(data=cmd)); t0 = time.time()
    while not got and time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.2)
    return got[-1] if got else 'no reply'
print('after list :', ask('list')[:200])
if len(sys.argv) > 1 and sys.argv[1] == 'stop':
    print('after stop :', ask('stop')[:200])
    print('after list :', ask('list')[:200])
