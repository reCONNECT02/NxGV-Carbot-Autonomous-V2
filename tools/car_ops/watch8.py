import time, rclpy
from carbot_interfaces.msg import CommandOwnerState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from carbot_common import topics as T
rclpy.init(); n = rclpy.create_node('watch8'); box = {'own': None, 'cmd': None, 'auto': None, 'odom': None}
n.create_subscription(CommandOwnerState, T.OWNER_STATE, lambda m: box.__setitem__('own', (m.winner, round(m.out_linear_x, 3), round(m.out_angular_z, 3), round(m.request_age_s, 2))), 10)
n.create_subscription(Twist, T.CMD_VEL_AUTO, lambda m: box.__setitem__('auto', (round(m.linear.x, 3), round(m.angular.z, 3))), 10)
n.create_subscription(Twist, T.CMD_VEL, lambda m: box.__setitem__('cmd', (round(m.linear.x, 3), round(m.angular.z, 3))), 10)
n.create_subscription(Odometry, '/odom', lambda m: box.__setitem__('odom', (round(m.twist.twist.linear.x, 3), round(m.pose.pose.position.x, 3))), qos_profile_sensor_data)
t0 = time.time(); nxt = 0.0; rows = []
while time.time() - t0 < 70:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - t0 >= nxt:
        rows.append((round(nxt), box['own'], box['auto'], box['cmd'], box['odom'])); nxt += 2.0
print('t(s) | owner (winner, out_linear=duty, out_angular, req_age) | /cmd_vel_auto sent | /cmd_vel applied | odom (speed m/s, x m)')
last = None
for r in rows:
    key = r[1:4]
    mark = '' if key != last else '  (same)'
    print(r[0], r[1], r[2], r[3], r[4], mark); last = key
