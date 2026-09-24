import time, rclpy, json
from carbot_interfaces.msg import CommandOwnerState, MotionRequest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from carbot_common import topics as T
rclpy.init(); n = rclpy.create_node('trig8'); box = {}
n.create_subscription(CommandOwnerState, T.OWNER_STATE, lambda m: box.__setitem__('own', (m.winner, m.reason[:40], round(m.out_linear_x, 3), round(m.out_angular_z, 3), round(m.request_age_s, 2), bool(m.armed))), 10)
n.create_subscription(MotionRequest, T.CALIBRATION_REQUEST, lambda m: box.__setitem__('req', (m.source, round(m.speed_mps, 3), round(m.steer_rad, 3), round(time.time(), 1))), 10)
n.create_subscription(Twist, T.CMD_VEL_AUTO, lambda m: box.__setitem__('auto', (round(m.linear.x, 3), round(m.angular.z, 3))), 10)
n.create_subscription(Twist, T.CMD_VEL, lambda m: box.__setitem__('cmd', (round(m.linear.x, 3), round(m.angular.z, 3))), 10)
n.create_subscription(Odometry, '/odom', lambda m: box.__setitem__('odom', (round(m.twist.twist.linear.x, 3), round(m.pose.pose.position.x, 3))), qos_profile_sensor_data)
out = open('/tmp/trig8.txt', 'w'); out.write('waiting for the first calibration request (up to 15 min)...\n'); out.flush()
t_wait = time.time(); t0 = None
while time.time() - t_wait < 900:
    rclpy.spin_once(n, timeout_sec=0.05)
    if t0 is None and 'req' in box:
        t0 = time.time(); nxt = 0.0
        out.write('GO seen. t(s) | owner(winner,reason,duty,angular,req_age,armed) | request(src,speed,steer) | /cmd_vel_auto | /cmd_vel applied | odom(v,x)\n')
    if t0 is not None:
        if time.time() - t0 >= nxt:
            out.write(f"{nxt:4.1f} {box.get('own')} {box.get('req', ('-',))[:3]} {box.get('auto')} {box.get('cmd')} {box.get('odom')}\n"); out.flush(); nxt += 1.0
        if time.time() - t0 > 40: break
out.write('done\n'); out.close()
