import sys, time, json, rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
dur = float(sys.argv[1]); rows = []
rclpy.init(); n = rclpy.create_node('ticks5')
n.create_subscription(Odometry, '/odom', lambda m: rows.append((time.time(), m.pose.pose.position.x)), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.2)
json.dump(rows, open('/tmp/ticks5.json', 'w'))
# 1 tick = 1 mm in this mode: ticks = dx * 1000
segs, cur, last_x, last_t = [], None, rows[0][1], rows[0][0]
for t, x in rows:
    moving = abs(x - last_x) > 1e-9
    if moving:
        if cur is None: cur = {'t0': t - t0, 'x0': last_x, 'peak_rate': 0.0}
        cur['t1'] = t - t0; cur['x1'] = x
        cur['peak_rate'] = max(cur['peak_rate'], abs(x - last_x) * 1000 / max(t - last_t, 1e-3))
        cur['last_move'] = t
    elif cur is not None and t - cur['last_move'] > 2.0:
        segs.append(cur); cur = None
    last_x, last_t = x, t
if cur is not None: segs.append(cur)
print(f'messages {len(rows)}; activity bursts (ticks = |dx| x 1000):')
for s in segs:
    print(f"  t {s['t0']:6.1f}-{s['t1']:6.1f} s  ticks {abs(s['x1'] - s['x0']) * 1000:10.0f}  sign {'+' if s['x1'] >= s['x0'] else '-'}  duration {s['t1'] - s['t0']:5.1f} s  peak {s['peak_rate']:8.0f} ticks/s")
print('total net ticks', round((rows[-1][1] - rows[0][1]) * 1000))
