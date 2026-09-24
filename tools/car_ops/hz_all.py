import time, rclpy
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node('hz_all')
time.sleep(1.0)
topics = sorted(t for t, ty in n.get_topic_names_and_types() if ('sensor_msgs/msg/Image' in ty) and 'compressed' not in t)
print('image topics:', topics)
cnt = {t: [] for t in topics}
for t in topics:
    n.create_subscription(Image, t, lambda m, t=t: cnt[t].append((time.time(), m.width, m.height, m.encoding)), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
for t, v in cnt.items():
    if len(v) > 1:
        print(f'{t}: {(len(v)-1)/(v[-1][0]-v[0][0]):.1f} Hz {v[0][1]}x{v[0][2]} {v[0][3]}')
    else:
        print(f'{t}: NO FRAMES')
