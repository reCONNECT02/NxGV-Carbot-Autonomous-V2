import time, rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from carbot_interfaces.msg import CommandOwnerState, SafetyStatus, MotionRequest
from std_msgs.msg import Bool
from carbot_common import topics as T
rclpy.init(); n = rclpy.create_node('own5'); box = {}
n.create_subscription(CommandOwnerState, T.OWNER_STATE, lambda m: box.__setitem__('owner', m), 10)
n.create_subscription(SafetyStatus, T.SAFETY_STATUS, lambda m: box.__setitem__('safety', m), 10)
n.create_subscription(MotionRequest, T.request_topic('CALIBRATION'), lambda m: box.__setitem__('req', m), 10)
n.create_subscription(MotionRequest, T.request_topic('CALIBRATION_RAW'), lambda m: box.__setitem__('raw', m), 10)
n.create_subscription(Bool, T.VEHICLE_ARM, lambda m: box.__setitem__('arm', m), QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1))
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.2)
o = box.get('owner')
print('OWNER', None if o is None else {f: getattr(o, f) for f in ('mode', 'winner', 'reason', 'armed', 'measured_speed_mps') if hasattr(o, f)})
s = box.get('safety')
if s is not None:
    print('SAFETY allowed', s.motion_allowed, '| veto', s.veto_check, '|', s.veto_reason)
    print('FAILED CHECKS', [(c.name, round(c.value, 3), round(c.limit, 3), c.detail) for c in s.checks if not c.ok])
print('CALIB request seen:', None if 'req' not in box else (box['req'].speed_mps, box['req'].steer_rad, box['req'].reason))
print('CALIB_RAW request seen:', None if 'raw' not in box else (box['raw'].speed_mps, box['raw'].steer_rad, box['raw'].reason))
print('ARM', None if 'arm' not in box else box['arm'].data)
