import time, rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
rclpy.init(); n = rclpy.create_node('param5')
c = n.create_client(GetParameters, '/servo_controller/get_parameters')
c.wait_for_service(timeout_sec=10)
names = ['servo_center', 'servo_range_left', 'servo_range_right', 'steering_max_deg', 'wheel_base', 'ticks_per_meter', 'odom_reverse_polarity', 'odom_velocity_deadband', 'max_linear_velocity',
         'odom_vel_alpha', 'encoder_jump_threshold', 'odom_distance_scale']
f = c.call_async(GetParameters.Request(names=names)); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
for k, v in zip(names, f.result().values): print(k, parameter_value_to_python(v))
