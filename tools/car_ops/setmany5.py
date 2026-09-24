import sys, json, rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as P, ParameterValue as PV, ParameterType as PT
rclpy.init(); n = rclpy.create_node('setmany5')
c = n.create_client(SetParameters, '/servo_controller/set_parameters'); c.wait_for_service(timeout_sec=10)
vals = json.loads(sys.argv[1]); params = []
for k, v in vals.items():
    if isinstance(v, bool): params.append(P(name=k, value=PV(type=PT.PARAMETER_BOOL, bool_value=v)))
    else: params.append(P(name=k, value=PV(type=PT.PARAMETER_DOUBLE, double_value=float(v))))
f = c.call_async(SetParameters.Request(parameters=params)); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
print([(p.name, r.successful, r.reason) for p, r in zip(params, f.result().results)])
