"""BLOCK 15 - Command owner: one final writer (V4 vehicle.js arbitrate) + BLOCK 16 interface.

Every block ASKS (MotionRequest on /carbot/request/<source>); only this node
COMMANDS the car. Priority: SAFETY STOP -> WATCHDOG (request older than
0.2 s) -> the source mission logic marked active. Converts the winning
speed (m/s, speed PID on /odom) and steering (rad, REP-103) into the base
servo_controller's /cmd_vel_auto units. The base auto_driver and
cmd_safety_controller are NOT launched (they would be competing writers).

Phase-1 stub: publishes an explicit ZERO command and winner DISARMED.
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import CommandOwnerState, MissionState, MotionRequest, SafetyStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

SPEC = BlockSpec(
    node='command_owner', block='15', title='One Command Owner', phase=5,
    required=['rate_hz', 'request_expiry_s', 'safety_max_age_s', 'mission_max_age_s',
              'speed_pid.kp', 'speed_pid.ki', 'speed_pid.kd', 'speed_pid.integral_limit',
              'feedforward.duty_per_mps', 'feedforward.static_duty', 'duty_max',
              'duty_reverse_max', 'duty_slew_per_s', 'steering.steer_sign',
              'steering.left_max_rad', 'steering.right_max_rad', 'steering.trim_rad',
              'steering.angular_limit', 'output_topic'],
    subs=[(MotionRequest, T.request_topic(s), 10) for s in T.REQUEST_SOURCES]
    + [(SafetyStatus, T.SAFETY_STATUS, 10), (MissionState, T.MISSION_STATE, LATCHED),
       (Odometry, T.ODOM, 10), (Bool, T.RACE_ARMED, LATCHED), (Bool, T.E_STOP, 10)],
    pubs=[(Twist, T.CMD_VEL_AUTO, 10), (CommandOwnerState, T.OWNER_STATE, 10)],
)


def _zero_loop(node):
    """Stub: never command motion. Publish explicit zeros so the path is testable."""
    cmd_pub = node.publishers_by_topic[T.CMD_VEL_AUTO]
    state_pub = node.publishers_by_topic[T.OWNER_STATE]

    def tick():
        cmd_pub.publish(Twist())
        st = CommandOwnerState()
        st.header.stamp = node.get_clock().now().to_msg()
        st.winner = 'DISARMED'
        st.reason = 'phase 1 skeleton: zero command only'
        st.watchdog_ok = True
        state_pub.publish(st)
    node.create_timer(1.0 / float(node.p('rate_hz')), tick)


def main(args=None):
    run_stub(SPEC, extra=_zero_loop, args=args)
