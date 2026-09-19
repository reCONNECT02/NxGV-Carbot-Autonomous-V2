"""System health: per-topic rate/latency, CPU/BPU/RAM/temperature, battery,
micro-ROS agent link, running camera node PIDs (phase 7).
"""
from carbot_common import topics as T
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import SystemHealth, UwbStatus
from std_msgs.msg import Float32

SPEC = BlockSpec(
    node='system_monitor', block='', title='System monitor', phase=7,
    required=['rate_hz', 'watch_topics', 'watch_expected_hz', 'soc_temp_path', 'bpu_ratio_path',
              'camera_process_patterns', 'uwb_agent_pattern'],
    subs=[(Float32, T.VEHICLE_BATTERY, 10), (UwbStatus, T.UWB_STATUS, 10)],
    pubs=[(SystemHealth, T.SYSTEM_HEALTH, 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
