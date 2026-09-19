"""Race supervisor: calibration gate, PREFLIGHT, READY, the one START (phase 8).

* Loads the ACTIVE calibration session; refuses to arm while any required step
  is missing or failed and lists which (this part already works in phase 1).
* Preflight (not a recalibration): 3 cameras + LiDAR at expected rate and
  fresh, all UWB anchors seen, battery, start pose matches the map start
  (UWB + camera agree within tolerance).
* START (std_srvs/Trigger /carbot/race/start) -> /carbot/race/armed latched
  True, auto rosbag, then no further input is accepted.
"""
from carbot_common import calibration_store as cs
from carbot_common import topics as T
from carbot_common.data import load_data
from carbot_common.qos import LATCHED, SENSOR
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import NodeStatus, PreflightCheck, PreflightReport, UwbStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

SPEC = BlockSpec(
    node='race_supervisor', block='', title='Race supervisor (preflight + START)', phase=8,
    required=['preflight_period_s', 'camera_min_rate_ratio', 'camera_max_age_s', 'lidar_min_hz',
              'uwb_anchor_max_age_s', 'battery_min_v', 'start_position_tolerance_m',
              'start_heading_tolerance_deg', 'start_camera_uwb_agreement_m', 'auto_record',
              'ready_hold_s', 'data_root', 'data.calibration_steps', 'data.cameras', 'data.uwb'],
    subs=[(NodeStatus, T.STATUS, 50), (LaserScan, T.SCAN, SENSOR), (UwbStatus, T.UWB_STATUS, 10),
          (Float32, T.VEHICLE_BATTERY, 10), (PoseWithCovarianceStamped, T.GLOBAL_POSE, 10)],
    pubs=[(PreflightReport, T.RACE_PREFLIGHT, 10), (Bool, T.RACE_ARMED, LATCHED)],
)


def _setup(node):
    root = cs.data_root(node.p('data_root'))
    session = cs.active_session(root)
    missing = cs.missing_required(cs.load_summary(session), load_data(node, 'calibration_steps'))
    node.publishers_by_topic[T.RACE_ARMED].publish(Bool(data=False))

    def report():
        r = PreflightReport()
        r.header.stamp = node.get_clock().now().to_msg()
        r.calibration_session = session or ''
        r.missing_calibrations = missing
        if missing:
            r.state = PreflightReport.STATE_CAL_MISSING
            r.summary = ('REFUSING TO ARM - missing/failed calibration: ' + ', '.join(missing)
                         + ('' if session else ' (no ACTIVE session in ' + root + ')'))
        else:
            r.state = PreflightReport.STATE_NOT_READY
            r.summary = 'calibration OK; preflight checks are implemented in phase 8'
        r.checks.append(PreflightCheck(name='calibration', ok=not missing,
                                       value=session or 'none', expected='all required PASS',
                                       detail=', '.join(missing)))
        node.publishers_by_topic[T.RACE_PREFLIGHT].publish(r)
    node.create_timer(float(node.p('preflight_period_s')), report)

    def start(req, resp):
        resp.success = False
        resp.message = 'START refused: race mode is implemented in phase 8'
        return resp
    node.create_service(Trigger, T.RACE_START_SRV, start)


def main(args=None):
    run_stub(SPEC, extra=_setup, args=args)
