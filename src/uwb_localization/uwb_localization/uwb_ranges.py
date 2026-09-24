"""UWB ranges + position (feeds BLOCK 06 only; UWB never reaches the servo).

In : /uwb3/input_json   std_msgs/String JSON from the micro-ROS tag, BEST_EFFORT
                        (a RELIABLE subscriber silently receives nothing)
Out: /carbot/uwb/ranges    UwbRanges, one per tag report (empty = no measurement)
     /carbot/uwb/status    UwbStatus at status_publish_hz (link, rate, per-anchor age)
     /carbot/uwb/raw_fix   PointStamped (venue): the solver's fix for this report,
                           unfiltered (Haffiz linear trilateration by default)
     /carbot/uwb/position  Odometry (venue): Haffiz's position = solver + constant-
                           velocity Kalman filter (positioning.py). pose covariance
                           [0,1,6,7] = filter covariance, twist = venue-frame velocity
                           (child_frame_id = venue). Block 06 (global_pose) uses this
                           by default (localization.yaml global_pose.uwb_input).

Solver / filter settings: common.yaml `/**` uwb_positioning (shared with the
calibration wizard and tools/uwb). The filter is reset when the tag reboots.

Per range (uwb_core.RangeProcessor, UWB_Handoff section 11): skip a repeated
sample_seq, drop ranges older than max_range_age_ms, stamp = estimated
measurement time (arrival - WiFi delay - age_ms), subtract range_offset_m,
flatten with the anchor/tag height difference.

Anchor positions, heights and offsets: data/uwb.yaml (calibration step 10
writes the session copy). Environment: ROS_DOMAIN_ID=1, ROS_LOCALHOST_ONLY=0.
"""
import time
from collections import deque

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from carbot_common import topics as T
from carbot_common.data import load_data
from carbot_common.node import CarbotNode
from carbot_common.qos import UWB
from carbot_interfaces.msg import NodeStatus, UwbRange, UwbRanges, UwbStatus
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .positioning import Positioner, PositioningCfg, report_ranges
from .uwb_core import AnchorSet, RangeProcessor, parse_report

REQUIRED = ['link_timeout_s', 'anchor_timeout_s', 'expected_rate_hz', 'max_range_age_ms',
            'min_range_m', 'max_range_m', 'publish_raw_fix', 'publish_position', 'status_publish_hz',
            'latency.mode', 'latency.window_s', 'latency.base_transit_ms',
            'latency.max_extra_ms', 'data.uwb', 'frames.venue',
            # common.yaml uwb_positioning (= positioning.KEYS; test_positioning checks they agree)
            'uwb_positioning.solver', 'uwb_positioning.filter', 'uwb_positioning.min_anchors',
            'uwb_positioning.skip_repeat_reports', 'uwb_positioning.cv_kf.process_noise',
            'uwb_positioning.cv_kf.measurement_noise_m', 'uwb_positioning.cv_kf.gate_mahalanobis2',
            'uwb_positioning.cv_kf.initial_variance_m2', 'uwb_positioning.cv_kf.reacquire_after_rejects',
            'uwb_positioning.moving_average.window', 'uwb_positioning.nlls.f_scale_m',
            'uwb_positioning.nlls.initial_xy_m']


def to_time_msg(t: float) -> TimeMsg:
    m = TimeMsg()
    m.sec = int(t // 1)
    m.nanosec = min(int(round((t - m.sec) * 1e9)), 999_999_999)
    return m


class UwbRangesNode(CarbotNode):

    def __init__(self):
        super().__init__('uwb_ranges', '06', REQUIRED)
        doc = load_data(self, 'uwb')
        self.anchors = AnchorSet.from_yaml(doc)
        topic = str((doc.get('tag') or {}).get('input_topic') or T.UWB_INPUT_JSON)
        if topic != T.UWB_INPUT_JSON:
            self.get_logger().warn(f'uwb.yaml tag.input_topic={topic} differs from {T.UWB_INPUT_JSON}')
        self.proc = RangeProcessor(
            self.anchors, float(self.p('max_range_age_ms')), float(self.p('min_range_m')),
            float(self.p('max_range_m')), str(self.p('latency.mode')),
            float(self.p('latency.window_s')), float(self.p('latency.base_transit_ms')),
            float(self.p('latency.max_extra_ms')))
        self.venue = str(self.p('frames.venue'))
        self.publish_fix = bool(self.p('publish_raw_fix'))
        self.publish_pos = bool(self.p('publish_position'))
        self.pos_cfg = PositioningCfg.from_dict(self.params_under('uwb_positioning'))
        self.positioner = Positioner(self.anchors, self.pos_cfg)
        self.last_fix_wall = None
        self.arrivals = deque(maxlen=64)
        self.last_arrival = None
        self.last_fresh = {a: None for a in self.anchors.ids}
        self.last_range = {a: float('nan') for a in self.anchors.ids}
        self.fresh_count = {a: 0 for a in self.anchors.ids}
        self.last_unknown = '0000'
        self.boot_id = ''
        self.latency_ms = 0.0
        self.bad_json = 0

        self.pub_ranges = self.create_publisher(UwbRanges, T.UWB_RANGES, 10)
        self.pub_status = self.create_publisher(UwbStatus, T.UWB_STATUS, 10)
        self.pub_fix = self.create_publisher(PointStamped, T.UWB_RAW_FIX, 10)
        self.pub_pos = self.create_publisher(Odometry, T.UWB_POSITION, 10)
        self.sub(String, T.UWB_INPUT_JSON, self._on_json, UWB)
        self.create_timer(1.0 / max(float(self.p('status_publish_hz')), 0.2), self._status)
        if not self.anchors.offsets_calibrated:
            self.get_logger().warn('uwb.yaml offsets_calibrated=false: ranges are uncorrected '
                                   '(~1 m long). Run calibration step 10.')
        self.set_status(NodeStatus.WARN, 'WAITING_INPUT', f'waiting for {T.UWB_INPUT_JSON}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_json(self, msg: String) -> None:
        arrival = self._now()
        rep = parse_report(msg.data)
        if rep is None:
            self.bad_json += 1
            return
        self.arrivals.append(time.monotonic())
        self.last_arrival = time.monotonic()
        res = self.proc.process(rep, arrival)
        if res.rebooted:
            self.get_logger().warn(f'UWB tag rebooted (boot_id {self.boot_id} -> {rep.boot_id}): '
                                   'position filter reset')
            self.positioner.reset()
        self.boot_id = rep.boot_id
        self.last_unknown = rep.last_unknown_id
        self.latency_ms = res.latency_ms

        out = UwbRanges()
        out.header.stamp = to_time_msg(arrival)
        out.header.frame_id = self.venue
        out.tag, out.boot_id, out.seq, out.t_ms = rep.tag, rep.boot_id, rep.seq, rep.t_ms
        out.last_unknown_id = rep.last_unknown_id
        for r in res.ranges:
            m = UwbRange()
            m.anchor_id = r.anchor
            m.range_raw_m = float(r.raw_m)
            m.range_corrected_m = float(r.corrected_m)
            m.age_ms = int(max(r.age_ms, 0))
            m.sample_seq = int(max(r.sample_seq, 0))
            m.fresh = bool(r.fresh)
            m.measured_stamp = to_time_msg(r.stamp)
            m.gated_out = False
            out.ranges.append(m)
            if r.reason in ('', 'repeat'):
                self.last_range[r.anchor] = r.corrected_m
            if r.fresh:
                self.last_fresh[r.anchor] = time.monotonic()
                self.fresh_count[r.anchor] += 1
        self.pub_ranges.publish(out)

        rng, t_fix, _ = report_ranges(res, self.pos_cfg.skip_repeat_reports)
        fix = self.positioner.update(rng, t_fix) if rng else None
        if fix is None:
            return
        self.last_fix_wall = time.monotonic()
        stamp = to_time_msg(t_fix)
        if self.publish_fix:
            f = PointStamped()
            f.header.stamp = stamp
            f.header.frame_id = self.venue
            f.point.x, f.point.y = float(fix.raw[0]), float(fix.raw[1])
            f.point.z = float(self.anchors.tag_z)
            self.pub_fix.publish(f)
        if self.publish_pos:
            o = Odometry()
            o.header.stamp = stamp
            o.header.frame_id = self.venue
            o.child_frame_id = self.venue            # twist is in the venue frame too
            o.pose.pose.position.x, o.pose.pose.position.y = float(fix.xy[0]), float(fix.xy[1])
            o.pose.pose.position.z = float(self.anchors.tag_z)
            o.pose.pose.orientation.w = 1.0          # UWB gives no heading
            cov = [0.0] * 36
            cov[0], cov[1], cov[6], cov[7] = fix.cov[0], fix.cov[1], fix.cov[1], fix.cov[2]
            cov[14] = cov[21] = cov[28] = cov[35] = 1e6
            o.pose.covariance = [float(c) for c in cov]
            if fix.vel is not None:
                o.twist.twist.linear.x, o.twist.twist.linear.y = float(fix.vel[0]), float(fix.vel[1])
            tc = [0.0] * 36
            tc[0] = tc[7] = 0.01 if fix.vel is not None else 1e6
            tc[14] = tc[21] = tc[28] = tc[35] = 1e6
            o.twist.covariance = tc
            self.pub_pos.publish(o)

    def _status(self) -> None:
        now = time.monotonic()
        link_ok = self.last_arrival is not None and now - self.last_arrival < float(self.p('link_timeout_s'))
        recent = [t for t in self.arrivals if now - t < 3.0]
        rate = (len(recent) - 1) / (recent[-1] - recent[0]) \
            if len(recent) > 2 and recent[-1] > recent[0] else 0.0
        st = UwbStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = self.venue
        st.agent_link_ok = bool(link_ok)
        st.rate_hz = float(rate)
        st.boot_id = self.boot_id
        st.tag_rebooted = self.proc.reboots > 0
        st.anchor_ids = self.anchors.ids
        timeout = float(self.p('anchor_timeout_s'))
        ages = [(now - self.last_fresh[a]) if self.last_fresh[a] is not None else -1.0
                for a in self.anchors.ids]
        st.anchor_seen = [bool(0.0 <= g < timeout) for g in ages]
        st.anchor_age_s = [float(g) for g in ages]
        st.anchor_range_m = [float(self.last_range[a]) for a in self.anchors.ids]
        st.last_unknown_id = self.last_unknown
        st.anchors_surveyed = self.anchors.surveyed
        st.offsets_calibrated = self.anchors.offsets_calibrated
        st.latency_ms = float(self.latency_ms)
        st.reboots = int(self.proc.reboots)
        st.anchor_fresh_count = [int(self.fresh_count[a]) for a in self.anchors.ids]
        self.pub_status.publish(st)

        missing = [a for a, s in zip(self.anchors.ids, st.anchor_seen) if not s]
        if not link_ok:
            self.set_status(NodeStatus.ERROR, 'NO_LINK',
                            'no tag data: agent running? ROS_LOCALHOST_ONLY=0, ROS_DOMAIN_ID=1?')
        elif missing:
            extra = f'; unknown id {self.last_unknown} heard' if self.last_unknown != '0000' else ''
            self.set_status(NodeStatus.WARN, 'ANCHOR_MISSING', f'not seen: {missing}{extra}')
        elif not self.anchors.offsets_calibrated or not self.anchors.surveyed:
            self.set_status(NodeStatus.WARN, 'UNCALIBRATED',
                            f'{rate:.1f} Hz, all anchors; step 10 not done '
                            f'(surveyed={self.anchors.surveyed}, offsets={self.anchors.offsets_calibrated})')
        else:
            kf = self.positioner.kf
            pos = (f'{self.pos_cfg.solver}+{self.pos_cfg.filter}: fixes {self.positioner.solved}'
                   + (f', gated {kf.rejected}, reacq {kf.reacquires}' if self.pos_cfg.filter == 'cv_kf' else ''))
            fix_age = None if self.last_fix_wall is None else now - self.last_fix_wall
            if fix_age is None or fix_age > float(self.p('link_timeout_s')):
                self.set_status(NodeStatus.WARN, 'NO_FIX', f'{rate:.1f} Hz but no position: fewer than '
                                f'{self.pos_cfg.min_anchors} usable ranges or a degenerate layout; {pos}')
                return
            lvl = NodeStatus.OK if rate >= 0.7 * float(self.p('expected_rate_hz')) else NodeStatus.WARN
            self.set_status(lvl, 'RUNNING', f'{rate:.1f} Hz, {len(self.anchors.ids)} anchors, '
                            f'latency {self.latency_ms:.0f} ms, bad json {self.bad_json}; {pos}')


def main(args=None):
    rclpy.init(args=args)
    node = UwbRangesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
