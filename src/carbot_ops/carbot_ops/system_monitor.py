"""System health (phase 8, replaces the phase-1 stub; same node, topic and message).

Publishes SystemHealth on /carbot/system/health at rate_hz:
  * per watched topic: rate over rate_window_s, age of the last message, header
    latency (now - header.stamp) and ok (rate >= ok_rate_ratio * expected, fresh);
  * CPU total + per core, RAM, SoC temperature, BPU load (-1 if the sysfs file
    is missing), battery (+ ok vs battery_min_v), micro-ROS agent process,
    camera driver processes with PIDs (wrappers filtered out).

Cheap by design, because it runs next to the autonomy stack in both modes:
  * RAW subscriptions (rclpy raw=True): messages are never deserialised; the
    header stamp is read from the first bytes of the CDR buffer. 960x544 camera
    frames cost a buffer copy, not a Python decode.
  * topics are subscribed as soon as they appear in the ROS graph
    (discovery_period_s), so the node never needs a restart after a driver
    restart, and a missing publisher is reported as "never", not as a crash.
  * image_watch_while_armed: false drops the image subscriptions once the race
    is armed (preflight has already checked them); they report rate -1.
Consumers: GUI System health tab, calibration step 1, race preflight.
"""
import time

import rclpy
from carbot_common import topics as T
from carbot_common.node import CarbotNode
from carbot_common.qos import LATCHED
from carbot_interfaces.msg import NodeStatus, SystemHealth, TopicHealth
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

from . import monitor_core as mc

REQUIRED = ['rate_hz', 'watch_topics', 'watch_expected_hz', 'soc_temp_path', 'bpu_ratio_path',
            'camera_process_patterns', 'uwb_agent_pattern',
            # phase 8
            'rate_window_s', 'ok_rate_ratio', 'max_age_s', 'latency_max_abs_ms', 'process_scan_period_s',
            'process_wrapper_names', 'battery_min_v', 'image_watch_while_armed', 'discovery_period_s']

WATCH_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
IMAGE_TYPES = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')


class SystemMonitor(CarbotNode):

    def __init__(self):
        super().__init__('system_monitor', '', REQUIRED)
        self.ok_cfg = all(self.has_parameter(k) for k in REQUIRED)
        if not self.ok_cfg:
            return                                   # CarbotNode already reports CONFIG_ERROR
        topics = [str(t) for t in self.p('watch_topics')]
        exp = [float(x) for x in self.p('watch_expected_hz')]
        if len(topics) != len(exp):
            self.set_status(NodeStatus.ERROR, 'CONFIG_ERROR',
                            f'watch_topics ({len(topics)}) and watch_expected_hz ({len(exp)}) differ in length')
            self.get_logger().error('ops.yaml system_monitor: watch_topics and watch_expected_hz differ in length')
            self.ok_cfg = False
            return
        self.expected = dict(zip(topics, exp))
        win = float(self.p('rate_window_s'))
        self.meters = {t: mc.RateMeter(win) for t in topics}
        self.subs, self.types, self.bad_types, self.warned = {}, {}, set(), set()
        self.armed = False
        self.battery = None
        self.procs, self.agent = [], False
        self.pub = self.create_publisher(SystemHealth, T.SYSTEM_HEALTH, 10)
        self.sub(Float32, T.VEHICLE_BATTERY, self._on_battery, 10)
        self.create_subscription(Bool, T.RACE_ARMED, self._on_armed, LATCHED)   # not an input: silent in calibrate
        try:
            import psutil
            self.psutil = psutil
            psutil.cpu_percent(percpu=True)          # prime: first call returns zeros
        except ImportError:
            self.psutil = None
            self.get_logger().error('python3-psutil missing: CPU/RAM/process checks disabled '
                                    '(sudo apt install python3-psutil)')
        self.create_timer(float(self.p('discovery_period_s')), self._discover)
        self.create_timer(float(self.p('process_scan_period_s')), self._scan)
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 0.1), self._publish)
        self._discover()
        self._scan()
        self.set_status(NodeStatus.OK, 'RUNNING', f'watching {len(topics)} topics')

    # ------------------------------------------------------------------ inputs
    def _on_battery(self, m):
        self.battery = (time.monotonic(), float(m.data))

    def _on_armed(self, m):
        self.armed = bool(m.data)
        if self.armed and not bool(self.p('image_watch_while_armed')):
            for t in [t for t, ty in self.types.items() if ty in IMAGE_TYPES and t in self.subs]:
                self.destroy_subscription(self.subs.pop(t))
            self.get_logger().info('race armed: image topic rate checks paused (image_watch_while_armed false)')

    def _paused(self, topic):
        return self.armed and not bool(self.p('image_watch_while_armed')) and self.types.get(topic) in IMAGE_TYPES

    def _discover(self):
        try:
            graph = dict(self.get_topic_names_and_types())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'topic discovery failed: {e!r}')
            return
        for topic in self.meters:
            if topic in self.subs or topic in self.bad_types or topic not in graph:
                continue
            types = graph[topic]
            self.types[topic] = types[0]
            if self._paused(topic):
                continue
            if len(types) > 1 and topic not in self.warned:
                self.warned.add(topic)
                self.get_logger().warn(f'{topic} has several types {types}: watching {types[0]}')
            try:
                from rosidl_runtime_py.utilities import get_message
                cls = get_message(types[0])
                fields = list(cls.get_fields_and_field_types().items())
                has_header = bool(fields) and fields[0][1] == 'std_msgs/Header'
            except Exception as e:  # noqa: BLE001  unknown message package: report, keep going
                self.bad_types.add(topic)
                self.get_logger().error(f'cannot watch {topic} ({types[0]}): {e!r}')
                continue
            self.track_input(topic)
            self.subs[topic] = self.create_subscription(
                cls, topic, lambda raw, t=topic, h=has_header: self._on_raw(t, raw, h), WATCH_QOS, raw=True)

    def _on_raw(self, topic, raw, has_header):
        now = time.monotonic()
        lat = -1.0
        if has_header:
            lat = mc.latency_ms(mc.cdr_stamp(raw), self.get_clock().now().nanoseconds * 1e-9,
                                float(self.p('latency_max_abs_ms')))
        self.meters[topic].add(now, lat)
        self.touch(topic)

    def _scan(self):
        if self.psutil is None:
            return
        try:
            procs = mc.psutil_procs()
            wraps = [str(w) for w in self.p('process_wrapper_names')]
            self.procs = mc.scan_processes(procs, [str(p) for p in self.p('camera_process_patterns')], wraps)
            self.agent = bool(mc.scan_processes(procs, [str(self.p('uwb_agent_pattern'))], wraps))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'process scan failed: {e!r}')

    # ------------------------------------------------------------------ output
    def _sys(self, h):
        if self.psutil is not None:
            try:
                cores = [float(c) for c in self.psutil.cpu_percent(percpu=True)]
                h.cpu_core_percent = cores
                h.cpu_percent = sum(cores) / len(cores) if cores else -1.0
                h.ram_percent = float(self.psutil.virtual_memory().percent)
            except Exception:  # noqa: BLE001
                h.cpu_percent = h.ram_percent = -1.0
        else:
            h.cpu_percent = h.ram_percent = -1.0
        t = mc.read_number(str(self.p('soc_temp_path')))
        h.soc_temp_c = t / 1000.0 if t > 1000 else t          # sysfs thermal zones report milli-degC
        bpu_path = str(self.p('bpu_ratio_path'))
        h.bpu_percent = mc.read_number(bpu_path)
        if h.bpu_percent < 0 and 'bpu' not in self.warned:
            self.warned.add('bpu')
            self.get_logger().warn(f'BPU load file {bpu_path} not readable: BPU shows -1 '
                                   '(ops.yaml system_monitor.bpu_ratio_path, still unverified on the RDK X5)')

    def _publish(self):
        now = time.monotonic()
        h = SystemHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        ratio, max_age = float(self.p('ok_rate_ratio')), float(self.p('max_age_s'))
        n_ok = 0
        for topic, m in self.meters.items():
            th = TopicHealth(topic=topic, expected_hz=float(self.expected[topic]))
            if self._paused(topic):
                th.rate_hz, th.age_s, th.latency_ms, th.ok = -1.0, -1.0, -1.0, True
            else:
                th.rate_hz = float(m.rate_hz(now))
                th.age_s = float(m.age_s(now))
                th.latency_ms = float(m.last_latency_ms)
                th.ok = th.rate_hz >= ratio * th.expected_hz and 0.0 <= th.age_s <= max_age
            n_ok += th.ok
            h.topics.append(th)
        self._sys(h)
        if self.battery is not None and now - self.battery[0] < 5.0:
            h.battery_v = self.battery[1]
            h.battery_ok = h.battery_v >= float(self.p('battery_min_v'))
        else:
            h.battery_v, h.battery_ok = -1.0, False
        h.uwb_agent_running = self.agent
        h.camera_process_names = [n for n, _ in self.procs]
        h.camera_process_pids = [int(p) for _, p in self.procs]
        self.pub.publish(h)
        bad = [t.topic for t in h.topics if not t.ok]
        if bad:
            self.set_status(NodeStatus.WARN, 'RUNNING', f'{n_ok}/{len(h.topics)} topics ok; not ok: '
                            + ', '.join(bad[:4]) + (' ...' if len(bad) > 4 else ''))
        else:
            self.set_status(NodeStatus.OK, 'RUNNING', f'{n_ok}/{len(h.topics)} topics ok')


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
