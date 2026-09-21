"""Detectors on the RDK X5 BPU (phase 6): traffic light, boom gate, signs.

Model: YOLO11n unified14 (team package), 640x640 NV12, installed with this
package (models/). The front camera (role `front` in cameras.yaml, the Astra Pro)
is the only input. What each class triggers: docs/DETECTORS.md.

Outputs
  /carbot/detections   DetectionArray, one per processed frame:
    detections[]        every mapped detection: canonical class, confidence, box
                        in the source image, base_link position (bearing always;
                        range when the object's size is known, else distance_m -1)
    traffic_light_state RED | GREEN | UNKNOWN (debounced; yellow counts as RED)
    boom_gate_state     OPEN | CLOSED | UNKNOWN (debounced; any gate in view)
    speed_bump_sign     debounced (class disabled in YAML until it is trained)
  -> mission_logic (block 08) uses the light state and, per map gate, the boom
     detections it can associate by bearing/range (mission_rules gate_association).
  /carbot/detections/debug/compressed  boxes drawn on the frame, only encoded
     while someone subscribes (GUI Detections tab), max debug.max_rate_hz.
  /traffic_light_state, /boom_gate_open  legacy base-repo topics
     (publish_legacy_topics). /boom_gate_open is only published while the gate
     state is known: the team node defaulted to OPEN, which is not safe.

NOT published (on purpose): /tunnel_detected (owned by the base LiDAR tunnel
trigger, which mission logic relies on), /parking_signboard_detected (the base
parking-playback chain is not used), hill / obstacle sign topics (no consumer).

Load: inference runs on the newest frame from a timer at rate_hz (the camera
gives 30 Hz; older frames are dropped, never queued).
"""
import os
import time

import numpy as np
import rclpy
from carbot_common import topics as T
from carbot_common.data import camera_role_topics, load_data
from carbot_common.node import CarbotNode
from carbot_common.qos import SENSOR
from carbot_interfaces.msg import Detection, DetectionArray, NodeStatus
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

from . import detector_core as dc

REQUIRED = ['camera_role', 'rate_hz', 'model_path', 'input_width', 'input_height', 'nms_iou', 'reg_max',
            'class_names', 'class_map', 'thresholds.traffic_light_red', 'thresholds.traffic_light_yellow',
            'thresholds.traffic_light_green', 'thresholds.speed_bump_sign', 'thresholds.boom_gate_open',
            'thresholds.boom_gate_closed', 'info_threshold', 'debounce_frames', 'state_expiry_s',
            'object_size_m.boom_gate_closed.width_m', 'object_size_m.boom_gate_open.height_m',
            'object_size_m.traffic_light_red.height_m', 'object_size_m.traffic_light_yellow.height_m',
            'object_size_m.traffic_light_green.height_m',
            'publish_legacy_topics', 'debug.enabled', 'debug.jpeg_quality', 'debug.max_rate_hz',
            'data.cameras']

THRESH_KEYS = ('traffic_light_red', 'traffic_light_yellow', 'traffic_light_green', 'speed_bump_sign',
               'boom_gate_open', 'boom_gate_closed')
SIZE_KEYS = {'boom_gate_closed': 'width_m', 'boom_gate_open': 'height_m', 'traffic_light_red': 'height_m',
             'traffic_light_yellow': 'height_m', 'traffic_light_green': 'height_m'}


def resolve_model(path: str) -> str:
    """Absolute, or relative to this package's share directory (models/...)."""
    if os.path.isabs(path):
        return path
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('carbot_detectors'), path)


def load_runtime(path: str):
    try:
        from hobot_dnn import pyeasy_dnn as dnn
    except ImportError:
        from hobot_dnn_rdkx5 import pyeasy_dnn as dnn
    return dnn.load(path)[0]


class BpuDetector(CarbotNode):

    def __init__(self):
        super().__init__('bpu_detector', '', REQUIRED)
        th = {k: float(self.p(f'thresholds.{k}')) for k in THRESH_KEYS}
        self.spec = dc.ModelSpec.from_yaml(
            list(self.p('class_names')), [str(c) for c in self.p('class_map')], th,
            float(self.p('info_threshold')), int(self.p('input_width')), int(self.p('reg_max')),
            float(self.p('nms_iou')))
        if int(self.p('input_width')) != int(self.p('input_height')):
            raise dc.ModelMismatch('input_width != input_height: the unified14 model is square')
        self.sizes = {k: {v: float(self.p(f'object_size_m.{k}.{v}'))} for k, v in SIZE_KEYS.items()}
        self.light = dc.Debounced(int(self.p('debounce_frames')), float(self.p('state_expiry_s')))
        self.gate = dc.Debounced(int(self.p('debounce_frames')), float(self.p('state_expiry_s')))
        self.bump = dc.Debounced(int(self.p('debounce_frames')), float(self.p('state_expiry_s')))

        cams = load_data(self, 'cameras')
        role = str(self.p('camera_role'))
        self.image_topic = camera_role_topics(cams)[role]
        self.cam_cfg = (cams, role)
        self.cam = None                         # built from the first frame (needs its width)

        self.pub = self.create_publisher(DetectionArray, T.DETECTIONS, 10)
        self.pub_dbg = self.create_publisher(CompressedImage, T.DETECTIONS_DEBUG, 1)
        self.legacy = bool(self.p('publish_legacy_topics'))
        if self.legacy:
            self.pub_tl = self.create_publisher(String, T.TRAFFIC_LIGHT_STATE, 10)
            self.pub_gate = self.create_publisher(Bool, T.BOOM_GATE_OPEN, 10)
        self.dbg_last = 0.0

        self.model, self.model_path, self.checked = None, resolve_model(str(self.p('model_path'))), False
        try:
            if not os.path.isfile(self.model_path):
                raise FileNotFoundError(self.model_path)
            self.model = load_runtime(self.model_path)
            self.set_status(NodeStatus.OK, 'LOADED', os.path.basename(self.model_path))
        except Exception as e:  # noqa: BLE001  no BPU runtime (laptop) or no model file
            self.set_status(NodeStatus.ERROR, 'NO_MODEL', f'{self.model_path}: {e}')
            self.get_logger().error(f'BPU model not loaded ({e}). Detector idles; mission holds '
                                    'that need it will never release (see mission_rules.yaml).')

        self.latest = None
        self.sub(Image, self.image_topic, self._on_image, SENSOR)
        self.create_timer(1.0 / max(float(self.p('rate_hz')), 0.5), self._tick)
        self.get_logger().info(f'bpu_detector: {len(self.spec.class_names)} classes on {self.image_topic} '
                               f'at {float(self.p("rate_hz")):.0f} Hz, model {self.model_path}')

    # ------------------------------------------------------------------ io
    def _on_image(self, msg: Image) -> None:
        self.latest = msg

    def _camera(self, width: int) -> dc.CameraGeom:
        cams, role = self.cam_cfg
        m = (cams.get('mounts') or {}).get(role) or {}
        mount = dict(mount_x=float(m.get('x_m', 0.0)), mount_y=float(m.get('y_m', 0.0)),
                     mount_yaw=np.radians(float(m.get('yaw_deg', 0.0))))
        sensor = cams['sensors'][cams['roles'][role]]
        k = dc.read_fx_cx(sensor.get('intrinsics_file') or '')
        if k is not None:
            fx, cx, w0 = k
            s = width / float(w0)
            return dc.CameraGeom(width, fx * s, cx * s, **mount)
        return dc.CameraGeom.from_hfov(width, float(m.get('hfov_deg', 60.0)), **mount)

    def _tick(self) -> None:
        msg, self.latest = self.latest, None
        if msg is None or self.model is None:
            return
        import cv2
        from carbot_perception.ros_image import image_to_bgr
        t0 = time.monotonic()
        bgr = image_to_bgr(msg)
        h, w = bgr.shape[:2]
        if self.cam is None:
            self.cam = self._camera(w)
        n = self.spec.input_size
        small = cv2.resize(bgr, (n, n), interpolation=cv2.INTER_LINEAR)
        yuv = cv2.cvtColor(small, cv2.COLOR_BGR2YUV_I420)
        uv = np.stack([yuv[n:n + n // 4].ravel(), yuv[n + n // 4:].ravel()], axis=1).reshape(n // 2, n)
        nv12 = np.vstack((yuv[:n], uv))
        try:
            outs = [np.asarray(o.buffer) for o in self.model.forward([nv12])]
            dets = dc.detect(outs, self.spec, w, h)
        except dc.ModelMismatch as e:
            self.model = None
            self.set_status(NodeStatus.ERROR, 'MODEL_MISMATCH', str(e))
            self.get_logger().error(f'Model does not match detectors.yaml: {e}. Detector stopped.')
            return
        ms = (time.monotonic() - t0) * 1000.0
        now = time.monotonic()
        light = self.light.update(dc.frame_light(dets), now)
        gate = self.gate.update(dc.frame_gate(dets), now)
        bump = self.bump.update('SEEN' if any(d.name == dc.BUMP_SIGN for d in dets) else '', now)
        self._publish(msg, dets, light, gate, bump == 'SEEN', ms)
        self._debug(msg, bgr, dets, light, gate)
        self.set_status(NodeStatus.OK, 'RUNNING', f'{len(dets)} det, {ms:.0f} ms, light {light}, gate {gate}')

    def _publish(self, msg, dets, light, gate, bump, ms) -> None:
        out = DetectionArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'base_link'
        role = str(self.p('camera_role'))
        for d in dets:
            x, y, rng = dc.locate(d, self.cam, self.sizes)
            m = Detection()
            m.class_name, m.confidence, m.camera_role = d.name, d.score, role
            x1, y1, x2, y2 = d.box
            m.x, m.y, m.width, m.height = int(x1), int(y1), int(x2 - x1), int(y2 - y1)
            m.distance_m = float(rng)
            m.position.x, m.position.y = float(x), float(y)
            out.detections.append(m)
        out.traffic_light_state, out.boom_gate_state = light, gate
        out.speed_bump_sign, out.inference_ms = bool(bump), float(ms)
        self.pub.publish(out)
        if self.legacy:
            self.pub_tl.publish(String(data=light))
            if gate in ('OPEN', 'CLOSED'):
                self.pub_gate.publish(Bool(data=gate == 'OPEN'))

    def _debug(self, msg, bgr, dets, light, gate) -> None:
        if not bool(self.p('debug.enabled')) or self.pub_dbg.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if now - self.dbg_last < 1.0 / max(float(self.p('debug.max_rate_hz')), 0.1):
            return
        self.dbg_last = now
        import cv2
        from carbot_perception.ros_image import jpeg_msg
        img = bgr.copy()
        for d in dets:
            x1, y1, x2, y2 = map(int, d.box)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f'{d.name} {d.score:.2f}', (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 255, 0), 1)
        cv2.putText(img, f'LIGHT {light}  GATE {gate}', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 2)
        j = jpeg_msg(img, msg.header.stamp, msg.header.frame_id, int(self.p('debug.jpeg_quality')))
        if j is not None:
            self.pub_dbg.publish(j)


def main(args=None):
    rclpy.init(args=args)
    node = BpuDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
