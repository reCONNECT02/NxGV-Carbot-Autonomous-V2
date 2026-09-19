"""Detectors on the RDK X5 BPU (phase 6).

Classes: traffic_light_red, traffic_light_green, speed_bump_sign,
boom_gate_open, boom_gate_closed. Debounced states go to mission logic via
DetectionArray. The boom-gate state is used ONLY for the Challenge 4
stop/proceed; at the roundabout it is only compared with the planned exit
(mismatch = log + GUI warning). Also republishes the legacy base topics
/traffic_light_state and /boom_gate_open.
Dataset layout, training and BPU conversion: tools/detector/ (phase 6).
"""
from carbot_common import topics as T
from carbot_common.data import load_data, camera_role_topics
from carbot_common.qos import SENSOR
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import DetectionArray
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

SPEC = BlockSpec(
    node='bpu_detector', block='', title='BPU detector (light, bump sign, gate)', phase=6,
    required=['camera_role', 'rate_hz', 'model_path', 'input_width', 'input_height', 'nms_iou',
              'class_names', 'class_map', 'thresholds.traffic_light_red',
              'thresholds.traffic_light_green', 'thresholds.speed_bump_sign',
              'thresholds.boom_gate_open', 'thresholds.boom_gate_closed', 'debounce_frames',
              'state_expiry_s', 'publish_legacy_topics', 'data.cameras'],
    pubs=[(DetectionArray, T.DETECTIONS, 10), (CompressedImage, T.DETECTIONS_DEBUG, 1),
          (String, T.TRAFFIC_LIGHT_STATE, 10), (Bool, T.BOOM_GATE_OPEN, 10)],
)


def _camera(node):
    topics = camera_role_topics(load_data(node, 'cameras'))
    node.sub(Image, topics[node.p('camera_role')], None, SENSOR)


def main(args=None):
    run_stub(SPEC, extra=_camera, args=args)
