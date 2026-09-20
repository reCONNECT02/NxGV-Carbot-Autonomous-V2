"""GUI + rosbag camera streams (support node, no block).

preview: downscaled JPEG for the GUI (all 3 cameras side by side), throttled.
record : throttled JPEG for the rosbag (raw 3 x 960x544 bgr8 is never recorded).
Replaces the one-camera-at-a-time hobot_codec + websocket viewer.

Lazy: a role's raw image topic is only subscribed while one of its outputs has
a subscriber (GUI tab open, or the recorder running). Every raw subscription
costs ~1.5 MB of deserialisation per frame at 30 fps, so an idle GUI must cost
nothing.
"""
import time
from typing import Dict

import rclpy
from carbot_common import topics as T
from carbot_common.data import camera_role_topics, load_data
from carbot_common.node import CarbotNode
from carbot_common.qos import SENSOR
from carbot_interfaces.msg import NodeStatus
from sensor_msgs.msg import CompressedImage, Image

from .ros_image import image_to_bgr, jpeg_msg, resize_to_width

REQUIRED = ['preview_width', 'preview_max_fps', 'preview_jpeg_quality', 'record_enabled',
            'record_width', 'record_max_fps', 'record_jpeg_quality', 'check_period_s',
            'data.cameras']


class CameraPreview(CarbotNode):

    def __init__(self):
        super().__init__('camera_preview', '', REQUIRED)
        self.topics = camera_role_topics(load_data(self, 'cameras'))
        self.pub_prev = {r: self.create_publisher(CompressedImage, T.cam_preview(r), 1)
                         for r in self.topics}
        self.pub_rec = {r: self.create_publisher(CompressedImage, T.cam_record(r), 1)
                        for r in self.topics}
        self.subs: Dict[str, object] = {}
        self.last = {(r, k): 0.0 for r in self.topics for k in ('prev', 'rec')}
        self.frames = {r: 0 for r in self.topics}
        for topic in self.topics.values():
            self.track_input(topic)
        self.create_timer(float(self.p('check_period_s')), self._check)
        self._check()

    def _wants(self, role: str) -> bool:
        if self.pub_prev[role].get_subscription_count() > 0:
            return True
        return bool(self.p('record_enabled')) and self.pub_rec[role].get_subscription_count() > 0

    def _check(self) -> None:
        for role, topic in self.topics.items():
            want = self._wants(role)
            if want and role not in self.subs:
                self.subs[role] = self.create_subscription(
                    Image, topic, lambda m, r=role, tp=topic: self._on_image(r, tp, m), SENSOR)
            elif not want and role in self.subs:
                self.destroy_subscription(self.subs.pop(role))
        active = sorted(self.subs)
        self.set_status(NodeStatus.OK, 'RUNNING' if active else 'IDLE',
                        'streaming: ' + (', '.join(active) if active else 'none (no viewers)'))

    def _on_image(self, role: str, topic: str, msg: Image) -> None:
        self.touch(topic)
        now = time.monotonic()
        jobs = []
        if self.pub_prev[role].get_subscription_count() > 0 and \
                now - self.last[(role, 'prev')] >= 1.0 / max(float(self.p('preview_max_fps')), 0.1):
            jobs.append(('prev', self.pub_prev[role], int(self.p('preview_width')),
                         int(self.p('preview_jpeg_quality'))))
        if bool(self.p('record_enabled')) and self.pub_rec[role].get_subscription_count() > 0 and \
                now - self.last[(role, 'rec')] >= 1.0 / max(float(self.p('record_max_fps')), 0.1):
            jobs.append(('rec', self.pub_rec[role], int(self.p('record_width')),
                         int(self.p('record_jpeg_quality'))))
        if not jobs:
            return
        try:
            img = image_to_bgr(msg)
        except ValueError as e:
            self.set_status(NodeStatus.ERROR, 'BAD_ENCODING', f'{role}: {e}')
            return
        for key, pub, width, q in jobs:
            m = jpeg_msg(resize_to_width(img, width), msg.header.stamp, f'cam_{role}', q)
            if m is not None:
                pub.publish(m)
                self.last[(role, key)] = now


def main(args=None):
    rclpy.init(args=args)
    node = CameraPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
