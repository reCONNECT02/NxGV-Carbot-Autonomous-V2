"""Camera frames for wizard steps that need images (step 3 intrinsics).

Subscribes to a sensor's image topic ONLY while a step asks for it (raw
images cost CPU on the RDK), keeps the newest message, and decodes it on
request: frame(sensor) -> (seq, bgr) | None.
"""
from typing import Dict, Iterable, Optional, Tuple

from carbot_common.qos import SENSOR
from sensor_msgs.msg import Image


class FrameTap:

    def __init__(self, node, topics: Dict[str, str]):
        self.node, self.topics = node, dict(topics)      # sensor -> image topic
        self.subs: Dict[str, object] = {}
        self.latest: Dict[str, Tuple[int, object]] = {}
        self.decoded: Dict[str, Tuple[int, object]] = {}
        self.seq: Dict[str, int] = {}

    def want(self, sensors: Iterable[str]) -> None:
        """Subscribe to exactly these sensors (others are dropped)."""
        want = {s for s in sensors if s in self.topics}
        for s in list(self.subs):
            if s not in want:
                self.node.destroy_subscription(self.subs.pop(s))
                self.latest.pop(s, None)
                self.decoded.pop(s, None)
        for s in want - set(self.subs):
            self.subs[s] = self.node.create_subscription(Image, self.topics[s], lambda m, s=s: self._on(s, m), SENSOR)
            self.node.get_logger().info(f'frame tap: {s} <- {self.topics[s]}')

    def _on(self, sensor: str, msg) -> None:
        n = self.seq.get(sensor, 0) + 1
        self.seq[sensor] = n
        self.latest[sensor] = (n, msg)

    def frame(self, sensor: str) -> Optional[Tuple[int, object]]:
        got = self.latest.get(sensor)
        if got is None:
            return None
        seq, msg = got
        d = self.decoded.get(sensor)
        if d is None or d[0] != seq:
            from carbot_perception.ros_image import image_to_bgr
            d = self.decoded[sensor] = (seq, image_to_bgr(msg))
        return d
