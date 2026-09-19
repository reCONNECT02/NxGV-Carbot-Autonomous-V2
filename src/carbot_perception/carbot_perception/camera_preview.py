"""GUI + rosbag camera streams (support node, no block).

preview: downscaled JPEG, published only while a GUI tab subscribes.
record : throttled JPEG for the rosbag (raw 3 x 960x544 bgr8 is never recorded).
Replaces the one-camera-at-a-time hobot_codec + websocket viewer.
"""
from carbot_common import topics as T
from carbot_common.data import subscribe_cameras
from carbot_common.stub import BlockSpec, run_stub
from sensor_msgs.msg import CompressedImage

SPEC = BlockSpec(
    node='camera_preview', block='', title='Camera preview/record streams', phase=2,
    required=['preview_width', 'preview_max_fps', 'preview_jpeg_quality', 'record_enabled',
              'record_width', 'record_max_fps', 'record_jpeg_quality', 'data.cameras'],
    pubs=[(CompressedImage, T.cam_preview(r), 1) for r in T.CAMERA_ROLES]
    + [(CompressedImage, T.cam_record(r), 1) for r in T.CAMERA_ROLES],
)


def main(args=None):
    run_stub(SPEC, extra=lambda n: subscribe_cameras(n), args=args)
