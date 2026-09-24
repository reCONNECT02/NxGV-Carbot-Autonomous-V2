"""sensor_msgs Image <-> numpy without cv_bridge (one less build dependency on the
RDK, and no extra copy). Encodings seen on the RISA Bot:
  (mipi_cam bgr8 / nv12 support stays: the decoder is generic)
  Astra Pro rgb8
"""
import array

import cv2
import numpy as np


def image_to_bgr(msg) -> np.ndarray:
    """sensor_msgs/Image -> HxWx3 uint8 BGR (a view where possible)."""
    h, w, step, enc = int(msg.height), int(msg.width), int(msg.step), msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('bgr8', 'rgb8'):
        img = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return img[:, :, ::-1] if enc == 'rgb8' else img
    if enc in ('bgra8', 'rgba8'):
        img = buf.reshape(h, step)[:, :w * 4].reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if enc == 'bgra8' else cv2.COLOR_RGBA2BGR)
    if enc in ('mono8', '8uc1'):
        return cv2.cvtColor(buf.reshape(h, step)[:, :w], cv2.COLOR_GRAY2BGR)
    if enc == 'nv12':
        return cv2.cvtColor(buf[:h * 3 // 2 * w].reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_NV12)
    raise ValueError(f'unsupported image encoding {msg.encoding!r}')


def compressed_to_bgr(msg) -> np.ndarray:
    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)


def jpeg_msg(img_bgr: np.ndarray, stamp, frame_id: str, quality: int):
    from sensor_msgs.msg import CompressedImage
    ok, enc = cv2.imencode('.jpg', img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    m = CompressedImage()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.format = 'jpeg'
    m.data = array.array('B', enc.tobytes())
    return m


def resize_to_width(img: np.ndarray, width: int) -> np.ndarray:
    h, w = img.shape[:2]
    if width <= 0 or width == w:
        return img
    height = int(round(h * width / w))
    interp = cv2.INTER_AREA if width < w else cv2.INTER_LINEAR
    return cv2.resize(img, (width, height), interpolation=interp)


def u8(a: np.ndarray) -> array.array:
    return array.array('B', np.ascontiguousarray(a, np.uint8).tobytes())


def f32(a: np.ndarray) -> array.array:
    return array.array('f', np.ascontiguousarray(a, np.float32).tobytes())
