"""Detector core (no ROS): YOLO11n BPU output decode, NMS, class mapping,
debounced states and a per-detection bearing/range estimate.

Model: tools/bpu_model/unified14 (team package unified14_rdk_x5_deploy), YOLO11n,
14 classes, 640x640 NV12 input. What is taken from the team's signage_detector.py
and what is not is listed in docs/DETECTORS.md.

BPU output protocol (D-Robotics ultralytics_yolo, YOLO11 detect head):
  6 tensors, per stride 8 / 16 / 32: class logits (nc channels) and DFL box
  regressions (4 x reg_max channels), NHWC or NCHW, float32.
  Unlike the team node, the tensors are identified by SHAPE (channel count ->
  cls or box, grid size -> stride), not by their position in the output list:
  pyeasy_dnn and hbm_runtime are not guaranteed to list them in the same order.

Decode (unchanged maths from the team node): sigmoid(class logits) -> best
class per cell, per-class threshold, DFL expected value -> l,t,r,b distances in
cells -> xyxy at 640 x 640 -> class-wise NMS -> scaled to the source image.
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# Canonical names the rest of the stack uses (DetectionArray, mission logic, GUI).
LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN = 'traffic_light_red', 'traffic_light_yellow', 'traffic_light_green'
GATE_OPEN, GATE_CLOSED = 'boom_gate_open', 'boom_gate_closed'
BUMP_SIGN = 'speed_bump_sign'
TRIGGER_CLASSES = (LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN, GATE_OPEN, GATE_CLOSED, BUMP_SIGN)


class ModelMismatch(RuntimeError):
    """The model's outputs do not fit the YAML (class count, tensor count)."""


@dataclass
class Det:
    cls: int              # model class index
    name: str             # canonical name ('' = not mapped)
    score: float
    box: Tuple[float, float, float, float]    # xyxy in the SOURCE image, pixels


# --------------------------------------------------------------------------- decode
def _hwc(t: np.ndarray, channels: int) -> Optional[np.ndarray]:
    a = np.squeeze(np.asarray(t, dtype=np.float32))
    if a.ndim != 3:
        return None
    if a.shape[-1] == channels:
        return a
    if a.shape[0] == channels:
        return np.transpose(a, (1, 2, 0))
    return None


def find_heads(outputs: Sequence[np.ndarray], nc: int, reg_max: int, input_size: int):
    """[(stride, cls HxWxnc, box HxWx4reg)] sorted by stride. Raises ModelMismatch."""
    if len(outputs) != 6:
        raise ModelMismatch(f'expected 6 BPU outputs (YOLO11 detect head), got {len(outputs)}')
    cls, box = {}, {}
    for t in outputs:
        c = _hwc(t, nc)
        b = _hwc(t, 4 * reg_max)
        if c is not None and nc != 4 * reg_max:
            cls[c.shape[0]] = c
        elif b is not None:
            box[b.shape[0]] = b
        else:
            raise ModelMismatch(f'output shape {np.shape(t)} matches neither {nc} classes '
                                f'nor {4 * reg_max} box channels: class_names does not fit this model')
    if sorted(cls) != sorted(box) or len(cls) != 3:
        raise ModelMismatch(f'class grids {sorted(cls)} / box grids {sorted(box)} do not pair up')
    heads = [(input_size // h, cls[h], box[h]) for h in cls]
    return sorted(heads, key=lambda x: x[0])


def decode_level(cls: np.ndarray, box: np.ndarray, stride: int, thresh: np.ndarray, reg_max: int):
    """One stride level -> (xyxy at the model input size, scores, class ids).

    Same result as sigmoid(cls) -> argmax -> threshold, without the sigmoid over every class score of
    every cell (BACKLOG #52, ~117k exp per frame on the RDK): sigmoid is monotonic, so the best class
    and the threshold test are done on the raw logits and only the cells that pass are converted."""
    with np.errstate(divide='ignore'):
        # logit(threshold); a threshold >= 1 (ignored class) can never pass -> +inf
        t = np.where(thresh >= 1.0, np.inf, np.log(thresh / np.maximum(1.0 - thresh, 1e-12)))
    finite = t[np.isfinite(t)]
    empty = (np.empty((0, 4), np.float32), np.empty((0,), np.float32), np.empty((0,), np.int32))
    if finite.size == 0:
        return empty
    # cheap pass: a cell can only pass if its best logit clears the lowest threshold; argmax only on those
    ys, xs = np.where(cls.max(axis=-1) >= finite.min())
    if ys.size == 0:
        return empty
    sub = cls[ys, xs]
    cid = np.argmax(sub, axis=-1)
    best = sub[np.arange(len(cid)), cid]
    ok = best >= t[cid]
    if not ok.any():
        return empty
    ys, xs, cid, best = ys[ok], xs[ok], cid[ok], best[ok]
    best_p = 1.0 / (1.0 + np.exp(-np.clip(best, -30.0, 30.0)))
    ltrb = box[ys, xs].reshape(-1, 4, reg_max)
    ltrb = ltrb - ltrb.max(axis=-1, keepdims=True)
    e = np.exp(ltrb)
    off = (e / e.sum(axis=-1, keepdims=True) * np.arange(reg_max, dtype=np.float32)).sum(axis=-1)
    gx, gy = xs.astype(np.float32) + 0.5, ys.astype(np.float32) + 0.5
    boxes = np.stack([(gx - off[:, 0]) * stride, (gy - off[:, 1]) * stride,
                      (gx + off[:, 2]) * stride, (gy + off[:, 3]) * stride], axis=1)
    return boxes.astype(np.float32), best_p.astype(np.float32), cid.astype(np.int32)


def nms(boxes: np.ndarray, scores: np.ndarray, iou: float) -> List[int]:
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        xx1, yy1 = np.maximum(x1[i], x1[order[1:]]), np.maximum(y1[i], y1[order[1:]])
        xx2, yy2 = np.minimum(x2[i], x2[order[1:]]), np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        ovr = inter / np.maximum(areas[i] + areas[order[1:]] - inter, 1e-9)
        order = order[np.where(ovr <= iou)[0] + 1]
    return keep


@dataclass
class ModelSpec:
    class_names: List[str]
    class_map: List[str]          # canonical name per model class ('' = ignored)
    thresholds: np.ndarray        # per model class
    input_size: int = 640
    reg_max: int = 16
    nms_iou: float = 0.45

    @classmethod
    def from_yaml(cls, class_names, class_map, thresholds: Dict[str, float], info_threshold: float,
                  input_size: int, reg_max: int, nms_iou: float) -> 'ModelSpec':
        if len(class_map) != len(class_names):
            raise ModelMismatch(f'class_map has {len(class_map)} entries, class_names {len(class_names)}')
        th = [float(thresholds[m]) if m in thresholds else (float(info_threshold) if m else 1.01)
              for m in class_map]
        return cls(list(class_names), list(class_map), np.asarray(th, np.float32),
                   int(input_size), int(reg_max), float(nms_iou))


def detect(outputs, spec: ModelSpec, src_w: int, src_h: int) -> List[Det]:
    """Raw BPU outputs -> detections in the source image (unmapped classes dropped)."""
    heads = find_heads(outputs, len(spec.class_names), spec.reg_max, spec.input_size)
    parts = [decode_level(c, b, s, spec.thresholds, spec.reg_max) for s, c, b in heads]
    boxes = np.concatenate([p[0] for p in parts])
    scores = np.concatenate([p[1] for p in parts])
    cids = np.concatenate([p[2] for p in parts])
    out = []
    sx, sy = src_w / spec.input_size, src_h / spec.input_size
    for c in np.unique(cids):
        m = np.where(cids == c)[0]
        for k in nms(boxes[m], scores[m], spec.nms_iou):
            i = m[k]
            name = spec.class_map[int(c)]
            if not name:
                continue
            x1, y1, x2, y2 = boxes[i]
            out.append(Det(int(c), name, float(scores[i]),
                           (float(np.clip(x1 * sx, 0, src_w)), float(np.clip(y1 * sy, 0, src_h)),
                            float(np.clip(x2 * sx, 0, src_w)), float(np.clip(y2 * sy, 0, src_h)))))
    return sorted(out, key=lambda d: -d.score)


# --------------------------------------------------------------------------- debounce
class Debounced:
    """A state becomes active after `frames` sightings of it with no OTHER state in
    between (single missed frames are tolerated, a gap longer than `expiry_s`
    is not) and falls back to UNKNOWN when nothing of this kind was seen for
    `expiry_s`. (The team node's counter could report a state it had never
    confirmed: two sightings + one miss left it active. Not taken.)"""

    def __init__(self, frames: int, expiry_s: float):
        self.frames, self.expiry = int(frames), float(expiry_s)
        self.state, self.candidate, self.count, self.last_seen = 'UNKNOWN', '', 0, -1e9

    def update(self, seen: str, t: float) -> str:
        if seen:
            self.last_seen = t
            self.count = self.count + 1 if seen == self.candidate else 1
            self.candidate = seen
            if self.count >= self.frames:
                self.state = seen
        elif t - self.last_seen > self.expiry:
            self.state, self.candidate, self.count = 'UNKNOWN', '', 0
        return self.state


def frame_light(dets: Sequence[Det]) -> str:
    """RED if any red or yellow lamp is seen (yellow is not green: the mission only
    ever proceeds on GREEN), else GREEN if green, else ''."""
    names = {d.name for d in dets}
    if LIGHT_RED in names or LIGHT_YELLOW in names:
        return 'RED'
    return 'GREEN' if LIGHT_GREEN in names else ''


def frame_gate(dets: Sequence[Det]) -> str:
    """CLOSED dominates (the safe reading), else OPEN, else ''."""
    names = {d.name for d in dets}
    if GATE_CLOSED in names:
        return 'CLOSED'
    return 'OPEN' if GATE_OPEN in names else ''


# --------------------------------------------------------------------------- geometry
@dataclass
class CameraGeom:
    """Front camera for bearing/range. fx/cx from intrinsics if calibrated, else
    from the mount's hfov (cameras.yaml, ASSUMED until calibration step 3)."""
    width: int
    fx: float
    cx: float
    mount_x: float = 0.0
    mount_y: float = 0.0
    mount_yaw: float = 0.0        # rad, 0 = forward

    @classmethod
    def from_hfov(cls, width: int, hfov_deg: float, **mount) -> 'CameraGeom':
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(int(width), fx, width / 2.0, **mount)


def locate(det: Det, cam: CameraGeom, sizes: Dict[str, Dict[str, float]]):
    """(x_m, y_m, range_m) in base_link. range_m = -1 when no size is known: x, y
    is then a unit bearing vector from the camera. Range comes from the apparent
    size of an object of known physical size (pinhole, pitch ignored): rough, and
    only used to tell gates apart, never to stop at a distance."""
    x1, y1, x2, y2 = det.box
    u = 0.5 * (x1 + x2)
    bearing = cam.mount_yaw + math.atan2(cam.cx - u, cam.fx)      # image right = -y
    rng = -1.0
    sz = sizes.get(det.name) or {}
    if sz.get('width_m'):
        px = max(x2 - x1, 1.0)
        rng = cam.fx * float(sz['width_m']) / px
    elif sz.get('height_m'):
        px = max(y2 - y1, 1.0)
        rng = cam.fx * float(sz['height_m']) / px
    if rng <= 0:
        return math.cos(bearing), math.sin(bearing), -1.0
    return cam.mount_x + rng * math.cos(bearing), cam.mount_y + rng * math.sin(bearing), rng


def read_fx_cx(camera_info_yaml: str):
    """(fx, cx, width) from a ROS camera_info yaml, or None."""
    import yaml
    try:
        d = yaml.safe_load(open(camera_info_yaml)) or {}
        k = d['camera_matrix']['data']
        return float(k[0]), float(k[2]), int(d['image_width'])
    except Exception:  # noqa: BLE001  no/invalid calibration -> hfov fallback
        return None
