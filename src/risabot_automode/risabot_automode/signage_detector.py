#!/usr/bin/env python3
"""
Signage Detector Node — YOLOv5 BPU Model Inference via hobot_dnn

Loads the compiled .bin model, subscribes to camera raw images,
runs hardware-accelerated BPU inference, and publishes processed state updates.
Features a platform-check so it runs gracefully on RDK X5 and idles on non-RDK systems.
"""

import time
from typing import Dict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

# Import topics from our shared module
from .topics import (
    CAMERA_IMAGE_TOPIC,
    HILL_SIGN_TOPIC,
    OBSTACLE_CAMERA_TOPIC,
    PARKING_SIGN_TOPIC,
    SIGNAGE_DEBUG_TOPIC,
    TRAFFIC_LIGHT_TOPIC,
)

# Graceful import of BPU runtime library
try:
    try:
        from hobot_dnn import pyeasy_dnn as dnn
    except ImportError:
        from hobot_dnn_rdkx5 import pyeasy_dnn as dnn
    BPU_AVAILABLE = True
except ImportError:
    BPU_AVAILABLE = False


class SignageDetector(Node):
    """BPU-accelerated signage detector node for competition signs & traffic lights."""

    def __init__(self):
        super().__init__('signage_detector')

        # ── Tunable parameters ─────────────────────────────────────────────
        self.declare_parameter('model_path',             '/home/sunrise/risabot_signs_640x640_nv12.bin')
        self.declare_parameter('conf_threshold',         0.05)
        self.declare_parameter('iou_threshold',          0.45)
        self.declare_parameter('show_debug',             False)
        self.declare_parameter('heartbeat_sec',          0.5)
        self.declare_parameter('min_parking_sign_width', 0)

        # ── Per-class confidence thresholds (ROS2 params — tunable from dashboard) ─
        self.declare_parameter('thresh_bumper',      0.15)   # Class 0 Bumper_signboard
        self.declare_parameter('thresh_hill',        0.15)   # Class 1 Hill_signboard
        self.declare_parameter('thresh_obstacle',    0.15)   # Class 2 Obstacle_signboard
        self.declare_parameter('thresh_parallelp',   0.10)   # Class 3 ParallelP_signboard
        self.declare_parameter('thresh_perpendp',    0.10)   # Class 4 PerpendP_signboard
        self.declare_parameter('thresh_roundabout',  0.10)   # Class 5 Roundabout_signboard
        self.declare_parameter('thresh_tl_green',    0.15)   # Class 6 Traffic_Green
        self.declare_parameter('thresh_tl_red',      0.15)   # Class 7 Traffic_Red
        self.declare_parameter('thresh_tl_generic',  0.15)   # Class 8 Trafficlight_signboard

        # ── Per-class bounding box colors as BGR strings "B,G,R" ──────────────
        self.declare_parameter('color_bumper',     '0,140,255')    # Orange
        self.declare_parameter('color_hill',       '180,0,255')    # Pink/Purple
        self.declare_parameter('color_obstacle',   '0,255,255')    # Yellow
        self.declare_parameter('color_parallelp',  '255,0,0')      # Blue
        self.declare_parameter('color_perpendp',   '0,100,0')      # Forest Green
        self.declare_parameter('color_roundabout', '255,255,255')  # White
        self.declare_parameter('color_tl_green',   '0,255,0')      # Pure Green
        self.declare_parameter('color_tl_red',     '0,0,255')      # Pure Red
        self.declare_parameter('color_tl_generic', '255,255,0')    # Cyan

        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        self.bridge = CvBridge()
        self.bpu_available = BPU_AVAILABLE
        self._last_log_time = 0.0  # rate-limit log (avoids hasattr in hot path)

        # Per-class thresholds and colors are now read dynamically from ROS2
        # parameters via _get_class_thresholds() and _get_class_colors().

        # ── Detection & Gating state ────────────────────────────────────────
        self.hill_sign_active = False
        self.parking_sign_active = False
        self.obstacle_sign_active = False
        self.traffic_light_active = 'unknown'

        self.detected_hill_consecutive = 0
        self.detected_parking_consecutive = 0
        self.detected_obstacle_consecutive = 0
        self.detected_tl_red_consecutive = 0
        self.detected_tl_green_consecutive = 0
        self.detected_tl_yellow_consecutive = 0

        # ── ROS publishers & subscribers ────────────────────────────────────
        self.parking_pub = self.create_publisher(Bool, PARKING_SIGN_TOPIC, 10)
        self.traffic_light_pub = self.create_publisher(String, TRAFFIC_LIGHT_TOPIC, 10)
        self.hill_pub = self.create_publisher(Bool, HILL_SIGN_TOPIC, 10)
        self.obstacle_pub = self.create_publisher(Bool, OBSTACLE_CAMERA_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, SIGNAGE_DEBUG_TOPIC, 10)

        # Heartbeat timer — continuously publishes last states to keep topics fresh
        self._heartbeat_timer = self.create_timer(
            float(self._param_cache['heartbeat_sec']),
            self.publish_states
        )

        # ── Initialize BPU Runtime ──────────────────────────────────────────
        if self.bpu_available:
            try:
                model_path = str(self._param_cache['model_path'])
                self.get_logger().info(f'Loading BPU model from: {model_path}')
                self.models = dnn.load(model_path)
                self.model = self.models[0]
                self.get_logger().info('BPU model loaded successfully.')
            except Exception as e:
                self.get_logger().error(f'Failed to load BPU model: {e}')
                self.bpu_available = False

        if not self.bpu_available:
            self.get_logger().warn(
                'hobot_dnn runtime not available or failed to load. '
                'Node will operate in dummy/idle mode (no BPU inference).'
            )

        # Camera raw subscriber
        self.color_sub = self.create_subscription(
            Image,
            CAMERA_IMAGE_TOPIC,
            self.image_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.get_logger().info('Signage Detector node initialized.')

    # ──────────────────────────────────────────────────────────────────────────
    # Parameter management
    # ──────────────────────────────────────────────────────────────────────────

    def _update_param_cache(self) -> None:
        self._param_cache = {
            'model_path':             str(self.get_parameter('model_path').value),
            'conf_threshold':         float(self.get_parameter('conf_threshold').value),
            'iou_threshold':          float(self.get_parameter('iou_threshold').value),
            'show_debug':             bool(self.get_parameter('show_debug').value),
            'heartbeat_sec':          float(self.get_parameter('heartbeat_sec').value),
            'min_parking_sign_width': int(self.get_parameter('min_parking_sign_width').value),
            # Per-class thresholds
            'thresh_bumper':     float(self.get_parameter('thresh_bumper').value),
            'thresh_hill':       float(self.get_parameter('thresh_hill').value),
            'thresh_obstacle':   float(self.get_parameter('thresh_obstacle').value),
            'thresh_parallelp':  float(self.get_parameter('thresh_parallelp').value),
            'thresh_perpendp':   float(self.get_parameter('thresh_perpendp').value),
            'thresh_roundabout': float(self.get_parameter('thresh_roundabout').value),
            'thresh_tl_green':   float(self.get_parameter('thresh_tl_green').value),
            'thresh_tl_red':     float(self.get_parameter('thresh_tl_red').value),
            'thresh_tl_generic': float(self.get_parameter('thresh_tl_generic').value),
            # Per-class colors
            'color_bumper':     str(self.get_parameter('color_bumper').value),
            'color_hill':       str(self.get_parameter('color_hill').value),
            'color_obstacle':   str(self.get_parameter('color_obstacle').value),
            'color_parallelp':  str(self.get_parameter('color_parallelp').value),
            'color_perpendp':   str(self.get_parameter('color_perpendp').value),
            'color_roundabout': str(self.get_parameter('color_roundabout').value),
            'color_tl_green':   str(self.get_parameter('color_tl_green').value),
            'color_tl_red':     str(self.get_parameter('color_tl_red').value),
            'color_tl_generic': str(self.get_parameter('color_tl_generic').value),
        }
        self._build_class_caches()

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        self._build_class_caches()  # rebuild cached arrays whenever any param changes
        return SetParametersResult(successful=True)

    def _build_class_caches(self) -> None:
        """Pre-build NumPy threshold array and color list from param cache.

        Called once at init and on every parameter change so the hot
        inference path never constructs these structures per-frame.
        """
        c = self._param_cache
        conf = c['conf_threshold']
        # Shape (10,) — indexed directly by class_id for vectorised filtering
        self._class_thresh_array = np.array([
            c.get('thresh_bumper',     conf),   # 0 Bumper_signboard
            c.get('thresh_hill',       conf),   # 1 Hill_signboard
            c.get('thresh_obstacle',   conf),   # 2 Obstacle_signboard
            c.get('thresh_parallelp',  conf),   # 3 ParallelP_signboard
            c.get('thresh_perpendp',   conf),   # 4 PerpendP_signboard
            c.get('thresh_roundabout', conf),   # 5 Roundabout_signboard
            c.get('thresh_tl_green',   conf),   # 6 Traffic_Green
            c.get('thresh_tl_red',     conf),   # 7 Traffic_Red
            c.get('thresh_tl_generic', conf),   # 8 Trafficlight_signboard
            1.0,                                # 9 null — always filtered out
        ], dtype=np.float32)
        self._class_colors_cache = [
            self._parse_color(c['color_bumper']),
            self._parse_color(c['color_hill']),
            self._parse_color(c['color_obstacle']),
            self._parse_color(c['color_parallelp']),
            self._parse_color(c['color_perpendp']),
            self._parse_color(c['color_roundabout']),
            self._parse_color(c['color_tl_green']),
            self._parse_color(c['color_tl_red']),
            self._parse_color(c['color_tl_generic']),
            (128, 128, 128),  # null (unused)
        ]

    def _get_class_thresholds(self) -> dict:
        """Build per-class threshold dict from current param cache."""
        c = self._param_cache
        return {
            0: c['thresh_bumper'],
            1: c['thresh_hill'],
            2: c['thresh_obstacle'],
            3: c['thresh_parallelp'],
            4: c['thresh_perpendp'],
            5: c['thresh_roundabout'],
            6: c['thresh_tl_green'],
            7: c['thresh_tl_red'],
            8: c['thresh_tl_generic'],
        }

    @staticmethod
    def _parse_color(color_str: str) -> tuple:
        """Parse 'B,G,R' string to a (B, G, R) int tuple for OpenCV."""
        try:
            parts = [int(x.strip()) for x in color_str.split(',')]
            if len(parts) == 3:
                return tuple(parts)
        except Exception:
            pass
        return (255, 255, 255)  # fallback white

    def _get_class_colors(self) -> list:
        """Build per-class color list from current param cache."""
        c = self._param_cache
        return [
            self._parse_color(c['color_bumper']),
            self._parse_color(c['color_hill']),
            self._parse_color(c['color_obstacle']),
            self._parse_color(c['color_parallelp']),
            self._parse_color(c['color_perpendp']),
            self._parse_color(c['color_roundabout']),
            self._parse_color(c['color_tl_green']),
            self._parse_color(c['color_tl_red']),
            self._parse_color(c['color_tl_generic']),
            (128, 128, 128),  # null (unused, always filtered)
        ]

    # ──────────────────────────────────────────────────────────────────────────
    # State publishing helper
    # ──────────────────────────────────────────────────────────────────────────

    def publish_states(self) -> None:
        """Publish the current latch states of the sign/light flags."""
        self.parking_pub.publish(Bool(data=self.parking_sign_active))
        self.traffic_light_pub.publish(String(data=self.traffic_light_active))
        self.hill_pub.publish(Bool(data=self.hill_sign_active))
        self.obstacle_pub.publish(Bool(data=self.obstacle_sign_active))

    # ──────────────────────────────────────────────────────────────────────────
    # Image preprocessing (BGR to NV12)
    # ──────────────────────────────────────────────────────────────────────────

    def bgr_to_nv12(self, bgr640: np.ndarray) -> np.ndarray:
        """Convert a pre-resized 640x640 BGR image to NV12 layout for BPU.

        Caller is responsible for passing an already-resized 640x640 frame so
        this method performs zero redundant resize work.
        """
        # 1. Convert to YUV I420 (input already 640x640)
        yuv = cv2.cvtColor(bgr640, cv2.COLOR_BGR2YUV_I420)
        # yuv has shape (960, 640)

        # 2. Extract planar components
        y = yuv[0:640, :]
        u = yuv[640:800, :]
        v = yuv[800:960, :]

        # 3. Interleave U and V for NV12 using column-stack (faster than strided assignment)
        uv_planar = np.stack([u.ravel(), v.ravel()], axis=1).ravel().reshape(320, 640)

        # 4. Stack Y and interleaved UV planes
        nv12 = np.vstack((y, uv_planar))
        return nv12

    # ──────────────────────────────────────────────────────────────────────────
    # Vectorized Non-Maximum Suppression
    # ──────────────────────────────────────────────────────────────────────────

    def nms(self, boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list:
        """Standard vectorized NMS in Numpy."""
        if len(boxes) == 0:
            return []
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        
        order = scores.argsort()[::-1]
        keep = []
        
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            
            inter = w * h
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            
            inds = np.where(ovr <= iou_threshold)[0]
            order = order[inds + 1]
            
        return keep

    # ──────────────────────────────────────────────────────────────────────────
    # Main camera callback
    # ──────────────────────────────────────────────────────────────────────────

    def image_callback(self, msg: Image) -> None:
        """Receive image, perform BPU inference, parse predictions, filter and publish."""
        if not self.bpu_available:
            return

        try:
            # Convert ROS Image to OpenCV BGR
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Resize ONCE here — reused for BPU NV12 input, CV post-processing,
            # and debug rendering. Avoids multiple redundant resize calls.
            bgr640 = cv2.resize(bgr, (640, 640), interpolation=cv2.INTER_LINEAR)

            # Preprocess to NV12 format (accepts pre-resized 640x640 image)
            nv12 = self.bgr_to_nv12(bgr640)
            
            # Forward pass on BPU
            # hobot_dnn forward takes list of inputs
            outputs = self.model.forward([nv12])
            pred = outputs[0].buffer
            
            # Reshape/squeeze predictions to 2D
            if len(pred.shape) > 2:
                pred = np.squeeze(pred)
                
            conf_threshold = float(self._param_cache['conf_threshold'])
            iou_threshold = float(self._param_cache['iou_threshold'])
            
            # YOLOv5 outputs: box coordinates [0:4], objectness score [4], class scores [5:]
            # Calculate absolute score = objectness * class_probability
            scores = pred[:, 4:5] * pred[:, 5:]
            class_ids = np.argmax(pred[:, 5:], axis=1)
            max_scores = pred[:, 4] * pred[np.arange(len(pred)), 5 + class_ids]
            
            # Filter by per-class confidence thresholds and ignore null class (9)
            # Vectorised NumPy op — replaces a ~25 000-iteration Python for-loop
            keep_indices = (max_scores >= self._class_thresh_array[class_ids]) & (class_ids != 9)
            
            filtered_boxes = pred[keep_indices, 0:4]
            filtered_scores = max_scores[keep_indices]
            filtered_class_ids = class_ids[keep_indices]
            
            if len(filtered_boxes) > 0:
                # Convert from [x_center, y_center, w, h] to [x1, y1, x2, y2]
                x_center = filtered_boxes[:, 0]
                y_center = filtered_boxes[:, 1]
                w = filtered_boxes[:, 2]
                h = filtered_boxes[:, 3]
                
                x1 = x_center - w / 2.0
                y1 = y_center - h / 2.0
                x2 = x_center + w / 2.0
                y2 = y_center + h / 2.0
                
                boxes_x1y1x2y2 = np.stack([x1, y1, x2, y2], axis=1)
                
                # Perform Non-Maximum Suppression
                keep = self.nms(boxes_x1y1x2y2, filtered_scores, iou_threshold)
                final_boxes = boxes_x1y1x2y2[keep]
                final_scores = filtered_scores[keep]
                final_class_ids = filtered_class_ids[keep]

                # Perform Hybrid CV classification for traffic light color (Option 1)
                # Reuse bgr640 already computed above — no second resize needed
                h_img, w_img = bgr640.shape[:2]
                for idx, cid in enumerate(final_class_ids):
                    if cid in (6, 7, 8):  # Run CV color verification on green, red, or generic detections
                        box = final_boxes[idx]
                        x1_c = max(0, int(box[0]))
                        y1_c = max(0, int(box[1]))
                        x2_c = min(w_img, int(box[2]))
                        y2_c = min(h_img, int(box[3]))

                        if x2_c > x1_c and y2_c > y1_c:
                            crop = bgr640[y1_c:y2_c, x1_c:x2_c]
                            new_cid = self.classify_traffic_light_color(crop)
                            final_class_ids[idx] = new_cid
            else:
                final_boxes = np.empty((0, 4))
                final_scores = np.array([])
                final_class_ids = np.array([])

            # Rate-limited status print (once per second) for diagnostics
            now = time.time()
            if now - self._last_log_time > 1.0:
                max_score_val = float(np.max(max_scores)) if len(max_scores) > 0 else 0.0
                self.get_logger().info(
                    f"BPU Inference: received frame | max_score={max_score_val:.4f} | raw_det={len(filtered_boxes)} | post_nms={len(final_boxes)} | "
                    f"classes={list(final_class_ids)}"
                )
                self._last_log_time = now

            # Update detection states and publish updates
            self.update_detection_states(final_boxes, final_class_ids)
            self.publish_states()

            # Render debug frames if requested (pass pre-resized frame — no extra resize)
            if self._param_cache['show_debug']:
                self.draw_debug(bgr640, final_boxes, final_scores, final_class_ids)

        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')

    # ──────────────────────────────────────────────────────────────────────────
    # Temporal filtering / confidence gating
    # ──────────────────────────────────────────────────────────────────────────

    def update_detection_states(self, boxes: np.ndarray, class_ids: np.ndarray) -> None:
        """Applies hysteresis / temporal filtering on current detections.

        10-class model mapping:
          0: Bumper_signboard       → obstacle_pub
          1: Hill_signboard         → hill_pub
          2: Obstacle_signboard     → obstacle_pub
          3: ParallelP_signboard    → parking_pub
          4: PerpendP_signboard     → parking_pub
          5: RISAbotRemastered      → ignored
          6: Traffic_Green          → traffic_light 'green'
          7: Traffic_Red            → traffic_light 'red'
          8: Trafficlight_signboard → traffic_light (CV-reclassified in image_callback)
          9: null                   → ignored
        """

        # 1. Hill sign (Class 1: Hill_signboard)
        saw_hill = 1 in class_ids
        if saw_hill:
            self.detected_hill_consecutive = min(10, self.detected_hill_consecutive + 1)
            if self.detected_hill_consecutive >= 3:
                self.hill_sign_active = True
        else:
            self.detected_hill_consecutive = max(0, self.detected_hill_consecutive - 1)
            if self.detected_hill_consecutive == 0:
                self.hill_sign_active = False

        # 2. Parking sign (Class 3: ParallelP_signboard OR Class 4: PerpendP_signboard)
        # Optional: check if the bounding box meets minimum width constraints
        saw_parking = False
        min_width = int(self._param_cache['min_parking_sign_width'])

        for idx, cid in enumerate(class_ids):
            if cid in (3, 4):  # ParallelP_signboard or PerpendP_signboard
                if min_width > 0:
                    box = boxes[idx]
                    box_w = box[2] - box[0]
                    if box_w >= min_width:
                        saw_parking = True
                        break
                else:
                    saw_parking = True
                    break

        if saw_parking:
            self.detected_parking_consecutive = min(10, self.detected_parking_consecutive + 1)
            if self.detected_parking_consecutive >= 3:
                self.parking_sign_active = True
        else:
            self.detected_parking_consecutive = max(0, self.detected_parking_consecutive - 1)
            if self.detected_parking_consecutive == 0:
                self.parking_sign_active = False

        # 3. Obstacle sign (Class 0: Bumper_signboard OR Class 2: Obstacle_signboard)
        saw_obstacle = (0 in class_ids) or (2 in class_ids)
        if saw_obstacle:
            self.detected_obstacle_consecutive = min(10, self.detected_obstacle_consecutive + 1)
            if self.detected_obstacle_consecutive >= 3:
                self.obstacle_sign_active = True
        else:
            self.detected_obstacle_consecutive = max(0, self.detected_obstacle_consecutive - 1)
            if self.detected_obstacle_consecutive == 0:
                self.obstacle_sign_active = False

        # 4. Traffic light states
        # Class 8: Trafficlight_signboard (generic) — CV-reclassified in image_callback to 6 or 7
        # Class 6: Traffic_Green
        # Class 7: Traffic_Red
        # No yellow class in 10-class model
        saw_red = 7 in class_ids
        saw_green = 6 in class_ids

        if saw_red:
            self.detected_tl_red_consecutive = min(10, self.detected_tl_red_consecutive + 1)
            self.detected_tl_green_consecutive = 0
            self.detected_tl_yellow_consecutive = 0
            if self.detected_tl_red_consecutive >= 3:
                self.traffic_light_active = 'red'
        elif saw_green:
            self.detected_tl_green_consecutive = min(10, self.detected_tl_green_consecutive + 1)
            self.detected_tl_red_consecutive = 0
            self.detected_tl_yellow_consecutive = 0
            if self.detected_tl_green_consecutive >= 3:
                self.traffic_light_active = 'green'
        else:
            # Decay all states
            self.detected_tl_red_consecutive = max(0, self.detected_tl_red_consecutive - 1)
            self.detected_tl_green_consecutive = max(0, self.detected_tl_green_consecutive - 1)
            self.detected_tl_yellow_consecutive = max(0, self.detected_tl_yellow_consecutive - 1)

            if (self.detected_tl_red_consecutive == 0 and
                    self.detected_tl_green_consecutive == 0 and
                    self.detected_tl_yellow_consecutive == 0):
                self.traffic_light_active = 'unknown'

    def classify_traffic_light_color(self, crop: np.ndarray) -> int:
        """Analyze cropped traffic light region in HSV to identify the active state.

        Returns class IDs matching the 10-class model:
            6 for Traffic_Green, 7 for Traffic_Red, 8 for unknown/generic.
        """
        if crop is None or crop.size == 0:
            return 2

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        
        # Define color thresholds (HSV)
        # Red wraps around 0 and 180 in Hue
        lower_red1 = np.array([0, 80, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 80, 80])
        upper_red2 = np.array([180, 255, 255])
        
        # Yellow/Orange: broadened Hue range
        lower_yellow = np.array([11, 80, 80])
        upper_yellow = np.array([38, 255, 255])
        
        # Green
        lower_green = np.array([40, 80, 80])
        upper_green = np.array([90, 255, 255])
        
        # Generate masks and count active pixels
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_count = cv2.countNonZero(mask_red1) + cv2.countNonZero(mask_red2)
        
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_count = cv2.countNonZero(mask_yellow)
        
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        green_count = cv2.countNonZero(mask_green)
        
        # Determine dominant color
        # Map to 10-class model IDs: 6=Traffic_Green, 7=Traffic_Red
        # Yellow pixels map to red (nearest match — no yellow class in model)
        counts = {6: green_count, 7: red_count + yellow_count}
        best_cls, max_pixels = max(counts.items(), key=lambda x: x[1])

        # Require a minimum count of pixels to prevent noise trigger (e.g. 2% of area, min 10 pixels)
        total_pixels = crop.shape[0] * crop.shape[1]
        min_required = max(10, int(total_pixels * 0.02))
        if max_pixels >= min_required:
            return best_cls

        return 8  # Trafficlight_signboard generic / unknown

    # ──────────────────────────────────────────────────────────────────────────
    # Debug visualization publisher
    # ──────────────────────────────────────────────────────────────────────────

    def draw_debug(self, bgr640: np.ndarray, boxes: np.ndarray, scores: np.ndarray, class_ids: np.ndarray) -> None:
        """Overlay detection boxes/labels on the pre-resized 640x640 frame and publish debug stream."""
        debug_img = bgr640.copy()
        
        # 10-class model — must match Roboflow alphabetical export order
        CLASS_NAMES = [
            'Bumper_signboard',       # Class 0
            'Hill_signboard',         # Class 1
            'Obstacle_signboard',     # Class 2
            'ParallelP_signboard',    # Class 3
            'PerpendP_signboard',     # Class 4
            'Roundabout_signboard',   # Class 5 (Renamed from RISAbotRemastered)
            'Traffic_Green',          # Class 6
            'Traffic_Red',            # Class 7
            'Trafficlight_signboard', # Class 8
            'null',                   # Class 9
        ]

        COLOR_MAP = self._class_colors_cache  # use pre-built cache, not per-frame rebuild

        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = map(int, box)
            score = scores[i]
            cid = class_ids[i]

            name = CLASS_NAMES[cid] if cid < len(CLASS_NAMES) else f'class_{cid}'
            color = COLOR_MAP[cid] if cid < len(COLOR_MAP) else (255, 255, 255)

            # Draw bounding box
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)

            # Draw label — bigger text with black outline (no filled background)
            label = f'{name}: {score:.2f}'
            font       = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.65
            thickness  = 2
            # Black outline for readability
            cv2.putText(debug_img, label, (x1, y1 - 6), font, font_scale,
                        (0, 0, 0), thickness + 2, lineType=cv2.LINE_AA)
            # Colored text on top
            cv2.putText(debug_img, label, (x1, y1 - 6), font, font_scale,
                        color, thickness, lineType=cv2.LINE_AA)
            
        # Draw status summaries on top left
        summary_text = (
            f"HILL: {'ACTIVE' if self.hill_sign_active else 'OFF'} "
            f"| PARK: {'ACTIVE' if self.parking_sign_active else 'OFF'} "
            f"| TL: {self.traffic_light_active.upper()}"
        )
        cv2.putText(
            debug_img,
            summary_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0) if self.parking_sign_active or self.hill_sign_active else (255, 255, 255),
            2,
            lineType=cv2.LINE_AA
        )

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SignageDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
