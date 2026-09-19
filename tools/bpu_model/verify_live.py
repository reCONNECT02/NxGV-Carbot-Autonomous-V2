#!/usr/bin/env python3
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

print("=== Live BPU Model Diagnostic Script ===")

# 1. Import BPU
try:
    try:
        from hobot_dnn import pyeasy_dnn as dnn
    except ImportError:
        from hobot_dnn_rdkx5 import pyeasy_dnn as dnn
    print("Successfully imported pyeasy_dnn.")
except ImportError as e:
    print(f"ERROR: Failed to import BPU library: {e}")
    sys.exit(1)

# 2. Load model
model_path = "/home/sunrise/risabot_signs_640x640_nv12.bin"
try:
    models = dnn.load(model_path)
    model = models[0]
    print("Successfully loaded BPU model.")
except Exception as e:
    print(f"ERROR: Failed to load model: {e}")
    sys.exit(1)


class FrameGrabber(Node):
    def __init__(self, model):
        super().__init__('verify_live_node')
        self.model = model
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_cb,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.frame_received = False
        print("Subscribed to /camera/color/image_raw. Waiting for a frame...")

    def bgr_to_nv12(self, bgr: np.ndarray) -> np.ndarray:
        resized = cv2.resize(bgr, (640, 640), interpolation=cv2.INTER_LINEAR)
        yuv = cv2.cvtColor(resized, cv2.COLOR_BGR2YUV_I420)
        y = yuv[0:640, :]
        u = yuv[640:800, :]
        v = yuv[800:960, :]
        
        u_flat = u.reshape(-1)
        v_flat = v.reshape(-1)
        
        uv_interleaved = np.zeros(len(u_flat) + len(v_flat), dtype=np.uint8)
        uv_interleaved[0::2] = u_flat
        uv_interleaved[1::2] = v_flat
        uv_planar = uv_interleaved.reshape(320, 640)
        return np.vstack((y, uv_planar))

    def image_cb(self, msg: Image):
        if self.frame_received:
            return
        self.frame_received = True
        print(f"Received a live frame ({msg.width}x{msg.height}). Running BPU inference...")
        
        try:
            # Preprocess
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            nv12 = self.bgr_to_nv12(bgr)
            
            # Forward pass
            outputs = self.model.forward([nv12])
            pred = outputs[0].buffer
            
            # Reshape/squeeze
            if len(pred.shape) > 2:
                pred = np.squeeze(pred)
                
            print(f"Prediction tensor squeezed shape: {pred.shape}")
            
            # YOLOv5 outputs: box coordinates [0:4], objectness [4], class scores [5:]
            # Calculate absolute score = objectness * class_probability
            scores = pred[:, 4:5] * pred[:, 5:]
            class_ids = np.argmax(pred[:, 5:], axis=1)
            max_scores = pred[:, 4] * pred[np.arange(len(pred)), 5 + class_ids]
            
            # Print diagnostic stats
            overall_max = np.max(max_scores)
            print(f"\n--- Diagnostic Results ---")
            print(f"Maximum absolute score in the frame: {overall_max:.6f}")
            
            # Count anchors passing different thresholds
            for th in [0.01, 0.05, 0.10, 0.20, 0.30, 0.40]:
                count = np.sum(max_scores >= th)
                print(f"  Anchors with score >= {th:.2f}: {count}")
                
            # Print top 5 highest scoring anchors
            top_indices = np.argsort(max_scores)[::-1][:5]
            print(f"\n--- Top 5 Highest Scoring Anchors ---")
            for rank, idx in enumerate(top_indices):
                score = max_scores[idx]
                box = pred[idx, 0:4]
                obj_conf = pred[idx, 4]
                cls_probs = pred[idx, 5:]
                best_cls = class_ids[idx]
                print(f"Rank {rank+1}: Anchor #{idx} | Score={score:.4f}")
                print(f"  Box   : [x={box[0]:.1f}, y={box[1]:.1f}, w={box[2]:.1f}, h={box[3]:.1f}]")
                print(f"  Obj   : {obj_conf:.4f}")
                print(f"  Classes probabilities: {[round(float(p), 4) for p in cls_probs]}")
                print(f"  Best Class ID: {best_cls}")
                
        except Exception as e:
            print(f"ERROR during processing: {e}")
            
        # Shutdown after processing first frame
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FrameGrabber(model)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
