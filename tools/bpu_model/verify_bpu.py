#!/usr/bin/env python3
import sys
import numpy as np

print("=== BPU Model Verification Script ===")

# 1. Import hobot_dnn / pyeasy_dnn
try:
    try:
        from hobot_dnn import pyeasy_dnn as dnn
        print("Successfully imported pyeasy_dnn from hobot_dnn")
    except ImportError:
        from hobot_dnn_rdkx5 import pyeasy_dnn as dnn
        print("Successfully imported pyeasy_dnn from hobot_dnn_rdkx5")
except ImportError as e:
    print(f"ERROR: Failed to import BPU library: {e}")
    sys.exit(1)

# 2. Load the model
model_path = "/home/sunrise/risabot_signs_640x640_nv12.bin"
print(f"Attempting to load BPU model from: {model_path}")

try:
    models = dnn.load(model_path)
    model = models[0]
    print("SUCCESS: BPU model loaded successfully.")
except Exception as e:
    print(f"ERROR: Failed to load BPU model: {e}")
    sys.exit(1)

# 3. Print model input / output properties
try:
    print("\n--- Model Inputs ---")
    for idx, inp in enumerate(model.inputs):
        print(f"  Input[{idx}]: name='{inp.name}', shape={inp.properties.shape}, layout={inp.properties.layout}, type={inp.properties.tensor_type}")
        
    print("\n--- Model Outputs ---")
    for idx, out in enumerate(model.outputs):
        print(f"  Output[{idx}]: name='{out.name}', shape={out.properties.shape}, layout={out.properties.layout}, type={out.properties.tensor_type}")
except Exception as e:
    print(f"Warning: Could not query model properties: {e}")

# 4. Generate dummy NV12 input
# A 640x640 NV12 image has size 640 * 640 * 1.5 = 614400 bytes, representing (960, 640) shape
dummy_nv12 = np.zeros((960, 640), dtype=np.uint8)
print(f"\nCreated dummy NV12 input tensor of shape: {dummy_nv12.shape}")

# 5. Run inference
print("Running forward inference pass on BPU...")
try:
    outputs = model.forward([dummy_nv12])
    print("SUCCESS: Forward inference completed.")
    
    # Process outputs
    pred = outputs[0].buffer
    print(f"Output raw prediction buffer shape: {pred.shape}")
    print(f"Sample prediction data (first 5 detections):\n{pred[0][:5] if len(pred.shape) == 3 else pred[:5]}")
except Exception as e:
    print(f"ERROR: Inference forward pass failed: {e}")
    sys.exit(1)

print("\n=== Verification Successful! BPU model is fully functional. ===")