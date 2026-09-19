# RISAbot YOLOv5s BPU Training Guide & Colab Script
# 
# Follow the cells below in Google Colab (with a GPU run time) to train,
# export, and package your custom YOLOv5s model for the RDK X5 BPU.
#
# ==============================================================================
# CELL 1: CLONE YOLOV5 v7.0 & INSTALL DEPENDENCIES
# ==============================================================================
"""
# Clone the exact YOLOv5 v7.0 release to ensure hb_mapper compatibility
!git clone --branch v7.0 https://github.com/ultralytics/yolov5.git
%cd yolov5
!pip install -qr requirements.txt
!pip install roboflow numpy pillow opencv-python
"""

# ==============================================================================
# CELL 2: DOWNLOAD DATASET FROM ROBOFLOW
# ==============================================================================
"""
from roboflow import Roboflow
import os

# Initialize Roboflow client.
rf = Roboflow(api_key="wSO9oU6yFMszgFn0Aulv")

# Download project dataset in YOLOv5 format
project = rf.workspace("shamsuls-workspace").project("risabot")
dataset = project.version(2).download("yolov5")

# Let's define the dataset path
dataset_dir = dataset.location
print(f"Dataset downloaded to: {dataset_dir}")
"""

# ==============================================================================
# CELL 3: RE-SPLIT DATASET (Fixes the 1-image validation split issue)
# ==============================================================================
"""
import glob
import os
import random
import shutil

# Roboflow export might place all annotations/images in train/ and only 1 in valid/.
# We'll merge valid/ into train/ first, then split 20% into valid/ to ensure stable evaluation.

train_img_dir = os.path.join(dataset_dir, "train", "images")
train_lbl_dir = os.path.join(dataset_dir, "train", "labels")
val_img_dir = os.path.join(dataset_dir, "valid", "images")
val_lbl_dir = os.path.join(dataset_dir, "valid", "labels")

# Ensure valid directories exist
os.makedirs(val_img_dir, exist_ok=True)
os.makedirs(val_lbl_dir, exist_ok=True)

# 1. Move everything from valid to train to start fresh
val_images = glob.glob(os.path.join(val_img_dir, "*"))
for img_path in val_images:
    filename = os.path.basename(img_path)
    label_filename = os.path.splitext(filename)[0] + ".txt"
    lbl_path = os.path.join(val_lbl_dir, label_filename)
    
    # Move image
    dest_img = os.path.join(train_img_dir, filename)
    if os.path.exists(img_path):
        shutil.move(img_path, dest_img)
    # Move label
    dest_lbl = os.path.join(train_lbl_dir, label_filename)
    if os.path.exists(lbl_path):
        shutil.move(lbl_path, dest_lbl)

# 2. Get list of all training images
all_train_images = glob.glob(os.path.join(train_img_dir, "*"))
random.seed(42)
random.shuffle(all_train_images)

# Let's take 20% for validation (about 50 images out of 250)
val_split_count = int(len(all_train_images) * 0.20)
val_images_to_move = all_train_images[:val_split_count]

print(f"Total merged train images: {len(all_train_images)}")
print(f"Moving {val_split_count} images to validation split...")

for img_path in val_images_to_move:
    filename = os.path.basename(img_path)
    label_filename = os.path.splitext(filename)[0] + ".txt"
    lbl_path = os.path.join(train_lbl_dir, label_filename)
    
    # Move image to valid/images/
    shutil.move(img_path, os.path.join(val_img_dir, filename))
    # Move label to valid/labels/
    if os.path.exists(lbl_path):
        shutil.move(lbl_path, os.path.join(val_lbl_dir, label_filename))

print(f"Split complete. Train images: {len(glob.glob(os.path.join(train_img_dir, '*')))}, Valid images: {len(glob.glob(os.path.join(val_img_dir, '*')))}")
"""

# ==============================================================================
# CELL 4: WRITE DATA.YAML CONFIG
# ==============================================================================
"""
import yaml

# Explicitly define our 6 target classes (mapped to index 0..5 in YOLO annotations)
data_yaml_content = {
    'train': train_img_dir,
    'val': val_img_dir,
    'nc': 6,
    'names': [
        'hill_sign',             # Class 0
        'parking_sign',          # Class 1
        'traffic_light',         # Class 2
        'traffic_light_green',   # Class 3
        'traffic_light_red',     # Class 4
        'traffic_light_yellow'   # Class 5
    ]
}

data_yaml_path = os.path.join(dataset_dir, "data.yaml")
with open(data_yaml_path, 'w') as f:
    yaml.dump(data_yaml_content, f, default_flow_style=None)

print(f"Updated data.yaml written to: {data_yaml_path}")
"""

# ==============================================================================
# CELL 4.5: PATCH TORCH.LOAD FOR PYTORCH 2.6+ COMPATIBILITY
# ==============================================================================
"""
# PyTorch 2.6+ changed the default value of weights_only to True, which crashes YOLOv5 loading.
# This script patches train.py and export.py to set weights_only=False globally.

patch_code = '''import torch
_org_load = torch.load
def _patched_load(*args, **kwargs):
    if 'weights_only' not in kwargs:
        kwargs['weights_only'] = False
    return _org_load(*args, **kwargs)
torch.load = _patched_load
'''

import os

# Ensure we are in the yolov5 directory
if os.path.basename(os.getcwd()) != 'yolov5':
    if os.path.exists('yolov5'):
        os.chdir('yolov5')
        print(f"Changed working directory to: {os.getcwd()}")
    else:
        print("[WARNING] 'yolov5' folder not found in current directory. Make sure Cell 1 ran successfully.")

for filename in ['train.py', 'export.py']:
    if os.path.exists(filename):
        with open(filename, 'r') as f:
            code = f.read()
        if 'patched_load' not in code:
            print(f"Patching {filename} for PyTorch 2.6+ compatibility...")
            with open(filename, 'w') as f:
                f.write(patch_code + "\\n" + code)
    else:
        print(f"[ERROR] Could not find {filename} at {os.getcwd()}")
"""

# ==============================================================================
# CELL 5: TRAIN YOLOV5s MODEL
# ==============================================================================
"""
# Train YOLOv5s on custom dataset.
# Cytron tutorial recommendations: --weights yolov5s.pt --hyp data/hyps/hyp.VOC.yaml
!python train.py --img 640 --batch-size 8 --epochs 150 --data {data_yaml_path} \
  --weights yolov5s.pt --hyp data/hyps/hyp.VOC.yaml --patience 30 --name risabot_signs
"""

# ==============================================================================
# CELL 6: EXPORT TO ONNX (OPSET 11)
# ==============================================================================
"""
# The RDK X5 hb_mapper requires opset 11, and we disable dynamo to use the legacy exporter
!python export.py --weights runs/train/risabot_signs/weights/best.pt \
  --img 640 --include onnx --opset 11
"""

# ==============================================================================
# CELL 7: GENERATE CALIBRATION DATA (1x3x640x640 uint8 CHW Binaries)
# ==============================================================================
"""
import cv2
import numpy as np

# BPU compiler needs raw binary calibration files instead of JPEG/NumPy.
# Format: CHW uint8, size is 1 * 3 * 640 * 640 = 1,228,800 bytes.

calibration_dir = "/content/calibration_data"
os.makedirs(calibration_dir, exist_ok=True)

# Select 50 training images for calibration
train_images = glob.glob(os.path.join(train_img_dir, "*"))
random.shuffle(train_images)
selected_cal_images = train_images[:50]

print(f"Generating {len(selected_cal_images)} raw binary calibration files in {calibration_dir}...")

for idx, img_path in enumerate(selected_cal_images):
    # Read image
    img = cv2.imread(img_path)
    if img is None:
        continue
    # Convert from BGR to RGB
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # Resize to model input shape
    img_resized = cv2.resize(img_rgb, (640, 640), interpolation=cv2.INTER_LINEAR)
    # HWC -> CHW
    img_chw = np.transpose(img_resized, (2, 0, 1)) # shape: (3, 640, 640)
    # Ensure type is uint8
    img_chw = img_chw.astype(np.uint8)
    
    # Write raw binary representation to file
    bin_filename = f"calib_{idx:03d}.bin"
    bin_path = os.path.join(calibration_dir, bin_filename)
    img_chw.tofile(bin_path)

print("Calibration data generation complete.")
"""

# ==============================================================================
# CELL 8: PACKAGE EVERYTHING FOR DOWNLOAD
# ==============================================================================
"""
# Zip the ONNX model and calibration data for easy downloading
!zip -q -r /content/bpu_package.zip \
  /content/yolov5/runs/train/risabot_signs/weights/best.onnx \
  /content/calibration_data

print("---------------------------------------------------------------")
print("Done! Download /content/bpu_package.zip from Colab.")
print("Unzip this package on your Windows machine to build the BPU bin!")
print("---------------------------------------------------------------")
"""
