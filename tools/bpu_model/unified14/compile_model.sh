#!/usr/bin/env bash
# Compile best_rdk.onnx -> model_output/unified14_yolo11n_640x640_nv12.bin with the
# Horizon Open Explorer Docker toolchain (the image the base repo used). Linux / WSL.
# Windows without WSL: run the same docker command from PowerShell in this folder.
set -euo pipefail
cd "$(dirname "$0")"
[[ -f best_rdk.onnx ]] || { echo "best_rdk.onnx missing (export_rdk_onnx.py)"; exit 1; }
[[ -d calibration_data ]] || { echo "calibration_data/ missing (make_calibration.py)"; exit 1; }
docker run --rm -v "$PWD":/workspace -w /workspace \
  openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8 \
  hb_mapper makertbin --config rdk_bpu_config.yaml --model-type onnx
echo "OK: model_output/unified14_yolo11n_640x640_nv12.bin"
echo "Next: verify on the board (verify_unified16.py), then copy it to src/carbot_detectors/models/"
