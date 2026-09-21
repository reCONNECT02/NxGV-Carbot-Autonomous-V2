# unified14: the detector model (YOLO11n, 14 classes, RDK X5 BPU)

The model the car runs is `src/carbot_detectors/models/unified14_yolo11n_640x640_nv12.bin`
(installed with `carbot_detectors`, loaded by `bpu_detector`). This folder is where it
comes from and how to make the next one. What each class does on the car:
`docs/DETECTORS.md`.

| File | What it is |
|---|---|
| `README_DEPLOY_team.md` | The team's original deploy notes (for the old `risabot_automode` stack; **do not follow its install steps here**, they replace base files we keep unchanged) |
| `verify_unified16.py`, `test_*.jpg` | Board smoke test from the team package (runs without ROS) |
| `dataset.yaml` | Class order + dataset folder layout |
| `train_yolo11.py` | Training (Ultralytics YOLO11n) |
| `export_rdk_onnx.py` | `best.pt` -> ONNX with the 6-output head the BPU decoder expects |
| `make_calibration.py` | Calibration images for the quantiser |
| `rdk_bpu_config.yaml`, `compile_model.sh` | ONNX -> `.bin` with the Horizon Docker toolchain |

Known limits of the current model (team package notes): the boom classes had **0
validation images** (31 closed / 19 open in training), and the speed-bump sign is not
trained yet (disabled in `detectors.yaml`). Collect real footage from our Astra at the
venue before trusting either.

## 1. Check the model on the board (no ROS)

```bash
cd ~/NxGV-Carbot-Autonomous-V2/tools/bpu_model/unified14
python3 verify_unified16.py --model ../../../src/carbot_detectors/models/unified14_yolo11n_640x640_nv12.bin --image test_boom_open.jpg
python3 verify_unified16.py --model ../../../src/carbot_detectors/models/unified14_yolo11n_640x640_nv12.bin --image test_sign.jpg
```

Both must end with `SMOKE PASS`. Run them from **inside this folder** (running
`python3 tools/...` from your home folder fails with "can't open file").

## 2. Dataset

YOLO format, one label file per image, same class order as `dataset.yaml`:

```
datasets/unified14/images/{train,val}/*.jpg
datasets/unified14/labels/{train,val}/*.txt     # class cx cy w h (0..1)
```

Record images from the car: GUI Recording tab (phase 7) or
`ros2 bag record /camera/color/image_raw`, then extract frames. Put boom gate images
(open AND closed, several distances and angles) in `val/` too.

## 3. Train, export, compile

```bash
pip install ultralytics==8.3.* onnx onnxsim
python train_yolo11.py --data dataset.yaml --epochs 150
python export_rdk_onnx.py runs/detect/unified14/weights/best.pt   # prints 6 output shapes
cp runs/detect/unified14/weights/best_rdk.onnx .
python make_calibration.py datasets/unified14/images/train calibration_data --count 100
./compile_model.sh                                                 # Docker must be running
```

`export_rdk_onnx.py` and `compile_model.sh` follow the D-Robotics / base-repo method
but have **not** been run in CI. Check that the export prints six outputs:
14 channels (classes) and 64 channels (boxes) at grids 80, 40 and 20.

## 4. Deploy

1. Run step 1 on the new `model_output/unified14_yolo11n_640x640_nv12.bin`.
2. Copy it over `src/carbot_detectors/models/` (same name), or give it a new name
   and change `model_path` in `detectors.yaml`.
3. If classes changed: update BOTH `class_names` and `class_map` in `detectors.yaml`.
   `bpu_detector` refuses to run (status `MODEL_MISMATCH`) if the model's class count
   does not match `class_names`.
4. `colcon build --packages-select carbot_detectors`, then check the GUI Detections tab.

## Roboflow key

Never write the key into a file. Use `ROBOFLOW_API_KEY=... python train_yolo11.py
--roboflow WORKSPACE/PROJECT/VERSION`. The pre-commit hook refuses committed keys.
