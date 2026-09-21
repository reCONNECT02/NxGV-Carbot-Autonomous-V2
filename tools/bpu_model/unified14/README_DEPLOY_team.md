# Unified14 YOLO11n RDK X5 Deployment

This package contains the clean 14-class checkpoint trained in `yolo11n_unified14_clean_v1`.

Final 14-class map (`unified16.yaml`):

- `0-6` signs (`end_of_tunnel`, `hill`, `obstacle`, `parallel`, `perpendicular`, `roundabout`, `speed_bump`)
- `7 traffic_signals_ahead_sign`
- `8 tunnel_sign`
- `9 traffic_red`, `10 traffic_yellow`, `11 traffic_green`
- `12 boom_closed`, `13 boom_open`

Important: boom validation currently has `0` images (`31 closed` / `19 open` in train). Keep `publish_boom_state: false` and retain the existing LiDAR boom detector until real camera footage has been collected and validated.

## Package Contents

- `unified14_yolo11n_640x640_nv12.bin`: compiled RDK X5 BPU model
- `ros/signage_detector.py`: 14-class BPU decoder and existing ROS topic integration
- `ros/params_nxgv_signage.yaml`: detector parameters
- `ros/dashboard_templates.py`: dashboard compatibility file from the prior deployment
- `board_tools/verify_unified16.py`: board-side BPU smoke test
- `board_tools/test_boom_open.jpg`: known validation image
- `board_tools/test_sign.jpg`: known sign validation image
- `SHA256SUMS.txt`: package file hashes

## 1. Back Up the Board

Run on the board before replacing anything:

```bash
cp /home/sunrise/nxgv_yolo11n_640x640_nv12.bin ~/nxgv_yolo11n_640x640_nv12.bin.bak
cp ~/risabotcar_ws/src/risabot_automode/risabot_automode/signage_detector.py ~/signage_detector.py.bak
cp ~/risabotcar_ws/src/risabot_automode/config/params.yaml ~/params.yaml.bak
```

## 2. Copy and Smoke-Test the Model

From the computer containing this extracted package:

```bash
scp unified14_yolo11n_640x640_nv12.bin board_tools/verify_unified16.py board_tools/test_boom_open.jpg board_tools/test_sign.jpg sunrise@<RDK_IP>:/home/sunrise/
```

On the board:

```bash
cd /home/sunrise
python3 verify_unified16.py --model unified14_yolo11n_640x640_nv12.bin --image test_boom_open.jpg
python3 verify_unified16.py --model unified14_yolo11n_640x640_nv12.bin --image test_sign.jpg
```

Both commands must end with `SMOKE PASS`. Stop if model loading, output count, or inference fails.

## 3. Install the ROS Files

From the extracted package on the development computer, copy the ROS files:

```bash
scp ros/signage_detector.py ros/dashboard_templates.py sunrise@<RDK_IP>:~/risabotcar_ws/src/risabot_automode/risabot_automode/
scp ros/params_nxgv_signage.yaml sunrise@<RDK_IP>:/home/sunrise/
```

`signage_detector.py` replaces:

```text
~/risabotcar_ws/src/risabot_automode/risabot_automode/signage_detector.py
```

On the board, replace the existing `signage_detector:` section in `~/risabotcar_ws/src/risabot_automode/config/params.yaml` with the section from `/home/sunrise/params_nxgv_signage.yaml`. Do not append a second section with the same name.

Keep this setting until boom-camera validation is complete:

```yaml
publish_boom_state: false
```

Build and launch with the wheels raised:

```bash
cd ~/risabotcar_ws
colcon build --packages-select risabot_automode
source install/setup.bash
ros2 launch risabot_automode bringup.launch.py
```

Expected messages include:

```text
Unified Detector (YOLO11n 14-class) initialized.
BPU model loaded successfully.
```

Enable the debug image only during static testing:

```bash
ros2 param set /signage_detector show_debug true
```

## 4. Roll Back

```bash
cp ~/nxgv_yolo11n_640x640_nv12.bin.bak /home/sunrise/nxgv_yolo11n_640x640_nv12.bin
cp ~/signage_detector.py.bak ~/risabotcar_ws/src/risabot_automode/risabot_automode/signage_detector.py
cp ~/params.yaml.bak ~/risabotcar_ws/src/risabot_automode/config/params.yaml
cd ~/risabotcar_ws
colcon build --packages-select risabot_automode
```
