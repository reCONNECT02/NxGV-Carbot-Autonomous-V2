# RDK X5 dual MIPI camera setup — verified working (2026-09-18)

## Hardware
- Board: D-Robotics RDK X5, hostname `risabot1`, user `sunrise`, IP 10.168.5.164
- OS: Ubuntu 22.04.5, kernel 6.1.83 aarch64, TROS Humble
- Camera A: OV5647 (Rev 1.3) → MIPI host 2, I2C bus 4, addr 0x36
- Camera B: IMX219 (v2.1) → MIPI host 0, I2C bus 6, addr 0x10
- Both sensors auto-detected by mipi_cam; native config 1920x1080 raw10 30fps 2-lane

## Packages
- tros-humble-mipi-cam 2.5.2, tros-humble-hobot-codec 2.3.5, tros-humble-websocket 2.3.2
- ~950 system packages still pending upgrade (not yet done; camera works anyway)

## Working commands (each in its own terminal, all as root)
Every terminal first: `sudo -i` then `source /opt/tros/humble/setup.bash`

1. OV5647:  ros2 run mipi_cam mipi_cam --ros-args -r __ns:=/cam_ov5647 -p channel:=2 -p image_width:=960 -p image_height:=544
2. IMX219:  ros2 run mipi_cam mipi_cam --ros-args -r __ns:=/cam_imx219 -p channel:=0 -p image_width:=960 -p image_height:=544
3. Encoder: ros2 launch hobot_codec hobot_codec_encode.launch.py codec_in_mode:=ros codec_in_format:=bgr8 codec_sub_topic:=/cam_imx219/image_raw codec_pub_topic:=/image_jpeg
4. Web:     ros2 launch websocket websocket.launch.py websocket_image_topic:=/image_jpeg websocket_only_show_image:=true
   → view at http://10.168.5.164:8000 → "Web display"

Published topics: /cam_ov5647/image_raw and /cam_imx219/image_raw (sensor_msgs/Image, bgr8, 960x544, ~30 fps)

## Gotchas found
- Must run as root; as normal user the node fails with `create_and_run_vflow failed`.
- With `ros2 run`, always set image_width/height. Default is 1088x1280, which makes
  `creat_vse_node failed, ret -10` (scaler can't output taller than the 1080p source).
- `There are no available host` = that camera port is already in use by a running node.
- Ctrl+C / closing terminal doesn't always kill nodes. Check with
  `ps -ef | grep -E "codec|websocket|mipi" | grep -v grep` and `kill <PID>`.
- Web page shows one camera at a time. To switch: stop encoder AND websocket,
  restart encoder with the other topic, restart websocket, hard-refresh (Ctrl+Shift+R).
- "get camera calibration parameters failed" warning is harmless (no calibration yet).
- `RCLError: context is invalid` after a failure is only a side effect, not the root cause.

## Not done yet
- No lens calibration files for either camera
- Not auto-starting at boot
- No side-by-side view of both cameras
- System packages not upgraded