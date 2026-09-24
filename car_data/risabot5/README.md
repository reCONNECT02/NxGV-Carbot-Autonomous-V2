# risabot5: calibration data, snapshot 2026-09-24 (evening)

Copied from `/home/sunrise/carbot_data` on risabot5 (RDK X5) so the work is not lost when moving to another
bot. Code: branch `phase8/complete-calibration`. Nothing here is a secret.

## `calibration/20260924_103632` = the session to resume (the wizard auto-resumes it within 12 h)
| Step | Status | Notes |
|---|---|---|
| 1 sensor health | PASS | front Astra only (side cameras removed, MIPI lanes never worked) |
| 2 camera identity | PASS | front confirmed |
| 3 camera intrinsics | PASS | Astra 640x480 @ 15 fps (`intrinsics/astra.yaml`) |
| 4 extrinsics + IPM | PASS | mount x 0.212 y 0.015 z 0.083 (hfov 56.0); board sheet `axle_offset_m` was 0.150 then, now 0.065 in the YAML: REDO if the car was really 65 mm back (BACKLOG #54) |
| 5 LiDAR-camera | PASS | depends on step 4 |
| 6 IMU + odometry | PASS | `ticks_per_meter` 299600, polarity reversed, `imu_yaw_scale` 1.0 (`params_overlay.yaml`) - done with the OLD motors' behaviour; new motors gave ~300000 as well (driven) |
| 7 servo centre + steering | NOT DONE | left radius 0.48 m (range 68), right 0.79 m (stops mechanically at ~15 deg), straight drift 3.3 cm/m; car needs a kick to start (BACKLOG #63) |
| 8-13 | NOT DONE | speed PID must be redone for the new motors |

## Live-only values on the car (NOT saved anywhere, a relaunch resets them; `~/lite5.sh` re-applies them)
`servo_range_left` 74, `servo_range_right` 70, `servo_center` 84 (found by step 7's straight runs; YAML default 90).

## Other cars
`calibration/20260923_101521` and `20260924_013152` are earlier sessions (320x240 intrinsics, old setups): kept, not used.
`load/*.csv` = CPU/temperature measurements (`tools/measure_load.py`), see BACKLOG #47-#52.

## Starting risabot1 (or any other car)
This data does NOT transfer: intrinsics, mounts, ticks/m, servo centre/ranges belong to the car. Use the code branch,
`tools/car_ops/README.md` for the helper scripts and lessons, and calibrate from step 1.
