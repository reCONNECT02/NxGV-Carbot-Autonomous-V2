# risabot5: calibration results (2026-09-24)

Everything measured or decided during the calibration wizard on **risabot5** (RDK X5), written for someone who
will use the numbers in his own code. Source files: `car_data/risabot5/calibration/20260924_103632/` on branch
`data/risabot5-calibration`; code on `phase8/complete-calibration`.

**These numbers belong to this car.** Another car needs its own calibration.

## How to read the status column

| Tag | Meaning |
|---|---|
| **SAVED** | measured by a wizard step that PASSED and was saved in the session |
| **LIVE** | measured / set on the car but never saved; a relaunch resets it |
| **PROVISIONAL** | measured, but the step failed or the value is known to be off; use with care |
| **ASSUMED** | not measured: a default from the YAML or the team |

---

## 0. Frames and sign conventions

* `base_link`: origin at the **centre of the rear axle on the ground**, `+x` forward, `+y` left, `+z` up (ROS REP-103).
* Angles: `+` = counter-clockwise seen from above, so **steering `+` = turn LEFT** in ROS messages (`MotionRequest.steer_rad`).
* The base `servo_controller` takes `angular.z` on `/cmd_vel_auto` as *normalised steering*; on this car
  **`steering.steer_sign = -1.0`**: a **negative** `angular.z` is a **left** turn (verified by the step 7 direction check:
  left lock turned the IMU yaw `+194°`, right lock `-168°`).
* Images: `/camera/color/image_raw`, **640 x 480, 15 Hz, published as `bgr8`** (the YAML says `rgb8`; the driver really
  publishes `bgr8`).
* Vehicle: length 0.30 m, width 0.192 m, wheelbase **0.216 m** (measured), rear overhang 0.042 m (assumed),
  wheel radius 0.033 m (measured), height 0.24 m (`common.yaml vehicle.*`).

---

## 1. Front camera (Astra Pro): intrinsics. SAVED

Step 3, 2026-09-24 10:40, file `intrinsics/astra.yaml` (ROS `camera_info` format). 18 views, all 9 image areas covered,
**reprojection RMS 0.468 px** (limit 0.8).

| Quantity | Value |
|---|---|
| Image size | 640 x 480 |
| Model | `plumb_bob` (OpenCV order `[k1, k2, p1, p2, k3]`) |
| `fx`, `fy` | 592.069, 595.056 px |
| `cx`, `cy` | 317.128, 239.389 px |
| Distortion | `k1 = 0.028034`, `k2 = 0.546394`, `p1 = -0.001158`, `p2 = -0.007166`, `k3 = -1.627180` |
| Horizontal FOV | **56.03 deg** |
| Fisheye fit (for comparison) | RMS 0.473 px (pinhole/plumb_bob is used) |

```yaml
camera_matrix: [[592.0687, 0.0, 317.1276], [0.0, 595.0560, 239.3887], [0.0, 0.0, 1.0]]
distortion_coefficients: [0.0280337, 0.5463935, -0.0011576, -0.0071660, -1.6271797]
```

**Caution:** `k2` and `k3` are large and of opposite sign (0.55 / -1.63). At the picture corners the undistortion
is dominated by them. The RMS is good, but check `cv2.undistort` on a real frame before trusting the outer 10 % of the
picture. The intrinsics are only valid at **640 x 480**: at any other size they must be re-measured.

---

## 2. Front camera: mounting (extrinsics + IPM). SAVED, one caveat

Step 4, 2026-09-24 10:41 (saved 10:44). Solved from one 6 x 4 inner-corner, 50 mm chessboard on the floor.
Pose of the camera in `base_link`:

| | x | y | z | yaw | pitch (down) | roll |
|---|---|---|---|---|---|---|
| **Solved** | **0.212 m** | **0.015 m** | **0.0828 m** | **0.79 deg** | **3.15 deg** | **0.23 deg** |
| CAD nominal | 0.217 m | 0.0114 m | 0.0935 m | 0.0 deg | 2.5 deg | 0.0 deg |

* Difference from the CAD: 1.24 cm in position, 1.04 deg in angle.
* Ground-projection error against the printed board: **RMS 3.9 mm, max 7.3 mm** (limit 20 mm); board-corner
  reprojection 0.735 px. Corner order was `flip_cols`.
* Saved in `data/cameras.yaml` under `mounts.front` (with `hfov_deg: 56.03`) and `extrinsics_calibrated: true`.

**Caveat (BACKLOG #54):** the floor sheet's board position is `centre_m + axle_offset_m`. When this step was saved the
YAML said `axle_offset_m: 0.150`; the owner later measured the car at **65 mm** and the YAML now says **0.065**. If
the car really stood 65 mm back, the mount `x` above is off by about 85 mm. The solved `x` (0.212) is within 5 mm
of the CAD (0.217), which fits an offset that was right at the time, but it is **not verified**. Re-run step 4 with
the final sheet before relying on `x` to better than a few centimetres.

---

## 3. LiDAR (YDLidar T-mini Plus) to base_link and to camera. SAVED, one warning

Step 5, 2026-09-24 10:53. Three target positions, spread 30.5 deg.

| | Value |
|---|---|
| `base_to_laser` `[x, y, z, yaw, pitch, roll]` | **`[0.0, 0.0, 0.12, 3.09731, 0.0, 0.0]`** (metres / radians) |
| Yaw correction found | **-2.537 deg** (from `3.14159` = pi to `3.09731` rad = 177.46 deg) |
| Bearing residual | 1.91 deg (limit 2.0) |
| Also written | `tunnel_wall_follower.lidar_angle_offset: 3.09731` |

The yaw of about pi is intended: the driver runs with `reversion: true`, so scan angle 0 points to the **rear** of the car.

| Capture | LiDAR xy | Ground (camera) xy | Bearing | Error | Residual | cam range | LiDAR range |
|---|---|---|---|---|---|---|---|
| 1 | (-0.724, 0.211) | (0.817, -0.250) | -17.0 deg | -0.75 deg | 1.78 deg | 0.854 m | 0.754 m |
| 2 | (-0.742, 0.011) | (0.872, -0.050) | -3.3 deg | -2.41 deg | 0.12 deg | 0.874 m | 0.742 m |
| 3 | (-0.762, -0.246) | (0.890, 0.213) | 13.4 deg | -4.44 deg | -1.91 deg | 0.915 m | 0.801 m |

**Warning from the wizard:** the camera and LiDAR distances to the target differ by up to **0.13 m** (limit 0.08). The
yaw result is valid, but the LiDAR **x** position is wrong: `base_to_laser` has `x = 0.0`, `common.yaml` still has the
V4 value `lidar_x_m: 0.09`. **Measure the LiDAR position on the car** (rear axle to LiDAR centre) and fix it.

---

## 4. Wheel odometry and IMU (servo_controller). SAVED

Step 6, saved 2026-09-24 14:14. Written to `params_overlay.yaml` under `servo_controller`:

```yaml
servo_controller:
  ros__parameters: {ticks_per_meter: 299599.9, odom_reverse_polarity: true, imu_yaw_scale: 1.0}
```

| Parameter | Value | Meaning |
|---|---|---|
| `ticks_per_meter` | **299599.9** | odometry distance = encoder ticks / this |
| `odom_reverse_polarity` | **true** | forward motion counts negative in the raw encoder, so it is flipped |
| `imu_yaw_scale` | **1.0** | IMU yaw needs no scaling |
| `drive_motor_index` | 0 | encoder channel used (YAML default) |

**Verification (2.00 m tape; the last two runs were driven with the joystick):**

| Run | Odom | Error |
|---|---|---|
| Distance, run 1 (pass) | 2.0149 m | +0.75 % |
| Distance, run 2 (pass) | 1.9831 m | -0.84 % |
| Spin, one full left turn | +356.19 deg | -1.06 % (limit 3 %) |
| IMU heading drift, car still | **0.00 deg/min** over 59.5 s (limit 2) | |

`/odom` runs at about 20 Hz and `/imu/rpy` at about 19.4 Hz (1155 messages in 59.5 s).

**Convergence history (shows why the number is odd):** the first runs read -10.67 m, +10.01, +11.26, +9.26, +4.36, +2.05,
+1.96 m with `ticks_per_meter` 1050, 1050, 5258, 29610, 137017, 298472, 306346, then 299600 and two passes. The scale
factor is **empirical**, about 62,000 ticks per wheel turn (wheel circumference 0.207 m), far more than a normal
encoder gives. Treat it as a fudge factor for this car and firmware, not as a physical encoder resolution.

**Requirements for using it (important):**

* `servo_controller.encoder_jump_threshold` **must be 100000** (set in `base_nodes.yaml`). The base default 800 drops
  almost every reading at this scale (BACKLOG #61). Also: `odom_velocity_deadband` 0.02 m/s (speeds below zero out),
  `max_linear_velocity` 1.0, `odom_vel_alpha` 0.3.
* The odometry **yaw** comes from Ackermann kinematics (steering command), not from the IMU. The IMU yaw is a
  separate string on `/imu/rpy` and it lags the true heading by about 0.28 s (EMA `alpha 0.15`): about 7 deg in a
  25 deg/s turn. `local_pose.heading_lead_s` (0.30-0.35) would compensate it in the sandbox, but it is **untested on
  the car and off by default**.
* The scale was found with the **old** motors and re-converged to nearly the same value with the **new** ones
  (about 300000 both times), but steps 7-8 have not been re-run with the new motors.
* `servo_controller.wheel_base` is still the base value 0.14; the measured wheelbase is 0.216 (`common.yaml`).
* The launch loads **no** session overlay. The wizard re-applies these three values after a relaunch; your own
  code must set them itself (e.g. `ros2 param set /servo_controller ticks_per_meter 299599.9`).

---

## 5. Steering and servo. PROVISIONAL (step 7 did not pass and was not saved)

Nothing from step 7 is saved. What was measured live:

| Quantity | Value | Status |
|---|---|---|
| Servo output | `set_pwm_servo(4, angle)`; `angle = center - range_left * s` (left), `center + range_right * s` (right), `s` in 0..1; auto mode multiplies right by 1.3 but clamps to `center + range_right` | code |
| `servo_center` | **84** (found by the straight runs) . YAML default 90 | LIVE |
| `servo_range_left` / `servo_range_right` | **74 / 70** now (YAML default 50 / 70; tried 68 / 90) | LIVE |
| Steering direction | `steer_sign = -1.0`; both locks turn the correct way | measured |
| Left full-lock radius | **0.576 m** at range 50; **0.482 m** at range 68 (centre 84) | PROVISIONAL |
| Right full-lock radius | **0.735 m** at range 70; **0.793 m** at range 90, no improvement | PROVISIONAL |
| Straight drift | **3.27 cm/m** after 4 runs (limit 2.0) | PROVISIONAL |

Notes:

* Radii come from odometry distance / IMU turn angle. The distance is from **one** drive wheel, so it is biased by
  about half the track width (about 0.09 m) in opposite directions on the two sides, and a difference between left and
  right is partly measurement bias. Use them as estimates of the turning circle, not of the rear-axle centre.
* Left angle at range 68 is about 24 deg of wheel angle (`atan(0.216 / 0.482)`); right about 15 deg. The **right lock
  stops mechanically** at about 15-16 deg (a range of 90 gave a larger radius than 70): a linkage / horn limit, not a
  software one. The design limit (radius <= 0.45 m) is **not met** on the right.
* `command_owner.steering.left_max_rad / right_max_rad = 0.495` and `vehicle.min_turning_radius_m = 0.40` are
  **assumed** (from a 0.40 m radius) and do **not** match this car (left about 0.48-0.58 m, right about 0.74-0.79 m).
  If you need a turning limit, use the measured radii, not these defaults.

---

## 6. Drive power and speed. PROVISIONAL

* Auto drive: `pwm = int(linear.x * 255)` on motor channel 1 (`linear.x` is a **duty** in `CALIBRATION_RAW` mode).
  The manual joystick path is separate: throttle x speed limit (`speed_levels [15, 25, 40, 60, 100]`, default 25).
* This car has **large static friction**: recorded run with full-left lock: it rolled at duty 0.20-0.24 (about
  0.17 m/s at 0.24) but did not start at 0.30 from standstill. Step 7 now starts each segment with a **kick**
  (duty 0.40 for 0.5 s, then 0.16), see BACKLOG #63. **Untested on the car.**
* Battery: 12.5 V charged, 11.4-11.8 V is low and coincided with USB drops of the motor board.
* **Speed PID (step 8) was never run for the new motors.** No feed-forward or speed-loop numbers exist for them.

---

## 7. Not done: do not use

| Step | State |
|---|---|
| 7 servo centre + steering limits | not saved (see section 5) |
| 8 speed PID | not done |
| 9 venue thresholds | not done |
| 10 UWB survey, 11 map / UWB alignment | not done (anchor positions and `track_to_venue` are still defaults) |
| 12 mission planner, 13 practice runs | not done |

The two side cameras were **removed** from the project (their MIPI lanes never worked): the car has **one** camera.

---

## 8. Using the numbers

```python
import numpy as np, yaml, cv2
ci = yaml.safe_load(open('intrinsics/astra.yaml'))
K = np.array(ci['camera_matrix']['data']).reshape(3, 3)
D = np.array(ci['distortion_coefficients']['data'])          # plumb_bob: k1 k2 p1 p2 k3
und = cv2.undistort(bgr_640x480, K, D)                        # image size must be 640x480
```

* Camera pose in `base_link`: `mounts.front` in `data/cameras.yaml` (x, y, z in metres; yaw, pitch_down, roll in degrees;
  `pitch_down > 0` tilts the optical axis towards the ground).
* Odometry: set the three `servo_controller` parameters (section 4) and `encoder_jump_threshold: 100000.0`
  before reading `/odom`.
* LiDAR: `base_to_laser` (section 3) as a static transform `base_link -> laser_frame`, with the x fix pending.
* Repo: `docs/BACKLOG.md` #42-#63 has the history of every problem; `tools/car_ops/README.md` has the helper scripts.

## 9. Provenance

* Session `20260924_103632` on risabot5, steps 1-6 saved between 10:36 and 14:14 (car clock).
* Code at commit `da5ee29` (branch `phase8/complete-calibration`); step 5 warning and step 6 history are in the
  session files `05_lidar_camera.yaml` and `06_imu_odometry.yaml`.
