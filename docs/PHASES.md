# Build phases

One phase per chat. Topic names (`carbot_common/topics.py`), message types
(`carbot_interfaces`) and YAML keys are a contract: later phases extend them and
never rename silently.

| # | Phase | Status |
|---|---|---|
| 1 | Repo skeleton, packages, launch files, YAML structure | **done** |
| 2 | Perception (3 cameras, IPM, stitch, road mask) | **done** |
| 3 | Localization + UWB | **done** |
| 4 | Global planner, mission logic, local planner | next |
| 5 | Parking, recovery, command owner + safety | |
| 6 | Detectors (traffic light, boom gate, bump sign) | |
| 7 | GUI main tab + diagnostic tabs | |
| 8 | Calibration wizard + race mode | |
| 9 | Docs | |

## Phase 1 — what exists

* **Interfaces** `carbot_interfaces`: 26 msgs, 2 srvs (NodeStatus, MotionRequest,
  SafetyStatus, CommandOwnerState, MissionState, GateRouteMismatch, LocalGrid,
  Corridor, CandidateArray, UwbRanges, DetectionArray, Scoreboard, PreflightReport,
  CalibrationState, …).
* **Shared library** `carbot_common`: `topics.py` is the single source of every topic
  name (base-repo names kept: `/cmd_vel_auto`, `/odom`, `/imu/rpy`, `/scan`,
  `/tunnel_detected`, `/tunnel_cmd_vel`, `/uwb3/input_json`, …); `CarbotNode`
  (all parameters from YAML, loud error on a missing key, 1 Hz NodeStatus heartbeat
  with input ages on `/carbot/status`); `geometry.py` (V4 core.js port, unit tested);
  `calibration_store.py` (session folders, ACTIVE marker, rollback).
* **YAML**: `config/params/*.yaml` (tunables, one section per node, `/**` for common
  vehicle geometry/limits) and `config/data/*.yaml` (track map = full V4 Course port,
  mission legs + roundabout exits, cameras + roles, UWB anchors, 13 challenges,
  12 calibration steps).
* **Every block is a running stub node** with its final publishers/subscribers, so
  `ros2 node info` already shows the final graph. Stubs report level `STUB` and
  publish nothing on their outputs, except: `command_owner` publishes an explicit
  zero Twist on `/cmd_vel_auto` with winner `DISARMED`; `scoreboard` publishes the
  13 challenges as PENDING; `race_supervisor` refuses to arm and names the missing
  calibrations; `gui_server` serves a status page (nodes, blocks, input ages,
  mission, scoreboard, e-stop) on port 8080.
* **Launch** (`carbot_bringup/stack.py`, used by both entry points):
  1. `ROS_DOMAIN_ID` from `uwb.yaml agent.domain_id` (1: the tag chooses it),
     `ROS_LOCALHOST_ONLY=0`, FastDDS UDP-only profile (base `disable_shm.xml`).
  2. Calibration session: ACTIVE (or `session:=NAME`); its `params_overlay.yaml` is
     applied last and its `data/*.yaml` replace the repo defaults.
  3. `kill_stale.sh` (patterns in `drivers.yaml carbot_launch.kill_patterns`) runs
     first; everything else starts only after it exits.
  4. Astra (base `astra_mini.launch.py`), both MIPI cameras through
     `run_mipi_cam.sh` as root (Camera_Setup.md commands exactly, width/height
     always set), LiDAR, micro-ROS agent (`run_uwb_agent.sh`), static TFs
     (`base_link -> laser_frame`, `cam_front`, `cam_left_rear`, `cam_right_rear`).
  5. Base `servo_controller` + `tunnel_wall_follower` unchanged (params:
     `base_nodes.yaml`, verbatim base values). Base `auto_driver`,
     `cmd_safety_controller` and `dashboard` are NOT started.
  6. All Carbot nodes, then the GUI.
* **Root handling**: `tools/setup/install_root_helpers.sh` installs root-owned copies
  of the two helpers into `/usr/local/lib/carbot` and a sudoers rule for exactly
  those two files. `tools/systemd/carbot-race.service` for optional auto-start.
* **Secrets**: `TagConfig.example.h` only; real `TagConfig.h` is gitignored and
  blocked by `tools/git-hooks/pre-commit`.

## Open items carried forward

| Item | Where | Phase |
|---|---|---|
| Camera image transport: 2 × 960×544 bgr8 @30 fps over UDP loopback (SHM disabled) is ~95 MB/s. Measure CPU; if too high, downscale in-process or use hobot shared-mem transport. | perception.yaml / cameras.yaml | 2 |
| `camera_calibration_file_path` parameter name of mipi_cam: confirm with `ros2 param list /cam_ov5647/mipi_cam`. | run_mipi_cam.sh | 2 |
| Optical frames (`cam_<role>_optical`) are not published yet; `cam_<role>` is body-style (+x along the optical axis). | stack.py `camera_static_tfs` | 2 |
| Wheelbase: base `servo_controller.wheel_base` 0.14 vs V4 measured 0.216. | base_nodes.yaml / common.yaml | 3/5 (calib step 6/7) |
| `servo_controller` has no battery topic or arm topic yet (`/carbot/vehicle/battery_v`, `/carbot/vehicle/arm`): add as a wrapper/extension without changing its motor code. | topics.py | 5 |
| `bpu_ratio_path` on RDK X5 unverified. | ops.yaml | 7 |
| `battery_min_v: 10.8` — confirm for the pack. | ops.yaml | 8 |
| Challenge 4 boom-gate position is provisional — MEASURE ON SITE. | mission.yaml | 4/8 |
| `bpu_detector.model_path` is relative to the repo root; resolve against the workspace in phase 6. | detectors.yaml | 6 |


## Phase 2 — what exists

* **`carbot_perception/camera_model.py`** (pure, unit tested): intrinsics in ROS
  camera_info format with `plumb_bob`, `rational_polynomial` or `equidistant`
  (fisheye); mounts in `base_link` (rear-axle centre on the ground),
  `R = Rz(yaw) Ry(pitch_down) Rx(roll)` = the static TF. Ideal pinhole reproduces
  V4 `projectionMap` exactly. Before step 3, an ideal pinhole from
  `mounts.<role>.hfov_deg` is used (node reports WARN).
* **Block 03 `road_perception`**: per camera one `cv2.remap` (fisheye correction +
  top-down warp in one step) onto the V4 grid (100 x 1.8 cm, x -0.65..1.15,
  y -0.9..0.9); strongest view per cell (1/(0.1+depth^2)) among fresh cameras;
  V4 luma/chroma classes; 4-neighbour growth from the seed box. Publishes
  `LocalGrid` on `/carbot/perception/road_grid` (base_link, camera stamp).
  Debug JPEGs (warped, stitched, mask, overlay/<role>) only while subscribed.
* **Block 04 `local_memory`**: V4 LocalMemory in the **odom** frame (`/odom`),
  pose interpolated at each grid's stamp; publishes a 3 m window with `age_s`
  and `travel_since_m`.
* **`camera_preview`**: subscribes to a raw camera only while its preview or
  record topic has a subscriber.
* **Calibration steps 3 and 4** as CLI tools (the phase-8 wizard calls the same
  functions in `calib_core.py`): `calib_intrinsics --sensor <s>` (auto view
  capture, fits standard + fisheye, keeps the better, needs all 9 image
  regions covered) and `calib_extrinsics` (floor boards from
  `calibration_steps.yaml`, pose per camera, ground error, CAD delta,
  `ipm_check.png`). Results go to the session layout of `calibration_store`.
* **Boards**: `tools/calibration/make_boards.py` -> `docs/calibration/*.pdf`;
  layout picture `tools/calibration/draw_mat.py` -> `docs/images/calib_mat.png`.
* **Floor sheet** (step-4 alternative to three A3 boards):
  `make_boards.py --sheet` -> `docs/calibration/floor_sheet.pdf`, one large
  print (1310 x 940 mm with the default YAML, fits a 1067 mm / 42 in roll) with
  all boards at their exact `centre_m` / `yaw_deg` plus the centre line and
  rear-axle line for placing the car. Tests: `python3 -m pytest -q tools/calibration`
  (rasterises the layout and the PDF and re-detects every board against
  `FloorBoard.ground_points`). No YAML keys or calibration code changed.

### Phase 2 contract changes
* `LocalGrid.msg`: + `float32[] travel_since_m` (additive).
* `local_memory` subscribes `/odom` instead of `local_pose` (memory must live in
  an uncorrected frame, as V4 `estimate.odom`).
* `cameras.yaml`: CAD mounts (z + 0.0325 m), default roles left=imx219,
  right=ov5647 (still unconfirmed), role keys unchanged.
* Static TFs: + `cam_<role>_optical`.
* `mipi_cam` no longer receives `intrinsics_file` (would undistort twice).
* New YAML keys: `road_perception.debug.{overlay_width,bev_scale}`,
  `local_memory.{window_m,ring_cells,odom_history_s,max_stamp_gap_s}`,
  `camera_preview.check_period_s`, step 3 `pass.min_coverage_cells`,
  step 3/4 `tool`, step 4 `target.boards` and warn limits.
* `calibration_store`: + `open_session`, `write_yaml`, `update_step`, `set_active`.

### For phase 3+
* Memory consumers apply their own age limit (drive 3 s, corridor 2 s, parking
  24 s) and the uncertainty rule `base + per_m * travel_since_m <= limit`.
* The road grid is in base_link at the newest camera stamp used.

## Repo housekeeping (after phase 2)

* `tools/map/map_builder.py`, `tools/map/mission_planner.py`: the team's
  offline block-01/07 tools, committed unchanged. They write `track_map.yaml` /
  `mission.yaml` **version 2**; the stack's `config/data/` files are still the
  phase-1 **version 1** layout. Phase 4 decides how the two meet (see
  `tools/map/README.md`). The VS Code sandbox world also reads version 1.
* `docs/reference/`: rulebook PDF, RISA Bot spec sheet, V4 simulator sources
  (`v4_simulator/*.js`, the reference every port follows), GUI reference
  screenshots (`gui/`, the look of the main tab for phase 7).


## Phase 3 — what exists

* **`carbot_common/course.py`** (block 01 geometry): V4 `Course` rebuilt from
  `track_map.yaml`; `clearance(x, y)` (> 0 drivable, = distance to the road edge),
  `gradient`, `surface` (hill/bump), `in_tunnel`, `in_area`, `pose(key)`,
  `load_course(path)` (cached by file sha1), `map_sha1`. Verified against V4
  `core.js` run in Node: identical to 4e-8 m within 25 cm of the road.
  **Phase 4 must use this, not a second map loader.**
* **`uwb_ranges`** (`uwb_localization`): `/uwb3/input_json` (BEST_EFFORT) ->
  `UwbRanges` per report, `UwbStatus` at 2 Hz, raw pairwise fix (GUI/calibration
  only). Repeat `sample_seq` skipped, reboot detected, offsets + height
  flattening applied, measurement time = WiFi-min-filtered encode time - age_ms.
  Pure logic in `uwb_core.py` (also layout checks, HDOP, offset math, flip-flop).
* **Block 05 `local_pose`**: V4 Estimator port. `/odom` distance + `/imu/rpy` yaw
  (offset to the track frame taken at the first IMU sample after a reset; falls
  back to `/odom` yaw if the IMU is stale), camera-edge-to-map registration at
  the grid's own timestamp, <= 1.5 mm per frame. No UWB input at all. Publishes
  `Odometry` on `/carbot/localization/local_pose` (track -> base_link, sigma^2 in
  covariance[0]/[7]) and **TF track -> base_link** (the base servo_controller
  publishes no TF).
* **Block 06 `global_pose`**: offset EKF (2x2), one gated update per fresh anchor
  range at the local pose of that range's time, using the **tag position**
  (`uwb.yaml tag.mount_xy_m`), V4 landmark pull-back, re-acquire safety net.
  Publishes `PoseWithCovarianceStamped` on `/carbot/localization/global_pose`,
  merged `LocalizationStatus`, static TF track -> venue. **No UWB updates until
  `track_to_venue.aligned` is true** (`require_alignment`).
* **Calibration CLIs** (the phase-8 wizard calls the same functions):
  step 6 `ros2 run carbot_localization calib_odometry`, step 10
  `ros2 run uwb_localization calib_uwb`, step 11
  `ros2 run carbot_localization calib_map_uwb [--mode points]`. All record raw
  captures into the session and support `--replay` (no ROS).
  Shared session helpers: `carbot_common/calib_tools.py`.
* **Sandbox** `tools/sandbox/run_localization.py`: simulated lap or `--bag` replay
  with a tuning report.

### Contract changes (additions only, nothing renamed)

* `UwbStatus`: + `anchors_surveyed`, `offsets_calibrated`, `latency_ms`, `reboots`,
  `anchor_fresh_count[]`.
* `LocalizationStatus`: + `visual_updates`, `imu_ok`, `heading_rad`, `uwb_accepted`,
  `uwb_rejected`, `uwb_reacquires`, `state`.
* Topic `/carbot/localization/reset` (`T.LOCALIZATION_RESET`,
  PoseWithCovarianceStamped, track): re-seeds blocks 05 + 06; refused unless the
  mission mode is IDLE / COMPLETE / empty.
* YAML: new keys in `localization.yaml` (all three nodes); `uwb.yaml tag.mount_xy_m`;
  `calibration_steps.yaml` steps 6/10/11 rewritten in block style with
  `tool`, `instructions` and new procedure/pass keys (ids, indices, `writes`
  targets unchanged in meaning; step 6 now writes `servo_controller.imu_yaw_scale`
  and `odom_reverse_polarity` instead of `imu_*_offset`).

### For phase 4

* Plan and track on **`local_pose`** (smooth). Use **`global_pose`** only for
  "where on the route am I" (checkpoint progress, roundabout branch identity,
  preflight start check); it can move by a few cm per second when UWB corrects.
  `LocalizationStatus.state == NO_ALIGNMENT` means global == local.
* Preflight start check (phase 8): `carbot_localization.alignment.start_pose_error`
  (raw UWB fix vs map start + tag lever arm).
* Mission logic should publish `MissionState.mode` != IDLE once the run starts,
  or `/carbot/localization/reset` stays accepted.
* Known real-car points to watch: base `servo_controller` EMA-filters IMU yaw
  (lag ~0.3 s in corners); `/odom` yaw comes from the servo command, not the IMU;
  `wheel_base` 0.14 (base) vs 0.216 (V4) is still unreconciled (step 7).
