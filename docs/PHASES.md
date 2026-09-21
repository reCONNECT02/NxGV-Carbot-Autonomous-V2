# Build phases

One phase per chat. Topic names (`carbot_common/topics.py`), message types
(`carbot_interfaces`) and YAML keys are a contract: later phases extend them and
never rename silently.

| # | Phase | Status |
|---|---|---|
| 1 | Repo skeleton, packages, launch files, YAML structure | **done** |
| 2 | Perception (3 cameras, IPM, stitch, road mask) | **done** |
| 3 | Localization + UWB | **done** |
| 4 | Global planner, mission logic, local planner | **done** |
| 5 | Parking, recovery, command owner + safety | next |
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
| Challenge 4 boom-gate position is provisional — MEASURE ON SITE. | track_features.yaml (phase 4) | 8 / on site |
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
  `mission.yaml` **version 2**, which the stack reads directly since phase 4
  (see `tools/map/README.md`).
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


## Phase 4 — what exists

The team's own map and mission are now the stack's data. **Only these six
nodes changed** from stubs to real code: `track_map_server` (01),
`global_planner` (07), `mission_logic` (08), `corridor` (09), `local_planner` (10),
`path_tracker` (13). Parking (11), recovery (12), safety (14) and the command
owner (15) are still stubs, so **the car does not drive itself yet**: the
command owner keeps publishing zeros.

* **Data** (`config/data/`):
  * `track_map.yaml` = the team's v2 map, `mission.yaml` = the team's v2 mission
    (re-planned on this map: same poses, same exits; fingerprint fixed).
  * NEW `track_features.yaml` (hand-edited): start + light stop poses as
    `{mission_pose: P0 / P1}`, traffic light, both boom gates, bump, hill,
    tunnel (along `tunnel_corner`). All marked `provisional`.
  * NEW `mission_rules.yaml` (hand-edited): per-leg `end_behaviour` /
    `parking_bay`, `roundabout_visits` (planned exits), `gate_route_check`,
    `challenge4_gate`, `traffic_light`, `transitions`, `speed_zones`.
  * `v4_reference/track_map.yaml` + `mission.yaml`: the phase-1 V4 files (v1),
    MOVED here. Tests and the phase-2/3 sandboxes use them explicitly.
* **`carbot_common`**: `course.py` reads v1 and v2 (v2 via `map_geometry.py`,
  a verbatim port of `map_builder.py` section 3; centrelines and areas are
  identical, the field agrees to < 1 cm at the edges) + V4 body checks
  (`body_clear_many`, `body_margin_many`, `road_clear_many`), `crossable()`,
  `in_area()` on polygons, `file_fingerprints()` (raw / LF / CRLF sha1),
  `course_from_params()`. NEW `mission.py`: one `Mission` object for v1/v2
  (pieces road / manoeuvre, rules, poses, `named_poses()`).
* **Cores** (`carbot_planning/*_core.py`, pure Python, used by the nodes, the tests and
  the sandbox): `route_core` (V4 hybridPlan port, v2 check, exits, 1 cm resample),
  `evidence` (V4 evidence/support over the road grid + memory), `corridor_core`,
  `local_core`, `tracker_core` (V4 Controller + splitGears sequencing), `mission_core`.
* **Verified against the V4 JS in Node** (`tools/v4_harness/`, `test_v4_equivalence.py`):
  hybridPlan/buildMission identical (1777 + 1017 points); local planner identical
  to 1e-9 given the same guide; corridor within one 9 mm probe step (probes sit
  exactly on cell edges, so 1e-7 route differences can flip one edge).
* **Closed loop** (`tools/sandbox/run_planning.py`, `test_closed_loop_full_mission`): the whole
  team mission completes in the sim: challenges 1-11 entered in order, observed-green
  release, gate check against visit 1 only, 3 manoeuvres (stand-in), COMPLETE.
  Lane driving body margin >= -0.6 cm (worst: roundabout west exit, 1.53, 0.78).
* **Rules enforced in code**: no timer anywhere in the light / gate logic (tests hold 5 min
  on RED and forever on a closed gate); gate-vs-route mismatch = GateRouteMismatch + banner,
  route untouched; TUNNEL needs the base `/tunnel_detected` AND the tunnel zone; e-stop
  -> SAFETY_STOP + event "MANUAL INTERVENTION ... = 0 marks".

### Contract changes (additions unless marked)
* **DATA_KEYS**: + `track_features`, `mission_rules` (every node gets `data.track_features`,
  `data.mission_rules`; a session may override them like the others).
* **MOVED**: phase-1 `config/data/track_map.yaml` + `mission.yaml` (v1) ->
  `config/data/v4_reference/`. The default files are now v2.
* **Path convention**: every planning Path is in `track`, `pose.position.z` = direction (+1/-1).
* **Corridor.msg**: header frame is `track` (comment only, approved).
* **ACTIVE_PATH** = the active route piece (road piece, or the manoeuvre preview / empty).
  **LOCAL_PATH** = guide + selected offset; EMPTY = no feasible candidate / tracking problem.
* **mission_logic** subscribes additionally to `/carbot/request/recovery` and the road grid
  (camera age). **local_planner** to `/carbot/owner/state` (steering estimate) and TF
  base_link -> laser_frame. **corridor** to `/odom` and route info.
* **v1 mission.yaml**: `roundabout_visits[].direction`, `challenge4_gate.near_route_m` added.
* **YAML**: new keys in `planning.yaml` (all six nodes, marked `# phase 4`) and
  `common.yaml memory_evidence.*`. No existing key renamed or re-valued.
* `local_pose` loads the map with `course_from_params` (v2 needs the features + mission);
  `calib_map_uwb.named_poses` uses `carbot_common.mission.named_poses`.
* `.gitattributes`: YAML/py/sh/md/js stored with LF.

### For phase 5
* **Parking (11)**: publish `/carbot/parking/path` (latched) for the active manoeuvre piece;
  `path_tracker` already follows it one gear at a time (0.4 s holds) and sets `arrived` at the
  end; `mission_logic` completes the piece when arrived AND within `parking_arrive_m` (3 cm)
  of the path end. The team's previews are drawn at the minimum turning radius: replaying
  them from a 1 cm / 3 deg start error ends 30 cm off, so plan from the ACTUAL pose (V4 does),
  replan at cusps and for terminal heading (sim stand-in: 2.5-6 deg end error, up to 5 cm
  over the edge leaving the parallel bay). Leg 3 = un-park (manoeuvre), road, park.
  The v2 bay polygons come from the map (`course.area_polys`), the observed bay from memory.
* **Recovery (12)**: publish `/carbot/request/recovery` while active (arrived = done);
  mission logic switches to RECOVERY on a fresh request (ROAD only). Problem signals:
  `local_candidates.selected_id == -1` / empty `local_path` (no feasible candidate or
  tracking error > 10 cm).
* **Safety (14)**: `corridor.branch_hold` + `branch_reason` is the V4 route-identity hold;
  mission logic shows SAFETY_STOP from `SafetyStatus` but the veto itself is the owner's.
* **Command owner (15)**: follow `MissionState.active_source`; `HOLD` = zero. Tracker requests
  already carry the lowest speed-zone cap. Steering is REP-103 (+ left).
* **Tunnel bridge**: activate while `MissionState.mode == TUNNEL` (`active_source` TUNNEL).
* Not yet measured on the car: roundabout exit tracking (0.6 cm over in the sim), real
  camera corridor stability, planner CPU (laptop: corridor 0.7 ms, local planner 11 ms,
  x2 when the relaxed pass runs; expect ~5x slower on the RDK X5 A55 cores, budget 200 ms at 5 Hz).
