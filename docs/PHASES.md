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
| 5 | Parking, recovery, command owner + safety | **done** |
| 6 | Detectors (traffic light, boom gate, bump sign) | **done** |
| 7 | GUI main tab + diagnostic tabs | **done** |
| 8 | Calibration wizard + race mode | **in progress** (page by page: steps 1-3 done) |
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

## Phase 5 — what exists

| Block | Node | Core (no ROS, tested) | Notes |
|---|---|---|---|
| 11 | `carbot_planning/parking_planner` | `parking_core.py`, `reeds_shepp.py` | V4 `parkingPlan` stages (docking 15/12/8 cm -> direct RS -> handoff extensions -> bounded hybrid + live RS connector). **Bit-identical to V4 JS** on the v1 course (stage, points to 1e-7, audit counts). Bay observed from memory tape (start/end/far edges); plans from the ACTUAL pose once stopped; one gear section at a time; cusp replan (> 2.5 cm, <= 4) and heading correction (> 0.045 rad, <= 2) from the pose; publishes DONE on `/carbot/parking/state` |
| 12 | `carbot_planning/recovery_planner` | `recovery_core.py` | Port of `recovery.js`: BRAKE / WAIT / ALIGN / TRACK, reverse 3-20 cm first then forward, exactly 2 gear sections, cost <= 1.6, paint allowance, LiDAR-clear swept footprint, >= 55 % observed rear road, 3 cm/s, <= 3 attempts. Paused (never bypasses) by a safety veto or mission hold |
| 13 | `carbot_control/tunnel_bridge` | `owner_core.SteeringMap.to_steer` | Base `tunnel_wall_follower` UNCHANGED; forwards `/tunnel_cmd_vel` as `/carbot/request/tunnel` only while `active_source == TUNNEL`, exact inverse of the owner's steering map |
| 14 | `carbot_control/safety_monitor` | `safety_core.py` | V4 order: e_stop, motion_fresh 0.2 s, local_sigma 3.5 cm, camera_fresh 0.45 s, route_identity (recoverable reason excluded), road_mask (60 cells, 0.5 s dwell), tunnel_clearance (+-0.14 rad, 0.24 m). UWB is not an input |
| 15/16 | `carbot_control/command_owner` | `owner_core.py` | Single writer of `/cmd_vel_auto` at 50 Hz. DISARMED -> SAFETY_STOP -> WATCHDOG (200 ms) -> HOLD -> active source. Feedforward + PID speed, slew on ramp-up only, stop immediate. Arms the base via `/carbot/vehicle/arm` |
| 16 | `control_servo/carbot_extension.py` | `arm_decision()` | Additive: `servo_controller.py` touched by 2 lines. `/carbot/vehicle/arm` -> AUTO / MANUAL+stop; `/carbot/vehicle/battery_v` |
| cal 7 | `ros2 run carbot_control calib_steering` | `calib_core.py` | Full-lock circles -> left/right_max_rad, min_turning_radius; straight runs -> integer servo_center (live) |
| cal 8 | `ros2 run carbot_control calib_speed` | `calib_core.py` | Duty sweep -> feedforward least squares; closed-loop verify + retune |

Closed-loop sim (`sim_core.simulate`, real blocks 08/09/10/11/12/13): the full team mission
completes (~390 s sim): challenges 1-11 in order, both bays PARKED (estimated footprint
inside, heading error < 1.1 deg), un-park via the time-reversed search, a stuck start
recovers (13 cm reverse, rejoin). Manoeuvre body margins > 0.

### Contract changes (additions unless marked)
* topics: `PARKING_STATE` `/carbot/parking/state` (String JSON, latched), `RECOVERY_STATE`
  `/carbot/recovery/state` (String JSON), `CALIBRATION_REQUEST` `/carbot/calibration/request`
  (MotionRequest, sources `CALIBRATION` / `CALIBRATION_RAW`, calibrate mode only).
* `MotionRequest.msg`: comment only (calibration sources). No field changes anywhere.
* `mission_logic.parking_requires_planner_done: true` — a manoeuvre piece now completes on
  block 11 DONE (not on the tracker's `arrived` + 3 cm, which is kept for `false`).
* **CHANGED** `drivers.yaml carbot_tf.base_to_laser` yaw `0 -> 3.14159265`: the driver runs
  `reversion: true` and the base tunnel follower already corrects with
  `lidar_angle_offset 3.1416`; blocks 10/12/14 read the TF, so it must agree (test enforces).
* New YAML keys: `control.yaml` (safety_monitor, command_owner `arm_publish_hz`,
  `estop_latch_in_race`, `measured_max_age_s`, `calibration.*`; tunnel_bridge `rate_hz`,
  `use_zone_cap`, `command_owner_steering.*` / `command_owner_feedforward.*` mirrors),
  `planning.yaml` (parking_planner / recovery_planner phase-5 blocks), `base_nodes.yaml`
  (`carbot_arm_enabled`, `carbot_battery_rate_hz`), `calibration_steps.yaml` steps 7/8
  `tool` / `instructions` / `procedure` (step 7 `writes` no longer lists servo_range_*:
  binding is checked by eye and set in Tuning).
* Deliberate differences from V4 (all YAML, all documented inline):
  `plan_radius_factor 1.08` (V4 1.0; sim un-park margin -3.0 -> +1.5 cm),
  `reverse_time_unpark` (V4 has no un-park; 0.4 s vs 5 s hybrid),
  `bay_observation.max_shift_m` / `fallback_after_s` (V4 holds forever = 0 marks),
  `fallback_time_budget_s 6`.
* `test_required_keys.py` (bringup): every `REQUIRED` key of the phase-4/5 nodes exists in
  the YAML the launch supplies; tunnel_bridge mirrors == owner values.

### For phase 6
* Detectors publish `DetectionArray` on the existing topic; mission logic already reads
  `traffic_light_state`, `boom_gate_state` (+ confidence) and the bump sign; nothing in
  phases 4/5 needs to change. Gate-vs-route mismatch stays a log + GUI warning (block 07).
* BPU load: planner CPU is unmeasured on the RDK. Parking/recovery searches run in their own
  callback-group thread (MultiThreadedExecutor) so status/subscriptions keep flowing; the
  hybrid fallback is time-boxed at 6 s. Keep detector inference off the planning cores.

### Not tested on hardware (phase 5)
* Nothing in phase 5 has run on the car. First drive: calibrate mode, steps 6 -> 7 -> 8.
* Feedforward / steering limits in `control.yaml` are UNCALIBRATED placeholders until
  steps 7/8 run. Wheelbase: base `wheel_base 0.14` vs V4 `0.216` still unreconciled —
  step 7 measures the radius; check `vehicle.wheelbase_m` against the chassis with a ruler.
* `/odom` speeds below `odom_velocity_deadband` (0.02 m/s) read as 0: creep speed checks
  and recovery (3 cm/s) sit just above it.
* Whether `Rosmaster.set_motor(0)` brakes or coasts (the speed controller never reverses
  the motor to brake inside one gear).

### Phase 5 follow-up (before phase 6)

* Wording fix: README / CHALLENGE_MAP / challenges.yaml said "pre-recorded motion
  for parking only", which read as our strategy. It is only the rulebook limit.
  Our parking is block 11: Reeds-Shepp planned into the observed bay, from the
  estimated pose, replanned at each gear change. Nothing replays motion.
* New guard: the base servo_controller record/playback writes straight to
  apply_hardware() (bypasses command owner + safety veto). carbot_extension now
  refuses playback unless CARBOT_MODE is in
  `servo_controller.carbot_playback_allowed_modes` (default `[calibrate]`).
  New YAML key only; no topic, message or existing key renamed.
* Phase 6 note: the base signage_detector / parking_controller / auto_driver
  "parking sign -> preset playback" chain stays NOT launched. The new BPU
  detector must not publish /record_playback_cmd.

## Phase 6 — what exists

| Piece | Where | Notes |
|---|---|---|
| Detector node | `carbot_detectors/bpu_detector.py` | Replaces the stub. Team YOLO11n unified14 model (installed as `share/carbot_detectors/models/`), front camera only, newest frame at 10 Hz, NV12 in, `DetectionArray` out, debug JPEG only while subscribed |
| Detector core | `carbot_detectors/detector_core.py` | YOLO11 6-output decode (tensors found by SHAPE, NHWC or NCHW), class-wise NMS, class map, debounce, bearing/range per detection. Tested on synthetic BPU outputs |
| Gate association | `carbot_planning/mission_core.py` | Team idea: every detection is matched to a MAP gate by bearing + range from the pose; only gates on the current route pieces can hold; the route check uses the roundabout gate's own associated state |
| Navigation first | `mission_rules.yaml` | `traffic_light.enabled: false`, `challenge4_gate.enabled: false` (race week). Banner + event `DETECTION HOLDS OFF` at START |
| Sim | `sim_core.simulate(detector=...)`, `run_planning.py --detector none|associated|scripted --holds yaml|on|off` | `--detector none` with the shipped YAML completes the whole mission (test enforces) |
| Model tooling | `tools/bpu_model/unified14/` | Dataset layout, `train_yolo11.py`, `export_rdk_onnx.py` (6-output head), `make_calibration.py`, `rdk_bpu_config.yaml`, `compile_model.sh`, board smoke test. Export + compile NOT run in CI |
| Docs | `docs/DETECTORS.md` | Every class, what it triggers, the wiring, the switches, what was not taken from the team package |

### Contract changes (additions unless marked)
* **CHANGED** `detectors.yaml`: `model_path` (now the unified14 model, relative to the
  package share dir), `class_names` / `class_map` (14 classes). New keys: `reg_max`,
  `thresholds.traffic_light_yellow`, `info_threshold`, `object_size_m.*`. All phase-1 keys
  kept. The base YOLOv5 model stays at `tools/bpu_model/model_output/` (unused).
* `DetectionArray` / `Detection`: no field changes. `position` (base_link) and
  `distance_m` (-1 = bearing only) are now filled; `boom_gate_state` is any gate in view.
* `mission_rules.yaml` (and `v4_reference/mission.yaml`): new `traffic_light.enabled`,
  `challenge4_gate.enabled`, `challenge4_gate.extra_gates`, new section `gate_association`
  (now a required rule key in `carbot_common/mission.py`). V4 reference keeps V4 behaviour
  (holds on, association off).
* `mission_core.Inputs`: new `gate_obs`, `gate_obs_t` (mission_logic fills them from
  `DetectionArray`).
* `/traffic_light_state` now RED/GREEN/UNKNOWN (as `topics.py` already documented);
  `/boom_gate_open` only while the state is known.
* `tools/git-hooks/pre-commit`: also refuses literal API keys / secrets / tokens.
  `tools/bpu_model/colab_training_script.py` reads `ROBOFLOW_API_KEY` from the environment.
  **The key that was committed must be rotated on Roboflow** (it is in git history).

### For phase 7 (GUI)
* Detections tab: `/carbot/detections` (boxes, class, confidence, `distance_m`),
  `/carbot/detections/debug/compressed` (subscribe only while the tab is open: the node
  encodes nothing otherwise), `/carbot/mission/gate_route_mismatch`.
* Main tab: `detections[].position` is base_link for rendering the light and gates;
  `traffic_light_state` / `boom_gate_state` for the labels. Show the `NAV ONLY` banner
  prominently (it comes in `MissionState.banner`).
* Tuning tab: `traffic_light.enabled` / `challenge4_gate.enabled` are the two switches the
  team will flip after validating the detector.

### For phase 8 (race mode)
* Preflight must **not** refuse to arm because `bpu_detector` reports `NO_MODEL` /
  `MODEL_MISMATCH` while both holds are disabled: show it as a warning. If a hold is
  enabled, a detector ERROR should block arming (the car would wait forever).

### Not tested on hardware (phase 6)
* `bpu_detector` has not run on the RDK (no ROS / BPU here): linted, core tested. First
  check: `ros2 topic hz /carbot/detections` and the node status (`docs/DETECTORS.md`).
* `pyeasy_dnn` output buffers are assumed float32 (as in the team node). If the model
  was compiled with quantised outputs, `MODEL_MISMATCH` will not catch it: the scores
  will look wrong on the debug image.
* Boom classes: 0 validation images in the team package. Association tolerances and
  `object_size_m` are provisional.
* CPU: detector decode + resize is on the CPU; planner CPU on the RDK is still unmeasured.


## Phase 7 — GUI (done)

Agreed design (mockup: claude.ai artifact "RISA Bot console mockup", v4):
race tabs Drive · 1 Global map (with legs strip) · 2 Perception + planner (candidates drawn on the
stitched drivable area) · 3 Memory + LiDAR · 4 Localization · 5 Detections · 6 Control + safety ·
7 System health · 8 Events + log; split view (2 tabs side by side) in race mode; header shows
"code running now" (Driving chain / Limiting / Stopped by / Faults), battery, Manual control
button, e-stop. Calibrate mode: Calibration (steps, phase 8 builds the pages) + Tuning + the same
diagnostics. Recording + Scoreboard tabs and nodes REMOVED (team decision, CPU).

### Checkpoint (verified against the files on disk)
- [x] `MANUAL_TAKEOVER` topic, command_owner winner `MANUAL` (+ test), joy_node in both modes
- [x] stack: scoreboard + run_recorder not launched; `session` param passed to nodes
- [x] ops.yaml system_monitor watch list extended (values only)
- [x] `carbot_gui/gui_core.py` + `test/test_gui_core.py` (8 tests pass)
- [x] `carbot_gui/gui_server.py` (lazy topic groups, /api/*, manual, start, e-stop) + `params_api.py` (tuning)
- [x] gui.yaml phase-7 keys, setup.py installs web/, package.xml deps
- [x] web/index.html, web/app.css, web/draw.js, web/app.js (shell)
- [x] web/tabs.js: drive, map (+ legs), percplan, memory, loc
- [x] web/tabs_diag.js: det, control, health, events, calibration (overview), tuning
- [x] tools/sandbox/gui_mock_server.py (no ROS, real web/ + synthetic data)
- [x] render test in headless Chromium (mock server): all 9 race tabs, calibrate overview + tuning, split view, manual confirm dialog, manual ON state: 0 JS errors
- [x] docs: handoff notes below, CHALLENGE_MAP regenerated with the new tab names

### What exists
* **`carbot_gui/gui_server`** (port 8080, both modes). JSON API: `GET /api/config`, `/api/core`
  (header + Drive), `/api/tab/<id>`, `/api/img/<key>` (JPEG or 204), `/api/events?since=N`,
  `/api/params`; `POST /api/estop`, `/api/estop_release` (calibrate), `/api/manual {on, confirm}`,
  `/api/start` (race, calls `/carbot/race/start`), `/api/params/get|set|save` (calibrate).
* **Lazy topic groups** (`gui_core.TAB_GROUPS`, `gui_server._build_groups`): `core` always
  (status, mission, owner, safety, battery, events, /rosout, armed, e-stop, /cmd_vel, local pose,
  global route + route_info, corridor, gate mismatch, preflight or calibration state); every other
  group only while a browser polls that tab (+ `idle_unsubscribe_s`). Images: one group per image
  key (`gui_core.IMAGE_KEYS`).
* **Header "running now"** = `gui_core.running_now()` from CommandOwnerState + SafetyStatus +
  MissionState + NodeStatus: Driving chain / Limiting / Stopped by (every blocker) / Faults (ERROR
  or silent nodes). Each item names its block and the tab that explains it.
* **Legs**: `gui_core.leg_progress()` = MissionState.route_leg + route_info pieces + nearest
  global-route index to the local pose (leg %, piece n of m, piece kind).
* **Tabs** (web/): race = drive, map, percplan, memory, loc, det, control, health, events;
  calibrate = calibration (overview from CalibrationState), tuning, + the same diagnostics.
  Split view in race mode (`gui.yaml race.split_view`).
* **Tuning** (`params_api.py`): catalogue of `config/params/*.yaml` (not drivers.yaml, not `/**`),
  live values via each node's parameter services (helper node, base-dashboard pattern), typed like
  the live value, Save merges into `<session>/params_overlay.yaml` (refused if no session).
* **Manual control**: GUI publishes `/carbot/manual/takeover` (Bool, latched). command_owner:
  winner `MANUAL`, zero `/cmd_vel_auto`, `/carbot/vehicle/arm` False, so the base servo_controller
  drives from `/joy` (its own joy_timeout watchdog). E-stop still wins. Race after START:
  `race.manual_confirm` requires a confirm and the event is logged as manual intervention.
* **Mock**: `tools/sandbox/gui_mock_server.py` (no ROS) + 2 VS Code entries.

### Contract changes (additions only)
* Topic `/carbot/manual/takeover` (`T.MANUAL_TAKEOVER`); CommandOwnerState.winner value `MANUAL`.
* Node parameter `session` (absolute session path or "") passed to every node by stack.py.
* gui.yaml phase-7 keys; ops.yaml system_monitor watch list: 7 topics appended (values only).
* challenges.yaml #13: `nodes` / `tab` values changed (scoreboard not launched). `tab:` ids in
  YAML are unchanged; `gui_core.YAML_TAB_ALIASES` maps them to GUI tab ids.
* stack.py: `scoreboard`, `run_recorder` NOT launched; joy_node in both modes (`start_joy`,
  race.launch.py default true).

### For phase 8
* The wizard pages go in `web/tabs_diag.js` `TABS.calibration` (currently the overview table).
  Data: `/api/tab/calibration` (CalibrationState). Add `POST /api/calibration/action` in
  gui_server -> CalibrationAction.srv. Each step's live view = embed an existing tab's renderer
  (`TABS[YAML_TAB_ALIASES[step.tab]].create(el, ctx)`); they are self-contained.
* Wizard is 13 steps (agreed in chat: 11 build map from a lap, 12 mission planner, 13 practice).
  calibration_steps.yaml still has 12: renumbering is a phase-8 contract change to announce.
* race_supervisor must serve `/carbot/race/start` (std_srvs/Trigger) and publish PreflightReport
  states 0-7; the Drive tab START button is enabled only at STATE_READY (4) and not in manual.
* Preflight should also refuse READY while `/carbot/manual/takeover` is true.
* `bpu_ratio_path` still unverified (System health shows BPU from SystemHealth.bpu_percent).

### PENDING ON CAR
1. `colcon build --symlink-install && source install/setup.bash`
2. `ros2 launch carbot_bringup calibrate.launch.py`, open http://<robot_ip>:8080 on a laptop.
3. Open each tab; `ros2 topic info /carbot/perception/road_grid -v` shows gui_server as a
   subscriber only while Perception + planner is open, and gone ~5 s after leaving it.
4. `top` on the RDK with the Drive tab open vs. no browser: gui_server < 10 % of one core.
5. Tuning: change `local_planner.lookahead_m`, Apply live, check `ros2 param get`; Save, check
   `<session>/params_overlay.yaml`.
6. Wheels off the ground: Manual control -> controller drives; Hand back -> owner resumes.
   E-stop while manual -> motors stop.
7. `ros2 launch carbot_bringup race.launch.py`: header shows "Waiting" (DISARMED) before START;
   after START (phase 8) Manual control asks for confirmation.


## Phase 8 — calibration wizard, page by page

Built one wizard page per chat. Status: **step 1 (sensor health) and step 2 (camera identity)
done**; steps 3-13 are placeholder pages; race mode (preflight / READY / START) not started.

### Page 1 — sensor health check (done, untested on the car)
| Piece | Where | Notes |
|---|---|---|
| system_monitor | `carbot_ops/system_monitor.py` (+ `monitor_core.py`) | REAL now (was a stub). Raw subscriptions (never deserialises images; stamp read from the CDR header), topics subscribed as they appear, rates/age/latency, CPU/RAM/temp/BPU, battery, agent, camera PIDs (sudo / bash / `ros2 run` wrappers filtered). Image checks pause after START (`image_watch_while_armed: false`) |
| Wizard node | `carbot_ops/calibration_wizard.py` | REAL now. Service actions SELECT/RUN/REDO/SAVE/KEEP_PREVIOUS/CANCEL/ROLLBACK/RESTART_CAMERAS; `/e_stop` cancels a running step; config errors reported (status, every reply, live JSON), never a crash |
| Wizard logic | `carbot_ops/wizard_core.py` | Order enforced on RUN; Save only a PASS; re-save moves the old file aside; session created on first Save; ACTIVE only when every required step passes; resume unfinished session (`resume_max_age_h`); picks up results the terminal CLIs write into the session; rollback |
| Step 1 | `step_sensor_health.py` + `sensor_checks.py` | 11 checks: 3 cameras (roles from cameras.yaml, "?" until step 2 confirms), LiDAR, odom, IMU, UWB tag, all anchors, battery, duplicate / old-viewer processes, ROS network env. Every failure has Why + Fix (Camera_Setup / UWB_Handoff gotchas). Run = 5 s, pass if each check ok in >= 80 % of samples. `sensor_checks` is meant for race preflight too |
| Restart camera drivers | `camera_restart.py` | Kill helper with camera patterns only, both MIPI cameras as root via the sudoers helper (width/height always passed, 2nd delayed), Astra via its base launch. Children of the wizard; stopped again on wizard exit |
| GUI | `web/tabs_calib.js` (+ app.js rail, app.css), `gui_server.py` | Rail: Overview, 13 numbered steps with status dots, Tuning, Diagnostics. Step page = stephead (Prev/Next, Next locked until pass/keep), what to do (live progress), controls, live view, result (Why/Fix per failed check, metrics), Run/Redo/Cancel, Save, Keep previous, embedded diagnostic tab (polled only while expanded). Placeholders show YAML instructions + the terminal command with `--session <wizard session>`. Overview: steps + sessions + two-click rollback |
| Mock | `tools/sandbox/gui_mock_server.py --mode calibrate [--sensors ok\|bad]` | Runs the REAL wizard_core + step 1 on the repo YAML with a synthetic feed |
| Tests | `carbot_ops/test/` (42), bringup key test + 2 nodes | Headless Chromium run of the whole step-1 flow + race tabs: 0 JS errors |

### Contract changes (additions unless marked)
* Topic `/carbot/calibration/live` (`T.CALIBRATION_LIVE`, String JSON, latched): open step's live view,
  result, instructions, sessions, running task.
* **CHANGED** `calibration_steps.yaml`: 13 steps. NEW step 12 `mission_planner` (`required: false`
  until its page exists, else race could never arm); `practice_runs` index 12 -> 13. Ids unchanged
  (all code looks steps up by id). Step 1: new `instructions`, `procedure`, `pass.max_age_s`,
  `pass.processes`.
* `ops.yaml`: calibration_wizard + system_monitor phase-8 keys (no existing key changed).
* Comments only: `CalibrationStepState.index` 1..13, `CalibrationAction.action` + CANCEL, RESTART_CAMERAS.
* gui_server: `POST /api/calibration/action`, `/api/tab/calibration` + `live` + `wizard`,
  `/api/config` + `calib_steps`, `/api/core` + `calib_steps` flags; `TAB_GROUPS.calibration`.

### Page 2 — camera identity (done, untested on the car)
| Piece | Where | Notes |
|---|---|---|
| Step | `carbot_ops/step_camera_identity.py` | Live: one row + preview tile per role (roles as the launch loaded them), picture state from system_monitor (live / frozen / no frames), sensors with `enabled: false` shown as "switched off, skipped". RUN argument JSON `{"confirm": true, "swap": false\|true}` (no argument = refused). Run = `procedure.measure_s` (3 s): every ENABLED role's raw image topic must be live (age <= `pass.max_image_age_s`) in >= `min_ok_fraction` of the reports. Swap refused when both side cameras are off |
| Save | `save_data` -> `<session>/data/cameras.yaml` | `roles` (swapped if asked; all three role keys always kept), `roles_confirmed: true`, `roles_confirmed_for: [enabled roles]`. Front-only car -> `[front]`. Complete file (session copy replaces the repo file on load), merged via `calib_tools.merge_data` so later steps' keys survive |
| Keep previous | `keep_data` | Copies those three keys from the older session; REFUSED if that session's confirmation does not cover every camera enabled now (e.g. a side camera switched back on) |
| Wizard hooks (shared) | `wizard_core.py` | `StepImpl.save_data(session, res)` (before the result file; paths -> `result.data_files`), `StepImpl.keep_data(src, session)` (before anything is recorded), `StepRefused` (abort with a message), RUN/REDO argument reaches `start()` as `inputs['argument']`. `calibration_wizard._setup`: `factories` dict, one line per built page |
| GUI | `tabs_calib.js` `STEP_PAGES.camera_identity`, `calstep` `cams` slot | `out.cams = [{key, label, note, off}]`: persistent camera tiles (ImgLoop keeps running across the 2 Hz re-render; rebuilt only when the camera set changes); `calActions(st, null)` = page has its own Run buttons. Controls: Confirm / Confirm swapped (only when a side camera is on) |
| Mock + tests | `gui_mock_server.py` (steps 1-2), `carbot_ops/test/test_step_camera_identity.py` (18), `carbot_common/test/test_data.py` (+2) | Headless Chrome run of the front-only flow on the mock: step 1 save -> step 2 confirm -> PASS -> Save -> `data/cameras.yaml` roles_confirmed_for [front]; 0 JS errors |

### Contract changes (page 2; additions only)
* `cameras.yaml`: NEW key `roles_confirmed_for: []` (required; only the wizard writes it). Repo file keeps
  `roles_confirmed: false`.
* `carbot_common.data.unconfirmed_roles(cameras)` -> enabled roles step 2 has not confirmed (`[]` = OK).
  Step 1 labels use it (`left side?`). **Race preflight must use it** instead of reading `roles_confirmed` alone.
* `calib_tools.merge_data(session, fname, base_doc, updates)`.
* `calibration_steps.yaml` step 2: `need`, `instructions`, `procedure {measure_s, min_samples,
  min_ok_fraction}`, `pass.max_image_age_s` (+ `writes` lists `roles_confirmed_for`). Id/index unchanged.

### Page 3 — camera intrinsics (done, untested on the car)
| Piece | Where | Notes |
|---|---|---|
| Step | `carbot_ops/step_camera_intrinsics.py` | One camera per Run: RUN argument = sensor name (none = first enabled sensor not passed yet). Sensors with `enabled: false` are listed as "Not detected" and refused. Auto-capture with `calib_core.ViewCollector` (board still for 2 frames, new pose), stops at `pass.min_views + procedure.extra_views` views with `pass.min_coverage_cells` areas, or at `procedure.capture_timeout_s`; the fit (`calib_core.calibrate_intrinsics`, pinhole vs fisheye) runs in a thread. Step PASSES when every ENABLED sensor in `per_sensor` passed (front-only: the Astra alone) |
| Frames | `carbot_ops/frame_tap.py` | The wizard subscribes to a sensor's raw image topic ONLY while step 3 captures it; newest message kept, decoded on demand (`ros_image.image_to_bgr`). `carbot_ops` now exec-depends on `carbot_perception` |
| Save / keep | `save_data` / `keep_data` | `<session>/intrinsics/<sensor>.yaml` (ROS camera_info) + `cameras.yaml sensors.<sensor>.intrinsics_file` via `calib_tools.merge_data` (step 2's roles survive). Keep copies the older session's intrinsics files and repoints `intrinsics_file` |
| GUI | `tabs_calib.js` `STEP_PAGES.camera_intrinsics`, `calstep` `cam` box | Controls: one Run/Redo per enabled camera, "Not detected" rows for switched-off ones. Live: state (no board / hold still / new view captured), views bar, 3x3 coverage grid. `out.cam = {key, label, size, points, color}`: ONE persistent camera box with an SVG overlay of the detected corners (normalised 0..1), next to step 2's `cams` tiles |
| Terminal tool | `carbot_perception/calib_intrinsics.py` | Now refuses a disabled `--sensor` and leaves disabled sensors out of the step's completion (was waiting for all three) |
| Tests | `carbot_ops/test/test_step_camera_intrinsics.py` (9, fake calib_core) | Front-only pass, refused disabled camera, fail reasons (reprojection, no frames), 3-camera ordering, loud missing key, save + keep files. JS syntax-checked (V8); NOT run in a browser or on the car |

### Contract changes (page 3; additions only)
* `calibration_steps.yaml` step 3: block style now, plus `instructions` and `procedure {extra_views,
  capture_timeout_s, novelty, still_px, max_live_corners}` (all required). Id/index/`per_sensor`/`target`/`pass` unchanged.

### For the next page (step 4, extrinsics + IPM)
* Same recipe (`step_<id>.py`, one `factories` line, `STEP_PAGES.<id>`); `FrameTap` gives frames.
* Front-only: only the `front` board counts; left/right boards must be skipped when their sensor is
  disabled (BACKLOG #24). Read intrinsics from this session's `data/cameras.yaml` (step 3 wrote it).
