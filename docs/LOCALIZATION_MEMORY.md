# Local localization and memory (front camera only)

Written 2026-09-24 after the two MIPI side cameras were removed (docs/PHASES.md "Front camera only"). Scope: what block 04
(memory) and block 05 (local pose) do today, an ICP-style correction against the accumulated camera edges, and what the
offline evaluation says. **Everything here is sandbox-only: nothing has run on the car (RDK X5).**

## 1. What exists (unchanged reference behaviour)

* **Block 03** (`road_perception`) turns the front camera into a top-down `LocalGrid` (`kind` 1 road / 2 paint / 3 other,
  `grown` = the connected road), 100 x 100 cells of 1.8 cm, at `process_rate_hz` (8 Hz, 4 Hz in the lite profile).
* **Block 04** (`local_memory`, `memory_core.py`, V4 `LocalMemory`): road/paint cells are stored in the **odom** frame (wheel +
  IMU dead reckoning, never corrected: a corrected frame would shift stored cells every time a correction lands, V4
  `estimate.odom`), each with time and travelled distance. Consumers choose their own limit: drive 3 s, corridor 2 s, parking
  24 s, and the V4 motion-uncertainty rule `0.004 + 0.006 * travel <= 0.02`. LiDAR is not stored. I found nothing in the V4
  reference that the port misses, and no memory change I could justify **and** test offline, so block 04 is untouched.
* **Block 05** (`local_pose`, `estimator_core.LocalEstimator`, V4 `Estimator`): pose = odom + transform. Predict: wheel
  distance + IMU heading (`heading_blend` 0.25). Correct (`visual_update`): paint-edge cells next to grown road are matched to the
  **prior map boundary** (`course.clearance == 0`), a damped weighted least-squares step, **translation only**, at most 1.5 mm and
  12 % of the solution per frame, so the pose cannot jump. Heading is never corrected by the camera. UWB never enters.

## 2. What was added

| Piece | Where | Default |
|---|---|---|
| ICP against a private edge memory | `carbot_localization/icp_core.py`, wired in `local_pose._on_grid`, YAML `local_pose.icp.*` | **off** (`icp.enabled: false`) |
| IMU lag lead | `LocalEstimator.predict`, YAML `local_pose.heading_lead_s` | **off** (0.0 = V4) |
| Evaluation harness | `tools/sandbox/run_icp_eval.py` | - |
| Unit tests | `carbot_localization/test/test_icp_core.py` (13) | - |

### ICP (`icp_core.py`, numpy only)

1. `edge_points`: paint cells touching grown road within `icp.max_radius_m` -> points in base_link (same selection as
   `visual_update`), thinned to `icp.max_points`; `normals`: local line normal by PCA of the k nearest points.
2. `EdgeMemory`: those points stored in the odom frame with time and travelled distance (a private, low-rate sibling of block 04:
   no new topic and no new subscriber, it lives in `local_pose` and reuses the road grid it already receives). A point is used only
   while `min_age_s <= age <= max_age_s` and the V4 rule `unc_base + unc_per_m * travel <= unc_limit` holds. Points younger than
   `min_age_s` are the same view (no information).
3. `register`: robust **point-to-line** ICP, 3 DOF (dx, dy, dtheta in the car frame): nearest-neighbour pairing within
   `pair_max_dist_m`, trimmed to the best `trim_fraction`, Huber weights, Levenberg-free normal equations with an **eigenvalue
   cut-off** (`eig_min_ratio`): on straight edges along-track motion is unobservable and gets *no* correction (`constrained == 2`).
4. `IcpCorrector`: gates (`min_inliers`, `min_inlier_ratio`, `max_rms_m`, implausible size, standing still) then applies only the
   share of the residual that accumulated since the last update (`interval / mean point age`) x `gain`, clamped per update
   (`max_step_m`, `max_step_rad`) and in total (`max_total_m`, `max_total_rad`). `LocalEstimator.apply_body_step` moves the transform
   and a new **heading trim** `ta` (pose heading = odom heading + ta). Rejected fits are counted (`icp acc/att` in the node status).

The estimate never jumps: every applied step is capped (test `test_corrector_never_exceeds_its_caps...`). UWB is not an input.
The memory stays in the uncorrected odom frame; only the pose receives the correction (the same rule as the map matching).

### IMU lag lead (`heading_lead_s`)

`/imu/rpy` yaw is `servo_controller`'s EMA: `alpha = 0.15` per IMU sample (`control_servo/servo_controller.py`, "strong filter for
micro-changes"), i.e. a time constant of ~0.28 s at 20 Hz. In a turn the reported heading **trails the real one by
yaw_rate x tau** (25 deg/s -> ~7 deg). `heading_lead_s > 0` adds `lead x odom_dyaw / dt` (the wheel-derived yaw rate: no lag, small
bias) to the IMU target. 0 = V4 behaviour.

## 3. Evaluation (`tools/sandbox/run_icp_eval.py`, laptop, Python 3.13, seed 1)

Simulated lap of the outer loop (28 m, smoothed so the heading changes at a finite rate), 0.3 m/s, camera grid rendered from the
map at the true pose (60 ms old, 8 Hz), wheel scale error 2 %, IMU noise 0.35 deg, IMU EMA alpha 0.15, `/odom` yaw-rate error 5 %.
Errors are against the **physical truth**; "sideways" is across the truth heading (what steering feels).

| Scenario | Mode | pos rms cm | sideways rms cm | along-track rms cm | heading rms deg | largest step beyond motion mm |
|---|---|---|---|---|---|---|
| nominal | odom only | 24.7 | 14.8 | 19.7 | 7.1 | 0.3 |
| nominal | map (today) | 15.3 | 8.7 | 12.5 | 7.1 | 1.8 |
| nominal | icp only | 23.7 | 14.3 | 18.9 | 7.1 | 0.8 |
| nominal | map + icp | 14.4 | 8.4 | 11.7 | 7.1 | 1.8 |
| map_offset (prior map off by 2 cm, 1.5 cm, 0.3 deg) | map | 13.5 | 7.7 | 11.1 | 7.1 | 1.8 |
| map_offset | map + icp | 12.6 | 7.3 | 10.3 | 7.1 | 1.95 |
| noisy (10 % paint dropout, false paint, jitter) | map / map + icp | 15.0 / 14.6 | 8.4 / 8.2 | 12.4 / 12.0 | 7.1 / 7.0 | 1.8 / 3.2 |
| imu drift 0.05 deg/s | map / map + icp | 27.4 / 26.1 | 13.2 / 12.7 | 24.0 / 22.8 | 8.0 / 7.8 | 1.8 / 2.1 |
| imu drift 0.3 deg/s | map / map + icp | 90.5 / 90.2 | 44.2 / 44.4 | 79.0 / 78.6 | 16.0 / 14.1 | 1.8 / 1.8 |

**With `heading_lead_s` (same runs, map matching on):**

| heading_lead_s | pos rms cm | sideways rms cm | heading rms deg |
|---|---|---|---|
| 0 (today) | 15.3 | 8.7 | 7.1 |
| 0.20 | 8.0 | 6.3 | 3.5 |
| 0.28 (= EMA tau) | 4.2 | 3.2 | 2.4 |
| 0.35 | 3.7 | 1.9 | 2.1 |
| 0.40 | 4.2 | 1.6 | 2.3 |
| 0.45 | 5.1 | 1.5 | 2.9 |
| 0.35, `/odom` yaw-rate error 15 % | 4.0 | 1.7 | 2.2 |
| IMU alpha 1.0 (no lag at all, for reference) | 4.3 | 3.2 | 2.7 |

ICP on top of `heading_lead_s = 0.35` (map matching on): nominal 3.70 -> 3.78 cm, map_offset 3.40 -> 3.68 cm, noisy 5.00 -> 6.14 cm.

### What the numbers say (honestly)

1. **The ICP against the accumulated camera edges does not pay for itself.** It moves the local estimate by 3-6 % at best when
   the heading lag is present (15.3 -> 14.4 cm; 13.5 -> 12.6 cm with a wrong map), is neutral or slightly worse once the lag is
   compensated, and worse with noisy edges. It recovers about 60 % of a 0.3 deg/s heading drift only with a narrow 1-2 s memory
   band and raised caps. Reasons: (a) the map matching already corrects sideways position well when the heading is right;
   (b) straight lane edges cannot constrain along-track error (the 2 % wheel scale error, 12 cm rms along-track, stays); (c) the
   heading signal per update (0.03 deg at 1.8 cm cells and ~1.5 s of baseline) is close to the raster noise, so the slew-limited
   correction can only follow very slow drift. **It stays off (`icp.enabled: false`).**
2. **The dominant local error is the IMU lag, not drift.** Heading rms 7 deg in the sim comes from the EMA (alpha 0.15) plus
   `heading_blend` 0.25; with no lag at all the map-matched error is 4.3 cm rms. `heading_lead_s` ~0.35 reaches the same (3.7 cm)
   with the real (lagged) IMU and is robust to a 15 % `/odom` yaw-rate error. The value that fits best (0.35-0.4) exceeds the EMA
   time constant (0.28 s) because `heading_blend` adds ~0.15 s of its own lag. **Recommended next step: try `heading_lead_s: 0.3-0.35`
   on the car** (it is a one-line YAML change and off by default).
3. Along-track error (wheel scale) is only fixed by landmarks (stop line, roundabout, corners), not by lane edges.
4. **Cost.** One ICP call (edge extraction + registration + memory add, 100-120 points, 300 reference points) is ~8 ms mean /
   30 ms max on the laptop in Python; extraction runs only on grids that are used. At `every_n_grids: 4` (2 Hz) that is ~1.6 % of a
   laptop core; on the RDK X5 (6-10 x slower Python, 97 % saturated, BACKLOG #47-#51) expect **10-16 % of a core** while enabled.
   No new topic and no new subscriber is added.

## 4. Tunables (`localization.yaml local_pose`)

`heading_lead_s`; `icp.enabled`, `icp.every_n_grids`, `icp.max_radius_m`, `icp.sample_stride`, `icp.max_points`, `icp.normal_k`,
`icp.memory.{add_every_n_grids,max_age_s,min_age_s,max_points,voxel_m,uncertainty_base_m,uncertainty_per_m,uncertainty_limit_m}`,
`icp.{max_iter,pair_max_dist_m,trim_fraction,huber_m,min_inliers,min_inlier_ratio,max_rms_m,min_ref_points,eig_min_ratio,rot_arm_m,
max_fit_translation_m,max_fit_rotation_rad,gain,max_step_m,max_step_rad,max_total_m,max_total_rad,min_speed_mps}`.
All are REQUIRED by `local_pose` (a missing key is a loud error). `python tools/sandbox/run_icp_eval.py --set icp.gain=1.0 ...`
overrides any of them.

## 5. Not tested / to verify on the car before enabling anything

* Everything above is the sandbox (rendered grids, an assumed noise model). The real camera grid has lighting, perspective
  and calibration errors the sandbox lacks; the prior map is not the real venue.
* **`heading_lead_s`**: on the RDK, record a run (`/odom`, `/imu/rpy`, `/carbot/perception/road_grid`, `/uwb3/input_json`) and
  replay it with `run_localization.py --bag ...` at 0 / 0.3 / 0.35; on the track check the local heading in a curve against the
  road (GUI Localization tab: sideways error should shrink); watch that `LocalizationStatus.local_sigma_m` and the `vis` row count do
  not get worse.
* **ICP** (only if someone insists): set `icp.enabled: true`, watch the `local_pose` status detail (`icp acc/att`, ms, trim), the
  CPU of `local_pose`, and that `trim` stays inside +-`max_total_rad`.
* The lite calibrate profile and race mode use the same YAML; nothing here is armed by default.
