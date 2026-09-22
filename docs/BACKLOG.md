# Backlog: things to change or add

Read at the start of every phase chat; updated at the end of it.
Status: TODO / DOING / DONE (phase) / WAITING (on the team).

| # | Request | Phase | Status | Notes |
|---|---|---|---|---|
| 1 | Stack reads track_map.yaml + mission.yaml **version 2** (from tools/map), still reads version 1 | 4 | DONE (4) | v2 is the default; v1 kept in `config/data/v4_reference/` |
| 2 | Leg 3: parallel bay -> perpendicular bay (Challenge 11) | 4 road part, 5 bay exit/entry | DONE (5) | Un-park = time-reversed parking search; park perpendicular via docking straight. Sim: both PARKED |
| 3 | Boom gate open/closed model (front camera) | 6 | WAITING | Team is training its own model; plug it in when supplied. Mission logic reads `DetectionArray.boom_gate_state` (+ confidence) only |
| 4 | Boom gate positions on the track (challenge4 + roundabout gate) | 4/8 | WAITING | `track_features.yaml boom_gates.*` (provisional, V4/rulebook values). Measure on site |
| 5 | Show all speed zones on the map tab | 7 | TODO | Zones: `mission_rules.yaml speed_zones`; active zone in `MissionState.speed_zone` |
| 6 | Change the speed of each zone from the GUI | 7 | TODO | Tuning tab edits `mission_rules.yaml speed_zones[].max_speed_mps`; calibrate mode only (race = read-only) |
| 7 | Corridor.msg frame comment: odom -> track | 4 | DONE (4) | |
| 8 | Confirm P1 (x 6.75, y 2.47, facing south) is the traffic-light stop line | 4 | WAITING | `track_features.yaml light_goal_pose` + `traffic_light` position derived from it (provisional) |
| 9 | Map lap must also drive the parking road (parking_spur, parking_corner, perp_row not fitted) | map / 8 | WAITING | `track_map.yaml fit_report`; then re-run `mission_planner.py` |
| 10 | Measure speed bump, hill, tunnel positions | on site | WAITING | `track_features.yaml` (provisional V4 values) |
| 11 | After ANY map edit, re-run `mission_planner.py` | always | note | Otherwise block 07 fails: "planned on a different track_map.yaml" |
| 12 | Parking previews are at the minimum turning radius: block 11 must replan from the actual pose | 5 | DONE (5) | Plans from the pose, cusp + heading replans, `plan_radius_factor 1.08` |
| 13 | Run calibration steps 7/8 on the car (feedforward / steering are placeholders) | 8 / on site | TODO | `calib_steering`, `calib_speed`; also confirm `vehicle.wheelbase_m` |
| 14 | Measure planner / parking CPU on the RDK X5 | on site | TODO | Laptop: parking 0.04-0.5 s, recovery search ~0.3 s |
| 15 | Collect boom gate footage (open + closed, several distances) from our Astra, validate, then set `challenge4_gate.enabled: true` | 6 / on site | TODO | Team model had 0 boom validation images. `docs/DETECTORS.md` |
| 16 | Validate the light on the car, then set `traffic_light.enabled: true` | 6 / on site | TODO | While false, challenge 7 = 0 but the run never waits. Check `object_size_m` and `gate_association` tolerances too |
| 17 | `run_planning.py --v4` crashes: v1 manoeuvre piece has no points, so `ParkingSession.start` has no goal | 5 | TODO | Found in phase 6. Team mission, race and tests unaffected. Fix: goal from the v1 bay, or `parking='preview'` for `--v4` |
| 18 | Run calibration step 1 on the car; check system_monitor CPU with 3 raw camera subscriptions | 8 / on site | TODO | `top`: system_monitor should be well under 10 % of a core. If high, lower watch set or add a sampled mode |
| 19 | `bpu_ratio_path` (BPU load) still unverified | 8 / on site | TODO | system_monitor logs one warning and reports -1 if the file is missing |
| 20 | Make step 12 `mission_planner` required once its page exists | 8 | TODO | calibration_steps.yaml `required: false` for now |
| 21 | KEEP_PREVIOUS for data-writing steps must copy their data files | 8 | TODO | Disabled for placeholders until each page implements it |
| 22 | mipi_cam auto-detects sensors and ignores `channel`: with one MIPI sensor missing, the other one is published under the wrong namespace | 8 | TODO | Seen 2026-09-23 on risabot5: IMX219 undetected, the `/cam_imx219` (ch 0) process opened the OV5647 (i2c4@0x36, host 2), the `/cam_ov5647` process then failed with "detected sensors are 1 less than expected". Step 1 should read the "cap <sensor> init success" line (/tmp/carbot_root_logs) or `dmesg` and fail the row on a sensor/namespace mismatch |
| 23 | Step 1 on the car: CPU load ~16 with calibrate.launch.py | 8 / on site | TODO | Seen 2026-09-23 on risabot5: gui_server ~60 %, system_monitor ~50 %, most planning/control nodes 35-45 % of a core each (old risabot5-track-stack.service was also running; now disabled). Re-measure alone, see #18 |
| 24 | Side MIPI cameras (OV5647, IMX219) switched OFF: `cameras.yaml sensors.<name>.enabled: false`, stack runs front-only | 8 | DOING | 2026-09-23 risabot5: IMX219 not detected, OV5647 MIPI frame errors (also alone, as in Camera_Setup.md), cable reseat did not help. Launch, camera restart, step 1 checks and road_perception skip disabled sensors. Still to honour it: steps 2 (roles), 3 (`per_sensor`), 4 (left/right boards), race preflight. Re-enable: fix hardware, set `enabled: true`, rebuild |

## How to add a request
Tell Claude in any chat, or add a row here yourself (next number, phase if known, status TODO).
