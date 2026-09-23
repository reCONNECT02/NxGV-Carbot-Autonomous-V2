# Phase 8 — wizard pages 7 to 13: handoff (2026-09-23)

Written at the end of a session that used one agent per wizard page, working in
parallel. **The code for pages 8-13 is NOT on `main` yet** (only this document is) and
**nothing has run on the car.** `docs/PHASES.md` has a "Phase 8 progress map" pointing here and
`docs/BACKLOG.md` has rows #36-#41; the full per-page PHASES sections (drafts at the end of this
file) go in when the branches below are merged.

## Branches on GitHub

| Page | Step id | Remote branch | Head | State |
|---|---|---|---|---|
| 7 | servo_steering | — (on `main`, 795d5d3, built by another session) | | done on main |
| 7 (duplicate) | servo_steering | `phase8/page-07-steering-superseded` | 1eca73f | **do not merge** — superseded by main's page 7; kept for reference (its `wizard_drive.py`) |
| 8 | speed_pid | `phase8/page-08-speed-pid` | 4afe92e | done; built on main 795d5d3 (reuses main's DriveRequests / MotionRecorder / ServoLink) |
| 9 | venue_thresholds | `phase8/page-09-venue-thresholds` | a34e501 | done (based on 09b604f) |
| 10 | uwb_survey | `phase8/page-10-uwb-survey` | b7c6b3c | done (based on 09b604f) |
| 11 | map_uwb_alignment | `phase8/page-11-map-uwb-alignment-wip` | 0540295 | **WIP**: backend, tests, GUI page and mock written; final test run + browser check + report not done. Contains page 10 merged with main 795d5d3 (4e14d33) |
| 12 | mission_planner | `phase8/page-12-mission-planner` | 20b30aa | done (based on 09b604f) |
| 13 | practice_runs | `phase8/page-13-practice-runs` | 6c41baa | done (based on 09b604f) |
| 9+12+13 | — | `phase8/integration-pages-9-12-13` (this branch) | see git log | main 795d5d3 + pages 13, 9, 12 merged (c13032f, cdddad1, 242fb78). **Post-merge review / tests / browser check NOT finished** (agent stopped by usage limit) |

## What is left (plan)

1. **Finish the integration branch** (this one): review the three merge resolutions —
   `calibration_wizard.py` (REQUIRED keeps main's keys + `data.challenges`; factories and
   docstring in step order), `gui_mock_server.py` (one MockWizard for all pages; unify the
   page-skip flags: main's `--skip-to N` vs the pages' `--practice`, `--prepass N`, `--unlock N`),
   `tabs_calib.js`, `wizard_core.py`, YAML. Then run pytest ONE PACKAGE AT A TIME
   (`carbot_ops`, `carbot_common`, `carbot_bringup`, `carbot_control`, `carbot_gui`, tools/map).
   Known Windows-only failures: `test_import::test_modules_import` (no rclpy),
   `test_unwritable_root_gives_message` (os.geteuid), `test_root_helper_fallback`.
   Browser: mock + headless Chrome, every page renders with 0 JS errors.
2. **Finish page 11** on its branch: run tests, browser check on the mock, final commit.
3. **Merge into the integration branch**: `phase8/page-08-speed-pid`, then
   `phase8/page-11-map-uwb-alignment-wip` (brings page 10). Resolve shared files keeping both sides.
4. **Deduplicate param helpers**: main has `servo_link.ServoLink`; page 9 added
   `param_link.ParamLink` (same interface). Make page 9 use ServoLink, delete param_link.py.
5. Update `docs/PHASES.md` (pages 8–13 sections, drafts below) + `docs/BACKLOG.md`, commit,
   merge to `main`.
6. Car tests on the RDK X5 (see "Untested on hardware").

## Shared contracts between pages

* **Steps 7 + 8 (driving)**: main's `calibration_wizard.DriveRequests` (`command(source, speed,
  steer, reason)` / `stop()`, repeated at `drive_request_hz`), `step_imu_odometry.MotionRecorder`
  (`self.motion`; page 8 adds optional `v` + `.trace`), `servo_link.ServoLink` (`self.servo`,
  `self.owner` = command_owner), `StepImpl.ops_while_running` + `handle()` for Go-per-segment.
  `CALIBRATION_RAW` = duty, `CALIBRATION` = closed-loop speed. STOP MOTORS calls `drive.stop()` first.
* **Steps 10 + 11 (UWB)**: `carbot_ops/wizard_uwb.py` — raw `/uwb3/input_json` (BEST_EFFORT)
  -> `inputs['uwb_raw']` UwbFeed (`cursor()`, `rows_since()`, `stats()`), `session_uwb(session, base)`
  gives step 10's saved offsets, `anchor_set`, `fresh_samples`, `fixes`, `SyntheticTag`.
  `<session>/data/uwb.yaml`: step 10 merges ONLY `anchors` (id, xyz_m, range_offset_m), `tag.z_m`,
  `tag.mount_xy_m`, `anchors_surveyed`, `offsets_calibrated`; step 11 merges ONLY `track_to_venue`.
* **Step 12**: reads `<session>/data/track_map.yaml` else the repo file (same for mission_rules,
  track_features, mission.yaml); records `map.sha1`; Keep previous refused after a map edit.
  Step 12 is now `required: true` (BACKLOG #20 -> DONE).
* Every data-writing page implements `keep_data` (BACKLOG #21).
* Shared GUI additions: `STEP_ARGS` + `setKeep` (page 10: form values survive the 2 Hz re-render),
  `out.widget = {id, create, data}` (page 12: persistent canvas), result `message` and
  `progress()['summary']` shown by wizard_core.

## Decisions / questions for the owner

1. **Practice runs can't drive (page 13)**: in calibrate mode nothing publishes `/carbot/race/armed`,
   so mission_logic stays IDLE. How should practice be armed? Until then attempts record time +
   the user's grade only.
2. **Probable bug (found by page 9)**: road_perception seed box `seed.x -0.25..0.45 m` is outside
   the front camera view (floor visible from x ≈ 0.47 m) on the front-only car -> `grown` /
   `connected` always 0. Move it forward (e.g. 0.47..0.70)?
3. Page 9 does not write `bpu_detector.thresholds.*` (needs labelled venue footage, #15/#16),
   but step 9 `writes:` still lists them. OK?
4. Page 12: P2/P3 must lie inside their `mission_rules.yaml` bays, and a PASS requires the race
   route check (exits must match `roundabout_visits`). Intended strictness? Planner car size comes
   from `map_builder.CAR`, not common.yaml — tie them together?
5. Page 13: step shows FAIL between attempts (no in-progress state in wizard_core); attempts are
   lost if the wizard restarts before Save; file names `practice/NN_<slug>.yaml` — OK?
6. Page 8 needs ~10 Go presses per run; creep speeds below servo_controller's 0.02 m/s odom
   deadband read 0.
7. `test_import.py` exists in several packages -> they can't run in one pytest command. Rename?

## Untested on hardware (all pages)

* Page 8: command_owner accepting the wizard's CALIBRATION / CALIBRATION_RAW, live
  feedforward/PID param set, 1.5 m of floor at duty 0.22 for 3 s.
* Page 9: ROI boxes on the floor the Astra sees, tunnel box on the curve, stitched-JPEG CPU
  cost, `road_max_chroma` tightness, ParamLink against real road_perception.
* Page 10: micro-ROS feed at 10 Hz into the wizard, offsets (~1 m) + verify ≤ 15 cm with raised
  anchors, RDK CPU.
* Page 11: everything (WIP).
* Page 12: planning time on the RDK (laptop: ~150 s for three legs), planner folder found from a
  `--symlink-install` build, OpenCV in the wizard node, canvas touch/drag on the tablet.
* Page 13: real MissionState/MissionEvent timing, challenge-exit detection.

## Draft PHASES.md sections (from each page's agent)

### Page 8 — speed PID
| Piece | Where | Notes |
|---|---|---|
| Step | `carbot_ops/step_speed_pid.py` | Supervised drive, Go before every segment. CALIBRATION_RAW duty sweep (`sweep_duties`, alternating direction) -> `calib_core.fit_feedforward` set live -> CALIBRATION steps at `step_targets_mps` (steady error, overshoot), kp/ki `retune` up to `max_runs`; stops early when a retune changes nothing. Aborts on stale /odom (`odom_timeout_s`) or a refused param set; a non-passing run restores the starting command_owner values |
| Save / keep | `calib_speed.write_overlay` / `OVERLAY_KEYS` | command_owner.feedforward.*, speed_pid.*, tunnel_bridge.command_owner_feedforward.* + captures/speed.json (`calib_speed --replay`). Save refused if live values differ from the verified ones. Keep copies + sets live |
| GUI / mock / tests | `STEP_PAGES.speed_pid`; mock motor model + real owner_core SpeedController (`--skip-to 8`); `test_step_speed_pid.py` (22) | Headless Chrome: 10 Go -> PASS -> Save, 0 JS errors |
Contract: step 8 `need`, `procedure.stop_settle_s`, `odom_timeout_s`, `instructions` list; `MotionRecorder.on_odom(..., v)`.

### Page 9 — venue colour / lighting thresholds
| Piece | Where | Notes |
|---|---|---|
| Step | `carbot_ops/step_venue_thresholds.py` | RUN phase `road` / `tunnel` / `apply` / `revert`. road_max_luma/chroma = P99 of the box + margin; paint_min_luma = midpoint road bright end / tape dark end (≥ road + min_paint_margin). PASS: coverage ≥ 0.5, false paint ≤ 0.05, tape seen, decode agreement ≥ 0.9, tunnel mean luma ≥ tunnel_min_luma, values applied + read back. bpu thresholds NOT changed |
| Inputs | `road_tap.py`, `param_link.py` | LocalGrid (page open) + stitched debug JPEG (while sampling), paired by stamp; no raw images in the wizard |
| Save / keep | overlay `road_perception.classify.*` (ints) | keep copies them, refused if missing |
| GUI / mock / tests | `STEP_PAGES.venue_thresholds` (SVG grid map); `MockVenue`; `test_step_venue_thresholds.py` (23) | Mock flow road -> tunnel -> apply -> Save ran (JS console not captured) |
Contract: step 9 block style + `need`, `instructions`, 16 `procedure` keys; inputs `road_grid`, `road_pair`, `road_params`.

### Page 10 — UWB anchor survey + offsets
| Piece | Where | Notes |
|---|---|---|
| Step | `carbot_ops/step_uwb_survey.py` | RUN `{"stage": link\|survey\|offsets\|verify, ...}`. Link: every anchor ≥ max(3, s) fresh samples. Survey: xyz per anchor + tag z/mount; spacing, triangle angle, extent (cm typo), HDOP. Offsets: median raw R − tape 3-D distance. Verify: ≥ `min_verify_separation_m` away, ≥ `min_fixes`, error ≤ 0.15 m. Survey change clears offsets/verify |
| Save / keep | `<session>/data/uwb.yaml` | only the step-10 keys (see contracts); keep refused if the old survey lacks a configured anchor |
| UWB feed | `carbot_ops/wizard_uwb.py` | shared with step 11 |
| GUI / mock / tests | `STEP_PAGES.uwb_survey`, `STEP_ARGS`, `setKeep`; synthetic tag, `--unlock`; `test_step_uwb_survey.py` (24) | Headless Chrome link -> survey -> offsets -> verify -> Save, 0 JS errors |
Contract: step 10 `need`, `procedure.min_verify_separation_m`, `min_fixes`, `max_extent_m`; ops.yaml `uwb_buffer_s`, `uwb_rate_window_s`; carbot_ops exec-depends uwb_localization; wizard_core result `message`.

### Page 11 — map-to-UWB alignment (WIP)
`carbot_ops/step_map_uwb_alignment.py` (lap + points modes, CLI robust fit, merges only
`track_to_venue`, keep refused if anchors/offsets differ), `test_step_map_uwb_alignment.py`,
`STEP_PAGES.map_uwb_alignment` + calstep map box, mock lap hand + parked points. Needs final
test run + browser check.

### Page 12 — mission planner
| Piece | Where | Notes |
|---|---|---|
| Step | `carbot_ops/step_mission_planner.py` | Imports tools/map/mission_planner.py (`procedure.planner_dir`, `$CARBOT_REPO`). Map = session track_map else repo. RUN `{"poses": [[x, y, yaw_deg] x4]}`. Thread plans leg by leg (unchanged legs reused). Checks: pose fits, parking legs end in their bay, every leg routed, block-07 race check |
| Save / keep | `<session>/data/mission.yaml` (map.file + map.sha1) | keep refused if sha1 ≠ this session's map; save refused if the map changed after Run |
| GUI / mock / tests | `STEP_PAGES.mission_planner` (click/drag poses, snap, routes, exits), `out.widget`; `--unlock 12`; `test_step_mission_planner.py` (16) | Headless Chrome plan -> PASS -> Save -> ACTIVE, 0 JS errors |
Contract: CHANGED step 12 `required: true`; + `need`, `procedure {...}`, `pass {...}`; `progress()['summary']`; exec-depends carbot_planning.

### Page 13 — practice runs (optional)
| Piece | Where | Notes |
|---|---|---|
| Step | `carbot_ops/step_practice_runs.py` | Never commands motion. RUN `{"op":"start","challenge":N}` / `grade` / `done`. Attempt ends on challenge exit, COMPLETE, manual intervention (auto FAIL), Stop, or `max_attempt_s` |
| Save / keep | `<session>/practice/<NN>_<slug>.yaml` + summary.yaml | keep copies `practice/` |
| Wizard node | read-only subs mission state/events, race/armed, manual/takeover | `data.challenges` required |
| GUI / mock / tests | `STEP_PAGES.practice_runs`; `--practice`; `test_step_practice_runs.py` (15) | Headless Chrome start -> end -> grade -> done -> Save, 0 JS errors |

## BACKLOG rows to add / update
* #13: step 7 page on main, step 8 page on branch; run both on the car, confirm `vehicle.wheelbase_m`.
* #20 -> DONE (8): step 12 required.
* #21: steps 8, 9, 10, 11, 12, 13 implement `keep_data`.
* #24: step 9 needs the front camera; side cameras irrelevant.
* NEW: practice arming in calibrate mode (Q1). NEW: road_perception seed box outside front view (Q2).
* NEW: car tests per page (list above). NEW: merge ParamLink into ServoLink. NEW: rename duplicate `test_import.py`.
