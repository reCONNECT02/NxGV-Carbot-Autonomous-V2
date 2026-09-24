# UWB positioning (Haffiz's method)

This replaces the position half of `docs/reference/UWB_Handoff.md` (which is kept
for the firmware, hardware and troubleshooting sections — read it first for those).
The tag firmware, message format, network and startup steps in that document are
UNCHANGED: Haffiz's code reads the same `/uwb3/input_json` JSON
(`links[].A`, `links[].R`) the current `TagMicroROS.ino` publishes.

## What changed

Two things replace the pre-Haffiz pipeline (pairwise trilateration + range offsets):

1. **Solver**: `uwb_localization/uwb_core.py` `solve_linear()` — Haffiz's closed-form
   trilateration (`turtle_uwb_visualizer.py solve_trilateration`), ported line for
   line. With exactly 3 anchors it is his 2x2 linear solve; with 4+ it is the
   least-squares solution of the same linear system.
2. **Filter**: `uwb_localization/positioning.py` `CvKalman` — his 2-D constant-velocity
   Kalman filter (`ExtendedKalmanFilter2D`), same process/measurement noise (0.2 /
   0.25) and the same Mahalanobis gate (16.0, ~4 sigma). A unit test loads his
   original class straight from `tools/uwb/haffiz/turtle_uwb_visualizer.py` and runs
   it step by step against the port: the output matches to 1e-12.

Both live in `uwb_localization/positioning.py`, configured once in
`common.yaml` `/**` `uwb_positioning` so the car, the wizard and every tool use the
same numbers:

```yaml
uwb_positioning:
  solver: linear        # linear (Haffiz) | nlls (his noEKF variant) | pairwise (old uwb_xy.py)
  filter: cv_kf          # cv_kf (Haffiz) | moving_average (his noEKF) | none
  cv_kf: {process_noise: 0.2, measurement_noise_m: 0.25, gate_mahalanobis2: 16.0,
          initial_variance_m2: 1.0, reacquire_after_rejects: 30}
```

`reacquire_after_rejects` is the one deliberate difference from his code: after that
many consecutive gated fixes, the filter re-initialises on the new measurement.
His original filter never does this and can stay stuck for a long time after a jump
of more than ~1 m (e.g. the tag rebooted mid-track) while its covariance grows back
up. Set it to `0` for his exact behaviour.

## What publishes what

`uwb_ranges` (block 06 input, unchanged topic ownership):

| Topic | Type | Content |
|---|---|---|
| `/carbot/uwb/ranges` | `UwbRanges` | per-anchor ranges, offset-corrected (offsets now optional, see below) |
| `/carbot/uwb/raw_fix` | `PointStamped` (venue) | one solver fix per report, **unfiltered** |
| `/carbot/uwb/position` | `Odometry` (venue) | Haffiz's **filtered** position: solver + `CvKalman`. Pose covariance = filter covariance, twist = filter velocity. **This is what block 06 uses.** |

Block 06 (`global_pose`) reads `localization.yaml` `global_pose.uwb_input`
(replaces the old `use_per_range_updates` boolean — a renamed key, noted here as
required by the project rules):

- `position` (default) — one whole-fix EKF update per `/carbot/uwb/position` fix,
  R = the filter's own covariance (rotated into the track frame) + a small floor.
- `ranges` — the old per-anchor-range mode, unchanged, still selectable.
- `raw_fix` — the old V4 whole-fix mode on the unfiltered solve.

UWB still never reaches the servo and never writes block 05 (local pose); only the
input to block 06 changed.

## Calibration wizard

**Step 10 (UWB anchor survey + offsets)**: order is now Link → Survey → **Verify**,
with **Offsets optional** (`calibration_steps.yaml` `uwb_survey.procedure.offsets_mode:
optional`). Haffiz's method needs no per-anchor range offset, so Verify runs right
after the survey and checks his **filtered** position against a tape-measured spot.
If it fails because every range reads consistently long or short (uncalibrated
antenna delay), the page sends you to Measure offsets, then Verify again at a
different spot. Set `offsets_mode: required` to force the old order back.

Any metre frame works for the anchor survey — the map is fitted onto it in step 11,
not the other way round. Haffiz's tested layout (metres, if your anchors stand
where he measured them): `1786 (-2.5, 0)`, `1782 (6.0, -0.6)`, `1783 (2.5, 8.0)`.

**Step 11 (map-to-UWB alignment)**: new default mode, **UWB lap**
(`calibration_steps.yaml` `map_uwb_alignment.procedure.modes: [uwb_lap, lap,
points]`). Push or drive the car slowly once around the whole track — any start
point, no odometry, no IMU, no exact start pose needed. The page records Haffiz's
filtered positions, moves each one from the tag antenna to the rear axle using the
filter's own velocity heading, and fits the team's map (`tools/map/map_builder.py
fit_rigid`) onto the lap. `track_map.yaml` is **not** rewritten, so `mission.yaml`
(step 12) stays valid; Save also writes `captures/step11_uwb_lap.csv` for
`map_builder.py edit` if the road shape itself needs a touch-up (redo steps 11 and
12 after that).

The old **Odometry lap** and **Points** modes are kept as fallbacks (car exactly on
the start pose, or parked on named map poses).

## Tools

| Tool | Replaces | Notes |
|---|---|---|
| `ros2 run uwb_localization record_lap` | — | writes `lap.csv` straight from `/carbot/uwb/position`, for `map_builder.py edit` outside the wizard |
| `tools/uwb/uwb_xy.py` | old pairwise viewer | Haffiz's method, anchors + settings from `uwb.yaml` / `common.yaml` |
| `tools/uwb/uwb_turtle.py` | Haffiz's `turtle_uwb_visualizer.py` | his turtle viewer, wired to read the stack's `uwb.yaml` and `uwb_positioning` settings instead of hard-coded anchors |
| `tools/uwb/uwb_calib.py` | `calib_uwb.py` offset check | now optional debug helper; step 10 does this in the wizard |
| `tools/uwb/haffiz/` | — | his original files, **committed unchanged**, used as the reference in `test_positioning.py` |

## Startup

Unchanged from `UWB_Handoff.md` section 9, except `run_uwb_agent.sh` now also
looks for `~/microros_ws` (his workspace name) if `uwb.yaml agent.workspace`
(default `~/uros_ws`) has no built agent.

## GUI

The Localization diagnostic tab draws Haffiz's filtered position as a purple dot
with a 2-sigma uncertainty ellipse and trail, alongside the existing local (blue),
global (green) and raw-solve (orange) traces. Steps 10 and 11 in the calibration
wizard show the same colour scheme and his method name (`linear + cv_kf`) in their
result panels.
