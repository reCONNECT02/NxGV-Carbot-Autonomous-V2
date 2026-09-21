# Node sandbox (VS Code, no ROS)

Test every phase-2 and phase-3 node on a laptop (Windows, macOS or Linux) before it goes on
the car. The runners import the **real** code from `src/` and read the **real**
YAML from `src/carbot_bringup/config/`. Only the ROS plumbing (topics, timers) is
replaced by a plain loop, so a fix made here is a fix on the car.

## Setup (once)

1. Install Python 3.8–3.12 and VS Code with the **Python** extension.
2. Open the repo folder in VS Code (`File > Open Folder`).
3. Create a virtual environment: `Ctrl+Shift+P` → **Python: Create Environment** →
   **Venv** → tick `tools/sandbox/requirements.txt`.
   Or in a terminal:
   ```bash
   python -m venv .venv
   .venv\Scripts\activate          # Windows   (macOS/Linux: source .venv/bin/activate)
   pip install -r tools/sandbox/requirements.txt
   ```
4. Open **Run and Debug** (`Ctrl+Shift+D`), pick a configuration, press **F5**.

The unit tests appear in the **Testing** panel (flask icon). The one test that
needs ROS (`test_import`) is skipped on a laptop.

## What each runner shows

| Node | Runner | What to look at |
|---|---|---|
| 03 `road_perception` | `run_road_perception.py` | top row: the 3 cameras with the road mask drawn on them (`o` toggles); bottom: per-camera warps, stitched top view, mask (green = connected road, cream = paint, grey = other, dark = unseen). Trackbars = classify thresholds (like calibration step 9). |
| 04 `local_memory` | `run_local_memory.py` | memory window around the car, shaded by age; right: what the driving rule still accepts. Add odom drift to see the memory smear. |
| 05/06 `local_pose` + `global_pose` | `run_localization.py` | a lap of the track: truth (black), block 05 local (blue), block 06 global (green dashed), raw UWB fixes (orange). Text report: errors, largest step (must stay ~1.5 mm: no jumps), visual match rate, per-anchor innovation and gate rate, suggested `range_sigma_m`. Add `--start-error`, `--drop 1783`, `--spikes`, `--no-camera`. **`--bag <folder>`** replays a rosbag from the car (needs `pip install rosbags`); add `--session` to use that calibration's `uwb.yaml`. |
| `camera_preview` | `run_camera_preview.py` | GUI preview after the real JPEG round trip, KB per frame, bandwidth. `p` switches to the rosbag record stream. |
| step 3 `calib_intrinsics` | `run_calib_intrinsics.py` | `--synth fisheye`: calibrates a virtual lens and prints the error against the truth. `--webcam 0`: the real auto-capture with your laptop camera and the printed board. |
| step 4 `calib_extrinsics` | `run_calib_extrinsics.py` | `--synth`: renders the floor boards through cameras with a known mount error and checks the tool recovers it; shows detections and the IPM check. |

Every runner takes `--help`. `--headless` saves frames to `tools/sandbox/out/`
instead of opening windows. `tools/sandbox/out/` is git-ignored.

## The virtual world

`sandbox_common.TrackWorld` rasterises `track_map.yaml` (5 mm per pixel):
dark road, white lane edges and markings, green outside. The virtual cameras
use the mounts in `cameras.yaml` and either the calibrated intrinsics it points
to or an ideal lens from `hfov_deg`. The colours are guesses: once you have
photos of the real mat, set `ROAD`, `PAINT` and `OUTSIDE` at the top of
`sandbox_common.py` to match.

## Using real data from the car

* **Camera recordings:** record the three topics on the car, export them to
  .mp4, then run the "recorded videos" configuration (edit the paths).
* **A calibration session:** copy `~/carbot_data/calibration/<session>/` from
  the RDK. Pass its `data/cameras.yaml` with `--cameras` so perception uses the
  real intrinsics and mounts.
* **Real floor photos for step 4:** use
  `run_calib_extrinsics.py --images front=... left_rear=... right_rear=...`
  with a session that already has the intrinsics.
