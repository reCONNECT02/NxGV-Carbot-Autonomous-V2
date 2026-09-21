# Detectors: what the car sees and what each detection does

One node, `bpu_detector` (package `carbot_detectors`), runs the team's YOLO11n
**unified14** model on the RDK X5 BPU, on the **front camera only** (the Astra Pro,
`cameras.yaml roles.front`). Settings: `src/carbot_bringup/config/params/detectors.yaml`.
The model and how to retrain it: `tools/bpu_model/unified14/README.md`.

## Race-day setting (25–26 Sept): navigation first

The car must drive the whole route even if the detector sees nothing, so both
detector stops are **OFF** in `src/carbot_bringup/config/data/mission_rules.yaml`:

```yaml
traffic_light:
  enabled: false      # true = stop at the light until GREEN is seen
challenge4_gate:
  enabled: false      # true = stop at a gate on our path until OPEN is seen
```

At START the main tab shows **NAV ONLY: no traffic light / boom gate stop**.

| Setting | What happens | Marks |
|---|---|---|
| `traffic_light.enabled: false` | The car drives past the light without stopping | Challenge 7 = 0 |
| `traffic_light.enabled: true` | The car stops near the light and waits for an observed GREEN. If GREEN is never seen, it waits forever: the rest of the run is lost | Challenge 7 scored normally |
| `challenge4_gate.enabled: false` | The car never stops for a gate. **A closed gate on the route is driven into**: the arm is 5.5 cm high, under the LiDAR, so the safety check does not see it | Challenge 4 = 0 if the gate is closed |
| `challenge4_gate.enabled: true` | The car stops before any gate **on its route** until it sees that gate OPEN | Challenge 4 scored normally |

There is deliberately **no "give up after N seconds"**. The rulebook scores
timer-based traffic-light logic 0, so a timeout gains nothing.

Proof that the route runs with no detections at all (laptop, no car):

```bash
python3 tools/sandbox/run_planning.py --detector none          # -> RESULT: MISSION COMPLETE
python3 tools/sandbox/run_planning.py --detector none --holds on   # -> stuck at GATE HOLD (why the holds are off)
```

Switch a hold back on only after the detector has been checked on the car
(Detections tab, real lighting, real distances).

## Every class and what it triggers

**Triggers** change what the car does. **Info** classes are only shown in the GUI
Detections tab and recorded in the rosbag; they never drive anything.

| # | Model class | Canonical name | Kind | What it does |
|---|---|---|---|---|
| 9 | `traffic_red` | `traffic_light_red` | Trigger | Light state RED. With `traffic_light.enabled`, the car keeps waiting at the light |
| 10 | `traffic_yellow` | `traffic_light_yellow` | Trigger | Counted as **RED** (only GREEN lets the car go) |
| 11 | `traffic_green` | `traffic_light_green` | Trigger | Light state GREEN. With `traffic_light.enabled`, a fresh GREEN releases the stop (event `LIGHT GREEN`) |
| 12 | `boom_closed` | `boom_gate_closed` | Trigger | Gate CLOSED. With `challenge4_gate.enabled`, the car keeps holding at that gate. At the roundabout gate it is compared with the planned exit (log + GUI warning only) |
| 13 | `boom_open` | `boom_gate_open` | Trigger | Gate OPEN. With `challenge4_gate.enabled`, releases the hold at that gate (event `GATE OPEN`) |
| 6 | `speed_bump_sign` | *(ignored)* | Disabled | **Not trained yet.** Mapped to `""` in `class_map`. The car still slows over the hill and bump from the map (`speed_zones.hill_and_bump`). When trained, map it to `speed_bump_sign`: the `bump_sign` speed zone then slows the car for 0.60 m after the sign |
| 0 | `end_of_tunnel_sign` | `end_of_tunnel_sign` | Info | Nothing. The tunnel is entered and left by the base **LiDAR** trigger (`/tunnel_detected`) plus the map, unchanged |
| 8 | `tunnel_sign` | `tunnel_sign` | Info | Nothing (same reason) |
| 1 | `hill_sign` | `hill_sign` | Info | Nothing. The hill speed zone comes from the map |
| 2 | `obstacle_sign` | `obstacle_sign` | Info | Nothing |
| 3 | `parallel_parking_sign` | `parallel_parking_sign` | Info | Nothing. Parking starts where the route says (block 11 plans it) |
| 4 | `perpendicular_parking_sign` | `perpendicular_parking_sign` | Info | Nothing (same reason) |
| 5 | `roundabout_sign` | `roundabout_sign` | Info | Nothing. Exits come from the global planner |
| 7 | `traffic_signals_ahead_sign` | `traffic_signals_ahead_sign` | Info | Nothing |

A state is reported after **3 sightings** (`debounce_frames`) with no other state in
between, and drops back to UNKNOWN after **0.4 s** with nothing seen
(`state_expiry_s`). When both red and green are seen in one frame, RED wins; when
both open and closed are seen, CLOSED wins.

## How the outputs reach the nodes

```
front camera (/camera/color/image_raw, 30 Hz)
   └─ bpu_detector (newest frame, 10 Hz, BPU)
        ├─ /carbot/detections  (DetectionArray)
        │     ├─ traffic_light_state RED|GREEN|UNKNOWN ──► mission_logic: traffic-light hold
        │     ├─ detections[] boom_gate_* with base_link position ──► mission_logic: gate association
        │     │        ├─► gate hold (gates on our route only)
        │     │        └─► roundabout gate vs planned exit ──► /carbot/mission/gate_route_mismatch (log + GUI)
        │     ├─ speed_bump_sign (bool) ──► mission_logic: bump_sign speed zone (class disabled for now)
        │     └─ all detections ──► GUI Detections tab, main-tab render, rosbag
        ├─ /carbot/detections/debug/compressed  (boxes drawn; only while the GUI tab is open, 4 Hz)
        └─ /traffic_light_state, /boom_gate_open  (legacy base topics; nothing in our stack reads them)
```

The detector never talks to the command owner or the motors. Holds go through
`mission_logic` (block 08), which sets the active source to HOLD. The command owner
(block 15) follows that, under the safety veto (block 14).

## Which gate is which (gate association)

The map knows where every boom gate is (`track_features.yaml boom_gates`:
`roundabout_exit1` on the west/tunnel exit, `challenge4` before the hill). For each
detected gate, `bpu_detector` estimates where it is relative to the car: the
**direction** from the box position in the image, and a rough **distance** from the
box size (arm 37.5 cm). `mission_logic` then counts a detection for a map gate only
if:

* that gate is in front of the car and within 1.5 m (`gate_association.max_range_m`),
* the detection points at it within 20° (`bearing_tol_deg`),
* and its distance is within 0.35 m of where the gate should be (`range_tol_m`),
* for 2 separate detector frames in a row (`min_frames`).

A gate can only **hold** the car on route pieces that pass within 0.40 m of it
(`challenge4_gate.near_route_m`). The gates that can hold are `challenge4_gate.gate`
plus `extra_gates` (`roundabout_exit1`).

What that means on the track:

* **Roundabout visit 1** (exit west, through `roundabout_exit1`): with holds on, the
  car stops at that gate until it sees it OPEN. If it sees it CLOSED, the planned-exit
  check also logs a mismatch. The route is never changed.
* **Roundabout visit 2** (exit north): the route does not pass that gate, so it is
  never judged. A closed gate seen off to the side does not stop the car.
* **Challenge 4** (before the hill): stop until OPEN (holds on).

The tolerances are **provisional**. Check them on the Detections and Localization
tabs by standing the car at known distances from each gate.

## What was taken from the team model package, and what was not

Taken: the model file, the YOLO11 decoding maths (sigmoid class scores, DFL boxes,
strides 8/16/32, class-wise NMS), the 14-class list, the per-class thresholds, and
the board smoke test (`tools/bpu_model/unified14/verify_unified16.py`).

Not taken:

| Team `signage_detector.py` did | Why not |
|---|---|
| Published `/tunnel_detected` from the tunnel signs | That topic belongs to the base LiDAR tunnel trigger; mission logic relies on it. Two publishers would fight |
| Published `/parking_signboard_detected` | Fed the base record/playback parking chain. Our parking is planned (block 11) |
| Published hill / obstacle sign topics | Nothing in our stack reads them |
| Boom gate defaulted to OPEN | Unsafe: ours is UNKNOWN until actually seen |
| 3-frame counter | Could report a state it had never confirmed (two sightings then one miss) |
| Took the six outputs in list order | The ROS runtime (`pyeasy_dnn`) and the board test (`hbm_runtime`) may order them differently. Ours identifies each tensor by its shape |
| `dashboard_templates.py`, old parameter aliases | Our GUI replaces that dashboard |

## Checks on the car

```bash
# 1. model smoke test, no ROS (run INSIDE the folder)
cd ~/NxGV-Carbot-Autonomous-V2/tools/bpu_model/unified14
python3 verify_unified16.py --model ../../../src/carbot_detectors/models/unified14_yolo11n_640x640_nv12.bin --image test_boom_open.jpg

# 2. the node, with the stack running (calibrate mode is fine)
ros2 topic hz /carbot/detections                 # ~10 Hz
ros2 topic echo /carbot/detections --field traffic_light_state
ros2 topic echo /carbot/status | grep -A3 bpu_detector    # RUNNING, not NO_MODEL / MODEL_MISMATCH
```

| Status | Meaning | Fix |
|---|---|---|
| `NO_MODEL` | Model file missing, or no BPU runtime (`hobot_dnn`) | `colcon build --packages-select carbot_detectors`; check `model_path` |
| `MODEL_MISMATCH` | The model's outputs don't fit `class_names` (wrong class count, or not a 6-output YOLO11 head) | Use the matching model, or fix both lists in `detectors.yaml` |
| `RUNNING` with 0 detections | Model loaded, nothing recognised | Point the car at the light or gate; check lighting; lower thresholds in Tuning |
