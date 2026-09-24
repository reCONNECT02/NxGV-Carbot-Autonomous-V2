# NxGV Carbot Autonomous V2 — RISA Bot

ROS 2 Humble (Ubuntu 22.04) stack for the **NxGV Driverless CarBot Challenge 26/27**
(MARii Cyberjaya, 25–26 Sept 2026) on the RISA Bot: RDK X5 8 GB, Orbbec Astra Pro
(the only camera; the two MIPI side cameras were removed 2026-09-24), YDLiDAR T-mini Plus,
IMU + wheel encoder, single rear motor, Ackermann steering, ESP32 + DW1000 UWB tag.

It implements the 16-block architecture of `docs/reference/Carbot_Architecture_V4.md`
(ported from the V4 simulator) and reuses the drivers, motor/servo bridge and tunnel
code of the [base repo](https://github.com/Aquadrox-Technologies/NXGV-Driveless-Carbot-Challenge)
unchanged.

## Two entry points only

| Command | Mode | GUI |
|---|---|---|
| `ros2 launch carbot_bringup calibrate.launch.py` | Guided calibration wizard (13 numbered steps) | `http://<robot_ip>:8080/` in calibrate mode |
| `ros2 launch carbot_bringup race.launch.py` | Start line: preflight → READY → one START → autonomous run | `http://<robot_ip>:8080/` in race mode |

Rollback to an older calibration: `ros2 launch carbot_bringup race.launch.py session:=20260924_170200`.

## Repository layout

```
src/
  carbot_interfaces/    msgs + srvs (the contract between blocks)
  carbot_common/        topics.py (ALL topic names), frames, QoS, CarbotNode base,
                        geometry (V4 core.js port), calibration folder layout
  carbot_bringup/       race/calibrate launches, config/params/*.yaml (tunables),
                        config/data/*.yaml (track_map + mission from tools/map, their
                        hand-edited track_features + mission_rules, cameras, uwb,
                        challenges, calibration steps, v4_reference/), root helpers
  carbot_perception/    03 road perception, 04 local memory, camera preview streams
  uwb_localization/     UWB range parser (feeds block 06 only)
  carbot_localization/  05 smooth local pose, 06 coarse global pose
  carbot_planning/      01 map server, 07 global planner, 08 mission logic, 09 corridor,
                        10 local planner, 11 parking, 12 recovery, 13 path tracker
  carbot_control/       14 safety monitor, 15 command owner (-> 16), tunnel bridge
  carbot_detectors/     BPU traffic light / boom gate / speed-bump sign
  carbot_ops/           calibration wizard, race supervisor, recorder, monitor, scoreboard
  carbot_gui/           browser GUI (main tab + 11 diagnostic tabs)
  control_servo/ risabot_automode/ obstacle_avoidance_camera/
  ros2_astra_camera/ ydlidar_ros2_driver/ YDLidar-SDK/     <- base repo, UNCHANGED
firmware/uwb_tag/       ESP32 + DW1000 micro-ROS tag (TagConfig.example.h only)
tools/uwb/              uwb_xy.py (debug trilateration), uwb_calib.py (anchor offsets)
tools/map/              map_builder.py (track_map.yaml), mission_planner.py (mission.yaml)
tools/sandbox/          laptop runners of the real node code (no ROS)
tools/v4_harness/       runs the V4 simulator JS in Node for the equivalence tests
tools/bpu_model/        base repo BPU model + training/conversion scripts
tools/setup/            install_root_helpers.sh (kill_stale.sh as root via sudoers)
tools/systemd/          carbot-race.service (optional auto-start)
tools/git-hooks/        pre-commit secret check
docs/                   SETUP, CALIBRATION, RUN, TROUBLESHOOTING, CHALLENGE_MAP, DETECTORS, reference/
```

## Quick start (full guide: `docs/SETUP.md`, written in phase 9)

```bash
cd ~/NxGV-Carbot-Autonomous-V2
git config core.hooksPath tools/git-hooks                 # secret check, once per clone
cp firmware/uwb_tag/TagMicroROS/TagConfig.example.h firmware/uwb_tag/TagMicroROS/TagConfig.h   # edit locally, never commit
sudo bash tools/setup/install_root_helpers.sh sunrise     # once: stale-process killer as root
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch carbot_bringup calibrate.launch.py
```

## Rules the code enforces

* Fully autonomous after START; the GUI e-stop is labelled
  "counts as manual intervention = 0 marks".
* No timer-based traffic-light logic (scores 0). The traffic light is judged from
  the detector's red/green state only.
* Parking is planned, not replayed: block 11 plans a Reeds-Shepp manoeuvre (V4
  `parkingPlan`) into the bay observed by local memory, from the car's actual
  estimated pose, and replans at every gear change. (The rulebook would allow
  pre-recorded motion for parking only; we don't use it anywhere.) The base
  repo's record/playback feature is blocked in race mode because playback drives
  the motors directly, bypassing the command owner and safety veto.
* Navigation first (race week): the traffic-light and boom-gate stops are OFF in
  `mission_rules.yaml` (`traffic_light.enabled`, `challenge4_gate.enabled`) so the car
  always finishes the route. What each detection does: `docs/DETECTORS.md`.
* Roundabout exits are chosen by the global planner (block 07) from `mission.yaml`.
  The boom-gate detector only drives the Challenge 4 stop/proceed; a gate/route
  disagreement is logged and shown in the GUI but never changes the route.
* One command owner with a safety veto is the only writer of `/cmd_vel_auto`.
* UWB never reaches the servo, and never makes the local estimate jump.
* Every tunable is in YAML; a missing key is an error, never a silent default.
* Secrets are never committed (`.gitignore` + `tools/git-hooks/pre-commit`).

## Build phases

See `docs/PHASES.md` for what each phase delivered and what the next one needs.
