# risabot1 live issues — 2026-09-25

Snapshot taken read-only over SSH (`sunrise@risabot1.local`) and the GUI at
`http://172.31.224.164:8080/` (`/api/core`, `/api/tab/health`, `/api/tab/control`,
`/api/tab/events`). Nothing on the car was changed.

## State at the time

- Race stack up about 9 minutes, preflight `RUNNING: race started`.
- Battery 11.8 V, e-stop released, calibration session present.
- **The car cannot move:** command_owner is in SAFETY_STOP, reason "Safety status stale".
- CPU 100% on all 8 cores, load average 23, SoC 79 °C. RAM 29%, disk 75%.

## Problems, most important first

1. **CPU saturated (main cause).** About 20 Python nodes run at 30–60% CPU each.
   Heaviest: `recovery_planner` 63%, `gui_server` 61%, `bpu_detector` 50%,
   `global_pose` 49%, `uwb_ranges` 42%. `recovery_planner` and `parking_planner`
   are busy while they should be idle. 79 °C is close to throttling.
2. **Safety and owner topics far below expected rate.**
   `/carbot/safety/status` 11.5 Hz (expected 50), `/carbot/localization/local_pose`
   19 Hz (50), `/carbot/owner/state` 30 Hz (50). command_owner flips between
   SAFETY_STOP ("Safety status stale") and WATCHDOG ("Request older than 200 ms"):
   about 60 SAFETY_STOP and 20 WATCHDOG events. Follows from problem 1.
3. **safety_monitor vetoes `motion_fresh`.** Motion sensor age 0.395 s vs a
   0.2 s limit, flapping. `/odom` 18.7 Hz and `/imu/rpy` 18.6 Hz, both under the
   expected 20 Hz. mission_logic logged "Local pose lost" 3 times and is stuck in
   SAFETY_STOP.
4. **Stale nodes.** road_perception, corridor, path_tracker, recovery_planner and
   gui_server are marked stale. `local_planner` is in PROBLEM: "0/0 feasible,
   selected -1; corridor or pose stale", so path_tracker has no candidate.
   `recovery_planner` says "Waiting for fresh rear camera and LiDAR" and "No
   checked reverse-and-rejoin route". The car has only the front camera (rear
   cameras removed), so the rear-camera wait can never clear.
5. **Front camera reports -1 Hz.** Preflight `camera_front` fails ("Restart camera
   drivers") and `/camera/color/image_raw` shows -1 Hz with no age. Yet
   road_perception reports 14 Hz on the front camera and `camera_preview` is
   streaming, so the preflight figure contradicts the live data. dmesg shows the
   Astra Pro re-enumerated at 552 s after boot (camera container launched twice)
   and "did not claim interface 0".
   - 5b. **UWB position stale:** `/carbot/uwb/position` 1.26 s old, 379 ms latency;
     `global_pose` offset 134 cm. Ranges themselves are healthy (3 anchors, about 10 Hz).
6. **Perception looks empty.** `bpu_detector` 0 detections, traffic light and gate
   UNKNOWN. `road_perception` coverage 0.16 (865 cells).
7. **Track map still provisional.** Provisional: light_goal_pose, traffic_light,
   boom_gates roundabout_exit1 and challenge4, speed_bump, elevation, tunnel.
   Not covered by the map lap: parking_corner, parking_spur, perp_row.
8. **Older log warnings.**
   - `servo_controller` repeatedly failed to reopen `/dev/myserial` (present now).
   - Calibration step 8 (Speed PID) failed several times; skipped on purpose by the owner.
   - `obstruction_avoidance` emergency-yielded at 0.07 m in an earlier session.

## Likely root cause

CPU starvation (problem 1) pushes safety, pose and odometry topics below their
rates, so safety and owner nodes see stale inputs.

## Suggested next steps (not done)

- Stop nodes not needed right now (`recovery_planner`, `parking_planner`) and
  reduce the GUI's polling load.
- Restart the camera driver and re-check `camera_front`.
- Find why `recovery_planner` and `gui_server` are so CPU-heavy.
- Re-check `/carbot/safety/status` and `/carbot/localization/local_pose` rates after.
