# Backlog: things to change or add

Read at the start of every phase chat; updated at the end of it.
Status: TODO / DOING / DONE (phase) / WAITING (on the team).

| # | Request | Phase | Status | Notes |
|---|---|---|---|---|
| 1 | Stack reads track_map.yaml + mission.yaml **version 2** (from tools/map), still reads version 1 | 4 | DONE (4) | v2 is the default; v1 kept in `config/data/v4_reference/` |
| 2 | Leg 3: parallel bay -> perpendicular bay (Challenge 11) | 4 road part, 5 bay exit/entry | DOING | Road part + mission sequencing done (4). Un-park / park manoeuvres = block 11 (5) |
| 3 | Boom gate open/closed model (front camera) | 6 | WAITING | Team is training its own model; plug it in when supplied. Mission logic reads `DetectionArray.boom_gate_state` (+ confidence) only |
| 4 | Boom gate positions on the track (challenge4 + roundabout gate) | 4/8 | WAITING | `track_features.yaml boom_gates.*` (provisional, V4/rulebook values). Measure on site |
| 5 | Show all speed zones on the map tab | 7 | TODO | Zones: `mission_rules.yaml speed_zones`; active zone in `MissionState.speed_zone` |
| 6 | Change the speed of each zone from the GUI | 7 | TODO | Tuning tab edits `mission_rules.yaml speed_zones[].max_speed_mps`; calibrate mode only (race = read-only) |
| 7 | Corridor.msg frame comment: odom -> track | 4 | DONE (4) | |
| 8 | Confirm P1 (x 6.75, y 2.47, facing south) is the traffic-light stop line | 4 | WAITING | `track_features.yaml light_goal_pose` + `traffic_light` position derived from it (provisional) |
| 9 | Map lap must also drive the parking road (parking_spur, parking_corner, perp_row not fitted) | map / 8 | WAITING | `track_map.yaml fit_report`; then re-run `mission_planner.py` |
| 10 | Measure speed bump, hill, tunnel positions | on site | WAITING | `track_features.yaml` (provisional V4 values) |
| 11 | After ANY map edit, re-run `mission_planner.py` | always | note | Otherwise block 07 fails: "planned on a different track_map.yaml" |
| 12 | Parking previews are at the minimum turning radius: block 11 must replan from the actual pose | 5 | TODO | See docs/PHASES.md "For phase 5" |

## How to add a request
Tell Claude in any chat, or add a row here yourself (next number, phase if known, status TODO).
