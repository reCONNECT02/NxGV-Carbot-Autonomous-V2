# Offline map + mission tools (blocks 01 and 07)

Written by the team, committed unchanged. No ROS; run on a laptop or the RDK.

| Script | Block | Makes |
|---|---|---|
| `map_builder.py` | 01 prior map | `track_map.yaml` **version 2**: the competition map fitted onto a recorded UWB lap (`venue_transform`, control points) |
| `mission_planner.py` | 07 global plan | `mission.yaml` **version 2**: start/end pose of each leg + the V4 hybridPlan route, with the map's sha1 |

```bash
pip install numpy opencv-python pyyaml
python tools/map/map_builder.py demo          # synthetic lap, checks the fit
python tools/map/map_builder.py edit lap.csv  # fit + drag points, s = save
python tools/map/mission_planner.py           # track_map.yaml -> mission.yaml
```

## How the stack uses these files (phase 4)

The stack reads **version 2** directly. `src/carbot_bringup/config/data/` holds:

| File | Written by | Edit by hand? |
|---|---|---|
| `track_map.yaml` (v2) | `map_builder.py` | no (the tool rewrites it) |
| `mission.yaml` (v2) | `mission_planner.py` | no (the tool rewrites it) |
| `track_features.yaml` | hand | yes: light, gates, bump, hill, tunnel positions (MEASURE ON SITE) |
| `mission_rules.yaml` | hand | yes: leg behaviours, planned roundabout exits, speed zones, gate/light rules |
| `v4_reference/` | phase 1 | no: the V4 simulator course (v1) for tests and sandboxes |

Block 07 does **not** replan a v2 mission. It checks it: the map fingerprint,
the whole car body on the road along every road piece (5 mm tolerance for the
two rasters), and the roundabout exits against `mission_rules.yaml`. Any failure
= route FAIL with the reason, and race mode refuses to arm.

**After every map edit, re-run `mission_planner.py`** (otherwise: "planned on a
different track_map.yaml"). Windows line endings are fine: the stack accepts
the LF and CRLF fingerprint, and `.gitattributes` stores YAML with LF.

Parking pieces in `mission.yaml` are previews. They are drawn at the minimum
turning radius, so they cannot be replayed from a slightly different start:
block 11 (phase 5) replans them from the car's actual pose and the observed bay.
