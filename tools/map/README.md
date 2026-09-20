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

## Open decision for phase 4 (not changed yet)

The ROS stack still reads `src/carbot_bringup/config/data/track_map.yaml` and
`mission.yaml` in the phase-1 **version 1** layout (V4 `Course` written out as
centrelines/areas). These scripts write **version 2**. Phase 4 must either make
the loaders read version 2, or have these scripts export version 1 too. Until
then, do not copy their output over the files in `config/data/`.
