# tools/car_ops: what was used on the RDK during the 2026-09-24 calibration sessions

Small helper scripts, copied here so the next car (risabot1, ...) does not have to rediscover them.
Copy the ones you need to `~` on the RDK. None of them touches the repo; they only read topics / set LIVE
parameters (a relaunch resets them) or start/stop the launch. No passwords are in these files.

| Script | What it does |
|---|---|
| `run_cal.sh` / `run_cal_lite.sh` | start `calibrate.launch.py` (full stack / `calibrate_profile:=lite`: ~11 nodes, load ~4 instead of ~25; use lite for steps 7-9) |
| `clean5.sh` | SIGINT/kill every leftover carbot node, `static_transform_publisher`, `micro_ros_agent`, `component_container` (run before a relaunch) |
| `lite5.sh` | relaunch with the lite profile, re-apply the live steering ranges, print load and the owner's rate (edit the two `servo_range_*` values for your car) |
| `param5.py` | print servo_controller's live parameters (servo centre/ranges, ticks_per_meter, polarity, speed levels, ...) |
| `setmany5.py` | set servo_controller parameters live: `python3 setmany5.py '{"ticks_per_meter": 1050.0}'` |
| `man5.py [release]` | read (or release) the latched Manual control flag; the GUI's `POST /api/manual {on:false}` does the same |
| `rp5.py [stop]` | ask servo_controller for its record/playback state; `stop` ends an accidental RECORDING |
| `own5.py` / `watch8.py` / `trig8.py` | command owner state, safety veto, calibration requests; `trig8.py` waits for the first Go and records 40 s |
| `batt5.py` | battery voltage + owner state |
| `ticks5.py` | raw encoder tick recorder (needs the odom filters switched off, see BACKLOG #61) |
| `hz_all.py` | rate + size of every image topic |

## Things that cost hours on risabot5 (all in docs/BACKLOG.md #42-#63)
* **Manual control ON** (GUI header) makes the command owner ignore steps 7/8 completely. Turn it OFF before Go.
* After every relaunch the servo controller ignores driving until a joystick button is pressed; the button also
  runs its function (A record, X playback, Y/Start auto/manual, D-pad speed limit): use LB or Back.
* The motor board can drop off USB ("disabled by hub (EMI?)"): `Rosmaster motor write failed` forever, only a
  relaunch (servo_controller restart) reopens `/dev/myserial`. Watch the battery (11.4 V was too low).
* `encoder_jump_threshold` 800 (base default) dropped almost every encoder reading at ~300000 ticks/m.
* The launch loads no session overlay: step 6 values are re-applied by the wizard after a relaunch; servo
  ranges/centre set live are NOT (they reset), see `lite5.sh`.
