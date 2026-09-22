# carbot_gui — browser GUI (phase 7)

`gui_server` serves `web/` on port 8080 (`gui.yaml`) in both launch modes.

| Mode | Tabs |
|---|---|
| race | Drive · 1 Global map · 2 Perception + planner · 3 Memory + LiDAR · 4 Localization · 5 Detections · 6 Control + safety · 7 System health · 8 Events + log (Split view: two tabs side by side) |
| calibrate | Calibration (steps; guided pages = phase 8) · Tuning · the same diagnostic tabs |

Header on every tab: mode, armed, **code running now** (Driving chain / Limiting /
Stopped by / Faults, each item opens the tab that explains it), battery,
**Manual control** (controller via the base servo_controller; race after START =
confirm, 0 marks), E-STOP.

## Why it does not starve the car
* Topics are grouped per tab (`gui_core.TAB_GROUPS`, `gui_server._build_groups`).
  A group is subscribed only while a browser polls that tab and dropped
  `idle_unsubscribe_s` after the last poll. Hidden browser tabs do not poll.
* Messages are stored raw and converted only when polled. Images are the
  camera_preview / debug JPEGs (already 320 px, ≤ 5 fps), pulled per image.
* Race mode halves diagnostic rates (`race.diagnostic_rate_scale`), tuning is off.

## Files
* `carbot_gui/gui_core.py` — pure logic (tested): lazy groups, running-now lines, leg progress, grid encoding, events.
* `carbot_gui/gui_server.py` — ROS node + HTTP API (`/api/config|core|tab/<id>|img/<key>|events|params`, POST `estop|manual|start|params/*`).
* `carbot_gui/params_api.py` — Tuning: YAML catalogue, live get/set, save to `<session>/params_overlay.yaml`.
* `web/` — `index.html`, `app.css`, `draw.js` (canvas helpers), `app.js` (shell), `tabs.js`, `tabs_diag.js`. No CDN: works offline.

## Without the car
`python3 tools/sandbox/gui_mock_server.py --mode race --scenario stop` then open
http://localhost:8081 (also in VS Code Run: "GUI mock").
