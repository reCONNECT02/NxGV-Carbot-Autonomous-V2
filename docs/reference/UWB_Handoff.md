# UWB Positioning — Handoff Notes (RISABOT, NxGV CarBot Challenge 26/27)

This document records everything done to get the UWB positioning system working, how it works, how to start it each session, and what is still open. It is written for whoever (human or AI) continues the work. Read it together with the project architecture document (`Carbot_Architecture_Vn.html`, "Fourteen boxes between the camera and the wheels"). The UWB system feeds **block 05 (EKF localisation)**.

Files that go with this note (attach them all):

| File | Runs on | Purpose |
|---|---|---|
| `TagMicroROS.ino` | Tag ESP32 | Main tag firmware: UWB ranging + micro-ROS publisher over WiFi |
| `TagConfig.h` | Tag ESP32 | WiFi, agent IP, ROS domain, timing settings |
| `BoardConfig.h` | Tag ESP32 | SPI / DW1000 pin numbers |
| `RangeProtocol.h` | Tag ESP32 | Anchor ID list + JSON encoder |
| `uwb_xy.py` | RDK X5 | Debug viewer: ranges → (x, y), prints and publishes `/uwb/position` |
| `uwb_calib.py` | RDK X5 | Measures a per-anchor distance offset at a known spot |

---

## 1. System overview

```
 Anchor 1786 ─┐
 Anchor 1782 ─┼── UWB radio (two-way ranging) ──► Tag (on car) ── WiFi/UDP ──► RDK X5
 Anchor 1783 ─┘                                   ESP32+DW1000                 micro-ROS agent (port 8888)
 (fixed around the track,                                                         │
  each on its own power bank)                                                     ▼
                                                                    /uwb3/input_json (std_msgs/String, JSON)
                                                                                  │
                                                                    uwb_xy.py (debug)  /  block 05 EKF (final)
```

- **Anchors** are fixed, powered by power banks, and only talk UWB radio to the tag. They never connect to WiFi or the RDK.
- **The tag** (ESP32 + DW1000, on the car) measures its distance to each anchor, packs the distances into JSON, and publishes it to ROS 2 through micro-ROS over WiFi.
- **The RDK X5** runs the micro-ROS agent, which turns the tag's UDP packets into a normal ROS 2 topic.
- The tag sends **ranges only**, never coordinates. Position is computed on the RDK.

### Architecture rules that apply (from the architecture doc)

- UWB owns *"which part of the track am I on"* (about ±20 cm). The camera owns the lane (±2 cm) and the steering. **UWB must never reach the servo.**
- The venue frame is the UWB anchor frame (cm). Only block 07 (prior map) and block 05 speak it.
- Camera wins on geometry, map wins on identity.

---

## 2. Hardware and IDs

| Item | Value |
|---|---|
| Tag MCU | Classic dual-core ESP32 (NOT ESP32-C3/S3) + DW1000 (NOT DW3000) |
| Tag USB-UART chip | Silicon Labs **CP2104** (needs the CP210x VCP driver on Windows) |
| Tag DW1000 address | `7D:17:22:EA:82:60:3B:9C` |
| Anchor IDs | **`0x1786`, `0x1782`, `0x1783`** (verified one by one) |
| Radio mode | `DW1000.MODE_LONGDATA_RANGE_LOWPOWER` (anchors must use the same) |
| Antenna delay | `16384` on the tag, **uncalibrated** |
| Pins (`BoardConfig.h`) | SCK 18, MISO 19, MOSI 23, CS 4, RST 27, IRQ 34 — provisional; the `BOARD_PINOUT_CONFIRMED` check is commented out in the `.ino`, but ranging works, so they are correct in practice |

**How anchor IDs work:** an anchor's ID is the first two bytes of the address in its own sketch, swapped. Address `"86:17:..."` → ID `0x1786`. The library computes `id = byte[1]*256 + byte[0]`.

**How to discover an unknown anchor ID:** power only that anchor and watch the tag's JSON. A known ID appears in `links`; an ID not in `ANCHOR_IDS` shows up as `last_unknown_id`.

---

## 3. Message format (`/uwb3/input_json`)

- Type: `std_msgs/msg/String`, JSON text in `data`.
- QoS: **BEST_EFFORT**, KEEP_LAST depth 1, VOLATILE. **Subscribers must be BEST_EFFORT**; a RELIABLE subscriber silently receives nothing.
- Rate: about 10 Hz. Each anchor produces fresh ranges at about 7 Hz, so roughly 20% of reports repeat an old range.

Example:

```json
{"tag":"RISA01","boot_id":"8543AB52","seq":15772,"t_ms":1577491,"last_unknown_id":"0000",
 "links":[{"A":"1786","R":2.5800,"age_ms":51,"sample_seq":6567},
          {"A":"1782","R":2.1800,"age_ms":65,"sample_seq":6023},
          {"A":"1783","R":2.8900,"age_ms":79,"sample_seq":6040}]}
```

| Field | Meaning |
|---|---|
| `boot_id` | Random per boot. A change means the tag restarted. |
| `seq` | Report counter; increments every 100 ms report. |
| `t_ms` | Tag `millis()` when the report was encoded. |
| `last_unknown_id` | Last anchor ID heard that is NOT in `ANCHOR_IDS` (`0000` = none). |
| `links[].A` | Anchor ID (hex string). |
| `links[].R` | Distance tag→anchor in **metres** (raw, 3D, uncalibrated). |
| `links[].age_ms` | How old that range was when the report was encoded. |
| `links[].sample_seq` | Increments ONLY on a new radio measurement for that anchor. Use it to avoid counting a repeated range twice. |

Ranges older than 400 ms (`OMIT_OLDER_THAN_MS`) are left out. **Empty `links` means "no measurement"** — never treat it as (0, 0).

---

## 4. Firmware design and the fixes that made it work

The firmware was originally USB-serial micro-ROS and was converted to WiFi. Key points:

1. **Agent IP must be a dotted string.** `micro_ros_arduino`'s `set_microros_wifi_transports(char* ssid, char* pass, char* agent_ip, uint32_t port)` parses the IP with `IPAddress::fromString()`. The original code passed a `uint8_t[4]` cast to `char*`, which parsed as garbage (0.0.0.0), so nothing ever reached the agent. Fixed: `constexpr char MICROROS_AGENT_IP[] = "10.168.5.164";`.
2. **BEST_EFFORT publisher.** RELIABLE over WiFi caused a publish failure on a lost ACK, which tore down the session and caused multi-second gaps. A late 10 Hz position sample is worthless, so no retransmission.
3. **`WiFi.setSleep(false)`.** ESP32 modem sleep delays inbound packets (agent pings) by ~100 ms+.
4. **No re-calling `set_microros_wifi_transports()` on WiFi loss.** It re-runs `WiFi.begin()` and blocks. The code now uses `WiFi.setAutoReconnect(true)` and forces `WiFi.reconnect()` after 10 s down.
5. **Agent ping** 100 ms timeout, 2 attempts (one lost UDP ping no longer drops the session).
6. **Anchor pre-registration (multi-anchor discovery fix).** In the thotro/Makerfabs `DW1000Ranging` library, the tag discovers anchors by broadcasting a BLINK. Every anchor replies with RANGING_INIT **at the same instant**, the replies collide, and the tag only learns the strongest anchor (1786). An anchor that has answered a BLINK never answers another while it still "knows" the tag, so the others never join. Symptom: each anchor works alone, but with all three on only `1786` appears. Fix in `TagMicroROS.ino`: `registerMissingAnchors()` adds every ID from `ANCHOR_IDS` to the tag's device list itself (via `searchDistantDevice` / `addNetworkDevices`, with `noteActivity()` so it isn't dropped as inactive). The tag's POLL then gives each anchor its own reply slot. It re-adds any missing anchor every 2 s (`ANCHOR_REREGISTER_MS`).
   - **Status:** all three anchors were seen together, but the `boot_id` suggested the fixed firmware may not have been flashed yet at that moment (the anchors may have joined one at a time while being switched on). **Verify with a cold start** (see section 10).
7. **Threading.** UWB ranging runs in Arduino `loop()` on core 1; all micro-ROS calls run in `communicationTask` pinned to core 0. They exchange a `Snapshot` through a 1-slot FreeRTOS queue (`xQueueOverwrite`, latest-only).
8. **Serial debug log** at 115200 on USB: WiFi status, IP, RSSI, agent found/lost, published count every 5 s.

Expected serial output when healthy:

```
[uwb] WiFi: connecting to 'OnePlus 13 7E44'...
[uwb] WiFi up: tag 10.168.5.122  RSSI -45 dBm  agent 10.168.5.164:8888  domain 1
[uwb] micro-ROS session up, publishing
[uwb] published 250  RSSI -20 dBm  last_unknown_id 0000
```

---

## 5. One-time setup: laptop (Arduino, Windows)

1. **Arduino IDE 2.x.**
2. **ESP32 board package 2.0.17** (Espressif). Boards manager URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`. micro_ros_arduino's precompiled ESP32 library targets the 2.0.x core; 3.x usually fails to link.
3. **micro_ros_arduino, `-humble` release.** From `github.com/micro-ROS/micro_ros_arduino/releases`, download "Source code (zip)" of the newest `*-humble` release and install with *Sketch → Include Library → Add .ZIP Library* (do not unzip). It installs to `Documents\Arduino\libraries\micro_ros_arduino`. Compile log shows "Library micro_ros_arduino has been declared precompiled".
4. **DW1000 library:** the same thotro/Makerfabs-style `DW1000` + `DW1000Ranging` library used to program the anchors. The anchor fix relies on `DW1000Ranging.searchDistantDevice()`, `DW1000Ranging.addNetworkDevices(dev, true)`, `DW1000Device(byte shortAddr[], true)` and `DW1000Device::noteActivity()`. `MAX_DEVICES` must be ≥ 3 (default 4).
5. **CP210x driver:** Silicon Labs "CP210x Universal Windows Driver". Without it, Device Manager shows "CP2104 USB to UART Bridge Controller — Code 28" and no COM port appears. Install via Device Manager → Update Driver → Browse → extracted folder.
6. **Sketch folder:** all four files in one folder named `TagMicroROS` (folder name must match the `.ino`).
7. **Board:** *ESP32 Dev Module*. Default partition works (sketch uses ~74% of 1.3 MB); switch to *Huge APP (3MB No OTA)* if it ever exceeds 100%.
8. If upload hangs at "Connecting...", hold the BOOT button until it starts.

---

## 6. One-time setup: RDK X5

Environment found on the RDK (user `sunrise`, host `risabot1`, `aarch64`):

- Both `/opt/ros/humble` (standard ROS 2) and `/opt/tros/humble` (TogetheROS) exist. The agent was built against **`/opt/ros/humble`**.
- The image sets **`ROS_DOMAIN_ID=1`** and **`ROS_LOCALHOST_ONLY=1`** (see section 8).

micro-ROS agent build (already done, lives in `~/uros_ws`):

```bash
sudo apt update
sudo apt install -y git python3-rosdep python3-colcon-common-extensions python3-pip
source /opt/ros/humble/setup.bash
sudo rosdep init        # "already exists" error is fine
rosdep update
mkdir -p ~/uros_ws/src && cd ~/uros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
ros2 pkg executables micro_ros_agent     # expect: micro_ros_agent micro_ros_agent
```

Viewer scripts: `~/uwb_xy.py` and `~/uwb_calib.py` (same folder; `uwb_calib.py` imports `ANCHORS` from `uwb_xy.py`).

---

## 7. Network

- The tag and the RDK must be on the **same 2.4 GHz network** (ESP32 cannot use 5 GHz; channels 1–13 = 2.4 GHz).
- **Current setup:** phone hotspot `OnePlus 13 7E44` (channel 11). RDK = `10.168.5.164`, tag = `10.168.5.122`.
- Earlier setup: Windows laptop hotspot `ARYAN PC` (RDK was `192.168.137.161`; Windows hotspots always use `192.168.137.x`).
- **The RDK's IP can change** whenever it reconnects. The tag has the agent IP compiled in, so an IP change means editing `MICROROS_AGENT_IP` in `TagConfig.h` and re-uploading. Always check `hostname -I` first.
- Check which WiFi the RDK is on: `nmcli -f IN-USE,SSID,CHAN dev wifi | grep '\*'`.
- Switch the RDK's WiFi: `sudo nmcli dev wifi connect "SSID" password "PASS"`.

**Recommended for competition (not done yet):** make the RDK its own 2.4 GHz hotspot so the car carries its own network and the IP is fixed:

```bash
sudo nmcli dev wifi hotspot ifname wlan0 ssid RISA_CAR password <8+ chars>
sudo nmcli con modify Hotspot 802-11-wireless.band bg connection.autoconnect yes
# RDK becomes 10.42.0.1 -> set MICROROS_AGENT_IP = "10.42.0.1", SSID/password in TagConfig.h
```

Note: in hotspot mode the RDK loses internet on wlan0; do any `apt`/`git` work first.

---

## 8. ROS 2 gotchas (both cost hours)

1. **Domain ID.** The **tag** chooses the DDS domain (`MICROROS_DOMAIN_ID` in `TagConfig.h`), not the agent. It is set to **1** to match the RDK's `ROS_DOMAIN_ID=1`. Any node reading UWB data must use the same domain.
2. **`ROS_LOCALHOST_ONLY=1` (set by the Yahboom image) hides the topic.** The micro-ROS agent creates its Fast DDS participant directly and ignores this variable, so it announces on the network interface, while ROS 2 nodes with localhost-only only look on loopback. Symptom: the agent log shows data being written (`DataWriter ... write`), but `ros2 topic list` shows only `/parameter_events` and `/rosout`. **Fix: `export ROS_LOCALHOST_ONLY=0`** in every terminal / launch environment that needs UWB data.
   - **Permanent fix still to do:** find where it is set and change it to 0:
     `grep -rn "ROS_LOCALHOST_ONLY" ~/.bashrc ~/.profile /etc/profile /etc/profile.d /etc/bash.bashrc /etc/environment 2>/dev/null`
3. After changing environment variables, run `ros2 daemon stop` (the CLI daemon caches old discovery results) or use `--no-daemon`.
4. `ros2 topic echo` truncates long strings; add `--full-length`.

---

## 9. Startup procedure (every session)

1. Turn on the hotspot (2.4 GHz). If it's a laptop, keep it plugged in and awake.
2. Power the RDK and wait ~1 min. Run `hostname -I`. **If the IP changed, update `TagConfig.h` and re-upload the tag.**
3. Power the anchors (power banks) and the tag.
4. RDK terminal 1 — start the agent (keep it running):
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/uros_ws/install/local_setup.bash
   export ROS_LOCALHOST_ONLY=0
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4
   ```
   Expect `running... port: 8888`, then `session established`, `participant/topic/publisher/datawriter created`. (`-v6` also dumps every packet in hex — useful for debugging only.)
5. RDK terminal 2 — check data:
   ```bash
   source /opt/ros/humble/setup.bash
   export ROS_LOCALHOST_ONLY=0
   export ROS_DOMAIN_ID=1
   ros2 topic hz /uwb3/input_json                   # expect ~10 Hz
   ros2 topic echo /uwb3/input_json --full-length   # expect 3 links
   ```
6. Optional viewer: `python3 ~/uwb_xy.py` → prints `(x, y)` in cm.

Verified result: `ros2 topic hz` gave **10.00 Hz** (message spacing 40–134 ms from WiFi jitter).

---

## 10. Coordinates

### Current anchor layout (cm, venue frame)

| Anchor | x | y | z |
|---|---|---|---|
| 1782 | 0 | 0 | 0 |
| 1786 | 750 | 0 | 0 |
| 1783 | 750 | 483 | 0 |

Frame: origin at anchor 1782, +x from 1782 towards 1786, +y from 1786 towards 1783. Right angle at 1786. Accuracy is best inside the triangle; the corner near (0, 483) is outside it. A 4th anchor at (0, 483) would cover the whole rectangle (library supports 4; `ANCHOR_COUNT`/`ANCHOR_IDS` would need updating).

Lesson learned: an earlier layout had the anchors only ~20 cm apart (positions were given in mm by mistake). Simulation with 5 cm range noise: 24×18 cm spacing → 33 cm typical / 101 cm worst-10% error; 300×250 cm spacing → 7 cm / 12 cm. **Anchor spacing matters far more than the algorithm.**

### `uwb_xy.py` (debug viewer, not block 05)

- Subscribes BEST_EFFORT to `/uwb3/input_json`.
- Per anchor: `range_cm = R*100 − RANGE_OFFSET_CM[id]`, then flattened to the floor plane using the anchor/tag z difference.
- For each **pair** of anchors (3 pairs), intersects the two range circles; the **third anchor's range** picks which of the two mirror-image intersections is correct. If the circles don't meet (noise), it uses the closest point on the pair's baseline.
- Averages the 3 pair estimates → prints only `(x, y)` in cm and publishes `geometry_msgs/PointStamped` on `/uwb/position` (metres, `frame_id: venue`).
- Requires all 3 anchors in the message; otherwise prints nothing.
- Tuning at the top: `ANCHORS`, `TAG_Z_CM`, `RANGE_OFFSET_CM` (per anchor).

### `uwb_calib.py` (per-anchor offset calibration)

Put the tag still at a tape-measured spot inside the triangle, >1 m from every anchor (e.g. (500, 150)), with the antenna oriented as on the car and nobody in the line of sight. Then:

```bash
python3 ~/uwb_calib.py 500 150      # optional 3rd arg: seconds (default 20)
```

It prints median measured vs true distance per anchor and a `RANGE_OFFSET_CM = {...}` block to paste into `uwb_xy.py`. Verify afterwards at a different spot.

### Observed accuracy so far

- **Before calibration:** tag placed on anchor 1782 at (0, 0) read about (−102, −6) → every range ≈ **100 cm too long** (uncalibrated antenna delay). Correct fix is a per-anchor range offset, not a position offset.
- **Stationary vs moving** (300 samples): typical jitter 3–4 cm in both cases, but while stationary the position **flip-flops between two clusters ~25 cm apart** along the direction of anchor 1786 (closest anchor), worst-10% 28 cm; while moving worst-10% 17 cm, with occasional spikes of ~70 cm. This is **multipath**: with a static geometry a strong reflection can make the receiver toggle between direct and reflected first-path. Most likely aggravated by the **anchors sitting on the floor (z = 0)** — ground reflection and partial blockage.
- Whether `RANGE_OFFSET_CM` values have been applied yet is **not confirmed** — check `uwb_xy.py` on the RDK.

---

## 11. How the main architecture should use this data (block 05)

- Consume **raw ranges** from `/uwb3/input_json`, not `/uwb/position`. Do one EKF measurement update **per anchor range** (h = distance from predicted position to anchor), each **individually outlier-gated** (innovation / Mahalanobis gate). This survives a missing anchor and rejects the multipath spikes above, which pair-averaging cannot.
- Timestamp each range as `arrival_time − age_ms`; WiFi adds 40–130 ms of variable latency.
- Use `sample_seq` per anchor to skip ranges already used (≈20% of reports repeat).
- Empty `links` or missing anchors = no update; dead-reckon on gyro + encoders. A WiFi dropout must never stop the car — UWB never steers.
- Apply per-anchor range offsets (from `uwb_calib.py`, or better, calibrated antenna delays in firmware).
- Subscriber QoS must be BEST_EFFORT; the node's environment needs `ROS_DOMAIN_ID=1` and `ROS_LOCALHOST_ONLY=0`.
- Suggested home: a node inside the ROS 2 package `risabot_automode`, plus the agent in the launch file:
  ```python
  Node(package='micro_ros_agent', executable='micro_ros_agent',
       name='uwb_agent', arguments=['udp4', '--port', '8888'], output='screen'),
  ```
  (the launch environment must also have `ROS_LOCALHOST_ONLY=0`, or the permanent fix from section 8 applied).
- The venue frame here must match the prior map (block 07). Anchors will be re-surveyed on-site at the competition (MARii Cyberjaya, Day 1) — budget time for it.

---

## 12. Open items / TODO

1. **Cold-start test of the anchor fix:** with all 3 anchors already on, power-cycle the tag. Expect a new `boot_id` and all 3 links within ~5 s.
2. **Make `ROS_LOCALHOST_ONLY=0` permanent** (section 8).
3. **Calibrate range offsets** with `uwb_calib.py`; later consider proper antenna-delay calibration in firmware (tag `TAG_ANTENNA_DELAY` and each anchor).
4. **Raise the anchors** to ~1–1.5 m with clear line of sight; set real `z` values and `TAG_Z_CM`.
5. **Fixed network:** RDK hotspot (10.42.0.1) or a static IP, so the tag doesn't need re-flashing when the IP changes.
6. **Tag mounting on the car:** highest point with clear all-round view, but the car must stay ≤ 25 cm tall and the tunnel clearance is only 27 cm.
7. **Write the block 05 EKF node** in `risabot_automode` (section 11).
8. Optional 4th anchor at (0, 483).
9. Organiser question still open: whether carrying a surveyed prior map / anchor layout counts as "pre-programming".

---

## 13. Troubleshooting quick reference

| Symptom | Cause | Fix |
|---|---|---|
| `micro_ros_arduino.h: No such file` | Library not installed / double-nested folder | Add .ZIP Library; check `Documents\Arduino\libraries` |
| No COM port; Device Manager "CP2104 … Code 28" | Missing driver | Install Silicon Labs CP210x VCP driver |
| Nothing at all in Device Manager | Charge-only cable / bad port | Use a data cable, direct laptop port |
| Serial stuck at `connecting to '...'` | Wrong SSID/password, or 5 GHz | Match exactly; force hotspot to 2.4 GHz |
| Serial repeats `no agent at ...` | Agent not running, or wrong IP | Start agent; compare `hostname -I` with `MICROROS_AGENT_IP` |
| Agent writes data but topic missing | `ROS_LOCALHOST_ONLY=1` or domain mismatch | `export ROS_LOCALHOST_ONLY=0`, `ROS_DOMAIN_ID=1`, `ros2 daemon stop` |
| Subscriber gets nothing, topic exists | RELIABLE subscriber vs BEST_EFFORT publisher | Use BEST_EFFORT QoS |
| Only `1786` with all anchors on | BLINK reply collision | Anchor pre-registration firmware (section 4.6) |
| `links` empty but `last_unknown_id` ≠ 0000 | Anchor ID not in `ANCHOR_IDS` | Add it in `RangeProtocol.h`, re-upload |
| Position offset ~1 m everywhere | Uncalibrated antenna delay | `uwb_calib.py` per-anchor offsets |
| Position flips between two spots while still | Multipath | Raise anchors, clear line of sight; EKF gating |
| Very noisy / nonsense coordinates | Anchors too close / collinear, or wrong units | Spread anchors 2–7 m in a triangle; positions in **cm** |
