# Dashboard (gui_server) on a PC instead of the RDK

The dashboard is one ROS 2 node (`carbot_gui/gui_server`) that subscribes to the robot's topics and serves the web
page on port 8080. It can run on any computer that sees the robot's DDS traffic. It needs ROS 2 Humble, so on
Windows it runs inside **WSL2 (Ubuntu 22.04)**. The wizard, the command owner and everything that drives the car
stay on the robot.

## Why
On risabot5 the dashboard took about 60 % of one core (BACKLOG #47/#50) on an RDK that was already saturated.
On a PC it costs the robot nothing except the messages it sends (camera previews only while a tab is open).

## What does NOT move
* The **wizard** (`calibration_wizard`), the command owner, servo, cameras and UWB agent: on the robot.
* The **session data** (`~/carbot_data/calibration/...`): on the robot. The dashboard's tuning **Set** (live) reaches the
  robot over ROS; the tuning **Save** writes to the PC's `~/carbot_data_pc`, not to the robot's session.

## One-time setup (Windows 11, this laptop is build 26100: mirrored networking is supported)
1. **Install WSL** (PowerShell as Administrator; needs a restart):
   `wsl --install -d Ubuntu-22.04`
2. **Mirrored networking** (so the robot's DDS discovery reaches WSL): copy `wslconfig.example` to
   `C:\Users\<you>\.wslconfig`, then `wsl --shutdown` and open Ubuntu again.
3. In Ubuntu: `bash <(curl -fsSL https://raw.githubusercontent.com/reCONNECT02/NxGV-Carbot-Autonomous-V2/phase8/complete-calibration/tools/dashboard_pc/setup_wsl.sh)`
   (installs ROS 2 Humble, clones the repo to `~/NxGV-Carbot-Autonomous-V2`, builds only the four packages the dashboard needs).

## Every time
1. **Robot:** start the calibration launch WITHOUT its own dashboard, e.g.
   `ros2 launch carbot_bringup calibrate.launch.py calibrate_profile:=lite start_gui:=false`
   (two `gui_server` nodes would clash).
2. **PC (Ubuntu):** `bash ~/NxGV-Carbot-Autonomous-V2/tools/dashboard_pc/check_link.sh` should list the robot's nodes and
   an `/odom` rate of about 20 Hz. Then `bash ~/NxGV-Carbot-Autonomous-V2/tools/dashboard_pc/run_gui_pc.sh`.
3. **Browser on Windows:** `http://localhost:8080/`.

## If it does not see the robot
* Same Wi-Fi network as the robot (the laptop and the RDK were both on 172.31.224.x).
* `ROS_DOMAIN_ID` must be 1 (`uwb.yaml agent.domain_id`) and `ROS_LOCALHOST_ONLY=0` on both sides (the scripts set it).
* Windows Firewall: allow UDP for `wsl.exe`/Ubuntu (private network) if `check_link.sh` shows nothing.
* Some Wi-Fi access points block multicast between clients. Then FastDDS needs the robot's IP as an *initial peer*
  (ask, and a peer-list profile can be added).
* Inside WSL2 **without** mirrored networking (the default NAT mode) the robot cannot reach WSL: it will not work.

## Files
| File | Use |
|---|---|
| `setup_wsl.sh` | one-time install inside Ubuntu 22.04 |
| `run_gui_pc.sh [calibrate\|race]` | start the dashboard on the PC |
| `check_link.sh` | is the robot visible? |
| `wslconfig.example` | mirrored networking for WSL2 |
| `src/carbot_bringup/launch/gui_pc.launch.py` | starts only `gui_server` with the same parameters as the robot's launch |
| `src/carbot_bringup/carbot_bringup/stack.py` | new launch argument `start_gui` (default `true`) |
