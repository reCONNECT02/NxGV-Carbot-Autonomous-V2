#!/bin/bash
cd ~/NxGV-Carbot-Autonomous-V2
sed 's/calibrate.launch.py/calibrate.launch.py calibrate_profile:=lite/' ~/run_cal.sh > ~/run_cal_lite.sh; chmod +x ~/run_cal_lite.sh; tail -1 ~/run_cal_lite.sh
L=$(pgrep -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch carbot_bringup" | head -1); [ -n "$L" ] && kill -INT $L; sleep 12
bash ~/clean5.sh | tail -1
setsid nohup ~/run_cal_lite.sh > /tmp/calibration_wizard.log 2>&1 < /dev/null &
sleep 100
curl -s -o /dev/null -w "gui http %{http_code}\n" localhost:8080/
echo "nodes died: $(grep -a -c 'process has died' /tmp/calibration_wizard.log)"; grep -a "process has died" /tmp/calibration_wizard.log | cut -c1-110
grep -a "profile=LITE" /tmp/calibration_wizard.log | head -1 | cut -c1-200
grep -a -E "Resumed|calibration wizard ready" /tmp/calibration_wizard.log | cut -c1-150
echo "carbot nodes running: $(ps -eo cmd | grep -c '[N]xGV-Carbot-Autonomous-V2/install')"
source /opt/ros/humble/setup.bash; source install/setup.bash; export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0 FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/install/carbot_bringup/share/carbot_bringup/config/fastdds/disable_shm.xml
echo "== re-apply the wider steering ranges (live)"
for kv in "servo_range_left 74" "servo_range_right 70"; do set -- $kv; curl -s -X POST -H "Content-Type: application/json" -d "{\"node\": \"servo_controller\", \"key\": \"$1\", \"value\": $2}" localhost:8080/api/params/set | head -c 90; echo; done
timeout 40 python3 ~/param5.py 2>&1 | grep -E "servo_|ticks_per|polarity"
sleep 20; echo "load now:"; uptime
timeout 40 python3 ~/own7.py 2>&1 | grep -E "owner state messages"
