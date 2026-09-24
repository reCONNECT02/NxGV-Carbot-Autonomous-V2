#!/bin/bash
me=$$
list() { ps -eo pid,cmd | grep -E "NxGV-Carbot-Autonomous-V2/install|static_transform_publisher|micro_ros_agent|ydlidar_ros2_driver_node|component_container|joy_node|mipi_cam" | grep -v -E "grep|clean5" | awk '{print $1}'; }
before=$(list | wc -l); echo "leftover before: $before"
for p in $(list); do kill -INT $p 2>/dev/null; done; sleep 6
for p in $(list); do kill $p 2>/dev/null; done; sleep 3
for p in $(list); do kill -9 $p 2>/dev/null; done; sleep 2
echo "leftover after: $(list | wc -l)"; uptime
