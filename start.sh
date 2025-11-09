#!/bin/bash
# 启动颜色跟踪节点 - 从get-toy包发布跟踪结果

cd /home/pfa5/slam
source install/setup.bash
ros2 launch color_tracking_node color_tracking.launch.py
sleep 3
ros2 launch navigation_control serial_debug.launch.py
