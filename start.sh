#!/bin/bash
# 同时启动颜色跟踪 + 串口通信节点

cd /home/pfa5/slam
source install/setup.bash

# 后台启动颜色跟踪节点
ros2 launch color_tracking_node color_tracking.launch.py &
COLOR_PID=$!

# 等待节点初始化
sleep 3

# 前台启动串口通信节点
ros2 launch navigation_control serial_debug.launch.py

# 退出时清理后台进程
trap "kill $COLOR_PID 2>/dev/null" EXIT
