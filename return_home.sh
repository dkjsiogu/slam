#!/bin/bash
# 返回起点任务

source install/setup.bash

echo "🏠 返回起点..."

# 先停止当前任务（如果有）
ros2 service call /mission/stop std_srvs/srv/Trigger 2>/dev/null

sleep 0.5

# 发送任务名称
ros2 topic pub --once /mission/name std_msgs/msg/String "{data: 'return_home'}"

sleep 0.5

# 启动任务
ros2 service call /mission/execute std_srvs/srv/Trigger

echo "✅ 返回任务已启动"
