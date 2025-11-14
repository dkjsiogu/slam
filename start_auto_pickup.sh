#!/bin/bash
# 快速启动自动取物任务

source install/setup.bash

echo "======================================"
echo "🤖 启动自动取物任务"
echo "======================================"
echo ""

# 等待系统启动
echo "⏳ 等待系统启动..."
sleep 3

echo ""
echo "🚀 发送任务: auto_pickup"
echo "   - 前往 waypoint_1"
echo "   - 启动颜色跟踪"
echo ""

# 发布任务名称
ros2 topic pub -1 /mission/name std_msgs/msg/String "{data: 'complete_workflow'}"
sleep 0.5

# 执行任务
ros2 service call /mission/execute std_srvs/srv/Trigger

echo ""
echo "======================================"
echo "✅ 任务已启动!"
echo ""
echo "💡 监控任务状态:"
echo "   ros2 topic echo /mission/status"
echo ""
echo "⛔ 停止任务:"
echo "   ros2 service call /mission/stop std_srvs/srv/Trigger"
echo "======================================"
