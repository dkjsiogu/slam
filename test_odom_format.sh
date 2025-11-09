#!/bin/bash
# 测试里程计消息格式是否符合 Cartographer 要求

echo "=========================================="
echo "测试轮式里程计消息格式"
echo "=========================================="
echo ""

echo "1. 检查 /odom 话题类型..."
ros2 topic info /odom -v

echo ""
echo "2. 显示一条里程计消息..."
timeout 3 ros2 topic echo /odom --once

echo ""
echo "=========================================="
echo "Cartographer 要求检查清单："
echo "=========================================="
echo ""
echo "✓ 消息类型: nav_msgs/msg/Odometry"
echo "✓ header.frame_id: 应该是 'odom'"
echo "✓ child_frame_id: 应该是 'base_link'"
echo "✓ pose.pose: 位置 (x, y, z) + 四元数姿态"
echo "✓ twist.twist: 速度应该在 child_frame_id 坐标系中"
echo "✓ pose.covariance: 6x6 对角矩阵 (36个元素)"
echo "✓ twist.covariance: 6x6 对角矩阵 (36个元素)"
echo ""
echo "⚠️  关键点："
echo "   - twist 应该在 base_link 坐标系（机器人坐标系）"
echo "   - pose 在 odom 坐标系（世界坐标系）"
echo ""
