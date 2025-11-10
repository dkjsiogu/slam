#!/bin/bash
# 坐标系对齐检查脚本

echo "========================================="
echo "   机器人坐标系对齐检查"
echo "========================================="
echo ""

echo "1. 检查当前TF树..."
timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -A 3 "Translation" | head -4

echo ""
echo "2. 请在RViz中检查以下内容："
echo ""
echo "   ✓ base_link 坐标轴（红绿蓝）"
echo "     - 红色箭头（X轴）应该指向机器人前方"
echo "     - 绿色箭头（Y轴）应该指向机器人左侧"
echo "     - 蓝色箭头（Z轴）应该向上"
echo ""
echo "   ✓ 机器人模型（蓝色方块）"
echo "     - 应该与 base_link 坐标轴原点重合"
echo ""
echo "   ✓ laser 坐标轴"
echo "     - 应该在机器人前方偏左位置"
echo "     - 旋转180° (X朝后)"
echo ""
echo "========================================="
echo ""
echo "如果发现不对齐，请描述："
echo "  1. base_link 红色箭头实际指向哪里？"
echo "  2. 机器人模型和坐标轴是否重合？"
echo "  3. 相差多少距离/角度？"
echo ""
