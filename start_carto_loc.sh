#!/bin/bash
# 使用真正的Cartographer纯定位模式

cd ~/slam
source install/setup.bash

echo "=========================================="
echo "  Cartographer 纯定位模式"
echo "=========================================="
echo ""
echo "✅ 使用真正的Cartographer定位"
echo "✅ 加载固定pbstream地图"
echo "✅ 不建图,只定位"
echo "✅ 自动全局定位"
echo "✅ 稳定不漂移"
echo ""
echo "使用的地图: src/navigation_control/maps/my_map.pbstream"
echo ""
echo "=========================================="
echo ""

ros2 launch navigation_control cartographer_localization.launch.py
