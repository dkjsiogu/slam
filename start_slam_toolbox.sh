#!/bin/bash
# SLAM Toolbox 纯定位启动脚本

cd ~/slam
source install/setup.bash

echo "=========================================="
echo "  SLAM Toolbox 纯定位模式 + 导航"
echo "=========================================="
echo ""
echo "📍 功能:"
echo "  - map_server: 加载 pgm+yaml 老图 (完整区域)"
echo "  - SLAM Toolbox: 实时扫描匹配优化 (/slam_map)"
echo "  - map_republisher: 智能融合 → /map_viz"
echo ""
echo "💡 使用 '2D Pose Estimate' 设置初始位姿"
echo ""
echo "🗺️  地图话题架构:"
echo "  /map (老图) + /slam_map (优化) → /map_viz (融合)"
echo ""
echo "✨ 优势:"
echo "  ✓ 可以到之前去过的地方 (老图保留)"
echo "  ✓ 实时优化障碍物位置 (SLAM 覆盖)"
echo "  ✓ 两全其美!"
echo ""
echo "=========================================="
echo ""

# 检查 pgm+yaml 地图文件
if [ ! -f "src/navigation_control/maps/my_map.yaml" ]; then
    echo "❌ 错误: 地图文件不存在!"
    echo ""
    echo "请确保地图文件位于:"
    echo "  src/navigation_control/maps/my_map.yaml"
    echo "  src/navigation_control/maps/my_map.pgm"
    echo ""
    exit 1
fi

ros2 launch navigation_control slam_toolbox_localization.launch.py
