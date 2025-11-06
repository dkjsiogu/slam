#!/bin/bash
# Cartographer 纯定位模式快速测试脚本

set -e

MAPS_DIR="$HOME/slam/src/navigation_control/maps"
PBSTREAM_FILE="$MAPS_DIR/my_map.pbstream"

echo "=========================================="
echo "Cartographer 纯定位模式测试"
echo "=========================================="

# 检查地图文件是否存在
if [ ! -f "$PBSTREAM_FILE" ]; then
    echo "❌ 错误: 找不到地图文件: $PBSTREAM_FILE"
    echo ""
    echo "请先运行建图模式并保存地图:"
    echo "  1. ros2 launch navigation_control mapping.launch.py"
    echo "  2. 建图完成后运行: ./save_cartographer_map.sh"
    echo ""
    exit 1
fi

echo "✓ 找到地图文件: $PBSTREAM_FILE"
echo "  文件大小: $(du -h "$PBSTREAM_FILE" | cut -f1)"
echo ""

# 检查设备
echo "检查设备连接..."
if [ -e /dev/radar ]; then
    echo "✓ 激光雷达: /dev/radar"
else
    echo "⚠ 激光雷达未找到 (将使用 /dev/ttyUSB0)"
fi

if [ -e /dev/stm32 ]; then
    echo "✓ STM32: /dev/stm32"
else
    echo "⚠ STM32未找到 (将使用 /dev/ttyACM0)"
fi

echo ""
echo "=========================================="
echo "启动纯定位模式..."
echo "=========================================="
echo ""
echo "注意事项:"
echo "  1. 确保机器人放在地图对应的实际位置"
echo "  2. 等待 5-10 秒让 Cartographer 初始化"
echo "  3. 在 RViz2 中检查机器人位姿是否正确"
echo "  4. 使用 '2D Goal Pose' 设置导航目标"
echo ""
echo "按 Ctrl+C 停止"
echo ""

sleep 3

# 启动纯定位模式
ros2 launch navigation_control cartographer_localization.launch.py \
  pbstream_file:="$PBSTREAM_FILE"
