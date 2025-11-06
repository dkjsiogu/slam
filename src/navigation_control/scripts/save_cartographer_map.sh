#!/bin/bash
# Cartographer 地图保存脚本
# 在建图完成后运行此脚本保存地图

set -e

# 默认保存路径
MAP_DIR="$HOME/slam/src/navigation_control/maps"
MAP_NAME="my_map"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --name)
            MAP_NAME="$2"
            shift 2
            ;;
        --dir)
            MAP_DIR="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            echo "用法: $0 [--name MAP_NAME] [--dir MAP_DIR]"
            exit 1
            ;;
    esac
done

PBSTREAM_FILE="$MAP_DIR/${MAP_NAME}.pbstream"
PGM_FILE="$MAP_DIR/${MAP_NAME}.pgm"
YAML_FILE="$MAP_DIR/${MAP_NAME}.yaml"

echo "=========================================="
echo "Cartographer 地图保存工具"
echo "=========================================="
echo "地图名称: $MAP_NAME"
echo "保存目录: $MAP_DIR"
echo "=========================================="

# 确保目录存在
mkdir -p "$MAP_DIR"

# 步骤1: 完成当前轨迹
echo ""
echo "[1/3] 完成当前轨迹..."
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
if [ $? -eq 0 ]; then
    echo "✓ 轨迹已完成"
else
    echo "✗ 完成轨迹失败"
    exit 1
fi

sleep 2

# 步骤2: 保存 .pbstream 文件
echo ""
echo "[2/3] 保存 Cartographer 地图文件..."
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$PBSTREAM_FILE'}"
if [ $? -eq 0 ]; then
    echo "✓ .pbstream 文件已保存: $PBSTREAM_FILE"
else
    echo "✗ 保存 .pbstream 文件失败"
    exit 1
fi

sleep 1

# 步骤3: 保存 .pgm 和 .yaml 文件（用于其他导航系统）
echo ""
echo "[3/3] 保存 ROS 地图文件 (.pgm + .yaml)..."
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME" --ros-args -p save_map_timeout:=10000.0
if [ $? -eq 0 ]; then
    echo "✓ .pgm/.yaml 文件已保存"
else
    echo "⚠ 保存 .pgm/.yaml 文件失败（可能 map_server 未运行）"
    echo "  可以手动运行: ros2 run nav2_map_server map_saver_cli -f $MAP_DIR/$MAP_NAME"
fi

echo ""
echo "=========================================="
echo "地图保存完成！"
echo "=========================================="
echo "文件列表:"
if [ -f "$PBSTREAM_FILE" ]; then
    echo "  ✓ $PBSTREAM_FILE ($(du -h "$PBSTREAM_FILE" | cut -f1))"
fi
if [ -f "$PGM_FILE" ]; then
    echo "  ✓ $PGM_FILE ($(du -h "$PGM_FILE" | cut -f1))"
fi
if [ -f "$YAML_FILE" ]; then
    echo "  ✓ $YAML_FILE"
fi
echo ""
echo "使用方法:"
echo "  纯定位: ros2 launch navigation_control cartographer_localization.launch.py pbstream_file:=$PBSTREAM_FILE"
echo "=========================================="
