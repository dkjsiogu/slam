#!/bin/bash
# 保存SLAM地图脚本
# 从 /slam_map 话题读取地图并保存到文件

# 默认保存路径
MAP_DIR="$HOME/slam/src/navigation_control/maps"
MAP_NAME="my_slam_map"

# 解析命令行参数
if [ $# -ge 1 ]; then
    MAP_NAME="$1"
fi

# 创建地图目录（如果不存在）
mkdir -p "$MAP_DIR"

# 完整路径
MAP_PATH="$MAP_DIR/$MAP_NAME"

echo "==========================================="
echo "保存SLAM地图"
echo "==========================================="
echo "地图名称: $MAP_NAME"
echo "保存路径: $MAP_PATH"
echo ""

# 检查 /slam_map 话题是否存在
if ! ros2 topic list | grep -q "/slam_map"; then
    echo "❌ 错误: /slam_map 话题不存在"
    echo ""
    echo "可用的地图话题:"
    ros2 topic list | grep map
    echo ""
    echo "请确保 SLAM Toolbox 正在运行:"
    echo "  ./start_slam_toolbox.sh"
    exit 1
fi

echo "📍 检测到 /slam_map 话题"
echo ""

# 使用 map_saver 保存地图
echo "💾 开始保存地图..."
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH" --ros-args -p save_map_timeout:=10000.0

# 检查是否成功
if [ -f "${MAP_PATH}.pgm" ] && [ -f "${MAP_PATH}.yaml" ]; then
    echo ""
    echo "==========================================="
    echo "✅ 地图保存成功！"
    echo "==========================================="
    echo "地图文件:"
    echo "  ${MAP_PATH}.yaml"
    echo "  ${MAP_PATH}.pgm"
    echo ""
    
    # 显示地图信息
    echo "地图信息:"
    cat "${MAP_PATH}.yaml"
    echo ""
    
    # 显示文件大小
    echo "文件大小:"
    ls -lh "${MAP_PATH}.pgm" | awk '{print "  " $9 ": " $5}'
    ls -lh "${MAP_PATH}.yaml" | awk '{print "  " $9 ": " $5}'
else
    echo ""
    echo "==========================================="
    echo "❌ 地图保存失败"
    echo "==========================================="
    echo "可能的原因:"
    echo "1. nav2_map_server 未安装"
    echo "   安装命令: sudo apt install ros-humble-nav2-map-server"
    echo "2. /slam_map 话题没有数据"
    echo "3. 磁盘空间不足"
    exit 1
fi
