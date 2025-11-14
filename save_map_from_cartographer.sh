#!/bin/bash
# Cartographer地图保存脚本 - 从.pbstream导出到pgm+yaml格式

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Cartographer地图格式转换工具${NC}"
echo -e "${GREEN}  .pbstream -> .pgm + .yaml${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查参数
if [ $# -lt 1 ]; then
    echo -e "${YELLOW}用法: $0 <pbstream文件路径> [输出目录]${NC}"
    echo -e "${YELLOW}示例: $0 ~/slam/src/navigation_control/maps/my_map.pbstream${NC}"
    exit 1
fi

PBSTREAM_FILE="$1"
OUTPUT_DIR="${2:-$(dirname "$PBSTREAM_FILE")}"
MAP_NAME=$(basename "$PBSTREAM_FILE" .pbstream)

# 检查输入文件
if [ ! -f "$PBSTREAM_FILE" ]; then
    echo -e "${RED}错误: 找不到文件 $PBSTREAM_FILE${NC}"
    exit 1
fi

# 创建输出目录
mkdir -p "$OUTPUT_DIR"

echo -e "${YELLOW}输入文件: $PBSTREAM_FILE${NC}"
echo -e "${YELLOW}输出目录: $OUTPUT_DIR${NC}"
echo -e "${YELLOW}地图名称: $MAP_NAME${NC}"
echo ""

# 执行转换
echo -e "${GREEN}开始转换...${NC}"

ros2 run nav2_map_server map_saver_cli \
    --fmt pgm \
    --occ 0.65 \
    --free 0.25 \
    -f "${OUTPUT_DIR}/${MAP_NAME}" \
    --ros-args -p map_subscribe_transient_local:=true

# 检查结果
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  转换成功！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "生成的文件:"
    echo -e "  地图图像: ${OUTPUT_DIR}/${MAP_NAME}.pgm"
    echo -e "  地图配置: ${OUTPUT_DIR}/${MAP_NAME}.yaml"
    echo ""
    echo -e "${YELLOW}提示:${NC}"
    echo -e "1. 请先运行 cartographer 并加载 .pbstream 文件"
    echo -e "2. 确保 /map 话题正在发布"
    echo -e "3. 然后运行此脚本进行转换"
else
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}  转换失败！${NC}"
    echo -e "${RED}========================================${NC}"
    echo -e "${YELLOW}可能的原因:${NC}"
    echo -e "1. Cartographer节点未运行"
    echo -e "2. /map 话题未发布"
    echo -e "3. nav2_map_server 未安装"
    echo ""
    echo -e "${YELLOW}解决方法:${NC}"
    echo -e "1. 先运行: ros2 launch navigation_control cartographer_localization.launch.py"
    echo -e "2. 等待地图加载完成后，再运行此脚本"
    echo -e "3. 安装依赖: sudo apt install ros-\$ROS_DISTRO-nav2-map-server"
    exit 1
fi
