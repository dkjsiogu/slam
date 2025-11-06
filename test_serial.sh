#!/bin/bash
# 串口通信测试脚本

echo "=========================================="
echo "  串口通信测试工具"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. 检查 USB 串口设备
echo -e "${YELLOW}[1/5] 检查 USB 串口设备...${NC}"
if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
    echo -e "${GREEN}✓ 找到 USB 串口设备:${NC}"
    ls -l /dev/ttyUSB*
    USB_PORTS=($(ls /dev/ttyUSB* 2>/dev/null))
elif ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo -e "${GREEN}✓ 找到 ACM 串口设备:${NC}"
    ls -l /dev/ttyACM*
    USB_PORTS=($(ls /dev/ttyACM* 2>/dev/null))
else
    echo -e "${RED}✗ 未找到任何 USB 串口设备${NC}"
    echo "  请确认:"
    echo "  1. 开发板已通过 USB 线连接到电脑"
    echo "  2. USB 线支持数据传输 (不是只充电的线)"
    echo "  3. 驱动程序已正确安装"
    exit 1
fi
echo ""

# 2. 检查串口权限
echo -e "${YELLOW}[2/5] 检查串口权限...${NC}"
for port in "${USB_PORTS[@]}"; do
    if [ -r "$port" ] && [ -w "$port" ]; then
        echo -e "${GREEN}✓ $port 有读写权限${NC}"
    else
        echo -e "${RED}✗ $port 没有读写权限${NC}"
        echo "  执行以下命令添加权限:"
        echo "  sudo chmod 666 $port"
        echo "  或者永久添加:"
        echo "  sudo usermod -a -G dialout \$USER"
        echo "  newgrp dialout"
    fi
done
echo ""

# 3. 检查 ROS2 环境
echo -e "${YELLOW}[3/5] 检查 ROS2 环境...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS2 环境未加载${NC}"
    echo "  执行: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓ ROS2 $ROS_DISTRO 环境已加载${NC}"
fi

if [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}✗ 工作空间未编译${NC}"
    echo "  执行: colcon build --packages-select navigation_control"
    exit 1
else
    source install/setup.bash
    echo -e "${GREEN}✓ 工作空间已加载${NC}"
fi
echo ""

# 4. 检查 serial_communication 节点
echo -e "${YELLOW}[4/5] 检查 serial_communication 节点...${NC}"
EXEC_PATH="install/navigation_control/lib/navigation_control/serial_communication"
if [ -f "$EXEC_PATH" ]; then
    echo -e "${GREEN}✓ serial_communication 节点已编译${NC}"
    echo "  路径: $EXEC_PATH"
else
    echo -e "${RED}✗ serial_communication 节点未找到${NC}"
    echo "  执行: colcon build --packages-select navigation_control"
    exit 1
fi
echo ""

# 5. 提供测试选项
echo -e "${YELLOW}[5/5] 测试选项${NC}"
echo "请选择要执行的测试:"
echo "  1) 查看可用串口详细信息"
echo "  2) 使用 minicom 测试串口 (需要 sudo apt install minicom)"
echo "  3) 启动 serial_communication 节点 (单独测试)"
echo "  4) 启动完整建图系统 (包含串口通信)"
echo "  5) 监控串口数据 (需要先启动系统)"
echo "  6) 退出"
echo ""
read -p "请输入选项 (1-6): " choice

case $choice in
    1)
        echo ""
        echo "=== USB 串口设备信息 ==="
        for port in "${USB_PORTS[@]}"; do
            echo ""
            echo "设备: $port"
            udevadm info --name=$port | grep -E "ID_VENDOR|ID_MODEL|ID_SERIAL"
        done
        ;;
    2)
        echo ""
        if ! command -v minicom &> /dev/null; then
            echo -e "${RED}minicom 未安装${NC}"
            echo "执行: sudo apt install minicom"
            exit 1
        fi
        
        if [ ${#USB_PORTS[@]} -gt 1 ]; then
            echo "发现多个串口设备:"
            for i in "${!USB_PORTS[@]}"; do
                echo "  $i) ${USB_PORTS[$i]}"
            done
            read -p "选择串口 (0-$((${#USB_PORTS[@]}-1))): " port_idx
            SELECTED_PORT="${USB_PORTS[$port_idx]}"
        else
            SELECTED_PORT="${USB_PORTS[0]}"
        fi
        
        echo "启动 minicom 连接到 $SELECTED_PORT (115200 8N1)"
        echo "提示: Ctrl+A 然后 Z 查看帮助, Ctrl+A 然后 X 退出"
        sudo minicom -D $SELECTED_PORT -b 115200
        ;;
    3)
        echo ""
        if [ ${#USB_PORTS[@]} -gt 1 ]; then
            echo "发现多个串口设备:"
            for i in "${!USB_PORTS[@]}"; do
                echo "  $i) ${USB_PORTS[$i]}"
            done
            read -p "选择开发板串口 (0-$((${#USB_PORTS[@]}-1))): " port_idx
            SELECTED_PORT="${USB_PORTS[$port_idx]}"
        else
            SELECTED_PORT="${USB_PORTS[0]}"
        fi
        
        read -p "输入波特率 [115200]: " baudrate
        baudrate=${baudrate:-115200}
        
        echo "启动 serial_communication 节点..."
        echo "设备: $SELECTED_PORT"
        echo "波特率: $baudrate"
        echo ""
        ros2 run navigation_control serial_communication \
            --ros-args \
            -p serial_port:=$SELECTED_PORT \
            -p baudrate:=$baudrate
        ;;
    4)
        echo ""
        if [ ${#USB_PORTS[@]} -gt 1 ]; then
            echo "发现多个串口设备,请分别指定雷达和开发板串口:"
            for i in "${!USB_PORTS[@]}"; do
                echo "  $i) ${USB_PORTS[$i]}"
            done
            read -p "选择雷达串口 (0-$((${#USB_PORTS[@]}-1))): " lidar_idx
            read -p "选择开发板串口 (0-$((${#USB_PORTS[@]}-1))): " board_idx
            LIDAR_PORT="${USB_PORTS[$lidar_idx]}"
            BOARD_PORT="${USB_PORTS[$board_idx]}"
        else
            echo -e "${YELLOW}只发现一个串口,假设为雷达${NC}"
            LIDAR_PORT="${USB_PORTS[0]}"
            BOARD_PORT="/dev/ttyUSB1"
        fi
        
        read -p "输入开发板波特率 [115200]: " baudrate
        baudrate=${baudrate:-115200}
        
        echo "启动建图系统..."
        echo "雷达串口: $LIDAR_PORT"
        echo "开发板串口: $BOARD_PORT"
        echo "开发板波特率: $baudrate"
        echo ""
        ros2 launch navigation_control mapping.launch.py \
            serial_port:=$LIDAR_PORT \
            dev_board_port:=$BOARD_PORT \
            dev_board_baudrate:=$baudrate
        ;;
    5)
        echo ""
        echo "请在另一个终端中启动系统,然后按回车键继续..."
        read
        
        echo "打开新终端窗口监控串口数据..."
        
        # 检测终端模拟器
        if command -v gnome-terminal &> /dev/null; then
            gnome-terminal -- bash -c "
                source /opt/ros/$ROS_DISTRO/setup.bash
                source install/setup.bash
                echo '=== 串口连接状态 ==='
                ros2 topic echo /serial_connection_status &
                PID1=\$!
                sleep 2
                echo ''
                echo '=== 发送数据 (十六进制) ==='
                ros2 topic echo /serial_tx_hex &
                PID2=\$!
                sleep 2
                echo ''
                echo '=== 接收数据 (十六进制) ==='
                ros2 topic echo /serial_rx_hex &
                PID3=\$!
                echo ''
                echo '按 Ctrl+C 停止监控'
                wait \$PID1 \$PID2 \$PID3
            "
        elif command -v xterm &> /dev/null; then
            xterm -e "
                source /opt/ros/$ROS_DISTRO/setup.bash
                source install/setup.bash
                ros2 topic echo /serial_connection_status &
                ros2 topic echo /serial_tx_hex &
                ros2 topic echo /serial_rx_hex &
                bash
            " &
        else
            echo "未找到终端模拟器,在当前终端监控:"
            echo ""
            echo "=== 串口连接状态 ==="
            ros2 topic echo /serial_connection_status --once
            echo ""
            echo "=== 发送数据 (最近一条) ==="
            ros2 topic echo /serial_tx_hex --once
            echo ""
            echo "要持续监控,请在新终端执行:"
            echo "  ros2 topic echo /serial_connection_status"
            echo "  ros2 topic echo /serial_tx_hex"
            echo "  ros2 topic echo /serial_rx_hex"
        fi
        ;;
    6)
        echo "退出测试工具"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}测试完成!${NC}"
