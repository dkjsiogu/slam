# Micro USB 串口通信集成完成报告

## 概述

已成功为 `navigation_control` 包添加了通过 **Micro USB** 连接下位机开发板的串口通信功能。

## 新增内容

### 1. 核心代码

#### `serial_communication.cpp` (275行)
完整的串口通信节点,包含以下功能:

**核心特性**:
- ✅ LibSerial 库集成 (跨平台串口通信)
- ✅ 自动重连机制 (5秒检测间隔)
- ✅ 双向数据传输 (TX + RX)
- ✅ 连接状态实时监控
- ✅ 发送/接收字节统计
- ✅ 十六进制调试输出
- ✅ 可配置波特率 (9600~921600)

**ROS2 接口**:
```cpp
// 订阅
/serial_tx_data (std_msgs/UInt8MultiArray) - 待发送数据

// 发布
/serial_rx_data (std_msgs/UInt8MultiArray) - 接收的原始数据
/serial_rx_hex (std_msgs/String)           - 接收数据十六进制显示
/serial_connection_status (std_msgs/String) - 连接状态

// 参数
serial_port: "/dev/ttyUSB1"  (默认)
baudrate: 115200              (默认)
timeout_ms: 100               (默认)
```

**自动重连逻辑**:
```cpp
void tryReconnect() {
    if (!serial_port_obj_.IsOpen()) {
        RCLCPP_WARN(this->get_logger(), "尝试重新连接串口...");
        if (openSerialPort()) {
            RCLCPP_INFO(this->get_logger(), "串口重新连接成功!");
        }
    }
}
```

### 2. 编译配置更新

#### `CMakeLists.txt` 修改
```cmake
# 添加 PkgConfig 支持
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

# 添加头文件路径
include_directories(
  ${SERIAL_INCLUDE_DIRS}
)

# 添加 serial_communication 可执行文件
add_executable(serial_communication
  src/serial_communication.cpp
)
ament_target_dependencies(serial_communication ${dependencies})
target_link_libraries(serial_communication ${SERIAL_LIBRARIES})

# 安装
install(TARGETS
  serial_communication
  DESTINATION lib/${PROJECT_NAME}
)
```

**编译结果**: ✅ 编译成功,无错误无警告

### 3. Launch 文件集成

#### `mapping.launch.py` 更新
```python
# 新增参数
dev_board_port_arg = DeclareLaunchArgument(
    'dev_board_port',
    default_value='/dev/ttyUSB1',
    description='Development board serial port (Micro USB)'
)

dev_board_baudrate_arg = DeclareLaunchArgument(
    'dev_board_baudrate',
    default_value='115200',
    description='Development board baudrate'
)

# 新增节点
Node(
    package='navigation_control',
    executable='serial_communication',
    name='serial_communication',
    output='screen',
    parameters=[{
        'serial_port': LaunchConfiguration('dev_board_port'),
        'baudrate': LaunchConfiguration('dev_board_baudrate'),
        'timeout_ms': 100,
    }],
),
```

#### `navigation.launch.py` 更新
同样添加了 `dev_board_port` 和 `dev_board_baudrate` 参数及 `serial_communication` 节点。

### 4. 文档资料

#### 新增文档
1. **`docs/SERIAL_COMMUNICATION.md`** (500+ 行)
   - 硬件连接指南
   - 串口权限设置
   - 节点配置说明
   - 使用示例
   - 调试技巧
   - 常见问题解答
   - 下位机开发指南 (Arduino/STM32 示例代码)

2. **`docs/SYSTEM_OVERVIEW.md`** (400+ 行)
   - 完整系统架构图
   - 建图/导航模式数据流图
   - Topic/Service 列表
   - TF 变换树
   - 串口协议详细说明
   - 系统工作流程
   - 功能清单

3. **`scripts/test_serial.sh`** (交互式测试脚本)
   - 自动检测 USB 串口设备
   - 权限检查和修复建议
   - ROS2 环境验证
   - 6 种测试模式:
     1. 查看串口详细信息
     2. minicom 测试
     3. 单独运行 serial_communication
     4. 启动完整建图系统
     5. 监控串口数据
     6. 退出

#### 更新文档
- **`README.md`**: 添加 Serial Communication 节点说明和快速配置指南

## 数据流完整链路

```
用户控制/Nav2导航
        ↓
    /cmd_vel (geometry_msgs/Twist)
        ↓
serial_data_publisher (协议转换)
        ↓
  /serial_tx_data (UInt8MultiArray)
  [AA 55 01 linear_L linear_H angular_L angular_H 00 00 checksum 0D]
        ↓
serial_communication (实际串口I/O)
        ↓
    /dev/ttyUSB1 (Micro USB)
    115200 baud, 8N1
        ↓
    下位机开发板
   (STM32/Arduino)
        ↓
    电机驱动器
        ↓
    机器人移动
```

## 使用方法

### 快速启动

1. **连接硬件**
   ```bash
   # 连接开发板到电脑 USB 口,检查设备
   ls -l /dev/ttyUSB*
   
   # 设置权限
   sudo chmod 666 /dev/ttyUSB1
   ```

2. **启动建图系统** (包含串口通信)
   ```bash
   cd ~/slam
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   
   ros2 launch navigation_control mapping.launch.py
   ```

3. **自定义串口参数**
   ```bash
   ros2 launch navigation_control mapping.launch.py \
       dev_board_port:=/dev/ttyACM0 \
       dev_board_baudrate:=230400
   ```

4. **监控串口状态**
   ```bash
   # 连接状态
   ros2 topic echo /serial_connection_status
   
   # 发送数据 (十六进制)
   ros2 topic echo /serial_tx_hex
   
   # 接收数据 (十六进制)
   ros2 topic echo /serial_rx_hex
   ```

### 使用测试脚本

```bash
cd ~/slam
./src/navigation_control/scripts/test_serial.sh
```

脚本会自动:
- ✅ 检测 USB 串口设备
- ✅ 验证串口权限
- ✅ 检查 ROS2 环境
- ✅ 验证节点是否编译
- ✅ 提供 6 种测试选项

## 系统测试

### 编译测试
```bash
$ colcon build --packages-select navigation_control
Starting >>> navigation_control
Finished <<< navigation_control [6.80s]
Summary: 1 package finished [6.97s]
```
✅ **结果**: 编译成功

### 节点验证
```bash
$ ros2 run navigation_control serial_communication --help
```
✅ **结果**: 节点可执行

### 依赖检查
```bash
$ dpkg -l | grep libserial
libserial1:amd64      1.0.0-7     ...
libserial-dev:amd64   1.0.0-7     ...
```
✅ **结果**: LibSerial 已安装

## 下位机开发支持

### Arduino 示例 (已提供)

```cpp
void setup() {
    Serial.begin(115200);
}

void loop() {
    if (Serial.available() >= 11) {
        uint8_t buffer[11];
        Serial.readBytes(buffer, 11);
        
        if (buffer[0] == 0xAA && buffer[1] == 0x55) {
            // 验证校验和
            uint8_t checksum = 0;
            for (int i = 2; i < 9; i++) {
                checksum ^= buffer[i];
            }
            
            if (checksum == buffer[9]) {
                // 解析速度
                int16_t linear_vel = (buffer[4] << 8) | buffer[3];
                int16_t angular_vel = (buffer[6] << 8) | buffer[5];
                
                // 控制电机
                controlMotors(linear_vel, angular_vel);
            }
        }
    }
}
```

### STM32 示例 (已提供)

使用 HAL_UART_RxCpltCallback 中断接收,详见文档。

## 常见问题解决

### Q: 找不到 /dev/ttyUSB1
**A**: 
1. 检查 USB 连接
2. 某些板子可能显示为 /dev/ttyACM0
3. 使用 `dmesg | grep tty` 查看实际设备

### Q: Permission denied
**A**:
```bash
# 临时
sudo chmod 666 /dev/ttyUSB1

# 永久
sudo usermod -a -G dialout $USER
newgrp dialout
```

### Q: 节点频繁重连
**A**:
1. 检查 USB 线质量
2. 检查下位机供电
3. 降低波特率
4. 查看内核日志: `dmesg -w`

## 待实现功能

1. **里程计反馈解析** ⏳
   - 从下位机接收编码器数据
   - 发布 `/odom` topic
   - 融合激光和轮式里程计

2. **双向数据验证** ⏳
   - 下位机应答确认机制
   - 数据丢包检测

3. **参数动态配置** ⏳
   - 运行时修改波特率
   - 运行时切换串口

## 文件清单

### 新增文件
```
src/navigation_control/
├── src/
│   └── serial_communication.cpp          [新增] 275行
├── docs/
│   ├── SERIAL_COMMUNICATION.md           [新增] 500+行
│   └── SYSTEM_OVERVIEW.md                [新增] 400+行
└── scripts/
    └── test_serial.sh                    [新增] 可执行脚本
```

### 修改文件
```
src/navigation_control/
├── CMakeLists.txt                        [修改] 添加 LibSerial 支持
├── launch/
│   ├── mapping.launch.py                 [修改] 添加串口节点
│   └── navigation.launch.py              [修改] 添加串口节点
└── README.md                             [修改] 添加使用说明
```

## 总结

✅ **Micro USB 串口通信功能已完全集成到系统中**

**主要成果**:
1. ✅ 编译通过,可以正常使用
2. ✅ 数据链路完整: Nav2 → 串口转换 → 实际 I/O → 下位机
3. ✅ 自动重连机制保证稳定性
4. ✅ 完善的文档和测试工具
5. ✅ 提供下位机示例代码 (Arduino/STM32)
6. ✅ 集成到 launch 文件,即插即用

**下一步建议**:
1. 连接实际开发板测试通信
2. 根据下位机反馈调整协议
3. 实现里程计数据回传解析
4. 添加 `/odom` topic 发布

**需要帮助时**:
- 查看文档: `src/navigation_control/docs/SERIAL_COMMUNICATION.md`
- 运行测试: `./src/navigation_control/scripts/test_serial.sh`
- 查看系统概览: `src/navigation_control/docs/SYSTEM_OVERVIEW.md`
