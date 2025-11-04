# 串口通信使用说明

## 概述

`serial_communication` 节点负责通过 Micro USB 与下位机开发板进行串口通信。它接收由 `serial_data_publisher` 准备好的数据包,并通过串口发送给下位机。

## 硬件连接

### 连接方式
- **接口类型**: Micro USB
- **默认设备**: `/dev/ttyUSB1` (如果雷达占用了 `/dev/ttyUSB0`)
- **波特率**: 115200 (可配置)

### 检查连接

1. 连接开发板到电脑的 USB 口
2. 检查设备是否被识别:
```bash
ls -l /dev/ttyUSB*
# 或
ls -l /dev/ttyACM*
```

3. 查看设备详细信息:
```bash
dmesg | tail -20
```

### 设置串口权限

**临时方法** (重启后失效):
```bash
sudo chmod 666 /dev/ttyUSB1
```

**永久方法** (推荐):
```bash
# 将当前用户添加到 dialout 组
sudo usermod -a -G dialout $USER

# 注销并重新登录使更改生效
# 或者执行:
newgrp dialout
```

## 节点配置

### 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `serial_port` | string | `/dev/ttyUSB1` | 串口设备路径 |
| `baudrate` | int | 115200 | 波特率 (支持: 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600) |
| `timeout_ms` | int | 100 | 读取超时时间 (毫秒) |

### 启动配置

在 launch 文件中修改参数:
```python
Node(
    package='navigation_control',
    executable='serial_communication',
    name='serial_communication',
    output='screen',
    parameters=[{
        'serial_port': '/dev/ttyUSB1',  # 根据实际设备修改
        'baudrate': 115200,              # 根据下位机配置修改
        'timeout_ms': 100,
    }],
),
```

命令行覆盖参数:
```bash
ros2 launch navigation_control mapping.launch.py \
    dev_board_port:=/dev/ttyACM0 \
    dev_board_baudrate:=230400
```

## 数据流

### 发送数据流程
1. Nav2 发布 `/cmd_vel` (导航控制命令)
2. `serial_data_publisher` 转换为 11 字节数据包 → `/serial_tx_data`
3. `serial_communication` 从 `/serial_tx_data` 接收 → 写入串口 → 发送到下位机

### 接收数据流程
1. `serial_communication` 从串口读取下位机数据
2. 发布到 `/serial_rx_data` (原始字节)
3. 发布到 `/serial_rx_hex` (十六进制字符串,方便调试)

## Topic 说明

### 订阅的 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/serial_tx_data` | `std_msgs/UInt8MultiArray` | 待发送的字节数据 (11字节协议包) |

### 发布的 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/serial_rx_data` | `std_msgs/UInt8MultiArray` | 从下位机接收的原始字节数据 |
| `/serial_rx_hex` | `std_msgs/String` | 接收数据的十六进制表示 (调试用) |
| `/serial_connection_status` | `std_msgs/String` | 连接状态信息 (包含发送/接收字节统计) |

## 使用示例

### 1. 启动建图模式 (包含串口通信)
```bash
# Source 环境
source /opt/ros/humble/setup.bash
source ~/slam/install/setup.bash

# 启动建图系统
ros2 launch navigation_control mapping.launch.py
```

### 2. 启动导航模式 (包含串口通信)
```bash
ros2 launch navigation_control navigation.launch.py
```

### 3. 监控串口连接状态
```bash
# 查看连接状态
ros2 topic echo /serial_connection_status

# 输出示例:
# data: "Connected | TX: 12450 bytes | RX: 3200 bytes"
```

### 4. 查看发送的数据
```bash
# 查看原始字节
ros2 topic echo /serial_tx_data

# 查看十六进制格式 (更易读)
ros2 topic echo /serial_tx_hex

# 输出示例:
# data: "AA 55 01 E8 03 00 00 00 00 EA 0D"
```

### 5. 查看接收的数据
```bash
# 原始字节
ros2 topic echo /serial_rx_data

# 十六进制格式
ros2 topic echo /serial_rx_hex
```

### 6. 手动测试串口通信节点
```bash
# 单独启动串口通信节点
ros2 run navigation_control serial_communication \
    --ros-args \
    -p serial_port:=/dev/ttyUSB1 \
    -p baudrate:=115200
```

## 调试技巧

### 1. 检查节点是否运行
```bash
ros2 node list | grep serial_communication
```

### 2. 查看节点参数
```bash
ros2 param list /serial_communication
ros2 param get /serial_communication serial_port
ros2 param get /serial_communication baudrate
```

### 3. 查看数据发送频率
```bash
ros2 topic hz /serial_tx_data
ros2 topic hz /serial_rx_data
```

### 4. 检查串口是否被占用
```bash
# 查看哪个进程在使用串口
sudo lsof /dev/ttyUSB1

# 或
fuser /dev/ttyUSB1
```

### 5. 使用串口调试工具
```bash
# 安装 minicom
sudo apt install minicom

# 配置并连接串口
sudo minicom -s
# 设置 Serial port setup -> A - Serial Device = /dev/ttyUSB1
# 设置 Serial port setup -> E - Bps/Par/Bits = 115200 8N1
# 选择 Save setup as dfl
# 选择 Exit
```

### 6. 查看内核日志
```bash
# 查看 USB 设备插拔日志
dmesg | grep tty

# 实时监控
dmesg -w
```

## 自动重连机制

`serial_communication` 节点内置了自动重连功能:

- **检测频率**: 每 5 秒检查一次连接状态
- **断线处理**: 检测到断线后自动尝试重新打开串口
- **状态反馈**: 通过 `/serial_connection_status` 发布连接状态

连接状态示例:
- 已连接: `"Connected | TX: 12450 bytes | RX: 3200 bytes"`
- 断开连接: `"Disconnected - trying to reconnect..."`
- 连接失败: `"Failed to open serial port: /dev/ttyUSB1"`

## 常见问题

### Q1: 提示 "Permission denied" 无法打开串口
**A**: 需要给串口设备添加权限或将用户加入 dialout 组 (见上文"设置串口权限")

### Q2: 找不到 /dev/ttyUSB1 设备
**A**: 
1. 检查 USB 线是否连接正确
2. 某些开发板可能显示为 `/dev/ttyACM0` 而不是 `/dev/ttyUSB1`
3. 使用 `dmesg | grep tty` 查看设备实际路径

### Q3: 串口连接成功但无数据发送
**A**: 
1. 检查 `/cmd_vel` 是否有数据: `ros2 topic echo /cmd_vel`
2. 检查 `serial_data_publisher` 是否运行: `ros2 node list | grep serial_data_publisher`
3. 查看 `/serial_tx_data` 是否有数据: `ros2 topic echo /serial_tx_data`

### Q4: 波特率应该设置为多少?
**A**: 必须与下位机配置一致。常见配置:
- Arduino: 9600, 115200
- STM32: 115200, 230400, 921600
- ESP32: 115200

### Q5: 如何验证串口通信是否正常?
**A**: 
1. 检查连接状态: `ros2 topic echo /serial_connection_status`
2. 查看发送数据: `ros2 topic echo /serial_tx_hex`
3. 如果下位机有回传数据,查看: `ros2 topic echo /serial_rx_hex`
4. 使用串口助手软件(如 minicom)连接同一串口,看是否能收到数据

### Q6: 节点频繁重连
**A**:
1. 检查 USB 线质量,尝试更换
2. 检查下位机供电是否稳定
3. 降低波特率尝试
4. 查看内核日志: `dmesg | tail -50`

## 下位机开发指南

### 接收数据格式

下位机应按照以下协议解析接收到的数据:

```c
typedef struct {
    uint8_t header1;      // 0xAA
    uint8_t header2;      // 0x55
    uint8_t cmd_id;       // 命令ID: 0x01=速度控制
    uint8_t linear_low;   // 线速度低字节 (mm/s)
    uint8_t linear_high;  // 线速度高字节 (mm/s)
    uint8_t angular_low;  // 角速度低字节 (mrad/s)
    uint8_t angular_high; // 角速度高字节 (mrad/s)
    uint8_t reserved1;    // 保留字节1
    uint8_t reserved2;    // 保留字节2
    uint8_t checksum;     // 校验和 (XOR)
    uint8_t tail;         // 0x0D
} __attribute__((packed)) SerialProtocol;
```

### Arduino 示例代码

```cpp
void setup() {
    Serial.begin(115200);  // 与 ROS2 节点配置一致
}

void loop() {
    if (Serial.available() >= 11) {
        uint8_t buffer[11];
        Serial.readBytes(buffer, 11);
        
        // 验证帧头和帧尾
        if (buffer[0] == 0xAA && buffer[1] == 0x55 && buffer[10] == 0x0D) {
            // 验证校验和
            uint8_t checksum = 0;
            for (int i = 2; i < 9; i++) {
                checksum ^= buffer[i];
            }
            
            if (checksum == buffer[9]) {
                // 解析数据
                int16_t linear_vel = (buffer[4] << 8) | buffer[3];   // mm/s
                int16_t angular_vel = (buffer[6] << 8) | buffer[5];  // mrad/s
                
                // 控制电机
                controlMotors(linear_vel, angular_vel);
            }
        }
    }
}
```

### STM32 示例代码

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        static uint8_t rx_buffer[11];
        static uint8_t rx_index = 0;
        
        uint8_t byte;
        HAL_UART_Receive(&huart1, &byte, 1, 100);
        
        if (rx_index == 0 && byte == 0xAA) {
            rx_buffer[rx_index++] = byte;
        } else if (rx_index == 1 && byte == 0x55) {
            rx_buffer[rx_index++] = byte;
        } else if (rx_index > 1 && rx_index < 11) {
            rx_buffer[rx_index++] = byte;
            
            if (rx_index == 11 && byte == 0x0D) {
                // 完整数据包接收完成
                processSerialPacket(rx_buffer);
                rx_index = 0;
            }
        } else {
            rx_index = 0;  // 重置
        }
        
        HAL_UART_Receive_IT(&huart1, &byte, 1);
    }
}
```

## 性能指标

- **发送延迟**: < 10ms (从 `/cmd_vel` 到串口发送)
- **数据完整性**: XOR 校验保证
- **断线恢复**: 5 秒内自动重连
- **最大吞吐**: 取决于波特率 (115200bps ≈ 11.5KB/s)

## 参考资料

- [串口协议详细说明](SERIAL_PROTOCOL.md)
- [LibSerial 文档](https://libserial.readthedocs.io/)
- [ROS2 Serial Driver](https://github.com/ros-drivers/serial_driver)
