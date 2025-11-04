# 串口通信协议文档

## 概述

本文档定义了上位机(ROS2导航系统)与下位机(电机控制器)之间的串口通信协议。

## 通信参数

- **波特率**: 115200 bps (可配置)
- **数据位**: 8
- **停止位**: 1
- **校验位**: None
- **流控制**: None

## 数据帧格式

所有数据包采用固定长度格式，总长度11字节。

### 通用帧结构

```
+--------+--------+--------+--------+--------+--------+--------+--------+--------+--------+--------+
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 | Byte 8 | Byte 9 | Byte10 |
+--------+--------+--------+--------+--------+--------+--------+--------+--------+--------+--------+
| 0xAA   | 0x55   | CMD_ID | DATA0  | DATA1  | DATA2  | DATA3  | RESV0  | RESV1  | CHKSUM | 0x0D   |
+--------+--------+--------+--------+--------+--------+--------+--------+--------+--------+--------+
 Header1  Header2  命令ID   数据字节0 数据字节1 数据字节2 数据字节3 保留0   保留1   校验和   帧尾
```

### 字段说明

| 字段 | 字节 | 说明 |
|------|------|------|
| Header1 | 0 | 帧头标识1，固定为0xAA |
| Header2 | 1 | 帧头标识2，固定为0x55 |
| CMD_ID | 2 | 命令标识符，见命令列表 |
| DATA0-3 | 3-6 | 数据字段，具体含义由CMD_ID决定 |
| RESV0-1 | 7-8 | 保留字段，默认为0x00 |
| CHKSUM | 9 | 校验和，所有字节XOR运算 |
| Tail | 10 | 帧尾标识，固定为0x0D |

## 校验和计算

校验和为除校验和字节本身外的所有字节进行XOR运算：

```
CHKSUM = Header1 ^ Header2 ^ CMD_ID ^ DATA0 ^ DATA1 ^ DATA2 ^ DATA3 ^ RESV0 ^ RESV1
```

C++示例代码:
```cpp
uint8_t calculateChecksum(uint8_t* data, int len) {
    uint8_t checksum = 0;
    for (int i = 0; i < len - 2; i++) {  // 不包括checksum和tail
        checksum ^= data[i];
    }
    return checksum;
}
```

## 命令列表

### 0x01 - 速度控制命令

控制机器人的线速度和角速度。

**数据格式**:
```
DATA0: 线速度低字节 (int16_t低字节)
DATA1: 线速度高字节 (int16_t高字节)
DATA2: 角速度低字节 (int16_t低字节)
DATA3: 角速度高字节 (int16_t高字节)
```

**单位**:
- 线速度: mm/s (毫米/秒)
- 角速度: mrad/s (毫弧度/秒)

**取值范围**:
- 线速度: -2000 ~ 2000 mm/s (即 -2.0 ~ 2.0 m/s)
- 角速度: -3140 ~ 3140 mrad/s (即 -π ~ π rad/s)

**示例1 - 停止**:
```
线速度 = 0 mm/s, 角速度 = 0 mrad/s
数据包: AA 55 01 00 00 00 00 00 00 FE 0D
```

**示例2 - 前进 0.5 m/s**:
```
线速度 = 500 mm/s (0x01F4), 角速度 = 0
DATA0 = 0xF4, DATA1 = 0x01, DATA2 = 0x00, DATA3 = 0x00
数据包: AA 55 01 F4 01 00 00 00 00 XX 0D
校验和 = 0xAA ^ 0x55 ^ 0x01 ^ 0xF4 ^ 0x01 ^ 0x00 ^ 0x00 ^ 0x00 ^ 0x00 = 0xFF
数据包: AA 55 01 F4 01 00 00 00 00 FF 0D
```

**示例3 - 原地旋转 1.0 rad/s**:
```
线速度 = 0, 角速度 = 1000 mrad/s (0x03E8)
DATA0 = 0x00, DATA1 = 0x00, DATA2 = 0xE8, DATA3 = 0x03
数据包: AA 55 01 00 00 E8 03 00 00 XX 0D
校验和 = 0xAA ^ 0x55 ^ 0x01 ^ 0x00 ^ 0x00 ^ 0xE8 ^ 0x03 ^ 0x00 ^ 0x00 = 0xEF
数据包: AA 55 01 00 00 E8 03 00 00 EF 0D
```

**示例4 - 前进并转弯**:
```
线速度 = 300 mm/s (0x012C), 角速度 = 500 mrad/s (0x01F4)
DATA0 = 0x2C, DATA1 = 0x01, DATA2 = 0xF4, DATA3 = 0x01
数据包: AA 55 01 2C 01 F4 01 00 00 XX 0D
校验和 = 0xAA ^ 0x55 ^ 0x01 ^ 0x2C ^ 0x01 ^ 0xF4 ^ 0x01 ^ 0x00 ^ 0x00 = 0x8C
数据包: AA 55 01 2C 01 F4 01 00 00 8C 0D
```

### 0x02 - 状态查询命令 (上位机→下位机)

查询下位机状态，DATA0-3暂时未使用。

**响应**: 下位机应返回0x82状态应答命令。

### 0x82 - 状态应答命令 (下位机→上位机)

下位机应答状态信息。

**数据格式**:
```
DATA0: 电池电压低字节 (uint16_t, 单位: mV)
DATA1: 电池电压高字节
DATA2: 错误代码
DATA3: 系统状态标志
```

**错误代码** (DATA2):
- 0x00: 正常
- 0x01: 左电机故障
- 0x02: 右电机故障
- 0x04: 电池电压过低
- 0x08: 通信超时
- 0x10: 编码器故障

**系统状态标志** (DATA3):
- Bit 0: 急停开关状态 (1=按下)
- Bit 1: 充电状态 (1=充电中)
- Bit 2-7: 保留

### 0x03 - 参数设置命令 (预留)

用于设置下位机参数，如PID参数、限速等。

### 0x83 - 里程计数据 (下位机→上位机)

下位机主动上报里程计数据。

**数据格式**:
```
DATA0: 左轮编码器增量低字节 (int16_t)
DATA1: 左轮编码器增量高字节
DATA2: 右轮编码器增量低字节 (int16_t)
DATA3: 右轮编码器增量高字节
```

**单位**: 编码器脉冲数

## 通信流程

### 正常运行流程

```
上位机                           下位机
  │                               │
  │──── 0x01 速度命令 ──────────→│  (20Hz, 每50ms发送一次)
  │                               │
  │←──── 0x83 里程计数据 ─────────│  (20Hz, 每50ms发送一次)
  │                               │
  │──── 0x02 状态查询 ──────────→│  (1Hz, 每1秒发送一次)
  │                               │
  │←──── 0x82 状态应答 ───────────│
  │                               │
```

### 超时处理

**上位机侧**:
- 如果1秒内未收到新的速度命令，下位机应自动停车
- 连续3秒未收到速度命令，下位机进入安全模式

**下位机侧**:
- 如果3秒内未收到状态应答，上位机应记录警告
- 如果5秒内未收到里程计数据，上位机应停止导航

## ROS2集成

### 订阅话题

**serial_tx_data** (std_msgs/UInt8MultiArray):
```cpp
void serialTxCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    // msg->data 包含11字节的完整数据包
    // 直接发送到串口
    serial_port.write(msg->data.data(), msg->data.size());
}
```

### 发布话题

**serial_rx_data** (std_msgs/UInt8MultiArray):
```cpp
// 从串口接收数据并发布
std::vector<uint8_t> buffer(11);
size_t bytes_read = serial_port.read(buffer.data(), 11);
if (bytes_read == 11) {
    auto msg = std_msgs::msg::UInt8MultiArray();
    msg.data = buffer;
    serial_rx_pub->publish(msg);
}
```

## 调试工具

### 查看发送的十六进制数据

```bash
ros2 topic echo /serial_tx_hex
```

输出示例:
```
data: 'TX: aa 55 01 f4 01 00 00 00 00 ff 0d'
```

### 手动发送测试命令

```bash
# 创建测试发布器
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --once
```

### Python测试脚本

```python
#!/usr/bin/env python3
import serial

# 打开串口
ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

# 构造速度命令包: 前进 0.5 m/s
packet = bytes([0xAA, 0x55, 0x01, 0xF4, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x0D])

# 发送
ser.write(packet)

print(f"Sent: {' '.join([f'{b:02x}' for b in packet])}")

# 读取响应
response = ser.read(11)
if len(response) == 11:
    print(f"Received: {' '.join([f'{b:02x}' for b in response])}")
```

## 安全注意事项

1. **超时保护**: 必须实现速度命令超时检测，超时自动停车
2. **限速保护**: 下位机应对速度命令进行范围检查
3. **急停功能**: 硬件急停按钮应直接切断电机电源
4. **校验失败**: 校验和错误的数据包应直接丢弃
5. **帧同步**: 接收端应检测帧头，确保数据包对齐

## 版本历史

- v1.0 (2025-01-04): 初始版本
  - 定义基本速度控制命令
  - 定义状态查询和应答
  - 定义里程计数据格式

## 附录

### 数据类型转换

**int16转字节数组** (小端序):
```cpp
int16_t value = 500;
uint8_t low = value & 0xFF;
uint8_t high = (value >> 8) & 0xFF;
```

**字节数组转int16** (小端序):
```cpp
uint8_t low = 0xF4;
uint8_t high = 0x01;
int16_t value = (int16_t)((high << 8) | low);  // = 500
```

### 常用速度值对照表

| 速度(m/s) | 速度(mm/s) | 十六进制(小端) |
|-----------|------------|----------------|
| 0.0 | 0 | 0x00 0x00 |
| 0.1 | 100 | 0x64 0x00 |
| 0.2 | 200 | 0xC8 0x00 |
| 0.5 | 500 | 0xF4 0x01 |
| 1.0 | 1000 | 0xE8 0x03 |
| -0.5 | -500 | 0x0C 0xFE |
| -1.0 | -1000 | 0x18 0xFC |
