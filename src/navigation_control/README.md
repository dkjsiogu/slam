# Navigation Control Package

完整的ROS2 SLAM导航系统，包含建图、定位、导航和串口通信准备功能。

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    Robot State Manager                      │
│              (系统模式切换和状态监控)                          │
└─────────────────────────────────────────────────────────────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
┌───────▼────────┐  ┌──────▼──────┐  ┌───────▼────────┐
│ Mapping Manager│  │Localization │  │  Navigation    │
│   (建图管理)    │  │  Manager    │  │  Controller    │
│                │  │  (定位管理)  │  │  (导航控制)    │
└───────┬────────┘  └──────┬──────┘  └───────┬────────┘
        │                  │                  │
        │        ┌─────────▼──────────┐       │
        │        │   AMCL / Nav2      │       │
        │        │    (定位/导航)      │       │
        │        └─────────┬──────────┘       │
        │                  │                  │
┌───────▼──────────────────▼──────────────────▼────────┐
│              Serial Data Publisher                   │
│          (准备发送给下位机的控制数据)                   │
└──────────────────────┬───────────────────────────────┘
                       │
                 ┌─────▼─────┐
                 │  /cmd_vel │
                 │ (速度命令) │
                 └───────────┘
```

## 核心功能模块

### 1. Mapping Manager (建图管理器)
- **功能**：管理Cartographer建图过程
- **服务**：
  - `start_mapping` - 开始建图
  - `stop_mapping` - 停止建图
  - `save_map` - 保存地图
- **话题**：
  - `mapping_status` - 发布建图状态

### 2. Localization Manager (定位管理器)
- **功能**：管理AMCL定位和重定位
- **服务**：
  - `start_localization` - 开始定位
  - `stop_localization` - 停止定位
  - `reset_localization` - 重定位
- **话题**：
  - `localization_status` - 发布定位状态
  - `current_pose` - 发布当前位姿

### 3. Navigation Controller (导航控制器)
- **功能**：管理Nav2导航和路径规划
- **服务**：
  - `navigate_to_goal` - 导航到目标点
  - `cancel_navigation` - 取消导航
- **话题**：
  - `navigation_status` - 发布导航状态

### 4. Serial Data Publisher (串口数据准备)
- **功能**：将速度命令转换为下位机协议格式
- **订阅**：`/cmd_vel` (geometry_msgs/Twist)
- **发布**：
  - `serial_tx_data` (std_msgs/UInt8MultiArray) - 原始字节数据
  - `serial_tx_hex` (std_msgs/String) - 十六进制字符串(调试)
  - `processed_cmd_vel` (geometry_msgs/Twist) - 处理后的速度

### 5. Robot State Manager (机器人状态管理)
- **功能**：统一管理系统状态和模式切换
- **服务**：
  - `switch_to_mapping` - 切换到建图模式
  - `switch_to_navigation` - 切换到导航模式
  - `emergency_stop` - 紧急停止
- **话题**：
  - `system_status` - 发布系统综合状态

## 串口数据协议

### 数据包格式

```
┌────────┬────────┬────────┬──────────┬───────────┬──────────┬──────────┬──────────┬────────┐
│ Header1│ Header2│ Cmd ID │ Linear_L │ Linear_H  │Angular_L │Angular_H │ Checksum │  Tail  │
│ 0xAA   │ 0x55   │ 0x01   │          │           │          │          │          │ 0x0D   │
└────────┴────────┴────────┴──────────┴───────────┴──────────┴──────────┴──────────┴────────┘
   1字节    1字节    1字节     1字节      1字节       1字节      1字节      1字节     1字节
```

### 字段说明

- **Header1/Header2**: 帧头标识 (0xAA 0x55)
- **Cmd ID**: 命令类型
  - `0x01` - 速度控制命令
  - `0x02` - 状态查询 (预留)
  - `0x03` - 参数设置 (预留)
- **Linear Velocity**: 线速度 (int16, mm/s)
  - 范围: -32768 ~ 32767 mm/s
  - 实际: -1000 ~ 1000 mm/s (±1 m/s)
- **Angular Velocity**: 角速度 (int16, mrad/s)
  - 范围: -32768 ~ 32767 mrad/s
  - 实际: -2000 ~ 2000 mrad/s (±2 rad/s)
- **Checksum**: 校验和 (XOR)
- **Tail**: 帧尾标识 (0x0D)

### 示例数据包

**停止命令**:
```
AA 55 01 00 00 00 00 00 00 FE 0D
```

**前进 0.5 m/s**:
```
AA 55 01 F4 01 00 00 00 00 XX 0D
```
(0x01F4 = 500 mm/s)

**旋转 1.0 rad/s**:
```
AA 55 01 00 00 E8 03 00 00 XX 0D
```
(0x03E8 = 1000 mrad/s)

## 编译和安装

```bash
cd ~/slam
source /opt/ros/<your-distro>/setup.bash
colcon build --packages-select navigation_control
source install/setup.bash
```

## 使用方法

### 1. 建图模式

```bash
# 启动建图模式
ros2 launch navigation_control mapping.launch.py

# 手动控制机器人移动建图
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 保存地图
ros2 service call /save_map std_srvs/srv/Trigger
```

### 2. 导航模式

```bash
# 启动导航模式
ros2 launch navigation_control navigation.launch.py map_file:=/path/to/your/map.yaml

# 发送导航目标
ros2 service call /navigate_to_goal std_srvs/srv/Trigger
```

### 3. 模式切换

```bash
# 切换到建图模式
ros2 service call /switch_to_mapping std_srvs/srv/Trigger

# 切换到导航模式
ros2 service call /switch_to_navigation std_srvs/srv/Trigger

# 紧急停止
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

### 4. 监控系统状态

```bash
# 查看系统综合状态
ros2 topic echo /system_status

# 查看建图状态
ros2 topic echo /mapping_status

# 查看定位状态
ros2 topic echo /localization_status

# 查看导航状态
ros2 topic echo /navigation_status

# 查看串口数据(十六进制)
ros2 topic echo /serial_tx_hex
```

## 参数配置

### Mapping Manager参数

```yaml
map_save_directory: "/path/to/maps"  # 地图保存目录
map_name: "my_map"                   # 地图名称
auto_save_interval: 60.0             # 自动保存间隔(秒)
```

### Localization Manager参数

```yaml
initial_pose_x: 0.0                  # 初始X坐标
initial_pose_y: 0.0                  # 初始Y坐标
initial_pose_yaw: 0.0                # 初始朝向
localization_quality_threshold: 0.5   # 定位质量阈值
```

### Navigation Controller参数

```yaml
navigation_timeout: 300.0            # 导航超时时间(秒)
goal_tolerance: 0.2                  # 目标容差(米)
```

### Serial Data Publisher参数

```yaml
max_linear_vel: 1.0                  # 最大线速度(m/s)
max_angular_vel: 2.0                 # 最大角速度(rad/s)
linear_scale: 1000.0                 # 线速度缩放(m/s -> mm/s)
angular_scale: 1000.0                # 角速度缩放(rad/s -> mrad/s)
velocity_timeout: 1.0                # 速度超时(秒)
smooth_factor: 0.8                   # 平滑因子(0-1)
```

## 下一步开发

### 串口通信模块
需要添加实际的串口通信节点，接收`serial_tx_data`话题的数据并发送给下位机。

推荐使用:
- `serial` 库 (C++)
- `boost::asio` (异步IO)

示例代码框架:
```cpp
#include <serial/serial.h>

// 创建串口对象
serial::Serial ser("/dev/ttyUSB1", 115200, serial::Timeout::simpleTimeout(1000));

// 订阅串口数据
void serialDataCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    ser.write(msg->data);
}
```

### 里程计反馈
添加从下位机接收里程计数据的功能:
- 接收编码器数据
- 计算里程计
- 发布到`/odom`话题
- 提供更准确的定位

### 电池监控
- 订阅下位机电池电压
- 低电量警告
- 自动返回充电桩

## 故障排除

### 1. 服务调用失败
```bash
# 检查节点是否运行
ros2 node list

# 检查服务是否可用
ros2 service list
```

### 2. 导航不工作
```bash
# 检查TF树
ros2 run tf2_tools view_frames

# 检查地图是否加载
ros2 topic echo /map --once

# 检查定位是否工作
ros2 topic echo /amcl_pose
```

### 3. 串口数据异常
```bash
# 查看原始数据
ros2 topic echo /serial_tx_hex

# 检查速度命令
ros2 topic echo /cmd_vel
```

## 依赖项

- ROS2 (Humble/Foxy及以上)
- Cartographer ROS
- Nav2导航栈
- AMCL
- Map Server
- TF2

安装依赖:
```bash
sudo apt install ros-<distro>-navigation2 \
                 ros-<distro>-nav2-bringup \
                 ros-<distro>-cartographer-ros
```

## 作者

SLAM Navigation System Developer

## 许可证

Apache 2.0
