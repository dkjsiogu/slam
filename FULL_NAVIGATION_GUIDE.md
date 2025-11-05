# 全向轮SLAM导航系统 - 完整功能文档

## 📋 系统概览

本系统是一个**完整的ROS2全向轮机器人SLAM导航解决方案**，包含以下核心功能：

### ✨ 新增功能

1. **障碍物距离监控** - 实时显示前后左右障碍物距离（米为单位）
2. **手动目标点设置** - 方便路径规划调试的服务接口
3. **全向轮运动控制** - 支持Vx、Vy、Wz三自由度控制
4. **CRC-16校验** - 确保串口通信数据完整性
5. **可视化标记** - RViz显示障碍物距离和目标点

---

## 🏗️ 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│                      SLAM & Navigation Layer                  │
│  ┌────────────┐  ┌──────────────┐  ┌───────────────────┐    │
│  │ RPLIDAR    │→→│ Cartographer │→→│ Nav2 (规划/控制)  │    │
│  └────────────┘  └──────────────┘  └───────────────────┘    │
└────────────────────────────┬─────────────────────────────────┘
                             │ /cmd_vel
                             ↓
┌──────────────────────────────────────────────────────────────┐
│                    Application Layer (新增)                   │
│  ┌─────────────────┐  ┌──────────────────┐  ┌─────────────┐ │
│  │ Obstacle Monitor│  │ Manual Goal      │  │ Omni Wheel  │ │
│  │ 障碍物监控      │  │ Setter           │  │ Controller  │ │
│  │ • 距离显示(米)  │  │ 手动目标点设置   │  │ 全向轮控制  │ │
│  │ • 分区域检测    │  │ • 预设点位       │  │ • Vx/Vy/Wz  │ │
│  │ • 安全警告      │  │ • 服务接口       │  │ • CRC-16    │ │
│  └─────────────────┘  └──────────────────┘  └──────┬──────┘ │
└─────────────────────────────────────────────────────┼────────┘
                                                       │ serial_tx_data
                                                       ↓
┌──────────────────────────────────────────────────────────────┐
│                      Hardware Layer                           │
│  ┌──────────────────┐              ┌──────────────────────┐  │
│  │ Serial Comm      │ Micro USB    │ 下位机 (STM32/ESP32) │  │
│  │ (LibSerial)      │←────────────→│ • 全向轮驱动         │  │
│  │ • 自动重连       │   115200 bps │ • CRC校验            │  │
│  │ • 数据监控       │              │ • 编码器反馈         │  │
│  └──────────────────┘              └──────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
```

---

## 📦 新增节点说明

### 1️⃣ obstacle_monitor (障碍物监控)

**功能**: 实时监控激光雷达数据，计算并显示各方向障碍物距离

**订阅话题**:
- `/scan` (sensor_msgs/LaserScan) - 激光雷达数据

**发布话题**:
- `/obstacle_info` (std_msgs/String) - 综合障碍物信息
- `/min_obstacle_distance` (std_msgs/Float32) - 最近障碍物距离(米)
- `/front_obstacle_distance` (std_msgs/Float32) - 前方障碍物距离(米)
- `/back_obstacle_distance` (std_msgs/Float32) - 后方障碍物距离(米)
- `/left_obstacle_distance` (std_msgs/Float32) - 左侧障碍物距离(米)
- `/right_obstacle_distance` (std_msgs/Float32) - 右侧障碍物距离(米)
- `/obstacle_markers` (visualization_msgs/MarkerArray) - RViz可视化标记

**参数**:
```yaml
warning_distance: 0.5   # 警告距离(米)
danger_distance: 0.3    # 危险距离(米)
update_rate: 10.0       # 更新频率(Hz)
```

**使用示例**:
```bash
# 查看综合信息
ros2 topic echo /obstacle_info

# 输出示例:
# data: "障碍物距离 [米] | 状态: WARNING | 最近: 0.452 | 前: 0.452 | 后: 1.234 | 左: 0.890 | 右: 1.120"

# 查看特定方向
ros2 topic echo /front_obstacle_distance
```

---

### 2️⃣ manual_goal_setter (手动目标点设置)

**功能**: 提供便捷的目标点设置接口，用于调试路径规划

**订阅话题**:
- `/goal_pose` (geometry_msgs/PoseStamped) - RViz 2D Nav Goal

**发布话题**:
- `/goal_setter_status` (std_msgs/String) - 状态信息
- `/waypoint_markers` (visualization_msgs/MarkerArray) - 目标点可视化

**服务**:
- `/set_goal_0` (std_srvs/Trigger) - 前往预设点0
- `/set_goal_1` (std_srvs/Trigger) - 前往预设点1
- `/set_goal_2` (std_srvs/Trigger) - 前往预设点2
- `/set_goal_3` (std_srvs/Trigger) - 前往预设点3
- `/start_patrol` (std_srvs/Trigger) - 开始巡航（循环访问所有点）
- `/cancel_goal` (std_srvs/Trigger) - 取消当前导航

**预设目标点** (在代码中修改 `initializeWaypoints()` 函数):
```cpp
waypoints_.push_back({"原点", 0.0, 0.0, 0.0});
waypoints_.push_back({"前方1米", 1.0, 0.0, 0.0});
waypoints_.push_back({"左前方", 1.0, 1.0, M_PI/4});
waypoints_.push_back({"左侧", 0.0, 1.0, M_PI/2});
```

**使用示例**:
```bash
# 前往预设点1
ros2 service call /set_goal_1 std_srvs/srv/Trigger

# 开始自动巡航
ros2 service call /start_patrol std_srvs/srv/Trigger

# 取消导航
ros2 service call /cancel_goal std_srvs/srv/Trigger

# 在RViz中使用"2D Nav Goal"工具手动设置目标点
```

---

### 3️⃣ omni_wheel_controller (全向轮控制器)

**功能**: 将Nav2的速度命令转换为全向轮控制协议，包含CRC-16校验

**订阅话题**:
- `/cmd_vel` (geometry_msgs/Twist) - 导航速度命令

**发布话题**:
- `/serial_tx_data` (std_msgs/UInt8MultiArray) - 待发送的串口数据
- `/omni_tx_hex` (std_msgs/String) - 十六进制调试信息
- `/omni_processed_vel` (geometry_msgs/Twist) - 处理后的速度
- `/omni_status` (std_msgs/String) - 状态信息

**服务**:
- `/emergency_stop` (std_srvs/Trigger) - 紧急停止

**参数**:
```yaml
max_vx: 1.0              # 最大X速度 (前后) m/s
max_vy: 1.0              # 最大Y速度 (左右) m/s
max_wz: 2.0              # 最大旋转速度 rad/s
velocity_scale: 1000.0   # m/s -> mm/s
angular_scale: 1000.0    # rad/s -> mrad/s
smooth_factor: 0.7       # 速度平滑系数 (0-1)
enable_lateral_motion: true  # 启用侧向运动
```

**全向轮运动学**:
```
输入 (来自Nav2):
  linear.x  → Vx (前后方向)
  linear.y  → Vy (左右方向) - 全向轮特有
  angular.z → Wz (旋转)

输出 (发送给下位机):
  15字节数据包 + CRC-16校验
```

**协议格式** (15字节):
```
+------+------+------+------+------+------+------+------+------+------+------+------+------+------+------+
| 0xAA | 0x55 | 0x10 | Vx_L | Vx_H | Vy_L | Vy_H | Wz_L | Wz_H | Mode | Res1 | Res2 |CRC_L |CRC_H | 0x0D |
+------+------+------+------+------+------+------+------+------+------+------+------+------+------+------+
 帧头1  帧头2  命令ID  X速度(mm/s)  Y速度(mm/s)  角速度(mrad/s) 模式  保留  保留  CRC-16  帧尾
```

**CRC-16校验算法** (MODBUS标准):
```cpp
uint16_t crc = 0xFFFF;
for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
        if (crc & 0x0001) {
            crc = (crc >> 1) ^ 0xA001;
        } else {
            crc = crc >> 1;
        }
    }
}
return crc;
```

**使用示例**:
```bash
# 查看发送的十六进制数据
ros2 topic echo /omni_tx_hex

# 输出示例:
# data: "TX [全向轮]: aa 55 10 e8 03 00 00 f4 01 00 00 00 3c 7f 0d  | Vx=1000mm/s, Vy=0mm/s, Wz=500mrad/s | CRC16=0x7f3c"

# 紧急停止
ros2 service call /emergency_stop std_srvs/srv/Trigger

# 查看状态
ros2 topic echo /omni_status
```

---

## 🚀 快速启动指南

### 1. 编译项目

```bash
cd ~/slam
source /opt/ros/humble/setup.bash  # 或你的ROS2版本
colcon build --packages-select navigation_control --symlink-install
source install/setup.bash
```

### 2. 设置串口权限

```bash
# 查看设备
ls -l /dev/ttyUSB*

# 设置权限（临时）
sudo chmod 666 /dev/ttyUSB0  # 雷达
sudo chmod 666 /dev/ttyUSB1  # 下位机

# 设置权限（永久）
sudo usermod -a -G dialout $USER
# 注销后重新登录
```

### 3. 启动系统

**方式1: 使用完整启动文件 (推荐)**
```bash
ros2 launch navigation_control full_navigation.launch.py
```

**方式2: 分步启动**
```bash
# 终端1: 启动雷达和SLAM
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
ros2 launch my_slam_config cartographer.launch.py

# 终端2: 启动导航功能
ros2 launch navigation_control full_navigation.launch.py
```

**方式3: 仅启动新增节点**
```bash
# 障碍物监控
ros2 run navigation_control obstacle_monitor

# 手动目标点设置
ros2 run navigation_control manual_goal_setter

# 全向轮控制器
ros2 run navigation_control omni_wheel_controller
```

---

## 🎮 使用场景

### 场景1: 障碍物距离监控

```bash
# 启动系统后，实时查看障碍物信息
ros2 topic echo /obstacle_info

# 在RViz中可视化
# 1. 打开RViz
# 2. 添加 MarkerArray 显示类型
# 3. 设置Topic为 /obstacle_markers
# 4. 你将看到在机器人周围显示的距离文字
```

### 场景2: 手动路径规划测试

```bash
# 快速前往预设点
ros2 service call /set_goal_0 std_srvs/srv/Trigger
ros2 service call /set_goal_1 std_srvs/srv/Trigger

# 开始自动巡航（依次访问所有点）
ros2 service call /start_patrol std_srvs/srv/Trigger

# 取消导航
ros2 service call /cancel_goal std_srvs/srv/Trigger
```

### 场景3: 全向轮运动控制

```bash
# 查看实时控制数据
ros2 topic echo /omni_tx_hex

# 手动发送速度命令测试
ros2 topic pub /cmd_vel geometry_msgs/Twist "{
  linear: {x: 0.5, y: 0.2, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.3}
}" --once

# 上面的命令会让机器人:
# - 以0.5 m/s前进
# - 以0.2 m/s向左平移（全向轮特性）
# - 以0.3 rad/s旋转

# 紧急停止
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

---

## 📊 数据流图

```
/scan (LaserScan)
  ↓
[obstacle_monitor]
  ↓
  ├→ /obstacle_info (障碍物距离信息)
  ├→ /min_obstacle_distance
  ├→ /front_obstacle_distance
  ├→ /back_obstacle_distance
  ├→ /left_obstacle_distance
  ├→ /right_obstacle_distance
  └→ /obstacle_markers (RViz可视化)

/cmd_vel (Twist)
  ↓
[omni_wheel_controller]
  ↓
  ├→ 速度限制 & 平滑处理
  ├→ 转换为下位机单位 (mm/s, mrad/s)
  ├→ 生成15字节数据包
  ├→ 计算CRC-16校验
  └→ /serial_tx_data (字节数组)
        ↓
[serial_communication]
        ↓
    串口发送 → 下位机 (全向轮驱动板)
```

---

## 🔧 自定义配置

### 修改预设目标点

编辑文件: `src/navigation_control/src/manual_goal_setter.cpp`

```cpp
void initializeWaypoints() {
    // 根据你的实际地图修改这些坐标
    waypoints_.push_back({"原点", 0.0, 0.0, 0.0});
    waypoints_.push_back({"前方1米", 1.0, 0.0, 0.0});
    waypoints_.push_back({"左前方", 1.0, 1.0, M_PI/4});
    waypoints_.push_back({"左侧", 0.0, 1.0, M_PI/2});
    // 添加更多点...
}
```

### 修改速度限制

编辑launch文件: `launch/full_navigation.launch.py`

```python
Node(
    package='navigation_control',
    executable='omni_wheel_controller',
    parameters=[{
        'max_vx': 1.5,      # 提高到1.5 m/s
        'max_vy': 1.0,
        'max_wz': 3.0,      # 提高到3.0 rad/s
    }],
),
```

### 修改安全距离

```python
Node(
    package='navigation_control',
    executable='obstacle_monitor',
    parameters=[{
        'warning_distance': 0.8,  # 更保守的警告距离
        'danger_distance': 0.5,   # 更保守的危险距离
    }],
),
```

---

## 🐛 调试技巧

### 查看所有话题
```bash
ros2 topic list
```

### 查看节点信息
```bash
ros2 node list
ros2 node info /obstacle_monitor
ros2 node info /omni_wheel_controller
```

### 监控数据流
```bash
# 查看话题频率
ros2 topic hz /scan
ros2 topic hz /cmd_vel
ros2 topic hz /serial_tx_data

# 查看话题内容
ros2 topic echo /obstacle_info
ros2 topic echo /omni_tx_hex
ros2 topic echo /serial_connection_status
```

### 检查串口连接
```bash
# 查看串口状态
ros2 topic echo /serial_connection_status

# 查看接收数据
ros2 topic echo /serial_rx_hex

# 查看发送数据
ros2 topic echo /omni_tx_hex
```

---

## 📝 下位机接口说明

### 下位机需要实现的功能

1. **接收15字节控制命令**:
   ```c
   // 伪代码
   uint8_t rx_buffer[15];
   if (receive_serial(rx_buffer, 15)) {
       // 检查帧头
       if (rx_buffer[0] == 0xAA && rx_buffer[1] == 0x55 && rx_buffer[2] == 0x10) {
           // 验证CRC
           uint16_t received_crc = (rx_buffer[13] << 8) | rx_buffer[12];
           uint16_t calculated_crc = calculate_crc16(rx_buffer, 12);
           
           if (received_crc == calculated_crc) {
               // 提取速度
               int16_t vx = (rx_buffer[4] << 8) | rx_buffer[3];  // mm/s
               int16_t vy = (rx_buffer[6] << 8) | rx_buffer[5];  // mm/s
               int16_t wz = (rx_buffer[8] << 8) | rx_buffer[7];  // mrad/s
               
               // 控制全向轮电机
               control_omni_wheels(vx, vy, wz);
           }
       }
   }
   ```

2. **发送反馈数据** (可选):
   - 编码器数据
   - 电池电压
   - 错误状态

---

## ⚠️ 注意事项

1. **速度单位转换**:
   - 上位机: m/s, rad/s
   - 下位机: mm/s, mrad/s
   - 缩放因子: 1000

2. **坐标系约定**:
   - X轴: 前进方向
   - Y轴: 左侧方向
   - Z轴(旋转): 逆时针为正

3. **CRC校验**:
   - 使用MODBUS标准CRC-16
   - 初始值: 0xFFFF
   - 多项式: 0xA001

4. **超时保护**:
   - 1秒无速度命令 → 逐渐减速停止
   - 建议下位机也实现超时停止

---

## 📚 参考资料

- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [Nav2文档](https://navigation.ros.org/)
- [Cartographer文档](https://google-cartographer-ros.readthedocs.io/)
- [MODBUS CRC-16标准](https://www.modbustools.com/modbus.html)

---

## 🎯 下一步改进建议

1. **里程计融合** - 将编码器数据反馈给导航系统
2. **动态避障** - 结合障碍物距离信息优化路径规划
3. **地图保存** - 自动保存和加载地图
4. **Web界面** - 通过浏览器监控和控制机器人
5. **任务调度** - 实现复杂的任务序列执行

---

**作者**: ROS2 SLAM导航系统  
**版本**: 2.0 (全向轮增强版)  
**日期**: 2025-11-05
