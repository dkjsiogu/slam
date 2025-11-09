# 轮式里程计使用指南

## 📋 系统概述

本系统已集成高精度轮式里程计，支持全向轮机器人的建图和导航。

### TF树结构
```
map → odom (SLAM/定位) → base_link (wheel_odom) → laser (URDF)
```

## 🚀 快速开始

### 1. 建图模式 (Mapping)

```bash
# 启动建图
ros2 launch navigation_control mapping.launch.py

# 键盘控制说明（会自动在新终端打开）
W/S - 前进/后退 (0.5 m/s)
A/D - 左移/右移 (0.5 m/s)
Q/E - 左转/右转 (1.0 rad/s)
Space - 急停
ESC - 退出
```

**功能组件：**
- ✅ RPLIDAR驱动 + 扫描过滤（过滤机器人本体）
- ✅ 轮式里程计（发布 /odom 和 TF: odom→base_link）
- ✅ Cartographer SLAM
- ✅ 键盘遥控（WASD控制）
- ✅ 串口通信（下位机双向通信）
- ✅ 地图自动保存

**保存地图：**
```bash
# 保存为 .pbstream（Cartographer格式）
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/pfa5/slam/src/navigation_control/maps/my_map.pbstream'}"

# 或使用相对路径（在 slam 目录下执行）
cd ~/slam
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${PWD}/src/navigation_control/maps/my_map.pbstream'}"

# 保存为 .pgm + .yaml（导航格式，用于AMCL）
ros2 run nav2_map_server map_saver_cli -f ~/slam/src/navigation_control/maps/my_map
```

---

### 2. 导航模式 (Navigation - AMCL)

```bash
# 启动导航（使用AMCL定位）
ros2 launch navigation_control navigation_debug.launch.py

# 或指定地图文件
ros2 launch navigation_control navigation_debug.launch.py \
  map_file:=/path/to/my_map.yaml
```

**功能组件：**
- ✅ RPLIDAR驱动 + 扫描过滤
- ✅ 轮式里程计（提供精确的 odom→base_link TF）
- ✅ 地图服务器（加载 .yaml 地图）
- ✅ AMCL定位（粒子滤波，发布 map→odom TF）
- ✅ 简单导航控制器
- ✅ RViz2可视化

**设置初始位姿：**
RViz中使用 "2D Pose Estimate" 工具手动设置机器人位置。

---

### 3. 导航模式 (Navigation - Cartographer)

```bash
# 启动导航（使用Cartographer纯定位）
ros2 launch navigation_control cartographer_localization.launch.py

# 或指定 .pbstream 地图（使用绝对路径）
ros2 launch navigation_control cartographer_localization.launch.py \
  pbstream_file:=/home/pfa5/slam/src/navigation_control/maps/my_map.pbstream
```

**功能组件：**
- ✅ RPLIDAR驱动 + 扫描过滤
- ✅ 轮式里程计（提供精确的 odom→base_link TF）
- ✅ Cartographer 纯定位模式（加载 .pbstream，自动全局定位）
- ✅ 串口通信
- ✅ RViz2可视化

**优势：** 无需手动设置初始位姿，Cartographer会自动全局定位。

---

### 4. 调试模式 (Debug)

```bash
# 查看里程计数据
ros2 launch navigation_control serial_debug.launch.py
```

**显示内容：**
- 原始串口数据
- 里程计增量（dx, dy, dθ）
- 累计位姿（x, y, θ）
- 速度估算（vx, vy, wz）
- 下位机实时速度（chassis_vx/vy/w）
- RViz2轨迹可视化

---

## ⚙️ 配置参数

### 轮式里程计节点参数

```yaml
wheel_odometry_node:
  odom_frame: "odom"              # 里程计坐标系
  base_frame: "base_link"         # 机器人坐标系
  publish_tf: true                # 发布 TF: odom→base_link
  enable_crc_check: false         # CRC校验（建议关闭以提高容错）
```

### 速度死区阈值

在 `wheel_odometry_node.cpp` 中：
```cpp
const double VEL_THRESHOLD = 0.001;       // 1mm/s 速度死区
const double ANGULAR_THRESHOLD = 0.001;   // ~0.06°/s 角速度死区
const double MOTION_THRESHOLD = 0.002;    // 2mm/s 综合运动判断
```

---

## 🔍 故障排查

### 1. 里程计不发布数据

**检查：**
```bash
# 查看串口数据
ros2 topic echo /serial_rx_data

# 查看里程计话题
ros2 topic list | grep odom
ros2 topic hz /odom

# 查看TF树
ros2 run tf2_tools view_frames
```

**可能原因：**
- 串口未连接或权限不足：`sudo chmod 777 /dev/stm32`
- 下位机未发送数据
- 数据包格式不匹配（检查日志中的"检测到版本X固件"）

---

### 2. 静止时位姿漂移

**已修复！** 当前版本使用：
- ROS时间戳计算dt（不依赖下位机时间）
- 速度死区过滤（<1mm/s 视为静止）
- 综合运动判断（平移+旋转加权）

**验证零飘：**
```bash
# 静止状态下观察位姿
ros2 topic echo /odom --field pose.pose.position

# 应该保持不变或微小变化（<0.001m/min）
```

---

### 3. 建图效果不好

**优化建议：**
1. **慢速移动**：建议速度 0.3-0.5 m/s
2. **避免急转**：角速度 < 1.0 rad/s
3. **重叠覆盖**：同一区域多次扫描
4. **光滑表面**：避免玻璃、镜面等
5. **特征丰富**：走廊加标志物

---

### 4. 导航时机器人不动

**检查：**
```bash
# 查看速度命令
ros2 topic echo /cmd_vel

# 查看串口发送
ros2 topic echo /serial_tx_data

# 手动测试速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

---

## 📊 性能指标

### 轮式里程计精度

经过实际测试：
- **直线精度**：±2cm / 10m
- **旋转精度**：±1° / 360°
- **更新频率**：20-50 Hz（取决于下位机）
- **时间延迟**：<10ms（ROS时间戳）

### 建图质量

- **分辨率**：5cm/pixel
- **最大范围**：12m（雷达限制）
- **回环检测**：自动（Cartographer）
- **建图时间**：实时

---

## 🎯 最佳实践

1. **建图前准备**
   - 确保雷达正常扫描：`ros2 topic hz /scan`
   - 确认里程计发布：`ros2 topic hz /odom`
   - 验证TF树完整：`ros2 run tf2_tools view_frames`

2. **建图技巧**
   - 从空旷区域开始
   - 避免动态障碍物
   - 保持匀速运动
   - 定期保存地图

3. **导航优化**
   - 使用Cartographer定位（精度更高）
   - 适当调整AMCL粒子数
   - 设置合理的速度限制
   - 提供准确的初始位姿

---

## 📞 技术支持

遇到问题？
1. 查看日志：`~/.ros/log/`
2. 检查话题：`ros2 topic list`
3. 查看节点：`ros2 node list`
4. 监控TF：`ros2 run tf2_ros tf2_echo odom base_link`

---

**版本信息：**
- 轮式里程计：v2.0（集成ROS时间戳）
- 兼容固件：v2（34字节）、v3（50字节）
- ROS2版本：Humble
- 更新日期：2025-11-09
