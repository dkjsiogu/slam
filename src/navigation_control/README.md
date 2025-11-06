# Navigation Control Package

完整的ROS2 SLAM导航系统，支持Cartographer建图与纯定位、全向轮控制、轮式里程计融合。

## 🎯 主要功能

- ✅ **Cartographer SLAM**: 高精度2D激光建图
- ✅ **纯定位模式**: 使用已有地图进行全局定位（支持自动重定位）
- ✅ **全向轮控制**: 三轮全向轮底盘控制
- ✅ **轮式里程计**: 编码器数据融合定位
- ✅ **激光过滤**: 过滤机器人本体干扰
- ✅ **串口通信**: STM32下位机数据交互

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                  RPLIDAR A1 激光雷达                     │
└────────────────────┬────────────────────────────────────┘
                     │ /scan_raw
                     ▼
┌─────────────────────────────────────────────────────────┐
│             Scan Filter (过滤机器人本体)                 │
└────────────────────┬────────────────────────────────────┘
                     │ /scan
        ┌────────────┴────────────┐
        ▼                         ▼
┌──────────────────┐      ┌──────────────────┐
│  Cartographer    │      │  轮式里程计节点   │
│  (建图/纯定位)    │      │  /odom + TF      │
└────────┬─────────┘      └─────────┬────────┘
         │                          │
         │ map→odom TF             │ odom→base_link TF
         ▼                          ▼
┌─────────────────────────────────────────────────────────┐
│                   TF Tree 完整链                         │
│        map → odom → base_link → laser                   │
└─────────────────────────────────────────────────────────┘
                     │
                     ▼
         ┌───────────────────────┐
         │   Navigation / 手动控制 │
         └───────────┬───────────┘
                     │ /cmd_vel
                     ▼
┌─────────────────────────────────────────────────────────┐
│       Serial Data Publisher (速度命令打包)               │
└────────────────────┬────────────────────────────────────┘
                     │ serial_tx_data
                     ▼
┌─────────────────────────────────────────────────────────┐
│       Serial Communication (UART通信)                    │
│              ↓                        ↑                 │
│       STM32下位机发送增量      STM32接收速度命令          │
└─────────────────────────────────────────────────────────┘
```

## 核心功能模块

### 1. Cartographer SLAM
- **建图模式** (`mapping.launch.py`): 实时建图
- **纯定位模式** (`cartographer_localization.launch.py`): 使用已有地图定位
- **全局定位**: 自动重定位，无需手动设置初始位姿
- **高精度**: ±2-5cm 定位精度

### 2. 轮式里程计系统
- **数据接收**: 从STM32接收编码器增量数据
- **坐标变换**: 机器人坐标系 → 世界坐标系
- **TF发布**: 发布 `odom → base_link` 动态变换
- **里程计融合**: 与Cartographer融合提高定位精度

### 3. 激光扫描过滤
- **功能**: 过滤机器人本体反射的激光点
- **范围**: -138.87° 到 163.00°, 0.35m以内
- **效果**: 消除底盘边缘和轮子的干扰点

### 4. 串口通信系统
- **下位机 → 上位机**: 里程计增量数据 (50Hz)
- **上位机 → 下位机**: 全向轮速度命令 (20Hz)
- **协议**: 自定义二进制协议 + CRC16校验
- **自动重连**: 连接断开自动恢复

---

## 🚀 快速开始

### 比赛推荐流程（使用 Cartographer 纯定位）

#### 步骤1: 赛前建图

```bash
cd ~/slam
source install/setup.bash

# 启动建图模式
ros2 launch navigation_control mapping.launch.py

# 控制机器人遍历比赛场地（慢速，3-5分钟）
# 使用手柄或键盘控制

# 建图完成后保存（新终端）
cd ~/slam/src/navigation_control/scripts
./save_cartographer_map.sh --name competition_map
```

#### 步骤2: 比赛时启动纯定位

```bash
cd ~/slam
source install/setup.bash

# 启动纯定位模式（机器人可放任意位置！）
ros2 launch navigation_control cartographer_localization.launch.py \
  pbstream_file:=$HOME/slam/src/navigation_control/maps/competition_map.pbstream

# 等待10-20秒自动全局定位
# 或在RViz2中用"2D Pose Estimate"手动提示位置
# 设置导航目标，开始比赛！
```

---

## 📁 Launch 文件说明

| Launch 文件 | 用途 | 定位方式 | 适用场景 |
|------------|------|---------|---------|
| `mapping.launch.py` | 建图 | Cartographer SLAM | 赛前建图 |
| `cartographer_localization.launch.py` | **纯定位** | Cartographer | **🏆 比赛推荐** |
| `navigation.launch.py` | 完整导航 | AMCL | 传统方案 |
| `navigation_debug.launch.py` | 调试 | AMCL | 快速测试 |

详见: [Launch文件选择指南](docs/LAUNCH_FILES_GUIDE.md)

---

## 📚 文档索引

### 使用指南
- **[Launch文件选择指南](docs/LAUNCH_FILES_GUIDE.md)** - 选择合适的启动文件 ⭐
- **[Cartographer纯定位指南](docs/CARTOGRAPHER_LOCALIZATION_GUIDE.md)** - 纯定位模式使用
- **[全局定位指南](docs/GLOBAL_LOCALIZATION_GUIDE.md)** - 自动重定位功能

### 技术文档
- **[里程计集成方案](docs/ODOMETRY_INTEGRATION.md)** - 轮式里程计数据流
- **[STM32参考代码](docs/STM32_ODOMETRY_REFERENCE.md)** - 下位机实现参考
- **[串口通信协议](docs/SERIAL_PROTOCOL.md)** - 通信协议规范
- **[串口通信文档](docs/SERIAL_COMMUNICATION.md)** - 串口节点说明

---

## 🔧 依赖项

### ROS2 包
```bash
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-nav2-*
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
```

### 系统库
```bash
sudo apt install libserial-dev
```

---

## 🛠️ 编译

```bash
cd ~/slam
colcon build --symlink-install
source install/setup.bash
```

---

## 📊 性能指标

| 指标 | Cartographer纯定位 | AMCL |
|------|-------------------|------|
| 定位精度 | ±2-5cm | ±10cm |
| 初始化方式 | 自动全局定位 | 需手动设置 |
| CPU占用 | 15-20% | 5-10% |
| 内存占用 | 200-500MB | 100-200MB |
| 适合比赛 | ✅✅✅ | ✅ |

---

## ⚠️ 常见问题

### 1. 串口权限问题
```bash
sudo chmod 666 /dev/ttyUSB0 /dev/ttyACM0
# 或创建udev规则
```

### 2. Cartographer定位失败
- 确保周围有足够障碍物
- 等待10-20秒让全局定位收敛
- 使用RViz2的"2D Pose Estimate"手动提示

### 3. 激光雷达无数据
```bash
# 检查设备
ls -l /dev/radar /dev/ttyUSB0

# 检查话题
ros2 topic hz /scan
ros2 topic echo /scan --once
```

### 4. 轮式里程计无数据
```bash
# 检查串口连接
ros2 topic echo /serial_connection_status

# 检查里程计话题
ros2 topic hz /odom
ros2 topic echo /odom --once
```

---

## 📞 技术支持

查看详细文档: `docs/` 目录

---

