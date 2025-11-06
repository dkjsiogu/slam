# Launch 文件使用指南

## 📁 可用的 Launch 文件

| Launch 文件 | 用途 | 定位方式 | 适用场景 |
|------------|------|---------|---------|
| `mapping.launch.py` | **建图模式** | Cartographer SLAM | 比赛前建图 |
| `cartographer_localization.launch.py` | **纯定位模式** | Cartographer | 🏆 **比赛推荐** |
| `navigation.launch.py` | 完整导航 | AMCL | 传统导航方案 |
| `navigation_debug.launch.py` | 调试导航 | AMCL | 简单测试 |

---

## 🎯 比赛推荐流程

### 赛前准备：使用 `mapping.launch.py`

```bash
# 步骤1: 建图（比赛场地开放时）
ros2 launch navigation_control mapping.launch.py

# 步骤2: 保存地图
cd ~/slam/src/navigation_control/scripts
./save_cartographer_map.sh --name competition_map
```

**建图要点**：
- ✅ 慢速移动（0.2-0.3 m/s）
- ✅ 覆盖所有比赛区域
- ✅ 多角度扫描障碍物
- ✅ 建图时间 3-5 分钟
- ✅ 保存多个备份地图

---

### 比赛时：使用 `cartographer_localization.launch.py`

```bash
# 启动纯定位模式
ros2 launch navigation_control cartographer_localization.launch.py \
  pbstream_file:=$HOME/slam/src/navigation_control/maps/competition_map.pbstream
```

**为什么选择 Cartographer 纯定位？**
- ✅ **定位精度更高** (±2-5cm vs AMCL ±10cm)
- ✅ **无需手动设置初始位姿** (AMCL需要)
- ✅ **对动态障碍物更鲁棒**
- ✅ **持续优化轨迹** (后端优化)
- ✅ **符合SLAM比赛要求** (使用SLAM技术)

---

## 📋 详细对比

### 1. `mapping.launch.py` - 建图模式

**功能**：
- ✅ RPLIDAR 激光扫描
- ✅ Cartographer SLAM 建图
- ✅ 激光过滤器（去除机器人本体）
- ✅ 轮式里程计（如果有）
- ✅ 串口通信（控制机器人）
- ✅ 实时地图显示

**启动命令**：
```bash
ros2 launch navigation_control mapping.launch.py

# 可选参数
ros2 launch navigation_control mapping.launch.py \
  lidar_port:=/dev/ttyUSB0 \
  dev_board_port:=/dev/ttyACM0
```

**何时使用**：
- 第一次到达比赛场地
- 场地环境有变化时
- 需要更新地图时

---

### 2. `cartographer_localization.launch.py` - 纯定位模式 ⭐

**功能**：
- ✅ 加载已保存的 .pbstream 地图
- ✅ Cartographer 纯定位（不建新图）
- ✅ 激光扫描 + 里程计融合
- ✅ 高精度定位
- ✅ 自动在地图中初始化

**启动命令**：
```bash
# 使用默认地图
ros2 launch navigation_control cartographer_localization.launch.py

# 指定地图文件
ros2 launch navigation_control cartographer_localization.launch.py \
  pbstream_file:=$HOME/maps/my_map.pbstream

# 完整参数
ros2 launch navigation_control cartographer_localization.launch.py \
  pbstream_file:=$HOME/maps/competition_map.pbstream \
  lidar_port:=/dev/radar \
  dev_board_port:=/dev/stm32
```

**何时使用**：
- ✅ **比赛正式运行时** ← 推荐！
- ✅ 已有高质量地图
- ✅ 需要高精度定位
- ✅ 需要自动初始化

**优点**：
- 定位精度高（基于扫描匹配）
- 无需手动设置初始位姿
- 对动态障碍物鲁棒
- 符合 SLAM 比赛要求

**缺点**：
- CPU占用稍高（~15-20%）
- 内存占用较大（~200-500MB）
- 需要提前建好地图

---

### 3. `navigation.launch.py` - 完整导航

**功能**：
- ✅ AMCL 定位
- ✅ Nav2 完整导航栈
- ✅ 全局路径规划
- ✅ 局部路径规划
- ✅ 代价地图
- ✅ 恢复行为

**启动命令**：
```bash
ros2 launch navigation_control navigation.launch.py \
  map_file:=$HOME/slam/src/navigation_control/maps/my_map.yaml
```

**何时使用**：
- 需要复杂导航功能
- 需要动态避障
- 需要路径重规划
- CPU性能有限（AMCL占用低）

**缺点**：
- 需要手动设置初始位姿
- 定位精度不如 Cartographer
- 对激光退化敏感

---

### 4. `navigation_debug.launch.py` - 调试导航

**功能**：
- ✅ AMCL 定位
- ✅ 简单目标点控制
- ✅ 轻量级（用于测试）

**启动命令**：
```bash
ros2 launch navigation_control navigation_debug.launch.py \
  map_file:=$HOME/slam/src/navigation_control/maps/my_map.yaml \
  initial_x:=0.0 \
  initial_y:=0.0
```

**何时使用**：
- 快速测试硬件
- 调试串口通信
- 验证激光雷达
- 不需要完整导航功能

---

## 🏆 比赛推荐配置

### 方案A：仅使用 Cartographer（推荐）

```bash
# 赛前
ros2 launch navigation_control mapping.launch.py
./save_cartographer_map.sh --name final_map

# 比赛时
ros2 launch navigation_control cartographer_localization.launch.py \
  pbstream_file:=$HOME/slam/src/navigation_control/maps/final_map.pbstream
```

**优点**：
- ✅ 完全使用 SLAM 技术
- ✅ 定位精度最高
- ✅ 配置简单

---

### 方案B：Cartographer + Nav2（高级）

如果需要复杂导航功能，可以组合使用：

```bash
# 1. 用 Cartographer 纯定位提供高精度位姿
# 2. 用 Nav2 进行路径规划和避障

# TODO: 创建组合 launch 文件
```

---

## 🔧 快速切换

### 创建启动脚本

```bash
# ~/start_mapping.sh - 建图
#!/bin/bash
ros2 launch navigation_control mapping.launch.py

# ~/start_competition.sh - 比赛
#!/bin/bash
ros2 launch navigation_control cartographer_localization.launch.py \
  pbstream_file:=$HOME/slam/src/navigation_control/maps/competition_map.pbstream

# 赋予执行权限
chmod +x ~/start_*.sh
```

---

## 📊 性能对比

| 指标 | mapping.launch.py | cartographer_localization.launch.py | navigation.launch.py |
|------|-------------------|-------------------------------------|---------------------|
| CPU占用 | 20-30% | 15-20% | 10-15% |
| 内存占用 | 300-600MB | 200-500MB | 100-200MB |
| 定位精度 | N/A (建图) | ±2-5cm | ±10cm |
| 启动时间 | 5s | 10s | 8s |
| 是否需要初始位姿 | N/A | ❌ 否 | ✅ 是 |
| 适合比赛 | 赛前准备 | ✅✅✅ | ✅ |

---

## ✅ 检查清单

**赛前准备**：
- [ ] 已用 `mapping.launch.py` 建立地图
- [ ] 已保存 .pbstream 文件
- [ ] 已验证地图质量良好
- [ ] 已测试 `cartographer_localization.launch.py`
- [ ] 已备份地图到多个位置

**比赛当天**：
- [ ] 检查设备连接（激光雷达、STM32）
- [ ] 启动 `cartographer_localization.launch.py`
- [ ] 等待 5-10 秒初始化
- [ ] 验证 RViz2 中位姿正确
- [ ] 设置导航目标点
- [ ] 开始比赛！

---

## 🆘 故障排除

### 找不到地图文件

```bash
# 检查地图是否存在
ls -lh ~/slam/src/navigation_control/maps/*.pbstream

# 如果没有，重新建图
ros2 launch navigation_control mapping.launch.py
# 建图后保存
./save_cartographer_map.sh
```

### 定位失败

```bash
# 检查激光数据
ros2 topic hz /scan
ros2 topic echo /scan --once

# 检查里程计
ros2 topic hz /odom

# 重启定位
# Ctrl+C 停止
ros2 launch navigation_control cartographer_localization.launch.py
```

### RViz2 崩溃

```bash
# 单独启动 RViz2
rviz2 -d ~/slam/src/navigation_control/config/navigation_debug.rviz

# 或使用默认配置
rviz2
```

---

**祝比赛成功！🏆🚀**
