# Cartographer 纯定位模式使用指南

## 📋 概述

本指南介绍如何使用 Cartographer 的纯定位模式进行比赛。与 AMCL 相比，Cartographer 纯定位具有以下优势：

- ✅ **更高精度**: 基于扫描匹配，定位更准确
- ✅ **更好的鲁棒性**: 对动态障碍物更宽容
- ✅ **无需初始位姿**: 自动在地图中定位
- ✅ **实时地图更新**: 可以微调地图（可选）

## 🔄 完整工作流程

### 步骤1: 建图阶段（比赛前）

使用 `mapping.launch.py` 建图：

```bash
# 启动建图模式
ros2 launch navigation_control mapping.launch.py
```

在 RViz2 中查看建图效果，确保地图质量良好。

### 步骤2: 保存地图

建图完成后，**不要关闭** mapping.launch.py，运行保存脚本：

```bash
# 方法1: 使用脚本（推荐）
cd ~/slam/src/navigation_control/scripts
./save_cartographer_map.sh

# 方法2: 指定地图名称
./save_cartographer_map.sh --name competition_map

# 方法3: 手动保存
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$HOME/slam/src/navigation_control/maps/my_map.pbstream'}"
ros2 run nav2_map_server map_saver_cli -f ~/slam/src/navigation_control/maps/my_map
```

### 步骤3: 验证保存的文件

```bash
cd ~/slam/src/navigation_control/maps
ls -lh

# 应该看到:
# my_map.pbstream  - Cartographer 地图文件（二进制）
# my_map.pgm       - 占用栅格地图图像
# my_map.yaml      - 地图元数据
```

### 步骤4: 比赛时使用纯定位模式

```bash
# 启动纯定位模式
ros2 launch navigation_control cartographer_localization.launch.py

# 或指定不同的地图文件
ros2 launch navigation_control cartographer_localization.launch.py \
  pbstream_file:=$HOME/slam/src/navigation_control/maps/competition_map.pbstream
```

## 🎯 纯定位模式工作原理

### 数据流程

```
激光雷达 → scan_filter → Cartographer → map→base_link TF
                              ↑
轮式里程计 → /odom topic + odom→base_link TF
```

### 坐标系关系

```
map (固定)
 └─ odom (Cartographer发布的累计误差修正)
     └─ base_link (轮式里程计发布)
         └─ laser (静态TF)
```

## ⚙️ 配置文件说明

### `cartographer_localization.lua` 关键参数

```lua
-- 纯定位模式配置
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 999999  -- 不建新图

-- 定位精度参数
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- 里程计融合
options.use_odometry = true  -- 使用轮式里程计
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
```

### Launch 文件参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pbstream_file` | `maps/my_map.pbstream` | Cartographer 地图文件路径 |
| `lidar_port` | `/dev/radar` | 激光雷达串口 |
| `dev_board_port` | `/dev/stm32` | STM32 下位机串口 |

## 🔍 调试和监控

### 查看定位状态

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames
evince frames.pdf

# 监控位姿
ros2 topic echo /tracked_pose

# 检查扫描匹配得分
ros2 topic echo /scan_matched_points2
```

### 常见问题排查

#### 1. 定位失败或漂移

**症状**: 机器人在地图中位置不对

**解决方案**:
```bash
# 检查是否加载了正确的地图
ros2 param get /cartographer_node load_state_filename

# 检查激光数据是否正常
ros2 topic hz /scan
ros2 topic echo /scan --once

# 检查里程计数据
ros2 topic hz /odom
ros2 topic echo /odom --once
```

#### 2. 地图不显示

**症状**: RViz2 中看不到地图

**解决方案**:
```bash
# 检查地图发布
ros2 topic list | grep map
ros2 topic echo /map --once

# 重启占用栅格节点
ros2 lifecycle set /occupancy_grid_node configure
ros2 lifecycle set /occupancy_grid_node activate
```

#### 3. 机器人不动

**症状**: 发送速度命令但机器人不移动

**解决方案**:
```bash
# 检查串口连接
ros2 topic echo /serial_connection_status

# 手动发送测试命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 查看串口发送数据
ros2 topic echo /serial_tx_hex
```

## 📊 性能对比

| 特性 | AMCL | Cartographer 纯定位 |
|------|------|-------------------|
| 定位精度 | ±10cm | ±2-5cm |
| 初始化 | 需要手动设置初始位姿 | 自动定位 |
| 动态障碍物 | 敏感 | 更鲁棒 |
| CPU 占用 | 低 (~5%) | 中等 (~15-20%) |
| 内存占用 | 低 (~50MB) | 高 (~200-500MB) |
| 适用场景 | 静态环境 | 半动态环境 |

## 🚀 比赛优化建议

### 1. 提前建好高质量地图

```bash
# 建图时慢速移动，多角度覆盖
# 建议速度: 0.2-0.3 m/s
# 建议采集时间: 至少 3-5 分钟
```

### 2. 备份多个地图

```bash
# 不同区域可能需要不同地图
./save_cartographer_map.sh --name area1_map
./save_cartographer_map.sh --name area2_map
./save_cartographer_map.sh --name full_map
```

### 3. 调整定位参数

如果定位不稳定，可以调整 `cartographer_localization.lua`:

```lua
-- 提高扫描匹配权重（更依赖激光）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 30.0

-- 降低里程计权重（如果里程计不准）
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4

-- 增加搜索窗口（初始定位困难时）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
```

## 📝 完整启动流程（比赛时）

```bash
# 1. 检查设备连接
ls -l /dev/radar /dev/stm32

# 2. 设置串口权限（如需要）
sudo chmod 666 /dev/ttyUSB0 /dev/ttyACM0

# 3. 启动纯定位模式
ros2 launch navigation_control cartographer_localization.launch.py

# 4. 等待 5-10 秒让 Cartographer 初始化

# 5. 在 RViz2 中设置导航目标点
# 使用 "2D Goal Pose" 工具

# 6. 监控机器人状态
ros2 topic hz /tracked_pose  # 定位频率
ros2 topic echo /cmd_vel     # 速度命令
```

## 🔧 高级功能

### 启用实时地图更新（可选）

如果比赛环境与建图时有变化，可以允许微调地图：

```lua
-- 在 cartographer_localization.lua 中修改
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- 允许小范围更新
```

**警告**: 这会增加 CPU 占用，谨慎使用！

### 保存定位轨迹

```bash
# 记录机器人运动轨迹（用于分析）
ros2 bag record /tracked_pose /odom /scan -o competition_run
```

## ✅ 检查清单（比赛前）

- [ ] 已建立高质量地图
- [ ] 已保存 .pbstream 文件
- [ ] 已验证纯定位模式可以正常启动
- [ ] 已测试机器人可以在地图中准确定位
- [ ] 已测试导航到目标点功能
- [ ] 已备份所有地图文件到U盘
- [ ] 已记录所有配置参数

## 📞 技术支持

如遇问题，请检查日志：

```bash
# Cartographer 日志
ros2 node info /cartographer_node

# 系统日志
journalctl -u ros2.service -f

# 完整日志记录
ros2 launch navigation_control cartographer_localization.launch.py 2>&1 | tee ~/cartographer.log
```

---

**祝比赛顺利！🏆**
