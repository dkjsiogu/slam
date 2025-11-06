# Cartographer 全局定位使用指南

## 🎯 自动全局定位（推荐）

### 工作原理

Cartographer 的纯定位模式已配置**自动全局定位**功能：

1. **启动时**: 机器人可以放在地图**任意位置**
2. **自动搜索**: Cartographer 在整个地图中搜索匹配
3. **扫描匹配**: 对比当前激光点云与地图中的障碍物
4. **自动定位**: 找到最匹配的位置并锁定

### 配置参数说明

```lua
-- 已在 cartographer_localization.lua 中配置

-- 全局搜索窗口
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0  
-- 搜索范围 ±7米

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
-- 角度搜索 ±30度

-- 全局定位最小匹配分数
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
-- 降低到60%以便初始匹配
```

---

## 🚀 使用方法

### 方法1: 完全自动定位（最简单）

```bash
# 1. 启动纯定位模式
ros2 launch navigation_control cartographer_localization.launch.py

# 2. 等待 10-20 秒
# Cartographer 会自动在地图中搜索当前位置

# 3. 观察 RViz2
# - 看到激光点云与地图逐渐对齐 = 定位成功
# - 机器人模型跳到正确位置 = 定位完成

# 4. 开始使用
# 定位成功后可以设置导航目标点
```

**注意事项**:
- ✅ 机器人可以放在地图**任意位置**
- ✅ 确保周围有**足够的障碍物**（墙、柱子等）
- ✅ 避免在**空旷区域**启动（激光无特征）
- ✅ 等待**10-20秒**让算法收敛

---

### 方法2: 手动辅助定位（推荐在难以定位时使用）

如果自动定位失败（等待30秒后仍不准确），可以手动提供初始位姿提示：

```bash
# 1. 启动后，在 RViz2 中：
#    - 点击顶部工具栏的 "2D Pose Estimate"
#    - 在地图上点击机器人**大概位置**
#    - 拖动鼠标指示机器人朝向
#    - 松开鼠标

# 2. Cartographer 收到提示后会：
#    - 以该位置为中心进行局部搜索
#    - 自动微调到准确位置
#    - 通常 3-5 秒完成

# 3. 观察激光点云对齐情况
#    - 对齐 = 定位成功
#    - 不对齐 = 重新设置一次
```

---

## 📊 定位质量判断

### ✅ 定位成功的标志

1. **视觉检查**:
   - RViz2 中激光点云与地图完美重合
   - 机器人模型位置准确
   - 移动机器人时，点云随之移动且持续对齐

2. **日志检查**:
```bash
# 查看 Cartographer 日志
ros2 topic echo /constraint_list

# 看到约束分数高于 0.7 = 定位可靠
```

3. **测试移动**:
```bash
# 发送小幅移动命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# 观察机器人在地图中移动是否平滑
```

### ❌ 定位失败的标志

1. 激光点云与地图错位
2. 机器人模型位置跳动
3. 移动时定位发散（越走越偏）

**解决方案**:
- 使用方法2手动提供初始位姿
- 移动到有更多障碍物的区域
- 检查激光雷达是否正常工作

---

## 🎮 RViz2 配置

确保 RViz2 显示以下内容（已在 navigation_debug.rviz 中配置）:

```
显示项:
✓ TF (查看坐标系关系)
✓ Map (地图)
✓ LaserScan (激光点云)
✓ RobotModel (机器人模型)
✓ Pose (机器人位姿)

工具栏:
✓ 2D Pose Estimate (手动设置初始位姿)
✓ 2D Goal Pose (设置导航目标)
```

---

## 🔧 高级调试

### 监控全局定位过程

```bash
# 1. 查看约束构建
ros2 topic echo /constraint_list

# 2. 查看子地图列表
ros2 topic echo /submap_list

# 3. 查看当前位姿
ros2 topic echo /tracked_pose

# 4. 查看定位状态
ros2 service call /get_trajectory_states cartographer_ros_msgs/srv/GetTrajectoryStates
```

### 调整全局定位参数

如果定位总是失败，可以编辑 `cartographer_localization.lua`:

```lua
-- 增加搜索范围（在更大区域搜索）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.0  -- 改为10米

-- 降低匹配阈值（更容易接受匹配）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.50  -- 改为50%

-- 增加采样率（检查更多子地图）
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5  -- 改为50%

-- 增加搜索深度（更彻底搜索）
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 8  -- 改为8
```

**注意**: 增加这些参数会提高CPU占用和初始化时间！

---

## 📋 对比：自动定位 vs 手动定位

| 特性 | 自动全局定位 | 手动初始位姿 |
|------|-------------|-------------|
| **便利性** | ✅ 完全自动 | ⚠️ 需要人工操作 |
| **准确性** | ✅ 高（算法优化） | ⚠️ 依赖人工估计 |
| **速度** | ⚠️ 10-20秒 | ✅ 3-5秒 |
| **可靠性** | ⚠️ 需要特征丰富 | ✅ 更稳定 |
| **适用场景** | 障碍物多的环境 | 任何环境 |

**推荐策略**:
1. 优先使用自动定位（启动后等待10-20秒）
2. 如果30秒后仍未定位，使用手动辅助
3. 比赛时可以预先手动设置初始位姿加快启动

---

## 🏆 比赛最佳实践

### 赛前准备

```bash
# 1. 在比赛场地建立高质量地图
ros2 launch navigation_control mapping.launch.py

# 2. 保存地图
./save_cartographer_map.sh --name competition_final

# 3. 测试自动定位
# 将机器人放在不同位置，每次重新启动测试
ros2 launch navigation_control cartographer_localization.launch.py

# 4. 记录定位时间
# - 障碍物多的区域: ~5-10秒
# - 空旷区域: ~15-30秒
# - 长走廊: 可能需要手动辅助
```

### 比赛时启动流程

```bash
# 方案A: 完全自动（推荐）
ros2 launch navigation_control cartographer_localization.launch.py
# 等待 10-20 秒
# 检查 RViz2 定位是否成功
# 开始导航

# 方案B: 快速启动（适合时间紧张）
ros2 launch navigation_control cartographer_localization.launch.py
# 立即在 RViz2 中用 "2D Pose Estimate" 设置大概位置
# 等待 3-5 秒微调
# 开始导航
```

---

## ❓ 常见问题

### Q1: 为什么定位后机器人位置还在漂移？

**A**: 这是正常的优化过程。Cartographer 会持续优化位姿，几秒后会稳定。

### Q2: 可以在完全空旷的区域定位吗？

**A**: 不能。自动定位需要激光能看到障碍物。建议移动到有墙或柱子的地方再启动。

### Q3: 定位成功后可以关闭 RViz2 吗？

**A**: 可以。定位由 Cartographer 维持，RViz2 只是可视化。

### Q4: 如何重置定位？

**A**:
```bash
# 方法1: 重启 Cartographer
Ctrl+C 停止 launch 文件
重新运行 launch 文件

# 方法2: 使用 RViz2 重新设置
在 RViz2 中用 "2D Pose Estimate" 重新设置位姿
```

### Q5: 定位精度有多高？

**A**: 
- 位置精度: ±2-5cm
- 角度精度: ±2-3度
- 比 AMCL (±10cm) 精度高 2-5 倍

---

**总结**: Cartographer 的全局定位功能已配置完成，机器人启动时可以放在地图任意位置，系统会自动找到正确定位！🎯
