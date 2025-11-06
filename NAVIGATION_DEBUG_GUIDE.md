# 导航调试 Launch 使用说明

## 功能概述

`navigation_debug.launch.py` 提供了一个简单的导航调试环境，包括：

1. **地图加载**：加载手动保存的地图文件
2. **AMCL 定位**：使用激光雷达数据进行机器人定位
3. **目标点导航**：在 RViz2 中设置目标点，自动计算速度
4. **全向轮控制**：将速度命令通过串口发送到 STM32 下位机
5. **可视化**：RViz2 显示地图、激光雷达、机器人位姿等

## 启动前准备

### 1. 确保地图文件存在

```bash
ls /home/pfa5/slam/src/navigation_control/maps/my_map.yaml
ls /home/pfa5/slam/src/navigation_control/maps/my_map.pgm
```

如果没有地图，先运行建图：
```bash
ros2 launch navigation_control mapping.launch.py
# 建图完成后在另一个终端保存地图
ros2 run nav2_map_server map_saver_cli -f ~/slam/src/navigation_control/maps/my_map
```

### 2. 确保串口设备可用

```bash
ls -l /dev/ttyACM0
ls -l /dev/ser  # udev 规则创建的符号链接
```

如果权限不足：
```bash
sudo chmod 666 /dev/ttyACM0
```

### 3. 确保激光雷达已连接

```bash
ls -l /dev/ttyUSB0
```

## 启动导航调试

### 基本启动（使用默认参数）

```bash
cd ~/slam
source install/setup.bash
ros2 launch navigation_control navigation_debug.launch.py
```

### 指定地图文件和串口

```bash
ros2 launch navigation_control navigation_debug.launch.py \
    map_file:=/path/to/your_map.yaml \
    dev_board_port:=/dev/ttyACM0
```

## 使用 RViz2 设置导航目标

### 1. 设置初始位姿（重要！）

1. 点击 RViz2 顶部工具栏的 **"2D Pose Estimate"** 按钮
2. 在地图上点击机器人的大致位置
3. 拖动鼠标设置机器人的朝向
4. 观察粒子云（绿色点）是否收敛到机器人真实位置

### 2. 设置导航目标

1. 点击 RViz2 顶部工具栏的 **"2D Nav Goal"** 按钮
2. 在地图上点击目标位置
3. 拖动鼠标设置目标朝向
4. 机器人会自动计算路径并开始移动

### 3. 查看速度命令

在新终端查看发送给下位机的速度命令：

```bash
source install/setup.bash
ros2 topic echo /cmd_vel
```

查看串口发送的数据：

```bash
ros2 topic echo /serial_tx_hex
```

## 系统架构

```
RViz2 (2D Nav Goal)
    ↓ /goal_pose
simple_goal_controller.py  (计算 vx, vy, wz)
    ↓ /cmd_vel
serial_data_publisher  (编码为 OmniWheelCmd 协议)
    ↓ /serial_tx_data
serial_communication  (串口发送)
    ↓ 串口
STM32 下位机 (全向轮控制)
```

## 参数说明

### 地图参数

- `map_file`: 地图 YAML 文件路径
  - 默认: `/home/pfa5/slam/src/navigation_control/maps/my_map.yaml`

### 串口参数

- `dev_board_port`: STM32 串口设备
  - 默认: `/dev/ser`
  - 可选: `/dev/ttyACM0`, `/dev/ttyACM1`

### 导航控制参数（在 simple_goal_controller.py 中）

可以编辑 launch 文件中的参数：

```python
parameters=[{
    'max_linear_vel': 0.5,      # 最大线速度 (m/s)
    'max_angular_vel': 1.0,     # 最大角速度 (rad/s)
    'goal_tolerance': 0.1,      # 目标容差 (m)
    'lookahead_distance': 0.3,  # 前瞻距离 (m)
}]
```

## 调试技巧

### 1. 查看所有节点

```bash
ros2 node list
```

应该看到：
- `/map_server`
- `/lifecycle_manager_map_server`
- `/amcl`
- `/lifecycle_manager_localization`
- `/simple_goal_controller`
- `/serial_data_publisher`
- `/serial_communication`
- `/rviz2`

### 2. 查看 TF 树

```bash
ros2 run tf2_tools view_frames
# 会生成 frames.pdf
evince frames.pdf
```

应该有完整的 TF 链：`map → odom → base_link → laser`

### 3. 监控话题频率

```bash
ros2 topic hz /scan          # 激光雷达数据
ros2 topic hz /amcl_pose     # 定位结果
ros2 topic hz /cmd_vel       # 速度命令
```

### 4. 查看串口状态

```bash
ros2 topic echo /serial_status
```

### 5. 日志查看

如果 simple_goal_controller 报错：
```bash
ros2 node info /simple_goal_controller
ros2 topic list | grep goal
```

## 常见问题

### Q1: RViz2 显示 "No map received"

**解决方案**：
1. 确保地图文件路径正确
2. 检查 map_server 节点是否启动：`ros2 node list | grep map_server`
3. 手动激活地图服务器：
   ```bash
   ros2 lifecycle set /map_server configure
   ros2 lifecycle set /map_server activate
   ```

### Q2: 粒子云不收敛

**解决方案**：
1. 确保激光雷达数据正常：`ros2 topic echo /scan`
2. 在 RViz2 中重新设置初始位姿（尽量准确）
3. 推动机器人移动几次，帮助粒子收敛

### Q3: 设置目标后机器人不动

**解决方案**：
1. 检查 simple_goal_controller 是否收到目标：
   ```bash
   ros2 topic echo /goal_pose
   ```
2. 检查是否有 /cmd_vel 发布：
   ```bash
   ros2 topic echo /cmd_vel
   ```
3. 查看控制器日志：
   ```bash
   ros2 node info /simple_goal_controller
   ```

### Q4: 串口连接失败

**解决方案**：
1. 确认串口设备存在：`ls -l /dev/ttyACM0`
2. 检查权限：`sudo chmod 666 /dev/ttyACM0`
3. 确认没有其他程序占用串口
4. 查看串口通信节点日志

## 与完整导航系统的区别

| 功能 | navigation_debug.launch.py | full_navigation.launch.py |
|------|---------------------------|---------------------------|
| 路径规划 | ❌ 简单直线导航 | ✅ Nav2 完整路径规划 |
| 障碍物避障 | ❌ 无避障 | ✅ 动态避障 |
| 恢复行为 | ❌ 无 | ✅ 自动恢复 |
| 配置复杂度 | ✅ 简单 | ❌ 复杂 |
| 适用场景 | 调试、简单移动 | 生产环境 |

## 下一步

如果简单导航工作正常，可以升级到完整的 Nav2 导航系统：

```bash
ros2 launch navigation_control full_navigation.launch.py
```

完整系统包括：
- 全局路径规划器
- 局部路径规划器
- 障碍物层
- 膨胀层
- 行为树导航
- 自动恢复机制
