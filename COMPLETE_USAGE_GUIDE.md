# 🚀 完整使用指南

## 📦 新增功能总结

### 1. 障碍物距离显示节点 (`obstacle_distance_monitor.cpp`)
- **功能**：实时监测激光雷达数据，显示最近障碍物距离（米）
- **发布话题**：`/obstacle_distance` (std_msgs/Float32)
- **可视化**：自动标记危险距离（<0.5m红色警告）

### 2. 手动目标点设置节点 (`manual_goal_setter.cpp`)
- **功能**：通过ROS2服务快速设置预定义目标点
- **预设4个目标点**：可在代码中自定义坐标
- **支持巡航模式**：自动依次访问所有目标点
- **服务接口**：
  - `/set_goal_0` ~ `/set_goal_3` - 设置单个目标点
  - `/start_patrol` - 开始巡航
  - `/cancel_goal` - 取消当前目标

### 3. 全向轮控制节点 (`omnidirectional_controller.cpp`)
- **功能**：将导航速度指令转换为全向轮控制数据并发送给下位机
- **协议格式**：11字节固定格式（帧头+速度+CRC）
- **CRC校验**：CRC-16/MODBUS确保数据完整性
- **自动重连**：串口断开自动尝试重连
- **支持参数**：
  - `serial_port` - 串口设备（默认：/dev/ttyUSB0）
  - `baudrate` - 波特率（默认：115200）
  - `max_linear_vel` - 最大线速度（m/s）
  - `max_angular_vel` - 最大角速度（rad/s）

---

## 🔧 编译和源码设置

```bash
cd ~/slam
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 🎯 完整系统启动流程

### 方法1：一键启动（推荐）

```bash
# 启动完整系统（SLAM + 导航 + 全向轮控制）
ros2 launch navigation_control full_navigation_system.launch.py

# 或者只启动导航控制部分（不含SLAM）
ros2 launch navigation_control navigation_system.launch.py
```

### 方法2：分步启动

#### 步骤1：启动SLAM建图
```bash
# 终端1：启动雷达
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py

# 终端2：启动Cartographer
ros2 launch my_slam_config cartographer.launch.py
```

#### 步骤2：启动导航控制
```bash
# 终端3：障碍物距离监测
ros2 run navigation_control obstacle_distance_monitor

# 终端4：手动目标点设置
ros2 run navigation_control manual_goal_setter

# 终端5：全向轮控制器
ros2 run navigation_control omnidirectional_controller --ros-args -p serial_port:=/dev/ttyUSB0 -p baudrate:=115200
```

---

## 📡 使用示例

### 1. 查看障碍物距离

```bash
# 实时查看距离数据
ros2 topic echo /obstacle_distance

# 查看发布频率
ros2 topic hz /obstacle_distance

# 查看话题信息
ros2 topic info /obstacle_distance
```

### 2. 设置目标点

```bash
# 前往目标点0（例如：原点）
ros2 service call /set_goal_0 std_srvs/srv/Trigger

# 前往目标点1
ros2 service call /set_goal_1 std_srvs/srv/Trigger

# 前往目标点2
ros2 service call /set_goal_2 std_srvs/srv/Trigger

# 前往目标点3
ros2 service call /set_goal_3 std_srvs/srv/Trigger

# 开始自动巡航（依次访问所有点）
ros2 service call /start_patrol std_srvs/srv/Trigger

# 取消当前目标
ros2 service call /cancel_goal std_srvs/srv/Trigger
```

### 3. 监控全向轮控制

```bash
# 查看发送给下位机的速度指令
ros2 topic echo /cmd_vel

# 查看节点状态
ros2 node info /omnidirectional_controller

# 查看参数
ros2 param list /omnidirectional_controller
ros2 param get /omnidirectional_controller serial_port
```

### 4. 在RViz2中手动设置目标点

```bash
# 启动RViz2（如果未自动启动）
rviz2 -d /home/dkjsiogu/slam/src/navigation_control/config/navigation.rviz

# 操作步骤：
# 1. 点击顶部工具栏的 "2D Goal Pose"
# 2. 在地图上点击目标位置
# 3. 拖动鼠标设置方向
# 4. 释放鼠标，机器人开始导航
```

---

## 🔍 调试和测试

### 测试脚本
```bash
# 使用自动化测试脚本
cd ~/slam/src/navigation_control/scripts
python3 test_system.py
```

### 查看串口通信
```bash
# 检查串口权限
ls -l /dev/ttyUSB0

# 如果需要权限
sudo chmod 777 /dev/ttyUSB0

# 或者添加用户到dialout组（永久方案）
sudo usermod -a -G dialout $USER
# 注销并重新登录生效

# 监听串口数据（调试用）
sudo apt install minicom
sudo minicom -D /dev/ttyUSB0 -b 115200
```

### 查看日志
```bash
# 查看节点日志
ros2 run navigation_control omnidirectional_controller --ros-args --log-level debug

# 查看系统所有话题
ros2 topic list

# 查看TF树
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

## ⚙️ 自定义配置

### 1. 修改预设目标点

编辑 `src/navigation_control/src/manual_goal_setter.cpp`：

```cpp
void ManualGoalSetter::initializeWaypoints() {
    // 目标点0：原点
    waypoints_.push_back(createPose(0.0, 0.0, 0.0));
    
    // 目标点1：前方2米
    waypoints_.push_back(createPose(2.0, 0.0, 0.0));
    
    // 目标点2：左转1米
    waypoints_.push_back(createPose(2.0, 1.0, 1.57));
    
    // 目标点3：返回原点
    waypoints_.push_back(createPose(0.0, 0.0, 3.14));
    
    // 添加更多目标点...
}
```

### 2. 调整障碍物警告距离

编辑 `src/navigation_control/src/obstacle_distance_monitor.cpp`：

```cpp
// 修改危险距离阈值
if (min_distance < 0.5) {  // 改为你需要的值
    RCLCPP_WARN(this->get_logger(), 
        "⚠️  危险！最近障碍物距离: %.2f 米", min_distance);
}
```

### 3. 调整全向轮速度限制

```bash
# 启动时设置参数
ros2 run navigation_control omnidirectional_controller \
  --ros-args \
  -p max_linear_vel:=1.5 \
  -p max_angular_vel:=2.0
```

或编辑 `launch/navigation_system.launch.py`：
```python
Node(
    package='navigation_control',
    executable='omnidirectional_controller',
    parameters=[{
        'max_linear_vel': 1.5,
        'max_angular_vel': 2.0
    }]
)
```

---

## 📊 通信协议详情

### 下位机协议格式（11字节）

| 字节 | 说明 | 数值范围 |
|------|------|----------|
| 0-1 | 帧头 | 0xAA 0x55 |
| 2-3 | X方向速度 | -3000 ~ 3000 (mm/s) |
| 4-5 | Y方向速度 | -3000 ~ 3000 (mm/s) |
| 6-7 | Z旋转速度 | -3000 ~ 3000 (°/s * 10) |
| 8 | 保留字节 | 0x00 |
| 9-10 | CRC-16校验 | MODBUS算法 |

### 示例数据包
```
停止: AA 55 00 00 00 00 00 00 00 [CRC]
前进: AA 55 E8 03 00 00 00 00 00 [CRC]  // 1000mm/s
右移: AA 55 00 00 E8 03 00 00 00 [CRC]  // 1000mm/s
左转: AA 55 00 00 00 00 2C 01 00 [CRC]  // 30°/s
```

---

## 🐛 常见问题

### 1. 串口打不开
```bash
# 检查设备是否存在
ls /dev/ttyUSB*

# 检查权限
ls -l /dev/ttyUSB0

# 临时授权
sudo chmod 777 /dev/ttyUSB0

# 永久方案
sudo usermod -a -G dialout $USER
```

### 2. 没有激光数据
```bash
# 检查雷达节点
ros2 node list | grep sllidar

# 检查话题
ros2 topic echo /scan

# 检查波特率是否匹配你的雷达型号
```

### 3. 导航不工作
```bash
# 检查TF树完整性
ros2 run tf2_tools view_frames

# 确保有这些坐标系：map -> odom -> base_link -> laser
ros2 run tf2_ros tf2_echo map base_link
```

### 4. 目标点无法到达
```bash
# 检查目标点是否在地图范围内
# 查看RViz2中的地图，确保目标点不在障碍物上

# 检查局部代价地图
ros2 topic echo /local_costmap/costmap
```

---

## 📈 性能优化建议

1. **降低激光数据频率**（如果CPU占用高）：
   - 编辑 `sllidar_ros2/launch` 中的发布频率参数

2. **调整Cartographer参数**：
   - 编辑 `my_slam_config/config/backpack_2d.lua`
   - 减少 `num_accumulated_range_data` 可降低延迟

3. **串口缓冲优化**：
   - 当前代码已使用非阻塞串口
   - 如果延迟高，考虑提高波特率

---

## 🎓 学习资源

- [ROS2 Humble文档](https://docs.ros.org/en/humble/)
- [Cartographer文档](https://google-cartographer-ros.readthedocs.io/)
- [全向轮运动学](https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383)

---

## 📝 更新日志

- **2025-01-05**：
  - ✅ 添加障碍物距离监测节点
  - ✅ 添加手动目标点设置功能
  - ✅ 实现全向轮控制器（带CRC校验）
  - ✅ 集成完整的launch文件
  - ✅ 完善文档和测试脚本

---

## 💡 下一步扩展建议

1. **添加更多传感器融合**：IMU、里程计
2. **实现路径录制和回放**
3. **添加动态避障算法**
4. **集成机械臂控制**
5. **开发Web控制界面**
6. **添加语音控制**

---

**祝你使用愉快！有问题随时查看文档或运行测试脚本。** 🎉
