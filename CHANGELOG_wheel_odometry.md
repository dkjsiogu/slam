# 轮式里程计节点更新日志

## 2025-11-07: 同步下位机数据包格式

### 主要改动

根据下位机最新的 `master_process.h` 中 `Vision_Send_s` 结构体定义，更新了ROS2节点的数据包解析逻辑。

### 数据包结构变更

**旧格式 (56字节)**:
```c
header (1) + detect_color (1) + disp_x (4) + disp_y (4) + delta_theta (4) 
+ target_point[4][2] (32) + reserved[4] (4) + timestamp (4) + crc16 (2)
```

**新格式 (38字节)** - 与下位机完全一致:
```c
header (1) + flags (1) + roll (4) + pitch (4) + yaw (4) + delta_theta (4) 
+ disp_x (4) + disp_y (4) + heading_diff (4) + game_time (2) 
+ timestamp (4) + checksum (2)
```

### 新增字段说明

1. **IMU姿态数据** (12字节):
   - `roll`: 横滚角 (rad)
   - `pitch`: 俯仰角 (rad)
   - `yaw`: 航向角 (rad)

2. **运动信息** (4字节):
   - `heading_diff`: 运动方向相对初始坐标系的角度 (rad)

3. **比赛信息** (2字节):
   - `game_time`: 比赛时间 (秒)

4. **标志位** (1字节位域):
   - `detect_color`: 颜色检测 (0-红 1-蓝)
   - `task_mode`: 任务模式 (0-自动 1-瞄准 2-大符)
   - `reset_tracker`: 重置跟踪器
   - `is_play`: 播放状态
   - `change_target`: 更改目标

### 移除字段

- `target_point[4][2]`: 目标点坐标数组 (32字节) - 已移除
- `reserved[4]`: 保留字节 (4字节) - 已移除

### 代码改动

1. **数据包结构体** (`VisionSendPacket`):
   - 从56字节缩减为38字节
   - 添加IMU姿态字段 (roll/pitch/yaw)
   - 添加运动方向字段 (heading_diff)
   - 添加比赛时间字段 (game_time)
   - 使用位域正确解析标志位

2. **调试输出增强**:
   - 发布的调试数据从9个字段增加到13个字段
   - 新增IMU姿态角输出 (roll/pitch/yaw)
   - 新增运动方向角输出 (heading_diff)
   - 日志输出包含IMU数据（角度以度显示）

3. **CRC校验**:
   - 字段名从 `crc16` 改为 `checksum` (与下位机一致)
   - CRC计算长度自动适应新的结构体大小

### 调试话题数据格式

`/wheel_odom_data` (Float32MultiArray):
```
[0] disp_x          - 机器人坐标系增量X (m)
[1] disp_y          - 机器人坐标系增量Y (m)
[2] delta_theta     - 角度增量 (rad)
[3] dt              - 时间增量 (s)
[4] delta_x_world   - 世界坐标系增量X (m)
[5] delta_y_world   - 世界坐标系增量Y (m)
[6] current_x       - 累计位姿X (m)
[7] current_y       - 累计位姿Y (m)
[8] current_theta   - 累计位姿θ (rad)
[9] roll            - IMU横滚角 (rad)
[10] pitch          - IMU俯仰角 (rad)
[11] yaw            - IMU航向角 (rad)
[12] heading_diff   - 运动方向角 (rad)
```

### 向后兼容性

**不兼容**: 此更新与旧版下位机固件不兼容。
- 必须使用匹配的下位机固件 (包含新的 `Vision_Send_s` 定义)
- 数据包大小从56字节变为38字节

### 验证

- ✅ 编译成功 (无警告)
- ✅ 数据包结构与下位机完全对应
- ✅ CRC16校验逻辑正确
- ⏳ 待现场测试验证数据解析正确性

### 下一步

1. 使用新固件测试串口通信
2. 验证IMU数据是否正确接收
3. 确认里程计积分精度
4. 考虑是否使用IMU的yaw角替代积分角度（减少累计误差）

---

**文件**: `src/navigation_control/src/wheel_odometry_node.cpp`  
**下位机参考**: `master_process.h` 中的 `Vision_Send_s` 结构体
