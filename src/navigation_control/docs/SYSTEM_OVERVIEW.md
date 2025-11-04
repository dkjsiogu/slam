# 系统完整数据流

## 建图模式数据流

```
┌─────────────┐
│  RPLIDAR    │ /dev/ttyUSB0 (雷达)
│   硬件设备   │
└──────┬──────┘
       │
       │ /scan (sensor_msgs/LaserScan)
       ▼
┌─────────────────┐
│ sllidar_node    │ (驱动节点)
└────────┬────────┘
         │
         │ /scan
         ▼
┌──────────────────┐         ┌──────────────────┐
│  Cartographer    │────────▶│ Mapping Manager  │
│    SLAM 建图     │  /map   │   建图管理节点    │
└────────┬─────────┘         └──────────────────┘
         │
         │ /map (nav_msgs/OccupancyGrid)
         │
         ▼
┌─────────────────────────────────────────────────┐
│              用户通过键盘控制移动                  │
│        ros2 run teleop_twist_keyboard ...       │
└────────────────────┬────────────────────────────┘
                     │
                     │ /cmd_vel (geometry_msgs/Twist)
                     │
       ┌─────────────┴─────────────┐
       │                           │
       ▼                           ▼
┌──────────────────┐      ┌─────────────────────┐
│ Serial Data      │      │  机器人底盘执行     │
│ Publisher        │      │  (通过串口通信)      │
│ 数据转换节点      │      └─────────────────────┘
└────────┬─────────┘
         │
         │ /serial_tx_data (std_msgs/UInt8MultiArray)
         │ 协议: AA 55 01 [linear_L] [linear_H] [angular_L] [angular_H] [resv] [resv] [checksum] 0D
         │
         ▼
┌──────────────────────────┐
│ Serial Communication     │
│ 实际串口通信节点          │
│ LibSerial I/O            │
└───────────┬──────────────┘
            │
            │ /dev/ttyUSB1 (开发板 Micro USB)
            │ 115200 baud, 8N1
            │
            ▼
    ┌───────────────┐
    │  下位机开发板  │
    │  (STM32/      │
    │   Arduino等)  │
    └───────────────┘
```

## 导航模式数据流

```
┌─────────────┐
│  RPLIDAR    │ /dev/ttyUSB0
│   硬件设备   │
└──────┬──────┘
       │
       │ /scan
       ▼
┌─────────────────┐
│ sllidar_node    │
└────────┬────────┘
         │
         │ /scan
         ▼
┌─────────────────┐         ┌──────────────────┐
│      AMCL       │────────▶│ Localization     │
│  自适应蒙特卡洛  │ /amcl_  │ Manager          │
│     定位算法     │  pose   │ 定位管理节点      │
└────────┬────────┘         └──────────────────┘
         │
         │ /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
         │
         ▼
┌──────────────────────────────────────────────────┐
│                    Nav2 导航栈                    │
│  ┌──────────────┐  ┌──────────────┐             │
│  │ bt_navigator │  │planner_server│             │
│  │  行为树调度  │  │  全局路径规划│             │
│  └──────┬───────┘  └──────┬───────┘             │
│         │                 │                      │
│         ▼                 ▼                      │
│  ┌─────────────────────────────────┐            │
│  │     controller_server           │            │
│  │     局部路径规划和控制           │            │
│  └────────────────┬────────────────┘            │
└───────────────────┼─────────────────────────────┘
                    │
                    │ /cmd_vel (geometry_msgs/Twist)
                    │
       ┌────────────┴────────────┐
       │                         │
       ▼                         ▼
┌──────────────────┐    ┌──────────────────┐
│ Navigation       │    │ Serial Data      │
│ Controller       │    │ Publisher        │
│ 导航控制节点      │    │ 数据转换节点      │
└──────────────────┘    └────────┬─────────┘
                                 │
                                 │ /serial_tx_data
                                 │
                                 ▼
                        ┌──────────────────────────┐
                        │ Serial Communication     │
                        │ 串口通信节点              │
                        └───────────┬──────────────┘
                                    │
                                    │ /dev/ttyUSB1
                                    │
                                    ▼
                            ┌───────────────┐
                            │  下位机开发板  │
                            │  接收速度命令  │
                            │  控制电机转动  │
                            └───────────────┘
```

## 节点通信总览

### Topic 列表

| Topic 名称 | 消息类型 | 发布者 | 订阅者 | 说明 |
|-----------|---------|--------|--------|------|
| `/scan` | sensor_msgs/LaserScan | sllidar_node | cartographer_node, amcl | 激光雷达扫描数据 |
| `/map` | nav_msgs/OccupancyGrid | cartographer_node | mapping_manager, rviz2 | 占据栅格地图 |
| `/cmd_vel` | geometry_msgs/Twist | teleop/nav2 | serial_data_publisher | 速度控制命令 |
| `/serial_tx_data` | std_msgs/UInt8MultiArray | serial_data_publisher | serial_communication | 待发送的字节数据 |
| `/serial_tx_hex` | std_msgs/String | serial_data_publisher | (调试) | 十六进制显示 |
| `/serial_rx_data` | std_msgs/UInt8MultiArray | serial_communication | (未来:里程计解析) | 接收的原始数据 |
| `/serial_rx_hex` | std_msgs/String | serial_communication | (调试) | 接收数据十六进制 |
| `/serial_connection_status` | std_msgs/String | serial_communication | (监控) | 连接状态统计 |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | amcl | localization_manager | 定位位姿 |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | localization_manager | amcl | 初始位姿设置 |
| `/mapping_status` | std_msgs/String | mapping_manager | robot_state_manager | 建图状态 |
| `/localization_status` | std_msgs/String | localization_manager | robot_state_manager | 定位状态 |
| `/navigation_status` | std_msgs/String | navigation_controller | robot_state_manager | 导航状态 |
| `/system_status` | std_msgs/String | robot_state_manager | (监控) | 系统综合状态 |

### Service 列表

| Service 名称 | 类型 | 提供者 | 说明 |
|-------------|------|--------|------|
| `/start_mapping` | std_srvs/Trigger | mapping_manager | 开始建图 |
| `/stop_mapping` | std_srvs/Trigger | mapping_manager | 停止建图 |
| `/save_map` | std_srvs/Trigger | mapping_manager | 保存地图 |
| `/start_localization` | std_srvs/Trigger | localization_manager | 开始定位 |
| `/stop_localization` | std_srvs/Trigger | localization_manager | 停止定位 |
| `/reset_localization` | std_srvs/Trigger | localization_manager | 重定位 |
| `/navigate_to_goal` | std_srvs/Trigger | navigation_controller | 导航到目标 |
| `/cancel_navigation` | std_srvs/Trigger | navigation_controller | 取消导航 |
| `/switch_to_mapping` | std_srvs/Trigger | robot_state_manager | 切换到建图模式 |
| `/switch_to_navigation` | std_srvs/Trigger | robot_state_manager | 切换到导航模式 |
| `/emergency_stop` | std_srvs/Trigger | robot_state_manager | 紧急停止 |

## TF 变换树

```
map
 └─ odom (由 AMCL 或 Cartographer 发布)
     └─ base_link (静态发布)
         └─ laser (静态发布)
```

**帧说明**:
- `map`: 全局地图坐标系
- `odom`: 里程计坐标系 (局部一致,全局可能漂移)
- `base_link`: 机器人底盘中心
- `laser`: 激光雷达坐标系

## 串口协议详细说明

### 上位机 → 下位机 (速度控制)

**数据包格式** (11 字节):
```
Byte 0:  0xAA        - 帧头1
Byte 1:  0x55        - 帧头2
Byte 2:  0x01        - 命令ID (速度控制)
Byte 3:  linear_L    - 线速度低字节
Byte 4:  linear_H    - 线速度高字节
Byte 5:  angular_L   - 角速度低字节
Byte 6:  angular_H   - 角速度高字节
Byte 7:  0x00        - 保留字节1
Byte 8:  0x00        - 保留字节2
Byte 9:  checksum    - 校验和 (Byte2 XOR Byte3 XOR ... XOR Byte8)
Byte 10: 0x0D        - 帧尾
```

**示例**:
- 停止: `AA 55 01 00 00 00 00 00 00 01 0D`
- 前进0.5m/s: `AA 55 01 F4 01 00 00 00 00 F5 0D`
- 后退0.3m/s: `AA 55 01 D8 FE 00 00 00 00 D9 0D` (-300 mm/s)
- 左转1.0rad/s: `AA 55 01 00 00 E8 03 00 00 E9 0D`

### 下位机 → 上位机 (里程计反馈) [待实现]

**预留协议** (15 字节):
```
Byte 0:  0xAA        - 帧头1
Byte 1:  0x55        - 帧头2
Byte 2:  0x83        - 命令ID (里程计数据)
Byte 3-4: left_encoder_L/H   - 左轮编码器值
Byte 5-6: right_encoder_L/H  - 右轮编码器值
Byte 7-8: velocity_L/H       - 当前速度
Byte 9-10: yaw_L/H           - 当前航向角
Byte 11:  battery_level      - 电池电量
Byte 12:  status_flags       - 状态标志
Byte 13:  checksum           - 校验和
Byte 14:  0x0D               - 帧尾
```

## 系统工作流程

### 建图流程

1. **启动系统**
   ```bash
   ros2 launch navigation_control mapping.launch.py
   ```

2. **系统初始化**
   - sllidar_node 连接雷达 → 发布 `/scan`
   - Cartographer 订阅 `/scan` → 开始 SLAM
   - mapping_manager 订阅 `/map` → 监控建图进度
   - serial_communication 连接开发板 → 准备接收速度命令

3. **遥控机器人移动**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   - 发布 `/cmd_vel`
   - serial_data_publisher 转换 → `/serial_tx_data`
   - serial_communication 发送 → 下位机
   - 下位机控制电机 → 机器人移动

4. **保存地图**
   ```bash
   ros2 service call /save_map std_srvs/srv/Trigger
   ```
   - mapping_manager 调用 map_saver_cli
   - 保存 `.pgm` 和 `.yaml` 文件

### 导航流程

1. **启动系统**
   ```bash
   ros2 launch navigation_control navigation.launch.py \
       map_file:=/path/to/map.yaml
   ```

2. **系统初始化**
   - map_server 加载地图
   - AMCL 开始定位
   - Nav2 栈启动
   - serial_communication 连接开发板

3. **设置初始位姿** (RViz2)
   - 点击 "2D Pose Estimate"
   - 在地图上标记机器人位置和方向
   - localization_manager 发布 → `/initialpose`

4. **发送导航目标** (RViz2)
   - 点击 "2D Goal Pose"
   - 在地图上标记目标位置
   - Nav2 规划路径 → 发布 `/cmd_vel`
   - serial_communication 传输 → 下位机执行

5. **自主导航**
   - Nav2 持续调整路径
   - AMCL 持续更新位姿
   - 机器人自主到达目标点

## 系统特性

### 已实现功能 ✅

1. **建图系统**
   - ✅ Cartographer 2D SLAM
   - ✅ 自动保存地图 (60秒间隔)
   - ✅ 手动保存地图服务
   - ✅ 建图状态监控

2. **定位系统**
   - ✅ AMCL 粒子滤波定位
   - ✅ 初始位姿设置
   - ✅ 重定位服务
   - ✅ 定位质量评估

3. **导航系统**
   - ✅ Nav2 完整导航栈
   - ✅ 全局路径规划
   - ✅ 局部路径规划
   - ✅ 动态避障
   - ✅ 预设航点导航

4. **串口通信**
   - ✅ 速度命令协议转换
   - ✅ 实际串口 I/O (LibSerial)
   - ✅ 自动重连机制
   - ✅ 连接状态监控
   - ✅ 数据统计 (TX/RX 字节数)
   - ✅ 十六进制调试输出

5. **状态管理**
   - ✅ 系统模式切换
   - ✅ 紧急停止
   - ✅ 综合状态监控

### 待实现功能 ⏳

1. **里程计反馈**
   - ⏳ 解析下位机发送的编码器数据
   - ⏳ 发布 `/odom` topic
   - ⏳ 融合轮式里程计和激光里程计

2. **多机器人支持**
   - ⏳ 命名空间隔离
   - ⏳ 多机器人协同建图

3. **高级功能**
   - ⏳ 路径规划优化
   - ⏳ 动态地图更新
   - ⏳ 电池监控和自动充电
