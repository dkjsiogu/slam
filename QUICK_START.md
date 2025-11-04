# 快速开始指南

## 系统概览

完整的SLAM导航系统已经创建完成，包含以下功能模块：

```
navigation_control/
├── src/                          # 源代码
│   ├── mapping_manager.cpp       # 建图管理器
│   ├── localization_manager.cpp  # 定位管理器
│   ├── navigation_controller.cpp # 导航控制器
│   ├── serial_data_publisher.cpp # 串口数据准备
│   └── robot_state_manager.cpp   # 系统状态管理
├── launch/                       # 启动文件
│   ├── mapping.launch.py         # 建图模式
│   └── navigation.launch.py      # 导航模式
├── config/                       # 配置文件
│   ├── cartographer_2d.lua       # Cartographer配置
│   ├── amcl.yaml                 # AMCL定位配置
│   └── nav2_params.yaml          # Nav2导航配置
├── maps/                         # 地图存储
└── docs/                         # 文档
    └── SERIAL_PROTOCOL.md        # 串口通信协议
```

## 第一步：设置LIDAR权限

```bash
sudo chmod 777 /dev/ttyUSB0
# 或者创建udev规则（推荐）
cd ~/slam/src/sllidar_ros2
source scripts/create_udev_rules.sh
```

## 第二步：建图模式

### 启动建图系统

```bash
cd ~/slam
source install/setup.bash
ros2 launch navigation_control mapping.launch.py
```

这将启动：
- ✅ RPLIDAR驱动
- ✅ Cartographer SLAM
- ✅ Mapping Manager（建图管理器）
- ✅ Serial Data Publisher（串口数据准备）
- ✅ Robot State Manager（系统状态管理）
- ✅ RViz2可视化

### 控制机器人移动建图

**新终端**中运行键盘控制：
```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

控制键位：
- `i` - 前进
- `,` - 后退
- `j` - 左转
- `l` - 右转
- `k` - 停止
- `q/z` - 增加/减少速度

### 监控建图状态

**新终端**：
```bash
# 查看建图状态
ros2 topic echo /mapping_status

# 查看系统综合状态
ros2 topic echo /system_status

# 查看串口数据（十六进制）
ros2 topic echo /serial_tx_hex
```

### 保存地图

**方法1 - 手动保存**：
```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

**方法2 - 自动保存**：
系统每60秒自动保存一次地图到 `~/slam/src/navigation_control/maps/`

地图文件：
- `my_map_<timestamp>.pgm` - 地图图像
- `my_map_<timestamp>.yaml` - 地图元数据

## 第三步：导航模式

### 切换到导航模式

**方法1 - 停止建图，启动导航**：
```bash
# 停止建图launch（Ctrl+C）
# 启动导航launch
ros2 launch navigation_control navigation.launch.py \
  map_file:=/home/dkjsiogu/slam/src/navigation_control/maps/my_map_XXXXX.yaml
```

**方法2 - 使用状态管理器切换**：
```bash
ros2 service call /switch_to_navigation std_srvs/srv/Trigger
```

### 设置初始位姿（重定位）

在RViz2中：
1. 点击 "2D Pose Estimate"
2. 在地图上点击机器人的实际位置
3. 拖动设置朝向

或通过服务：
```bash
ros2 service call /reset_localization std_srvs/srv/Trigger
```

### 发送导航目标

**方法1 - RViz2图形界面**：
1. 点击 "2D Goal Pose"
2. 在地图上点击目标位置
3. 拖动设置目标朝向

**方法2 - 服务调用**：
```bash
# 导航到预设路点
ros2 service call /navigate_to_goal std_srvs/srv/Trigger
```

**方法3 - 取消导航**：
```bash
ros2 service call /cancel_navigation std_srvs/srv/Trigger
```

## 第四步：监控和调试

### 查看所有运行的节点

```bash
ros2 node list
```

应该看到：
- `/mapping_manager` 或 `/localization_manager`
- `/navigation_controller`
- `/serial_data_publisher`
- `/robot_state_manager`
- `/sllidar_node`
- `/cartographer_node` 或 `/amcl`

### 查看话题列表

```bash
ros2 topic list
```

关键话题：
- `/scan` - 激光扫描数据
- `/map` - 地图
- `/cmd_vel` - 速度命令
- `/serial_tx_data` - 串口发送数据（字节数组）
- `/serial_tx_hex` - 串口发送数据（十六进制字符串）
- `/system_status` - 系统状态

### 查看TF树

```bash
ros2 run tf2_tools view_frames
# 会生成 frames.pdf 文件
evince frames.pdf
```

TF树应该是：
```
map → odom → base_link → laser
```

### 实时监控速度命令

```bash
# 查看速度命令
ros2 topic echo /cmd_vel

# 查看处理后的速度（平滑+限制）
ros2 topic echo /processed_cmd_vel
```

### 查看串口数据协议

```bash
# 查看十六进制格式
ros2 topic echo /serial_tx_hex
```

输出示例：
```
data: 'TX: aa 55 01 f4 01 00 00 00 00 ff 0d'
```

解析：
- `aa 55` - 帧头
- `01` - 命令ID（速度控制）
- `f4 01` - 线速度 = 0x01F4 = 500 mm/s
- `00 00` - 角速度 = 0 mrad/s
- `00 00` - 保留字节
- `ff` - 校验和
- `0d` - 帧尾

## 第五步：紧急情况处理

### 紧急停止

```bash
ros2 service call /emergency_stop std_srvs/srv/Trigger
```

这会：
1. 立即发送停止命令到 `/cmd_vel`
2. 取消所有导航任务
3. 标记系统为紧急停止状态

### 检查错误

```bash
# 查看节点日志
ros2 node info /navigation_controller

# 查看话题详情
ros2 topic info /cmd_vel
```

## 串口数据说明

### 当前准备的数据格式

Serial Data Publisher节点已经将`/cmd_vel`转换为下位机协议格式：

**发布话题**：
- `/serial_tx_data` (std_msgs/UInt8MultiArray) - 原始字节数据，可直接发送到串口
- `/serial_tx_hex` (std_msgs/String) - 十六进制字符串，用于调试

### 下一步：实际串口通信

需要创建串口通信节点：

```cpp
// 伪代码示例
#include <serial/serial.h>

class SerialCommunication : public rclcpp::Node {
    serial::Serial ser_("/dev/ttyUSB1", 115200);
    
    void serialTxCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        // 直接发送到串口
        ser_.write(msg->data.data(), msg->data.size());
    }
};
```

详细协议请参考：`docs/SERIAL_PROTOCOL.md`

## 系统架构说明

### 建图模式流程

```
LIDAR → /scan → Cartographer → /map → Mapping Manager
                     ↓
              Robot Movement
                     ↓
         Serial Data Publisher → /serial_tx_data
```

### 导航模式流程

```
LIDAR → /scan → AMCL → /amcl_pose
         ↓
    Localization Manager → /current_pose
                              ↓
         User Goal → Nav2 → /cmd_vel → Serial Data Publisher
                              ↓
                      /serial_tx_data → 下位机
```

## 常见问题

### 1. 编译失败

```bash
# 确保安装所有依赖
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-cartographer-ros

# 重新编译
cd ~/slam
colcon build --packages-select navigation_control
```

### 2. LIDAR无数据

```bash
# 检查权限
ls -l /dev/ttyUSB0

# 检查话题
ros2 topic hz /scan
```

### 3. 定位不准

```bash
# 重新设置初始位姿
ros2 service call /reset_localization std_srvs/srv/Trigger

# 或在RViz2中使用"2D Pose Estimate"
```

### 4. 导航失败

```bash
# 检查地图是否加载
ros2 topic echo /map --once

# 检查定位是否工作
ros2 topic echo /amcl_pose

# 检查TF树
ros2 run tf2_tools view_frames
```

## 性能参数调优

### 建图质量调优

编辑 `config/cartographer_2d.lua`:
```lua
-- 提高分辨率（更精细的地图）
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.025

-- 增加扫描匹配权重（更准确但更慢）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 2.0
```

### 导航速度调优

编辑 `config/nav2_params.yaml`:
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 1.0        # 增加最大线速度
      max_vel_theta: 2.0    # 增加最大角速度
```

### 串口数据调优

修改参数：
```bash
ros2 param set /serial_data_publisher max_linear_vel 1.5
ros2 param set /serial_data_publisher smooth_factor 0.9
```

## 下一步开发建议

1. **实现实际串口通信**
   - 使用`serial`库或`boost::asio`
   - 订阅`/serial_tx_data`并发送到串口
   - 从串口接收下位机反馈数据

2. **添加里程计反馈**
   - 接收编码器数据
   - 发布到`/odom`话题
   - 提高定位精度

3. **实现多目标点巡航**
   - 从文件加载路点列表
   - 自动按顺序导航
   - 循环巡航功能

4. **添加Web界面**
   - 使用rosbridge_server
   - 创建Web控制面板
   - 远程监控和控制

5. **电池监控**
   - 订阅电池电压
   - 低电量自动返回充电

## 总结

✅ 建图系统 - 完成
✅ 定位系统 - 完成
✅ 导航系统 - 完成
✅ 串口数据准备 - 完成
✅ 系统状态管理 - 完成
⏳ 实际串口通信 - 待开发
⏳ 里程计反馈 - 待开发

整个上位机框架已经完成，串口数据已经准备好，只需要添加实际的串口通信模块即可与下位机通信！
