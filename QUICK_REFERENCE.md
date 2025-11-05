# 🚀 快速参考卡片

## 一键启动
```bash
cd ~/slam
source install/setup.bash
ros2 launch navigation_control full_navigation_system.launch.py
```

## 常用命令

### 设置目标点
```bash
ros2 service call /set_goal_0 std_srvs/srv/Trigger  # 去目标点0
ros2 service call /set_goal_1 std_srvs/srv/Trigger  # 去目标点1
ros2 service call /start_patrol std_srvs/srv/Trigger  # 开始巡航
ros2 service call /cancel_goal std_srvs/srv/Trigger  # 取消
```

### 查看数据
```bash
ros2 topic echo /obstacle_distance  # 障碍物距离
ros2 topic echo /cmd_vel            # 速度指令
ros2 topic echo /scan               # 激光数据
```

### 串口权限
```bash
sudo chmod 777 /dev/ttyUSB0  # 临时
sudo usermod -a -G dialout $USER  # 永久（需重新登录）
```

## 协议格式
```
帧头: AA 55
X速度: 2字节 (-3000~3000 mm/s)
Y速度: 2字节 (-3000~3000 mm/s)  
Z速度: 2字节 (-3000~3000 °/s*10)
保留: 00
CRC16: 2字节
```

## 调试
```bash
# 查看节点
ros2 node list

# 查看话题
ros2 topic list

# 查看TF树
ros2 run tf2_tools view_frames && evince frames.pdf

# 测试脚本
cd ~/slam/src/navigation_control/scripts
python3 test_system.py
```

## 目录结构
```
slam/
├── src/
│   ├── my_slam_config/       # SLAM配置
│   ├── sllidar_ros2/         # 雷达驱动
│   └── navigation_control/   # 导航控制（新增）
│       ├── src/              # C++节点
│       │   ├── obstacle_distance_monitor.cpp
│       │   ├── manual_goal_setter.cpp
│       │   └── omnidirectional_controller.cpp
│       ├── launch/           # 启动文件
│       └── scripts/          # 测试脚本
└── COMPLETE_USAGE_GUIDE.md   # 完整文档
```
