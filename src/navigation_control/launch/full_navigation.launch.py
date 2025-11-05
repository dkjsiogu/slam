#!/usr/bin/env python3
"""
全功能导航系统启动文件
包含SLAM、导航、障碍物监控、手动目标点设置、全向轮控制
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='LIDAR serial port'
    )
    
    dev_board_port_arg = DeclareLaunchArgument(
        'dev_board_port',
        default_value='/dev/ttyUSB1',
        description='Development board serial port (Micro USB)'
    )
    
    dev_board_baudrate_arg = DeclareLaunchArgument(
        'dev_board_baudrate',
        default_value='115200',
        description='Development board baudrate'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable RViz visualization markers'
    )
    
    # 获取参数
    lidar_port = LaunchConfiguration('lidar_port')
    dev_board_port = LaunchConfiguration('dev_board_port')
    dev_board_baudrate = LaunchConfiguration('dev_board_baudrate')
    enable_visualization = LaunchConfiguration('enable_visualization')
    
    return LaunchDescription([
        # 参数声明
        lidar_port_arg,
        dev_board_port_arg,
        dev_board_baudrate_arg,
        enable_visualization_arg,
        
        # 启动信息
        LogInfo(msg='启动全功能SLAM导航系统...'),
        LogInfo(msg=['激光雷达端口: ', lidar_port]),
        LogInfo(msg=['下位机端口: ', dev_board_port]),
        
        # ============ 障碍物监控节点 ============
        Node(
            package='navigation_control',
            executable='obstacle_monitor',
            name='obstacle_monitor',
            output='screen',
            parameters=[{
                'warning_distance': 0.5,      # 警告距离 (米)
                'danger_distance': 0.3,       # 危险距离 (米)
                'scan_topic': '/scan',
                'update_rate': 10.0,          # Hz
                'enable_visualization': enable_visualization,
            }],
            remappings=[
                ('/scan', '/scan'),
            ],
        ),
        
        # ============ 手动目标点设置节点 ============
        Node(
            package='navigation_control',
            executable='manual_goal_setter',
            name='manual_goal_setter',
            output='screen',
            parameters=[{
                'goal_tolerance': 0.3,
                'enable_visualization': enable_visualization,
            }],
        ),
        
        # ============ 全向轮控制器 ============
        Node(
            package='navigation_control',
            executable='omni_wheel_controller',
            name='omni_wheel_controller',
            output='screen',
            parameters=[{
                'max_vx': 1.0,                # 最大前后速度 m/s
                'max_vy': 1.0,                # 最大左右速度 m/s
                'max_wz': 2.0,                # 最大旋转速度 rad/s
                'velocity_scale': 1000.0,     # m/s -> mm/s
                'angular_scale': 1000.0,      # rad/s -> mrad/s
                'velocity_timeout': 1.0,      # 超时时间
                'smooth_factor': 0.7,         # 平滑系数
                'enable_lateral_motion': True,# 启用侧向运动
            }],
        ),
        
        # ============ 串口通信节点 ============
        Node(
            package='navigation_control',
            executable='serial_communication',
            name='serial_communication',
            output='screen',
            parameters=[{
                'serial_port': dev_board_port,
                'baudrate': dev_board_baudrate,
                'timeout_ms': 100,
                'auto_reconnect': True,
                'reconnect_interval': 5.0,
            }],
        ),
        
        # ============ 机器人状态管理器 ============
        Node(
            package='navigation_control',
            executable='robot_state_manager',
            name='robot_state_manager',
            output='screen',
        ),
        
        # ============ 建图管理器 ============
        Node(
            package='navigation_control',
            executable='mapping_manager',
            name='mapping_manager',
            output='screen',
        ),
        
        # ============ 定位管理器 ============
        Node(
            package='navigation_control',
            executable='localization_manager',
            name='localization_manager',
            output='screen',
        ),
        
        # ============ 导航控制器 ============
        Node(
            package='navigation_control',
            executable='navigation_controller',
            name='navigation_controller',
            output='screen',
            parameters=[{
                'navigation_timeout': 300.0,
                'goal_tolerance': 0.2,
            }],
        ),
        
        LogInfo(msg='所有节点已启动！'),
        LogInfo(msg=''),
        LogInfo(msg='====== 可用服务 ======'),
        LogInfo(msg='[目标点设置]'),
        LogInfo(msg='  ros2 service call /set_goal_0 std_srvs/srv/Trigger'),
        LogInfo(msg='  ros2 service call /set_goal_1 std_srvs/srv/Trigger'),
        LogInfo(msg='  ros2 service call /set_goal_2 std_srvs/srv/Trigger'),
        LogInfo(msg='  ros2 service call /set_goal_3 std_srvs/srv/Trigger'),
        LogInfo(msg='  ros2 service call /start_patrol std_srvs/srv/Trigger'),
        LogInfo(msg=''),
        LogInfo(msg='[紧急控制]'),
        LogInfo(msg='  ros2 service call /emergency_stop std_srvs/srv/Trigger'),
        LogInfo(msg='  ros2 service call /cancel_goal std_srvs/srv/Trigger'),
        LogInfo(msg=''),
        LogInfo(msg='====== 监控话题 ======'),
        LogInfo(msg='[障碍物距离]'),
        LogInfo(msg='  ros2 topic echo /obstacle_info'),
        LogInfo(msg='  ros2 topic echo /min_obstacle_distance'),
        LogInfo(msg='  ros2 topic echo /front_obstacle_distance'),
        LogInfo(msg=''),
        LogInfo(msg='[串口通信]'),
        LogInfo(msg='  ros2 topic echo /omni_tx_hex'),
        LogInfo(msg='  ros2 topic echo /serial_rx_hex'),
        LogInfo(msg=''),
        LogInfo(msg='[状态信息]'),
        LogInfo(msg='  ros2 topic echo /omni_status'),
        LogInfo(msg='  ros2 topic echo /serial_connection_status'),
    ])
