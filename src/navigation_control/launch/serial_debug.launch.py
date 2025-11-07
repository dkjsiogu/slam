#!/usr/bin/env python3
"""
串口调试工具启动文件
用于查看和调试下位机发送的原始数据

功能:
1. 启动串口通信节点 - 接收下位机数据
2. 启动轮式里程计节点 - 解析并发布 /odom 和 TF
3. 启动串口监控节点 - 实时显示接收到的数据
4. 显示轮式里程计数据 (dx, dy, dw, dt)
5. RViz2 可视化 - 显示里程计轨迹和机器人坐标系

使用场景:
- 调试串口通信
- 验证数据格式和 CRC
- 检查里程计精度
- 监控通信频率
- 可视化机器人轨迹

使用方法:
    ros2 launch navigation_control serial_debug.launch.py
    ros2 launch navigation_control serial_debug.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 获取包路径和配置文件
    nav_control_dir = get_package_share_directory('navigation_control')
    config_dir = os.path.join(nav_control_dir, 'config')
    urdf_file = os.path.join(nav_control_dir, 'urdf', 'robot.urdf')
    rviz_config = os.path.join(config_dir, 'odometry_debug.rviz')
    
    # 读取 URDF 文件
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 声明启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/stm32',
        description='STM32 serial port (udev rule or /dev/ttyUSB0)'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate'
    )
    
    enable_crc_arg = DeclareLaunchArgument(
        'enable_crc_check',
        default_value='false',
        description='Enable CRC16 validation (disable for debugging)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        enable_crc_arg,
        use_rviz_arg,
        
        # ============ 机器人模型发布 ============
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
        ),
        
        # ============ 串口通信节点 (双向通信) ============
        Node(
            package='navigation_control',
            executable='serial_communication',
            name='serial_communication',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'timeout_ms': 100,
                'auto_reconnect': True,
                'reconnect_interval': 5.0,
            }],
            remappings=[
                # serial_communication 订阅 serial_tx_data 发送给下位机
                # serial_communication 发布 serial_rx_data 从下位机接收
            ],
        ),
        
        # ============ 串口数据发布器 (发送控制命令) ============
        # 订阅 /cmd_vel，转换为下位机协议并发送
        Node(
            package='navigation_control',
            executable='serial_data_publisher',
            name='serial_data_publisher',
            output='screen',
            parameters=[{
                'max_vx': 1.0,
                'max_vy': 1.0,
                'max_wz': 2.0,
                'velocity_timeout': 1.0,
                'smooth_factor': 0.7,
            }],
        ),
        
        # ============ 轮式里程计节点 ============
        Node(
            package='navigation_control',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'enable_crc_check': LaunchConfiguration('enable_crc_check'),
            }],
        ),
        
        # ============ 串口数据监控节点 ============
        Node(
            package='navigation_control',
            executable='serial_monitor.py',
            name='serial_monitor',
            output='screen',
        ),
        
        # ============ 里程计轨迹发布器 ============
        # 将 /odom 转换为 Path 消息用于 RViz 显示
        Node(
            package='navigation_control',
            executable='odom_to_path.py',
            name='odom_to_path',
            output='screen',
        ),
        
        # ============ RViz2 可视化 ============
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
