#!/usr/bin/env python3
"""
联合启动文件 - 颜色跟踪 + 串口通信
同时启动 color_tracking_node 和 serial_data_publisher
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    navigation_control_dir = get_package_share_directory('navigation_control')
    
    # 声明参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='串口设备路径'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='串口波特率'
    )
    
    # 颜色跟踪节点
    color_tracking_node = Node(
        package='color_tracking_node',
        executable='color_tracking_node',
        name='color_tracking_node',
        output='screen',
        parameters=[{
            'debug': True
        }]
    )
    
    # 串口数据发布节点
    serial_publisher = Node(
        package='navigation_control',
        executable='serial_data_publisher',
        name='serial_data_publisher',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'max_vx': 1.0,
            'max_vy': 1.0,
            'max_wz': 2.0,
            'velocity_timeout': 0.5,
            'smooth_factor': 0.3
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        color_tracking_node,
        serial_publisher,
    ])
