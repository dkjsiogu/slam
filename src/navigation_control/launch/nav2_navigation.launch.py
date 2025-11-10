#!/usr/bin/env python3
"""
完整导航系统 - Cartographer定位 + Nav2路径规划 + 避障
功能:
1. RPLIDAR驱动
2. 轮式里程计
3. Cartographer纯定位（加载已保存地图）
4. Nav2完整导航栈（路径规划 + 避障）
5. 串口通信

使用方法:
1. 先用 mapping.launch.py 建图并保存
2. 运行此文件进行导航:
   ros2 launch navigation_control nav2_navigation.launch.py
3. 在RViz中使用 "2D Goal Pose" 设置目标
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    nav_control_dir = get_package_share_directory('navigation_control')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    config_dir = os.path.join(nav_control_dir, 'config')
    maps_dir = os.path.join(nav_control_dir, 'maps')
    urdf_file = os.path.join(nav_control_dir, 'urdf', 'robot.urdf')
    
    # 读取 URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 参数
    pbstream_file_arg = DeclareLaunchArgument(
        'pbstream_file',
        default_value=os.path.join(maps_dir, 'my_map.pbstream'),
        description='Cartographer地图文件'
    )
    
    nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(config_dir, 'nav2_params.yaml'),
        description='Nav2参数文件'
    )
    
    return LaunchDescription([
        pbstream_file_arg,
        nav2_params_arg,
        
        # ============ 机器人模型 ============
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        
        # ============ 轮式里程计 ============
        Node(
            package='navigation_control',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'enable_crc_check': False,
            }],
        ),
        
        # ============ 串口通信 ============
        Node(
            package='navigation_control',
            executable='serial_communication',
            name='serial_communication',
            output='screen',
            parameters=[{
                'serial_port': '/dev/stm32',
                'baudrate': 115200,
            }],
        ),
        
        # ============ 激光雷达 ============
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/radar',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
            }],
            remappings=[('/scan', '/scan_raw')],
        ),
        
        # ============ 扫描过滤 ============
        Node(
            package='navigation_control',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'filter_angle_min': -2.30,
                'filter_angle_max': 2.69,
                'filter_range_max': 0.35,
                'input_topic': '/scan_raw',
                'output_topic': '/scan',
            }],
        ),
        
        # ============ Cartographer 纯定位 ============
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_localization.lua',
                '-load_state_filename', LaunchConfiguration('pbstream_file'),
            ],
        ),
        
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}],
        ),
        
        # ============ 地图重发布 ============
        Node(
            package='navigation_control',
            executable='map_republisher.py',
            name='map_republisher',
            output='screen',
        ),
        
        # ============ Nav2 导航栈 ============
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true',
            }.items(),
        ),
        
        # ============ 全向轮控制器 ============
        Node(
            package='navigation_control',
            executable='serial_data_publisher',
            name='serial_data_publisher',
            output='screen',
            parameters=[{
                'max_vx': 0.5,
                'max_vy': 0.5,
                'max_wz': 1.0,
            }],
        ),
        
        # ============ RViz2 ============
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config_dir, 'navigation_debug.rviz')],
        ),
    ])
