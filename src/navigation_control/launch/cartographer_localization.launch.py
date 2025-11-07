#!/usr/bin/env python3
"""
Cartographer 纯定位模式启动文件
功能:
1. 加载已保存的 .pbstream 地图
2. 使用 Cartographer 进行纯定位（不建图）
3. 提供比 AMCL 更准确的激光定位
4. 发布 map→odom TF（Cartographer 自动处理）

Cartographer vs AMCL 的区别:
- Cartographer: 激光SLAM定位，精度高，自动全局定位，发布 map→odom TF
- AMCL: 粒子滤波定位，需要初始位姿，需要手动设置

使用方法:
1. 先用 mapping.launch.py 建图
2. 保存地图: ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$(pwd)/my_map.pbstream'}"
3. 运行此文件进行定位和导航
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取配置文件路径
    nav_control_dir = get_package_share_directory('navigation_control')
    config_dir = os.path.join(nav_control_dir, 'config')
    maps_dir = os.path.join(nav_control_dir, 'maps')
    urdf_file = os.path.join(nav_control_dir, 'urdf', 'robot.urdf')
    
    # 读取 URDF 文件
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 声明启动参数
    pbstream_file_arg = DeclareLaunchArgument(
        'pbstream_file',
        default_value=os.path.join(maps_dir, 'my_map.pbstream'),
        description='Path to Cartographer .pbstream map file'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/radar',
        description='LIDAR serial port'
    )
    
    dev_board_port_arg = DeclareLaunchArgument(
        'dev_board_port',
        default_value='/dev/stm32',
        description='Development board serial port'
    )
    
    return LaunchDescription([
        pbstream_file_arg,
        lidar_port_arg,
        dev_board_port_arg,
        
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
        
        # ============ TF: odom → base_link (静态，因为没有轮式里程计) ============
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),
        
        # ============ 激光雷达 ============
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            remappings=[
                ('/scan', '/scan_raw'),  # 发布到 /scan_raw
            ],
        ),
        
        # ============ 激光扫描过滤器 (过滤机器人本体) ============
        Node(
            package='navigation_control',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'filter_angle_min': -2.42,  # -138.87°
                'filter_angle_max': 2.84,   # 163.00°
                'filter_range_max': 0.35,   # 只过滤 0.35m 以内
                'input_topic': '/scan_raw',
                'output_topic': '/scan',    # 输出到标准 /scan
            }],
        ),
        
        # ============ Cartographer 纯定位节点 ============
        # 功能: 加载 .pbstream 地图，进行激光定位
        # 发布: map → odom TF (自动全局定位)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_localization.lua',
                '-load_state_filename', LaunchConfiguration('pbstream_file'),
            ],
        ),
        
        # ============ Cartographer 占用栅格节点 (发布地图) ============
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05,
            }],
        ),
        
        # ============ 串口数据发布器 (接收 /cmd_vel) ============
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
        
        # ============ 串口通信 (发送到 STM32) ============
        Node(
            package='navigation_control',
            executable='serial_communication',
            name='serial_communication',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('dev_board_port'),
                'baudrate': 115200,
                'timeout_ms': 100,
                'auto_reconnect': True,
                'reconnect_interval': 5.0,
            }],
        ),
        
        # ============ RViz2 可视化 ============
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config_dir, 'navigation_debug.rviz')],
        ),
    ])
