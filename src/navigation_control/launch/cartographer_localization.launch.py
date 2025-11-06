#!/usr/bin/env python3
"""
Cartographer 纯定位模式启动文件
功能:
1. 加载已保存的 .pbstream 地图
2. 使用 Cartographer 进行纯定位（不建图）
3. 提供比 AMCL 更准确的激光定位
4. 适合比赛场景：使用赛前建好的地图

使用方法:
1. 先用 mapping.launch.py 建图并保存 .pbstream 文件
2. 运行此文件进行定位和导航

保存地图命令:
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/path/to/map.pbstream'}"
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
        description='Development board serial port (Micro USB)'
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
        
        # ============ RPLIDAR 节点 ============
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
                ('/scan', '/scan_raw'),
            ],
        ),
        
        # ============ 激光扫描过滤器 ============
        Node(
            package='navigation_control',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'filter_angle_min': -2.42,
                'filter_angle_max': 2.84,
                'filter_range_max': 0.35,
                'input_topic': '/scan_raw',
                'output_topic': '/scan',
            }],
        ),
        
        # ============ 轮式里程计接收节点 ============
        Node(
            package='navigation_control',
            executable='wheel_odometry_receiver',
            name='wheel_odometry_receiver',
            output='screen',
            parameters=[{
                'publish_tf': True,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
        ),
        
        # ============ Cartographer 纯定位节点 ============
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
                '-start_trajectory_with_default_topics=false',  # 重要！
            ],
            remappings=[
                ('/scan', '/scan'),  # 使用过滤后的扫描
            ],
        ),
        
        # ============ Cartographer 占用栅格节点 ============
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
        
        # ============ TF 静态变换已由 robot.urdf 定义 ============
        # base_link → laser 的变换在 URDF 中定义：
        # - 位置: xyz="0.098 0.065 0.077"
        # - 旋转: rpy="0 0 1.5708" (绕Z轴旋转90度对齐雷达坐标系)
        
        # ============ Cartographer 初始位姿辅助节点 ============
        # 监听 RViz2 的 "2D Pose Estimate" 工具，辅助全局定位
        Node(
            package='navigation_control',
            executable='cartographer_initial_pose_setter.py',
            name='cartographer_initial_pose_setter',
            output='screen',
        ),
        
        # ============ 串口数据发布器 ============
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
        
        # ============ 串口通信 ============
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
