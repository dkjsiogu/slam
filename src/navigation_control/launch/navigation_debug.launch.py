#!/usr/bin/env python3
"""
导航调试启动文件 (已集成轮式里程计)
功能:
1. RPLIDAR驱动 + 扫描过滤
2. 轮式里程计 (发布 /odom 和 TF: odom->base_link)
3. 地图服务器 (加载已保存的地图)
4. AMCL定位 (map->odom TF)
5. 简单导航控制器
6. 串口通信 (全向轮控制)
7. RViz2可视化

TF树: map -> odom (AMCL) -> base_link (wheel_odom) -> laser (URDF)

使用方法:
    ros2 launch navigation_control navigation_debug.launch.py
    ros2 launch navigation_control navigation_debug.launch.py map_file:=/path/to/map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    nav_control_dir = get_package_share_directory('navigation_control')
    config_dir = os.path.join(nav_control_dir, 'config')
    urdf_file = os.path.join(nav_control_dir, 'urdf', 'robot.urdf')
    
    # 读取 URDF 文件
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 地图文件路径 (用户手动保存的地图)
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/pfa5/slam/src/navigation_control/maps/my_map.yaml',
        description='Path to the map file'
    )
    
    dev_board_port_arg = DeclareLaunchArgument(
        'dev_board_port',
        default_value='/dev/stm32',
        description='STM32 serial port'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/radar',
        description='LIDAR serial port'
    )
    
    # 初始位姿参数 (应与地图原点对应，或在地图中心)
    # 地图: origin=[-3.3, -5.4], size=209x220, resolution=0.05
    # 地图中心约为: x = -3.3 + (209*0.05)/2 = 1.925, y = -5.4 + (220*0.05)/2 = 0.1
    initial_x_arg = DeclareLaunchArgument(
        'initial_x',
        default_value='1.9',  # 地图中心 X
        description='Initial X position in map frame'
    )
    
    initial_y_arg = DeclareLaunchArgument(
        'initial_y',
        default_value='0.1',  # 地图中心 Y
        description='Initial Y position in map frame'
    )
    
    initial_yaw_arg = DeclareLaunchArgument(
        'initial_yaw',
        default_value='0.0',
        description='Initial yaw angle in radians'
    )
    
    return LaunchDescription([
        map_file_arg,
        dev_board_port_arg,
        lidar_port_arg,
        initial_x_arg,
        initial_y_arg,
        initial_yaw_arg,
        
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
        
        # ============ 轮式里程计节点 (发布 /odom 和 TF: odom->base_link) ============
        Node(
            package='navigation_control',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'enable_crc_check': False,  # 导航时关闭CRC检查以提高容错性
            }],
        ),
        
        # ============ 串口通信 (双向通信) ============
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
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
            remappings=[
                ('/scan', '/scan_raw'),  # 重命名原始扫描
            ],
        ),
        
        # ============ 激光扫描过滤器 (过滤机器人本体) ============
        # 雷达倒装(X朝后Y朝右)，需过滤机器人后方本体
        Node(
            package='navigation_control',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'filter_angle_min': -2.30,  # -132° (左后角，雷达坐标系)
                'filter_angle_max': 2.69,   # 154° (右后角，雷达坐标系)
                'filter_range_max': 0.35,   # 只过滤 0.35m 以内的点
                'input_topic': '/scan_raw',
                'output_topic': '/scan',  # 输出到标准话题，替换原始扫描
            }],
        ),
        
        # ============ 地图服务器 ============
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': LaunchConfiguration('map_file'),
            }],
        ),
        
        # Map Server Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server']
            }],
        ),
        
        # ============ AMCL 定位 ============
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'scan_topic': '/scan',  # 使用过滤后的扫描 (已经替换)
                # 粒子滤波参数
                'min_particles': 500,
                'max_particles': 2000,
                'update_min_d': 0.2,
                'update_min_a': 0.5,
                # 增加全局定位能力
                'recovery_alpha_slow': 0.001,
                'recovery_alpha_fast': 0.1,
                # 激光模型参数
                'laser_max_range': 12.0,
                'laser_min_range': 0.15,
                'max_beams': 60,
                'laser_model_type': 'likelihood_field',
                # 初始位姿分散
                'set_initial_pose': False,
                'initial_pose.x': 0.0,
                'initial_pose.y': 0.0,
                'initial_pose.yaw': 0.0,
                # TF 发布
                'tf_broadcast': True,
                'transform_tolerance': 1.0,
            }],
        ),
        
        # AMCL Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['amcl']
            }],
        ),
        
        # ============ 地图重发布节点 (解决 RViz2 QoS 问题) ============
        Node(
            package='navigation_control',
            executable='map_republisher.py',
            name='map_republisher',
            output='screen',
        ),
        
        # ============ 自动设置初始位姿 ============
        Node(
            package='navigation_control',
            executable='initial_pose_setter.py',
            name='initial_pose_setter',
            output='screen',
            parameters=[{
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                'delay': 3.0,  # 延迟3秒，等待 AMCL 启动
            }]
        ),
        
        # ============ 简单导航控制器 (计算vx, vy) ============
        Node(
            package='navigation_control',
            executable='simple_goal_controller.py',
            name='simple_goal_controller',
            output='screen',
            parameters=[{
                'max_linear_vel': 0.5,
                'max_angular_vel': 1.0,
                'goal_tolerance': 0.1,
                'lookahead_distance': 0.3,
            }]
        ),
        
        # ============ 全向轮控制器 (cmd_vel -> 下位机协议) ============
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
        
        # ============ RViz2 可视化 ============
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config_dir, 'navigation_debug.rviz')],
        ),
    ])
