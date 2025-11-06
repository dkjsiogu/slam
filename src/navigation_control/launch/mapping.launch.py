"""
建图模式启动文件
启动组件:
1. RPLIDAR驱动
2. Cartographer SLAM
3. Mapping Manager
4. Serial Data Publisher
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
    
    # 声明启动参数
    lidar_model_arg = DeclareLaunchArgument(
        'lidar_model',
        default_value='a1',
        description='LIDAR model (a1, a2, s1, etc.)'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/radar',
        description='LIDAR serial port'
    )
    
    dev_board_port_arg = DeclareLaunchArgument(
        'dev_board_port',
        default_value='/dev/stm32',
        description='Development board serial port (Micro USB)'
    )
    
    dev_board_baudrate_arg = DeclareLaunchArgument(
        'dev_board_baudrate',
        default_value='115200',
        description='Development board baudrate'
    )
    
    return LaunchDescription([
        lidar_model_arg,
        serial_port_arg,
        dev_board_port_arg,
        dev_board_baudrate_arg,
        
        # RPLIDAR节点
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            remappings=[
                ('/scan', '/scan_raw'),  # 重命名原始扫描
            ],
        ),
        
        # 激光扫描过滤器 (过滤机器人本体)
        Node(
            package='navigation_control',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'filter_angle_min': -2.42,  # -138.87° (右后角)
                'filter_angle_max': 2.84,   # 163.00° (左后角)
                'filter_range_max': 0.35,   # 只过滤 0.35m 以内的点
                'input_topic': '/scan_raw',
                'output_topic': '/scan',  # 输出到标准话题
            }],
        ),
        
        # Cartographer节点 (使用过滤后的扫描数据)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_2d.lua'
            ],
        ),
        
        # Cartographer占用栅格节点
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05'],
        ),
        
        # TF静态变换 - base_link -> laser (移除odom->base_link，由里程计节点发布)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        # Wheel Odometry Node (轮式里程计 - 发布odom->base_link TF)
        Node(
            package='navigation_control',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'enable_crc_check': True,
            }],
        ),
        
        # Mapping Manager节点
        Node(
            package='navigation_control',
            executable='mapping_manager',
            name='mapping_manager',
            output='screen',
            parameters=[{
                'map_save_directory': os.path.join(nav_control_dir, 'maps'),
                'map_name': 'my_map',
                'auto_save_interval': 60.0,
            }],
        ),
        
        # Serial Data Publisher节点
        Node(
            package='navigation_control',
            executable='serial_data_publisher',
            name='serial_data_publisher',
            output='screen',
            parameters=[{
                'max_linear_vel': 1.0,
                'max_angular_vel': 2.0,
                'linear_scale': 1000.0,
                'angular_scale': 1000.0,
                'velocity_timeout': 1.0,
                'smooth_factor': 0.8,
            }],
        ),
        
        # Serial Communication节点 (Micro USB连接下位机)
        Node(
            package='navigation_control',
            executable='serial_communication',
            name='serial_communication',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('dev_board_port'),
                'baudrate': LaunchConfiguration('dev_board_baudrate'),
                'timeout_ms': 100,
            }],
        ),
        
        # Robot State Manager节点
        Node(
            package='navigation_control',
            executable='robot_state_manager',
            name='robot_state_manager',
            output='screen',
        ),
        
        # RViz2可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(config_dir, 'mapping.rviz')],
        ),
    ])
