"""
导航模式启动文件
启动组件:
1. RPLIDAR驱动
2. Map Server (加载已保存的地图)
3. AMCL定位
4. Nav2导航栈
5. Localization Manager
6. Navigation Controller
7. Serial Data Publisher
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    nav_control_dir = get_package_share_directory('navigation_control')
    config_dir = os.path.join(nav_control_dir, 'config')
    maps_dir = os.path.join(nav_control_dir, 'maps')
    
    # 声明启动参数
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(maps_dir, 'my_map.yaml'),
        description='Path to map file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
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
    
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
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
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
        
        # Map Server节点
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
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
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server']
            }],
        ),
        
        # AMCL定位节点
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(config_dir, 'amcl.yaml')],
        ),
        
        # AMCL Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['amcl']
            }],
        ),
        
        # Nav2 BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(config_dir, 'nav2_params.yaml')],
        ),
        
        # Nav2 Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'nav2_params.yaml')],
        ),
        
        # Nav2 Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'nav2_params.yaml')],
        ),
        
        # Nav2 Recoveries Server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'nav2_params.yaml')],
        ),
        
        # Nav2 Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': [
                    'bt_navigator',
                    'controller_server',
                    'planner_server',
                    'recoveries_server'
                ]
            }],
        ),
        
        # TF静态变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        # Localization Manager节点
        Node(
            package='navigation_control',
            executable='localization_manager',
            name='localization_manager',
            output='screen',
            parameters=[{
                'map_file': LaunchConfiguration('map_file'),
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_yaw': 0.0,
                'localization_quality_threshold': 0.5,
            }],
        ),
        
        # Navigation Controller节点
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
            arguments=['-d', os.path.join(config_dir, 'navigation.rviz')],
        ),
    ])
