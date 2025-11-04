# SLAM ROS2 Workspace - AI Coding Agent Instructions

## Project Overview

This is a ROS2 workspace integrating SLAMTEC RPLIDAR hardware with Google Cartographer SLAM for 2D mapping and localization. The workspace contains two primary packages:

- **sllidar_ros2**: ROS2 driver for SLAMTEC RPLIDAR sensors (fork from Slamtec/sllidar_ros2)
- **my_slam_config**: Custom Cartographer SLAM configuration and launch files

## Architecture & Data Flow

1. **Hardware → Driver**: RPLIDAR connects via serial/TCP/UDP → `sllidar_node` publishes to `/scan` topic
2. **Driver → SLAM**: Cartographer subscribes to `/scan` → generates occupancy grid maps
3. **TF Tree**: Static transforms chain `odom` → `base_link` → `laser` (defined in `my_slam_config/launch/cartographer.launch.py`)

The system is configured for **IMU-less operation** - Cartographer's `use_imu_data` is explicitly disabled in `backpack_2d.lua`.

## Build System (Colcon + CMake)

**Standard build workflow from workspace root:**
```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**Key build details:**
- `sllidar_ros2/CMakeLists.txt` compiles vendored SDK from `sdk/` directory (glob pattern for arch/linux/*.cpp)
- Produces two executables: `sllidar_node` (main driver) and `sllidar_client` (test subscriber)
- `my_slam_config` only installs resources (launch files + Lua configs), no compilation
- Use `--symlink-install` for faster iteration on Python launch files

## Critical Device Configuration

**Serial permissions required before running:**
```bash
sudo chmod 777 /dev/ttyUSB0
# OR create udev rules (recommended):
cd src/sllidar_ros2 && source scripts/create_udev_rules.sh
```

**Baudrate mapping by LIDAR model** (from launch files):
- A1/A2M8: 115200
- A2M7/A2M12/A3/S1: 256000  
- S2/S2E/S3/T1: 1000000

Wrong baudrate = no data. Check launch files in `sllidar_ros2/launch/*_launch.py` for model-specific configs.

## Launch File Patterns

**Two launch patterns in sllidar_ros2:**
1. `sllidar_<model>_launch.py`: Driver only
2. `view_sllidar_<model>_launch.py`: Driver + RViz2 with preconfigured view

**Connection types supported:**
- Serial (default): `channel_type='serial'`, requires `serial_port` + `serial_baudrate`
- TCP: `channel_type='tcp'` (see `sllidar_s1_tcp_launch.py` example)
- UDP: `channel_type='udp'`

**Custom SLAM launch:**
- `my_slam_config/launch/cartographer.launch.py` hardcodes absolute path to config: `/home/dkjsiogu/slam/src/my_slam_config/config`
- **Important**: This path is user-specific and will break on different machines

## Cartographer Configuration

**Lua config stack** (`my_slam_config/config/backpack_2d.lua`):
```lua
include "map_builder.lua"          -- Cartographer base configs
include "trajectory_builder.lua"    -- (from Cartographer installation)

-- Key customizations:
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- IMU disabled
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10
MAP_BUILDER.use_trajectory_builder_2d = true
```

Frame configuration:
- `map_frame = "map"`
- `tracking_frame = "base_link"`  
- `published_frame = "odom"`
- `provide_odom_frame = false` (expects external odometry)

## ROS2 Services & Topics

**Published by sllidar_node:**
- Topic: `/scan` (sensor_msgs/LaserScan)

**Services** (src/sllidar_node.cpp:303-306):
- `start_motor` (std_srvs/Empty): Resume LIDAR scanning
- `stop_motor` (std_srvs/Empty): Pause LIDAR (preserves connection)

**Published by Cartographer:**
- `/map` (occupancy grid)
- `/submap_list`
- TF transforms for map→odom

## Common Issues & Debugging

**No scan data appearing:**
1. Check serial permissions: `ls -l /dev/ttyUSB0`
2. Verify correct baudrate for LIDAR model
3. Check node is running: `ros2 node list`
4. Monitor topic: `ros2 topic echo /scan`

**Cartographer not mapping:**
1. Verify TF tree is complete: `ros2 run tf2_tools view_frames`
2. Ensure `/scan` topic is publishing
3. Check Lua config path exists (absolute path issue)
4. Confirm `num_laser_scans = 1` in backpack_2d.lua

**Build failures:**
- Missing ROS2 dependencies: Install `rclcpp`, `sensor_msgs`, `std_srvs` via rosdep
- Cartographer not found: Install `ros-<distro>-cartographer-ros`

## Code Organization

```
src/
├── sllidar_ros2/          # LIDAR driver (upstream fork)
│   ├── src/
│   │   ├── sllidar_node.cpp      # Main node (publishes /scan)
│   │   └── sllidar_client.cpp    # Example subscriber
│   ├── sdk/                      # Vendored SLAMTEC SDK
│   ├── launch/                   # Model-specific launchers
│   └── rviz/                     # RViz configs
│
└── my_slam_config/        # Custom SLAM integration
    ├── config/
    │   └── backpack_2d.lua       # Cartographer tuning
    └── launch/
        └── cartographer.launch.py # SLAM + TF publishers
```

## Development Conventions

- **Launch files**: Python-based (ROS2 style), use `LaunchConfiguration` for parameters
- **SDK modifications**: Avoid editing `sdk/` - it's vendored upstream code
- **TF frames**: Follow REP-105 naming (`base_link`, `laser`, `odom`, `map`)
- **Hardcoded paths**: Aware that `cartographer.launch.py` has absolute path dependency

## Testing Workflow

**Quick verification:**
```bash
# Terminal 1: Launch LIDAR (example for A1)
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py

# Terminal 2: Start Cartographer
ros2 launch my_slam_config cartographer.launch.py

# Verify data flow
ros2 topic hz /scan
ros2 topic hz /map
```

## External Dependencies

- **Cartographer**: Expects system installation with standard Lua includes
- **SLAMTEC SDK**: Bundled in `sllidar_ros2/sdk/` (Linux arch only)
- **ROS2 distro**: Not pinned, tested on Humble/Foxy based on imports
