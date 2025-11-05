cmake_minimum_required(VERSION 3.8)
project(navigation_control)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)

# 串口库
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_util
)

# Include directories
include_directories(
  include
  ${SERIAL_INCLUDE_DIRS}
)

# Mapping Manager Node
add_executable(mapping_manager
  src/mapping_manager.cpp
)
ament_target_dependencies(mapping_manager ${dependencies})

# Localization Manager Node
add_executable(localization_manager
  src/localization_manager.cpp
)
ament_target_dependencies(localization_manager ${dependencies})

# Navigation Controller Node
add_executable(navigation_controller
  src/navigation_controller.cpp
)
ament_target_dependencies(navigation_controller ${dependencies})

# Serial Data Publisher Node (准备串口数据)
add_executable(serial_data_publisher
  src/serial_data_publisher.cpp
)
ament_target_dependencies(serial_data_publisher ${dependencies})

# Robot State Manager Node (机器人状态管理)
add_executable(robot_state_manager
  src/robot_state_manager.cpp
)
ament_target_dependencies(robot_state_manager ${dependencies})

# Serial Communication Node (实际串口通信)
add_executable(serial_communication
  src/serial_communication.cpp
)
ament_target_dependencies(serial_communication ${dependencies})
target_link_libraries(serial_communication ${SERIAL_LIBRARIES})

# Obstacle Monitor Node (障碍物监控)
add_executable(obstacle_monitor
  src/obstacle_monitor.cpp
)
ament_target_dependencies(obstacle_monitor ${dependencies})

# Manual Goal Setter Node (手动目标点设置)
add_executable(manual_goal_setter
  src/manual_goal_setter.cpp
)
ament_target_dependencies(manual_goal_setter ${dependencies})

# Omni Wheel Controller Node (全向轮控制器)
add_executable(omni_wheel_controller
  src/omni_wheel_controller.cpp
)
ament_target_dependencies(omni_wheel_controller ${dependencies})

# Install executables
install(TARGETS
  mapping_manager
  localization_manager
  navigation_controller
  serial_data_publisher
  robot_state_manager
  serial_communication
  obstacle_monitor
  manual_goal_setter
  omni_wheel_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install maps directory
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
