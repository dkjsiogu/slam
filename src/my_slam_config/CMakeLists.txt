cmake_minimum_required(VERSION 3.8)
project(my_slam_config)

# 添加对Python启动文件的支持
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装Python启动文件
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装Lua配置文件
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 标准配置（保留原有内容）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()