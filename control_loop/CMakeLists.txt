cmake_minimum_required(VERSION 3.5)
project(control_loop)

# 默认使用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 编译可执行文件
add_executable(control_loop_node_250517 src/control_loop_node_250517.cpp)
ament_target_dependencies(control_loop_node_250517
  rclcpp std_msgs nav_msgs tf2_geometry_msgs)

# 安装
install(
  TARGETS 
    control_loop_node_250517
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
