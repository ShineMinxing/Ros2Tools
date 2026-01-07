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
# 既然你之前的代码能跑，tf2 可能是作为 tf2_geometry_msgs 的传递依赖存在的
# 但为了保险，建议加上 find_package(tf2 REQUIRED)

add_executable(control_loop_node_250517 src/control_loop_node_250517.cpp)
ament_target_dependencies(control_loop_node_250517 
  rclcpp std_msgs nav_msgs tf2_geometry_msgs
)

add_executable(control_loop_node_260107 src/control_loop_node_260107.cpp)
ament_target_dependencies(control_loop_node_260107 
  rclcpp std_msgs nav_msgs tf2_geometry_msgs
)

# 安装
install(
  TARGETS 
  control_loop_node_250517
  control_loop_node_260107
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()