cmake_minimum_required(VERSION 3.5)
project(control_message)

find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(std_msgs      REQUIRED)

add_executable(control_message_node src/control_message_node.cpp)
ament_target_dependencies(control_message_node
  rclcpp sensor_msgs std_msgs)

install(TARGETS control_message_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
