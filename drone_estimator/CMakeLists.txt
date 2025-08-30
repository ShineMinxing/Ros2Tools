cmake_minimum_required(VERSION 3.10)
project(drone_estimator)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)  # ★ 新增

set(SMX_DIR ${CMAKE_CURRENT_SOURCE_DIR}/SMXFusionEstimator)

set(SMX_C_SRCS
  ${SMX_DIR}/StateSpaceModel3.c
)
file(GLOB SMX_CPP_SRCS
  ${SMX_DIR}/Cpp_Estimators/*.cpp
)

add_library(smx_fusion_estimator STATIC
  ${SMX_C_SRCS}
  ${SMX_CPP_SRCS}
)
target_include_directories(smx_fusion_estimator PUBLIC
  ${SMX_DIR}
  ${SMX_DIR}/Cpp_Estimators
)
if (UNIX)
  target_link_libraries(smx_fusion_estimator m)
endif()

add_executable(drone_estimator_node
  src/drone_estimator_node.cpp
)
target_include_directories(drone_estimator_node PRIVATE
  ${SMX_DIR}
)
target_link_libraries(drone_estimator_node
  smx_fusion_estimator
)

ament_target_dependencies(drone_estimator_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
)

install(TARGETS
  smx_fusion_estimator
  drone_estimator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  SMXFusionEstimator/
  DESTINATION include/${PROJECT_NAME}/SMXFusionEstimator
  FILES_MATCHING PATTERN "*.h"
)

ament_package()
