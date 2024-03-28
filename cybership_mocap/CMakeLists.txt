cmake_minimum_required(VERSION 3.8)
project(cybership_mocap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)


add_library(cybership_mocap
  src/cybership_mocap.cpp
)
ament_target_dependencies(cybership_mocap
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
  std_msgs
  geometry_msgs
  mocap4r2_msgs
  nav_msgs
)

add_executable(cybership_mocap_node
  src/cybership_mocap_node.cpp
)

ament_target_dependencies(cybership_mocap_node
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
  std_msgs
  geometry_msgs
  mocap4r2_msgs
  nav_msgs
)
target_link_libraries(cybership_mocap_node
  cybership_mocap
)
install(
  DIRECTORY
    launch
    param
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS
    cybership_mocap
    cybership_mocap_node
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
