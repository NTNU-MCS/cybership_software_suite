cmake_minimum_required(VERSION 3.8)
project(cybership_observer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(ned_to_enu_transformer src/ned_to_enu_transformer.cpp)
ament_target_dependencies(ned_to_enu_transformer
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
)

install(
  PROGRAMS
    nodes/ned_to_enu_transformer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    ned_to_enu_transformer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
