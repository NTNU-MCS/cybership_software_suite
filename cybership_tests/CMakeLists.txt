cmake_minimum_required(VERSION 3.10)
project(cybership_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(joy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


ament_python_install_package(${PROJECT_NAME})

# Find all python scripts under the "nodes" directory.
file(GLOB NODE_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/cybership_tests/*.py)

install(
  PROGRAMS
    ${NODE_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()