cmake_minimum_required(VERSION 3.10)
project(cybership_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cybership_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR src/${PROJECT_NAME}
)

# Install LOSGuidance action server node
file(GLOB LOS_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/nodes/*.py")
install(
  PROGRAMS
    ${LOS_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()