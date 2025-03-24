cmake_minimum_required(VERSION 3.10)
project(cybership_thrusters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/voith_schneider.cpp
  src/fixed.cpp
  src/azimuth.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
