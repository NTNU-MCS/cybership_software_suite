cmake_minimum_required(VERSION 3.10)
project(cybership_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    bin/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
