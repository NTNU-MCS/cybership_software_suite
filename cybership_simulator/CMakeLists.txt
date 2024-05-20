cmake_minimum_required(VERSION 3.8)
project(cybership_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
ament_cmake_symlink_install_directory(DIRECTORY ${PROJECT_SOURCE_DIR})
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    ${PROJECT_NAME}/cybership_enterprise.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
