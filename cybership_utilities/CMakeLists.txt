cmake_minimum_required(VERSION 3.10)
project(cybership_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(
  DIRECTORY
    launch html
  DESTINATION
    share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/force_mux_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()