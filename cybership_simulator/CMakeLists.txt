cmake_minimum_required(VERSION 3.10)
project(cybership_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cybership_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/${PROJECT_NAME}/voyager.py
    src/${PROJECT_NAME}/enterprise.py
    src/${PROJECT_NAME}/drillship.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
