cmake_minimum_required(VERSION 3.10)
project(cybership_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR src/${PROJECT_NAME}
)

file(GLOB NODE_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/nodes/*.py)

install(
  PROGRAMS
    ${NODE_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()