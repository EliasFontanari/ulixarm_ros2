cmake_minimum_required(VERSION 3.16)
project(damiao_motor_control_standalone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(pybind11 REQUIRED)

add_library(damiao_core STATIC
  ${CMAKE_CURRENT_LIST_DIR}/../src/motor_control.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../src/motor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../src/serial_port.cpp
)

target_include_directories(damiao_core PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/../include
)

pybind11_add_module(damiao_motor_control
  ${CMAKE_CURRENT_LIST_DIR}/../src/motor_control_pybind.cpp
)

target_include_directories(damiao_motor_control PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../include
)

target_link_libraries(damiao_motor_control PRIVATE
  damiao_core
)
