cmake_minimum_required(VERSION 3.8)
project(damiao_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  hardware_interface
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
"Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(pinocchio REQUIRED)
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  damiao_hardware_interface
  SHARED
  src/damiao_hardware_interface.cpp
  src/motor_control.cpp
  src/motor.cpp
  src/serial_port.cpp
)

# target_include_directories(damiao_hardware_interface
#   SYSTEM PRIVATE
#   src/damiao_api
# )

target_compile_features(damiao_hardware_interface PUBLIC cxx_std_17)
target_include_directories(damiao_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/damiao_hardware_interface>
)
target_link_libraries(damiao_hardware_interface PUBLIC
  ${trajectory_msgs_TARGETS}
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
  pinocchio::pinocchio
)


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_pluginlib.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/damiao_hardware_interface
)

install(TARGETS damiao_hardware_interface
  EXPORT export_damiao_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_damiao_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()
