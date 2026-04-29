cmake_minimum_required(VERSION 3.8)
project(damiao_hardware_interface)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ros2_control build helpers
find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# Dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  hardware_interface
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(pinocchio REQUIRED)
find_package(controller_manager 4.0.0 REQUIRED)

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Library
add_library(${PROJECT_NAME} SHARED
  src/damiao_hardware_interface.cpp
  src/motor_control.cpp
  src/motor.cpp
  src/serial_port.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  pinocchio::pinocchio
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface hardware_pluginlib.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Exports
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})
ament_package()