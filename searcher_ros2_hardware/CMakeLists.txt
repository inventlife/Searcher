cmake_minimum_required(VERSION 3.16)
project(searcher_ros2_hardware LANGUAGES CXX)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


find_package(controller_manager REQUIRED)
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the right version of searcher_ros2_hardware?")
endif()

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  searcher_ros2_hardware
  SHARED
  hardware/searcher_system.cpp
)
target_compile_options(searcher_ros2_hardware PRIVATE -Wall -Wextra -Wpedantic)
target_compile_features(searcher_ros2_hardware PUBLIC cxx_std_17)
target_include_directories(searcher_ros2_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware>
$<INSTALL_INTERFACE:include/searcher_ros2_hardware>
)

find_package(hardware_interface REQUIRED)

target_include_directories(searcher_ros2_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware>
  $<INSTALL_INTERFACE:include/searcher_ros2_hardware>
)
ament_target_dependencies(searcher_ros2_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface searcher_ros2_hardware.xml)

install(TARGETS searcher_ros2_hardware
  EXPORT export_searcher_ros2_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  FILES searcher_ros2_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY hardware
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

endif()

ament_export_targets(export_searcher_ros2_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
