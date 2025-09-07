cmake_minimum_required(VERSION 3.5)
project(searcher_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosgraph_msgs REQUIRED)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

ament_python_install_package(${PROJECT_NAME})

add_executable(system_clock_publisher src/system_clock_publisher.cpp)
ament_target_dependencies(system_clock_publisher
  rclcpp
  rosgraph_msgs
)
install(TARGETS system_clock_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(servo_confirmation_node src/servo_confirmation_node.cpp)
ament_target_dependencies(servo_confirmation_node
  rclcpp
  std_msgs
  sensor_msgs
)
install(TARGETS servo_confirmation_node DESTINATION lib/${PROJECT_NAME})

add_executable(custom_servo_trajectory_server src/custom_servo_trajectory_server.cpp)
ament_target_dependencies(custom_servo_trajectory_server
  rclcpp
  rclcpp_action
  std_msgs
  control_msgs
)
install(TARGETS custom_servo_trajectory_server DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()