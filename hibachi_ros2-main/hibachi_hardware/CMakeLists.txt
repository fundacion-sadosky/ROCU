cmake_minimum_required(VERSION 3.8)
project(hibachi_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)

## COMPILE
add_library(
  hibachi_hardware
  SHARED
  src/hibachi_hardware.cpp
  src/serial_port.cpp
  src/message.cpp
)

target_include_directories(
  hibachi_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  hibachi_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

pluginlib_export_plugin_description_file(hardware_interface hibachi_hardware.xml)

# INSTALL
install(
  TARGETS hibachi_hardware
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
