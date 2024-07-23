cmake_minimum_required(VERSION 3.8)
project(turtlebot_motorBridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

add_library(diffdrive_arduino SHARED src/diffdrive_turtlebot.cpp src/wheel.cpp src/esp_comms.cpp)

# Include directories
include_directories(include)

# Add executable
# add_executable(low_level_bridge src/low_level_bridge.cpp)

target_include_directories(
  diffdrive_turtlebot
  PRIVATE
  include
)

# Link libraries
ament_target_dependencies(
  diffdrive_turtlebot 
  rclcpp 
  hardware_interface
  std_msgs 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_geometry_msgs
  Boost
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

add_library(fake_robot SHARED src/fake_robot.cpp src/wheel.cpp)
set_property(TARGET fake_robot PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(
  fake_robot
  PRIVATE
  include
)
ament_target_dependencies(
  fake_robot
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

install(TARGETS
  diffdrive_turtlebot fake_robot
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()