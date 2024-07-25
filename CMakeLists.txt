cmake_minimum_required(VERSION 3.8)
project(turtlebot_motorBridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Boost
)

# Find dependencies
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(rclcpp_lifecycle REQUIRED)
#find_package(pluginlib REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(hardware_interface REQUIRED)
#find_package(Boost REQUIRED COMPONENTS thread)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  turtlebot_motorBridge
  SHARED
  src/low_level_bridge.cpp
)
target_compile_features(turtlebot_motorBridge PUBLIC cxx_std_17)
target_include_directories(turtlebot_motorBridge PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
$<INSTALL_INTERFACE:include/turtlebot_motorBridge>
)
ament_target_dependencies(
  turtlebot_motorBridge PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Include directories
#include_directories(include)

# Add executable
#add_executable(low_level_bridge src/low_level_bridge.cpp)

# Link libraries
#ament_target_dependencies(low_level_bridge 
#  rclcpp 
#  std_msgs 
#  tf2 
#  tf2_geometry_msgs
#  Boost
#)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface turtlebot_motorBridge.xml)

install(TARGETS
  turtlebot_motorBridge
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