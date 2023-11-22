cmake_minimum_required(VERSION 3.5)
project(pointcloud_relay)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tier4_localization_msgs REQUIRED)

# Declare a C++ executable
add_executable(pointcloud_relay_node src/pointcloud_relay.cpp)

# Specify dependencies for the executable
ament_target_dependencies(pointcloud_relay_node
  rclcpp
  sensor_msgs
  tier4_localization_msgs)

install(TARGETS
  pointcloud_relay_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# Export dependencies
ament_package()

