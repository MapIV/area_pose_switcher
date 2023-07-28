cmake_minimum_required(VERSION 3.5)
project(pose_switcher)

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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
# find_package(ndt_scan_matcher REQUIRED)

# ndt_scan_matcher
include_directories(
    include
)
# Declare a C++ executable
add_executable(pose_switcher
  src/pose_switcher.cpp
  src/util_func.cpp
  src/pose_array_interpolator.cpp
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(pose_switcher rclcpp std_msgs geometry_msgs tf2_ros tf2_eigen tf2_geometry_msgs std_msgs tier4_debug_msgs)

install(TARGETS
  pose_switcher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Exporting dependencies and setup for ament
ament_package()
