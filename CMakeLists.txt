cmake_minimum_required(VERSION 3.0.2)
project(pose_switcher)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_switcher
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pose_switcher
src/pose_switcher_node.cpp
src/pose_switcher.cpp)
add_dependencies(pose_switcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_switcher ${catkin_LIBRARIES})

install(TARGETS
 pose_switcher
 DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
            launch
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

