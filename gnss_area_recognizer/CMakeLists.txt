cmake_minimum_required(VERSION 3.5)
project(gnss_area_filter)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME})

ament_auto_add_library(lib_${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_gtest(test_${PROJECT_NAME} ${test_files})

  target_link_libraries(test_${PROJECT_NAME}
    lib_${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
