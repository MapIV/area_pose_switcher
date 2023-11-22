cmake_minimum_required(VERSION 3.5)
project(area_visualizar)

# ROS 2パッケージの依存関係を探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_mapping_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 実行可能ファイルを追加
add_executable(area_node src/area.cpp)

# 実行可能ファイルに依存関係を関連付ける
ament_target_dependencies(area_node
  rclcpp
  autoware_auto_mapping_msgs
  geometry_msgs
  visualization_msgs
)

include_directories(
    include
)

# インストール指示
install(TARGETS
  area_node
  DESTINATION lib/${PROJECT_NAME}
)

# 必要な場合は追加のインクルードディレクトリやライブラリをここに追加

# amentパッケージの設定
ament_package()