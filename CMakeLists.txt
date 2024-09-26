cmake_minimum_required(VERSION 3.5)
project(path_planner)

# 必要なパッケージを検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# インクルードディレクトリの設定
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 実行可能ファイルの追加
add_executable(path_planner_node src/path_planner_node.cpp)

# 依存関係の設定
ament_target_dependencies(path_planner_node
  rclcpp
  mavros_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  cv_bridge
)

# ライブラリのリンク
target_link_libraries(path_planner_node
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

# インストール設定
install(TARGETS path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
