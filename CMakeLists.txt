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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# インクルードディレクトリの設定
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 実行可能ファイルの追加
add_executable(path_planner_node src/path_planner_node.cpp)
add_executable(corrected_pose_publisher src/corrected_pose_publisher.cpp)
add_executable(corrected_pose_publisher_imu src/corrected_pose_publisher_imu.cpp)

# 依存関係の設定
ament_target_dependencies(path_planner_node
  rclcpp
  mavros_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  cv_bridge
)

ament_target_dependencies(corrected_pose_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

ament_target_dependencies(corrected_pose_publisher_imu
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

# ライブラリのリンク
target_link_libraries(path_planner_node
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(corrected_pose_publisher
  Eigen3::Eigen
)

# インストール設定
install(TARGETS path_planner_node corrected_pose_publisher corrected_pose_publisher_imu
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
