cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(corrected_pose_publisher src/corrected_pose_publisher.cpp)
add_executable(corrected_pose_publisher_imu src/corrected_pose_publisher_imu.cpp)

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

# target_link_libraries(path_planner_node
#   ${OpenCV_LIBRARIES}
#   Eigen3::Eigen
# )

target_link_libraries(corrected_pose_publisher
  Eigen3::Eigen
)

install(TARGETS corrected_pose_publisher corrected_pose_publisher_imu
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_library(path_planner SHARED src/path_planner.cpp)

ament_target_dependencies(path_planner
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

target_include_directories(
  path_planner PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

rclcpp_components_register_node(
  path_planner PLUGIN "path_planner::PathPlanner"
  EXECUTABLE path_planner_node)

install(
  TARGETS path_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  # add_subdirectory(test)
endif()

ament_package()
