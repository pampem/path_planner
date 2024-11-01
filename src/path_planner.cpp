/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#include "path_planner/path_planner.hpp"

#include "path_planner/rotational_force.hpp"

namespace path_planner
{

PathPlanner::PathPlanner(const rclcpp::NodeOptions & options)
: Node("path_planner", options),
  tf_buffer_(this->get_clock(), tf2::Duration(std::chrono::seconds(10))),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
  this->declare_parameter("attractive_force.max_distance", 3.0F);
  this->declare_parameter("attractive_force.gain", 0.5F);
  this->declare_parameter("repulsive_force.min_distance", 2.0F);
  this->declare_parameter("repulsive_force.gain", 1.0F);

  this->get_parameter("attractive_force.max_distance", attractive_force_max_distance_);
  this->get_parameter("attractive_force.gain", attractive_force_gain_);
  this->get_parameter("repulsive_force.min_distance", repulsive_force_min_distance_);
  this->get_parameter("repulsive_force.gain", repulsive_force_gain_);

  slam_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "glim_ros/pose", 10, std::bind(&PathPlanner::slam_pose_callback, this, std::placeholders::_1));

  gridmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "gridmap", 10, std::bind(&PathPlanner::gridmap_callback, this, std::placeholders::_1));

  robot_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "drone1/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

  current_robot_position_publisher_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("current_robot_position_gridmap", 10);

  RCLCPP_INFO(this->get_logger(), "PathPlanner node has been initialized.");

  // 仮に置いてある。
  target_position_.x() = -5;
  target_position_.y() = 10;
}

// GridMap Callback
void PathPlanner::gridmap_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received gridmap data.");
  current_gridmap_ = *msg;
}

// SLAM Pose Callback
void PathPlanner::slam_pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped current_pose = *msg;

  current_position_(current_pose.pose.position.x, current_pose.pose.position.y);

  Eigen::Vector2f attractive_force = calculate_attractive_force(
    current_position_, target_position_, attractive_force_gain_, attractive_force_max_distance_);

  Eigen::Vector2f repulsive_force = calculate_repulsive_force(current_position_);

  Eigen::Vector2f total_force = attractive_force + repulsive_force;

  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = total_force.x();
  twist_msg.linear.y = total_force.y();

  robot_velocity_publisher_->publish(twist_msg);

  RCLCPP_INFO(
    this->get_logger(),
    "Published velocity command towards goal: [linear.x: %f, linear.y: %f], "
    "Attractive Force: [x: %f, y: %f], Repulsive Force: [x: %f, y: %f]",
    twist_msg.linear.x, twist_msg.linear.y, attractive_force.x(), attractive_force.y(),
    repulsive_force.x(), repulsive_force.y());
}

// Goalに引かれる力を計算する関数。この力はそのCellにいるときだけ計算すればいいので、行列の形式で保存しておく必要はない。
Eigen::Vector2f PathPlanner::calculate_attractive_force(
  const Eigen::Vector2f & current_position, const Eigen::Vector2f & goal_position,
  float attractive_gain, float attractive_force_max_distance)
{
  // Goalまでの方向ベクトルを計算 vector2f最高!
  Eigen::Vector2f direction = goal_position - current_position;
  // Goalまでの距離
  float distance = direction.norm();
  // 距離がmax_distance以上の場合は力が小さくなりすぎないように制限
  if (distance > attractive_force_max_distance) {
    distance = attractive_force_max_distance;
  }
  // 引力を計算 normalized()では単位ベクトルを生成しているよ。
  // つまり、attractive_force = Gain*単位方向ベクトル*距離
  Eigen::Vector2f attractive_force = attractive_gain * direction.normalized() * distance;

  return attractive_force;
}

Eigen::Vector2f PathPlanner::calculate_repulsive_force(const Eigen::Vector2f & current_position)
{
  int width = current_gridmap_.info.width;
  int height = current_gridmap_.info.height;
  float resolution = current_gridmap_.info.resolution;

  if (repulsive_forces_.rows() != width || repulsive_forces_.cols() != height) {
    repulsive_forces_.resize(width, height);
  }

  // グリッドマップをスキャンして障害物の位置を行列に保存
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;
      int8_t cell_value = current_gridmap_.data[index];

      // 障害物があるセルにのみ反発力ベクトルを割り当てる
      // 障害物があるセルからロボットに向かう力が生成される。
      if (cell_value > 0) {
        float cell_x = current_gridmap_.info.origin.position.x + x * resolution;
        float cell_y = current_gridmap_.info.origin.position.y + y * resolution;
        repulsive_forces_(x, y) = Eigen::Vector2f(cell_x, cell_y);
      } else {
        repulsive_forces_(x, y) = Eigen::Vector2f(0.0F, 0.0F);
      }
    }
  }

  Eigen::Vector2f total_repulsive_force(0.0F, 0.0F);

  for (int i = 0; i < repulsive_forces_.rows(); ++i) {
    for (int j = 0; j < repulsive_forces_.cols(); ++j) {
      Eigen::Vector2f cell_position = repulsive_forces_(i, j);
      // ここだなあ。current_positionとcell_positionの座標系一致が求められている
      Eigen::Vector2f direction = current_position - cell_position;
      float distance = direction.norm();
      if (distance < std::numeric_limits<float>::epsilon()) {
        continue;  // 極端に小さい距離の場合、計算をスキップ
      }

      // 反発力の影響範囲内にある場合のみ計算
      // つまり、近い距離にある障害物のみから反発力を計算するということ。
      if (distance < repulsive_force_min_distance_ && distance > 0.0F) {
        float repulsive_magnitude = repulsive_force_gain_ *
                                    (1.0F / distance - 1.0F / repulsive_force_min_distance_) /
                                    (distance * distance);
        Eigen::Vector2f repulsive_force = direction.normalized() * repulsive_magnitude;
        total_repulsive_force += repulsive_force;
        RCLCPP_DEBUG(
          this->get_logger(),
          "Repulsive force for cell (%d, %d): [x: %f, y: %f], magnitude: %f, distance: %f", i, j,
          repulsive_force.x(), repulsive_force.y(), repulsive_magnitude, distance);
      }
    }
  }

  // (For Debugging) 現在のロボット位置を示す OccupancyGrid メッセージを生成
  auto robot_position_gridmap = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  robot_position_gridmap->header.stamp = this->now();
  robot_position_gridmap->header.frame_id = "odom";
  robot_position_gridmap->info = current_gridmap_.info;
  robot_position_gridmap->data.resize(width * height, 0);

  // 現在のロボット位置をワールド座標系で取得して、セルに値を設定
  float world_origin_x = current_gridmap_.info.origin.position.x;
  float world_origin_y = current_gridmap_.info.origin.position.y;

  // ロボットの位置をワールド座標系に基づいて OccupancyGrid のセル位置に変換
  // この式は他で使われているものと一緒???要確認
  int robot_x = static_cast<int>((current_position_.x() - world_origin_x) / resolution);
  int robot_y = static_cast<int>((current_position_.y() - world_origin_y) / resolution);

  if (robot_x >= 0 && robot_x < width && robot_y >= 0 && robot_y < height) {
    int robot_index = robot_y * width + robot_x;
    robot_position_gridmap->data[robot_index] = 100;  // ロボット位置を示すための値
  }

  current_robot_position_publisher_->publish(*robot_position_gridmap);

  return total_repulsive_force;
}

}  // namespace path_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_planner::PathPlanner)
