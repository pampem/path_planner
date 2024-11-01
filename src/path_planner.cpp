/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#include "path_planner/path_planner.hpp"

namespace path_planner
{

PathPlanner::PathPlanner(const rclcpp::NodeOptions & options)
: Node("path_planner", options),
  tf_buffer_(this->get_clock(), tf2::Duration(std::chrono::seconds(10))),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
  this->declare_parameter("attractive_force.max_distance", 3.0F);
  this->declare_parameter("attractive_force.gain", 0.5F);

  this->get_parameter("attractive_force.max_distance", attractive_force_max_distance_);
  this->get_parameter("attractive_force.gain", attractive_force_gain_);

  slam_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "glim_ros/pose", 10, std::bind(&PathPlanner::slam_pose_callback, this, std::placeholders::_1));

  auto gridmap_subscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "gridmap", 10, std::bind(&PathPlanner::gridmap_callback, this, std::placeholders::_1));

  robot_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "drone1/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

  RCLCPP_INFO(this->get_logger(), "PathPlanner node has been initialized.");

  // 仮に置いてある。
  target_position_.x() = 5;
  target_position_.y() = 10;
}

// GridMapのコールバック
void PathPlanner::gridmap_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received gridmap data.");

  // process the gridmap here for path planning
}

// SLAMのPoseコールバック
void PathPlanner::slam_pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 現在のポーズを更新
  current_pose_ = *msg;

  Eigen::Vector2f current_position(current_pose_.pose.position.x, current_pose_.pose.position.y);

  // 引力を計算
  Eigen::Vector2f attractive_force = calculate_attractive_force(
    current_position, target_position_, attractive_force_gain_, attractive_force_max_distance_);

  // 計算された引力をTwistメッセージに変換して移動指令を生成
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = attractive_force.x();
  twist_msg.linear.y = attractive_force.y();

  // 速度指令をパブリッシュしてドローンに渡す
  robot_velocity_publisher_->publish(twist_msg);

  RCLCPP_INFO(
    this->get_logger(), "Published velocity command towards goal: [linear.x: %f, linear.y: %f]",
    twist_msg.linear.x, twist_msg.linear.y);
}

// これいるかな
void PathPlanner::calculate_path()
{
  // Placeholder path calculation - add the real algorithm here

  // Example: Create a Twist message to move towards the target position
  geometry_msgs::msg::Twist twist_msg;

  twist_msg.linear.x = target_position_.x() - current_pose_.pose.position.x;
  twist_msg.linear.y = target_position_.y() - current_pose_.pose.position.y;

  // Publish the velocity command
  robot_velocity_publisher_->publish(twist_msg);

  RCLCPP_INFO(
    this->get_logger(), "Published velocity command: [linear.x: %f, linear.y: %f]",
    twist_msg.linear.x, twist_msg.linear.y);
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

}  // namespace path_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_planner::PathPlanner)
