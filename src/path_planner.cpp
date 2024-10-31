/*
  Copyright (c) 2024
  Masashi Izumita
 */
#include "path_planner/path_planner.hpp"

namespace path_planner
{

PathPlanner::PathPlanner(const rclcpp::NodeOptions & options)
: Node("path_planner", options),
  tf_buffer_(this->get_clock(), tf2::Duration(std::chrono::seconds(10))),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
  slam_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "slam_pose", 10, std::bind(&PathPlanner::slam_pose_callback, this, std::placeholders::_1));

  auto gridmap_subscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "gridmap", 10, std::bind(&PathPlanner::gridmap_callback, this, std::placeholders::_1));

  robot_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  RCLCPP_INFO(this->get_logger(), "PathPlanner node has been initialized.");
}

void PathPlanner::gridmap_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received gridmap data.");

  // process the gridmap here for path planning
}

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

}  // namespace path_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_planner::PathPlanner)
