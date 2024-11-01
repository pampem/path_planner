/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#ifndef PATH_PLANNER__PATH_PLANNER_HPP_
#define PATH_PLANNER__PATH_PLANNER_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace path_planner
{

class PathPlanner : public rclcpp::Node
{
public:
  explicit PathPlanner(const rclcpp::NodeOptions & options);

private:
  void gridmap_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  /**
   * @brief glimなどのSLAMからPoseを受け取るコールバック。
   * @param msg 自己位置姿勢推定結果。
   */
  void slam_pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  static Eigen::Vector2f calculate_attractive_force(
    const Eigen::Vector2f & current_position, const Eigen::Vector2f & goal_position,
    float attractive_gain, float attractive_force_max_distance);

  Eigen::Vector2f calculate_repulsive_force(const Eigen::Vector2f & current_position);

  Eigen::Vector2f target_position_;
  Eigen::Vector2f goal_position_;
  Eigen::Vector2f current_position_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr gridmap_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_velocity_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr current_robot_position_publisher_;

  float attractive_force_max_distance_;
  float attractive_force_gain_;
  float repulsive_force_gain_;
  float repulsive_force_min_distance_;
  Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic> repulsive_forces_;
};

}  // namespace path_planner

#endif  // PATH_PLANNER__NEW_PATH_PLANNER_HPP_
