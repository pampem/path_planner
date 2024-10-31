/*
  Copyright (c) 2024
  Masashi Izumita
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
  /**
   * @brief glimなどのSLAMからPoseを受け取るコールバック。
   * @param msg 自己位置姿勢推定結果。
   */
  void slam_pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void gridmap_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  geometry_msgs::msg::PoseStamped current_pose_;

  Eigen::Vector2d target_position_;

  void calculate_path();
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_velocity_publisher_;
};

}  // namespace path_planner

#endif  // PATH_PLANNER__NEW_PATH_PLANNER_HPP_
