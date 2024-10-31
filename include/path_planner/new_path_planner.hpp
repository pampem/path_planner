// Copyright (c) 2024, Masashi Izumita
#ifndef PATH_PLANNER__NEW_PATH_PLANNER_HPP_
#define PATH_PLANNER__NEW_PATH_PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"

#include <Eigen/Dense>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>

namespace path_planner
{

class NewPathPlanner : public rclcpp::Node
{
public:
  // コンストラクタ
  explicit NewPathPlanner(const rclcpp::NodeOptions & options);

private:
  void lidar_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

  geometry_msgs::msg::PoseStamped current_pose_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::PointCloud2::SharedPtr latest_lidar_msg_;

  Eigen::Vector2d target_position_;

  void calculate_path();
  void send_velocity_command(const geometry_msgs::msg::Twist & cmd_vel);
  std::vector<Eigen::Vector2d> interpolate_spline(
    const std::vector<pcl::PointXYZ> & points, int samples_per_segment);
};

}  // namespace path_planner

#endif  // PATH_PLANNER__NEW_PATH_PLANNER_HPP_
