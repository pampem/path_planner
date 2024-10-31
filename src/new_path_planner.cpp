// // new_path_planner.cpp

// #include "path_planner/new_path_planner.hpp"

// #include <Eigen/Dense>

// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <cmath>
// #include <limits>
// #include <vector>

// namespace path_planner
// {

// NewPathPlanner::NewPathPlanner(const rclcpp::NodeOptions & options)
// : Node("new_path_planner", options), target_position_(5.0, 5.0, 0.0)  // 3つの引数で初期化
// {
//   RCLCPP_INFO(this->get_logger(), "NewPathPlanner node has been started.");

//   // LiDARデータの購読設定
//   lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//     "iris0/lidar/points", 10,  // livox/lidar or iris0/lidar/points
//     std::bind(&::path_planner::NewPathPlanner::lidar_callback, this, std::placeholders::_1));

//   // ドローンの現在位置の購読設定
//   pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//     "glim_ros/pose", 10, std::bind(&NewPathPlanner::pose_callback, this, std::placeholders::_1));

//   // 速度指令のパブリッシュ設定
//   velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
//     "drone1/mavros/setpoint_velocity/cmd_vel", 10);

//   // タイマーの設定（100msごとに経路計画を実行）
//   timer_ =
//     this->create_wall_timer(std::chrono::milliseconds(100), [this]() { this->calculate_path(); });
// }

// void NewPathPlanner::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// {
//   RCLCPP_DEBUG(this->get_logger(), "LiDAR callback triggered.");
//   latest_lidar_msg_ = msg;
// }

// void NewPathPlanner::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//   RCLCPP_DEBUG(this->get_logger(), "Pose callback triggered.");
//   current_pose_ = *msg;
// }

// void NewPathPlanner::calculate_path()
// {
//   RCLCPP_INFO(this->get_logger(), "Starting calculate_path.");

//   if (!latest_lidar_msg_) {
//     RCLCPP_WARN(this->get_logger(), "No LiDAR data received yet.");
//     return;
//   }

//   // 1. LiDARデータの取得とPCL形式への変換
//   pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::fromROSMsg(*latest_lidar_msg_, *pcl_cloud);
//   RCLCPP_DEBUG(
//     this->get_logger(), "Converted PointCloud2 to PCL format. Total points: %zu",
//     pcl_cloud->points.size());

//   // 2. ポイントクラウドのフィルタリング（ダウンサンプリング）
//   // 2.1 VoxelGrid フィルタでダウンサンプリング
//   pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
//   voxel_filter.setInputCloud(pcl_cloud);
//   voxel_filter.setLeafSize(0.3F, 0.3F, 0.3F);  // 間引きの閾値
//   pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   voxel_filter.filter(*voxel_filtered_cloud);
//   RCLCPP_DEBUG(
//     this->get_logger(), "VoxelGrid filtering done. Points after filtering: %zu",
//     voxel_filtered_cloud->points.size());

//   // 2.2 PassThrough フィルタで z 軸をフィルタリング（2D対応）
//   pcl::PassThrough<pcl::PointXYZ> pass;
//   pass.setInputCloud(voxel_filtered_cloud);
//   pass.setFilterFieldName("z");
//   pass.setFilterLimits(-0.5, 0.5);  // 高度の範囲を適宜設定
//   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   pass.filter(*filtered_cloud);
//   RCLCPP_DEBUG(
//     this->get_logger(), "PassThrough filtering done. Points after filtering: %zu",
//     filtered_cloud->points.size());

//   // 3. ポイントクラウドの並べ替え
//   std::vector<pcl::PointXYZ> ordered_points;
//   if (filtered_cloud->points.empty()) {
//     RCLCPP_WARN(this->get_logger(), "Filtered point cloud is empty after 2D filtering.");
//     return;
//   }

//   // 最初の点を選択
//   ordered_points.push_back(filtered_cloud->points[0]);
//   RCLCPP_DEBUG(
//     this->get_logger(), "Added first point to ordered_points: (%.2f, %.2f)",
//     ordered_points.back().x, ordered_points.back().y);

//   // 残りの点を訪問リストに追加
//   std::vector<pcl::PointXYZ> points_to_visit(
//     filtered_cloud->points.begin() + 1, filtered_cloud->points.end());
//   RCLCPP_DEBUG(
//     this->get_logger(), "Initialized points_to_visit with %zu points.", points_to_visit.size());

//   const double max_distance = 0.5;

//   while (!points_to_visit.empty()) {
//     const pcl::PointXYZ & last_point = ordered_points.back();
//     // 最も近い点を探す（2D距離）
//     double min_dist = std::numeric_limits<double>::max();
//     size_t min_index = 0;
//     for (size_t i = 0; i < points_to_visit.size(); ++i) {
//       double dist = std::sqrt(
//         std::pow(last_point.x - points_to_visit[i].x, 2) +
//         std::pow(last_point.y - points_to_visit[i].y, 2));
//       if (dist < min_dist) {
//         min_dist = dist;
//         min_index = i;
//       }
//     }

//     RCLCPP_DEBUG(
//       this->get_logger(), "Closest point distance: %.2f at index: %zu", min_dist, min_index);

//     if (min_dist <= max_distance) {
//       // 閾値以内の点を追加
//       ordered_points.push_back(points_to_visit[min_index]);
//       RCLCPP_DEBUG(
//         this->get_logger(), "Added point within max_distance: (%.2f, %.2f)",
//         ordered_points.back().x, ordered_points.back().y);
//       points_to_visit.erase(points_to_visit.begin() + min_index);
//       RCLCPP_DEBUG(
//         this->get_logger(), "Removed point from points_to_visit. Remaining points: %zu",
//         points_to_visit.size());
//     } else {
//       // 閾値を超える場合、中間点を挿入して経路の連続性を保つ
//       pcl::PointXYZ last_pt = ordered_points.back();
//       pcl::PointXYZ next_pt = points_to_visit[min_index];
//       RCLCPP_DEBUG(
//         this->get_logger(), "Adding intermediate points between (%.2f, %.2f) and (%.2f, %.2f)",
//         last_pt.x, last_pt.y, next_pt.x, next_pt.y);

//       // 中間点の数を計算
//       int num_intermediate = static_cast<int>(std::ceil(min_dist / max_distance)) - 1;
//       RCLCPP_DEBUG(
//         this->get_logger(), "Number of intermediate points to add: %d", num_intermediate);

//       // 中間点を生成
//       for (int i = 1; i <= num_intermediate; ++i) {
//         pcl::PointXYZ intermediate_point;
//         float ratio = static_cast<float>(i) / (num_intermediate + 1);
//         intermediate_point.x = last_pt.x + (next_pt.x - last_pt.x) * ratio;
//         intermediate_point.y = last_pt.y + (next_pt.y - last_pt.y) * ratio;
//         intermediate_point.z = 0.0;  // 2D環境のため

//         ordered_points.push_back(intermediate_point);
//         RCLCPP_DEBUG(
//           this->get_logger(), "Added intermediate point: (%.2f, %.2f)", intermediate_point.x,
//           intermediate_point.y);
//       }

//       // 最も近い点を追加
//       ordered_points.push_back(next_pt);
//       RCLCPP_DEBUG(this->get_logger(), "Added point: (%.2f, %.2f)", next_pt.x, next_pt.y);
//       points_to_visit.erase(points_to_visit.begin() + min_index);
//       RCLCPP_DEBUG(
//         this->get_logger(), "Removed point from points_to_visit. Remaining points: %zu",
//         points_to_visit.size());
//     }
//   }

//   RCLCPP_INFO(this->get_logger(), "Ordered points size: %zu", ordered_points.size());

//   // 4. スプライン曲線の生成（Catmull-Romスプライン）
//   std::vector<Eigen::Vector2d> spline_points = interpolate_spline(ordered_points, 10);
//   RCLCPP_INFO(this->get_logger(), "Generated spline points: %zu", spline_points.size());

//   // 5. 力の計算
//   Eigen::Vector2d total_force(0.0, 0.0);
//   Eigen::Vector2d drone_pos(current_pose_.pose.position.x, current_pose_.pose.position.y);
//   Eigen::Vector2d target_pos(target_position_.x(), target_position_.y());
//   Eigen::Vector2d target_vector = target_pos - drone_pos;

//   RCLCPP_DEBUG(this->get_logger(), "Drone position: (%.2f, %.2f)", drone_pos.x(), drone_pos.y());
//   RCLCPP_DEBUG(this->get_logger(), "Target position: (%.2f, %.2f)", target_pos.x(), target_pos.y());

//   if (target_vector.norm() > 0) {
//     target_vector.normalize();
//     target_vector *= 10.0;  // ターゲットへの引力係数
//     RCLCPP_DEBUG(
//       this->get_logger(), "Calculated target vector: (%.2f, %.2f)", target_vector.x(),
//       target_vector.y());
//   }

//   // スプライン曲線上の各ポイントに対して反作用力を計算
//   for (const auto & point : spline_points) {
//     Eigen::Vector2d point_vec(point.x(), point.y());
//     Eigen::Vector2d diff = drone_pos - point_vec;
//     double distance = diff.norm();
//     if (distance < 1.5 && distance > 0.0) {
//       Eigen::Vector2d repulsive_force = diff.normalized() * (10.0 / distance);  // 反作用力の計算
//       total_force += repulsive_force;
//       RCLCPP_DEBUG(
//         this->get_logger(), "Applied repulsive force from point (%.2f, %.2f): (%.2f, %.2f)",
//         point.x(), point.y(), repulsive_force.x(), repulsive_force.y());
//     }
//   }

//   // 合力の計算
//   total_force += target_vector;
//   RCLCPP_DEBUG(this->get_logger(), "Total force: (%.2f, %.2f)", total_force.x(), total_force.y());

//   // 6. 速度指令の生成
//   geometry_msgs::msg::Twist cmd_vel;
//   cmd_vel.linear.x = total_force.x();
//   cmd_vel.linear.y = total_force.y();
//   cmd_vel.linear.z = 0.0;  // 2D環境のため
//   cmd_vel.angular.x = 0.0;
//   cmd_vel.angular.y = 0.0;
//   cmd_vel.angular.z = 0.0;

//   RCLCPP_INFO(
//     this->get_logger(), "Publishing velocity command: linear.x=%.2f, linear.y=%.2f",
//     cmd_vel.linear.x, cmd_vel.linear.y);

//   // 7. 速度指令の送信
//   send_velocity_command(cmd_vel);
// }

// // Catmull-Romスプライン補間の実装
// std::vector<Eigen::Vector2d> NewPathPlanner::interpolate_spline(
//   const std::vector<pcl::PointXYZ> & points, int samples_per_segment)
// {
//   std::vector<Eigen::Vector2d> spline_points;

//   for (size_t i = 0; i < points.size() - 1; ++i) {
//     // 制御点の取得
//     Eigen::Vector2d p0 = (i == 0) ? Eigen::Vector2d(points[i].x, points[i].y)
//                                   : Eigen::Vector2d(points[i - 1].x, points[i - 1].y);
//     Eigen::Vector2d p1 = Eigen::Vector2d(points[i].x, points[i].y);
//     Eigen::Vector2d p2 = Eigen::Vector2d(points[i + 1].x, points[i + 1].y);
//     Eigen::Vector2d p3 = (i + 2 < points.size())
//                            ? Eigen::Vector2d(points[i + 2].x, points[i + 2].y)
//                            : Eigen::Vector2d(points[i + 1].x, points[i + 1].y);

//     for (int j = 0; j < samples_per_segment; ++j) {
//       double t = static_cast<double>(j) / samples_per_segment;
//       double t2 = t * t;
//       double t3 = t2 * t;

//       // Catmull-Romスプラインの計算
//       Eigen::Vector2d point =
//         0.5 * ((2.0 * p1) + (-p0 + p2) * t + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
//                (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);

//       spline_points.push_back(point);
//       RCLCPP_DEBUG(this->get_logger(), "Spline point added: (%.2f, %.2f)", point.x(), point.y());
//     }
//   }

//   // 最後の点を追加
//   spline_points.emplace_back(points.back().x, points.back().y);
//   RCLCPP_DEBUG(
//     this->get_logger(), "Spline final point added: (%.2f, %.2f)", points.back().x, points.back().y);

//   return spline_points;
// }

// void NewPathPlanner::send_velocity_command(const geometry_msgs::msg::Twist & cmd_vel)
// {
//   RCLCPP_DEBUG(this->get_logger(), "Sending velocity command.");
//   velocity_publisher_->publish(cmd_vel);
// }

// }  // namespace path_planner

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(path_planner::NewPathPlanner)
