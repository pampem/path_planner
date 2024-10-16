// corrected_pose_publisher_imu.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <vector>

class CorrectedPosePublisherIMU : public rclcpp::Node
{
public:
  CorrectedPosePublisherIMU();

private:
  void fcuImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidarImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void timerCallback();

  tf2::Quaternion computeAverageQuaternion(const std::vector<tf2::Quaternion>& quaternions);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr fcu_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr lidar_imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr corrected_pose_pub1_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr corrected_pose_pub2_;

  std::vector<tf2::Quaternion> orientation_differences_;
  tf2::Quaternion last_fcu_orientation_;
  tf2::Quaternion last_lidar_orientation_;
  bool has_fcu_orientation_;
  bool has_lidar_orientation_;

  bool collecting_data_;
  tf2::Quaternion orientation_correction_;

  rclcpp::Time start_time_;
};

CorrectedPosePublisherIMU::CorrectedPosePublisherIMU()
: Node("corrected_pose_publisher_imu"), collecting_data_(true),
  has_fcu_orientation_(false), has_lidar_orientation_(false)
{
  // QoS設定を定義
  rclcpp::QoS qos(10);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // FCU IMUサブスクリプション
  fcu_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/drone1/mavros/imu/data", qos,
    std::bind(&CorrectedPosePublisherIMU::fcuImuCallback, this, std::placeholders::_1));

  // LiDAR IMUサブスクリプション
  lidar_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/iris0/lidar/imu", qos,
    std::bind(&CorrectedPosePublisherIMU::lidarImuCallback, this, std::placeholders::_1));

  // Poseサブスクリプション
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/glim_ros/pose", qos,
    std::bind(&CorrectedPosePublisherIMU::poseCallback, this, std::placeholders::_1));

  // パブリッシャーはデフォルトのQoSを使用
  corrected_pose_pub1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/drone1/mavros/vision_pose/pose", 10);
  corrected_pose_pub2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/drone1/mavros/mocap/pose", 10);

  start_time_ = this->now();

  // 100ミリ秒ごとにtimerCallbackを呼び出すタイマーを設定
  auto timer_callback = std::bind(&CorrectedPosePublisherIMU::timerCallback, this);
  this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
}

void CorrectedPosePublisherIMU::fcuImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (collecting_data_)
  {
    tf2::fromMsg(msg->orientation, last_fcu_orientation_);
    has_fcu_orientation_ = true;

    if (has_lidar_orientation_)
    {
      tf2::Quaternion diff = last_fcu_orientation_ * last_lidar_orientation_.inverse();
      orientation_differences_.push_back(diff);
      has_fcu_orientation_ = false;
      has_lidar_orientation_ = false;
    }
  }
}

void CorrectedPosePublisherIMU::lidarImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (collecting_data_)
  {
    tf2::fromMsg(msg->orientation, last_lidar_orientation_);
    has_lidar_orientation_ = true;

    if (has_fcu_orientation_)
    {
      tf2::Quaternion diff = last_fcu_orientation_ * last_lidar_orientation_.inverse();
      orientation_differences_.push_back(diff);
      has_fcu_orientation_ = false;
      has_lidar_orientation_ = false;
    }
  }
}

void CorrectedPosePublisherIMU::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!collecting_data_)
  {
    geometry_msgs::msg::PoseStamped corrected_pose = *msg;

    tf2::Quaternion pose_orientation;
    tf2::fromMsg(msg->pose.orientation, pose_orientation);

    // 補正を適用
    tf2::Quaternion corrected_orientation = orientation_correction_ * pose_orientation;
    corrected_orientation.normalize();

    corrected_pose.pose.orientation = tf2::toMsg(corrected_orientation);

    corrected_pose.header.stamp = this->now(); // タイムスタンプを更新

    corrected_pose_pub1_->publish(corrected_pose);
    corrected_pose_pub2_->publish(corrected_pose);
  }
}

void CorrectedPosePublisherIMU::timerCallback()
{
  if (collecting_data_)
  {
    auto elapsed_time = this->now() - start_time_;
    if (elapsed_time.seconds() >= 5.0)
    {
      collecting_data_ = false;

      if (orientation_differences_.size() > 0)
      {
        orientation_correction_ = computeAverageQuaternion(orientation_differences_);
        RCLCPP_INFO(this->get_logger(), "Orientation correction calculated.");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Not enough IMU data collected for correction.");
      }
    }
  }
}

tf2::Quaternion CorrectedPosePublisherIMU::computeAverageQuaternion(const std::vector<tf2::Quaternion>& quaternions)
{
  if (quaternions.empty())
    return tf2::Quaternion(0, 0, 0, 1);

  double x = 0, y = 0, z = 0, w = 0;
  for (const auto& q : quaternions)
  {
    x += q.x();
    y += q.y();
    z += q.z();
    w += q.w();
  }

  x /= quaternions.size();
  y /= quaternions.size();
  z /= quaternions.size();
  w /= quaternions.size();

  tf2::Quaternion avg_q(x, y, z, w);
  avg_q.normalize();
  return avg_q;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CorrectedPosePublisherIMU>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
