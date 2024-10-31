// Copyright 2024, pampem
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class CorrectedPosePublisherIMU : public rclcpp::Node {
public:
  CorrectedPosePublisherIMU();

private:
  void imuCallbackFCU(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void imuCallbackLiDAR(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
  tf2::Quaternion computeRotationBetweenVectors(const tf2::Vector3 &vec1,
                                                const tf2::Vector3 &vec2);

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_fcu_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_lidar_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      vision_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_pub_;

  // Calibration variables
  tf2::Vector3 sum_accel_fcu_;
  tf2::Vector3 sum_accel_lidar_;
  size_t sample_count_;
  rclcpp::Time start_time_;
  bool calibration_done_;
  tf2::Quaternion correction_quaternion_;

  // Flags to check if IMU data has been received
  bool imu_received_fcu_;
  bool imu_received_lidar_{};
};

CorrectedPosePublisherIMU::CorrectedPosePublisherIMU()
    : Node("corrected_pose_publisher_imu"), sum_accel_fcu_(0.0, 0.0, 0.0),
      sum_accel_lidar_(0.0, 0.0, 0.0), sample_count_(0),
      calibration_done_(false), imu_received_fcu_(false)
{
  // Set QoS settings to match the publishers (Best Effort, Volatile)
  rclcpp::QoS qos_settings(
      rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  qos_settings.best_effort();
  qos_settings.durability_volatile();

  // Subscriber for the /glim_ros/pose topic (using default QoS)
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/glim_ros/pose", 10,
      std::bind(&CorrectedPosePublisherIMU::poseCallback, this,
                std::placeholders::_1));

  // Subscribers for the IMU data with adjusted QoS settings
  imu_sub_fcu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/drone1/mavros/imu/data_raw", qos_settings,
      std::bind(&CorrectedPosePublisherIMU::imuCallbackFCU, this,
                std::placeholders::_1));

  imu_sub_lidar_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/iris0/lidar/imu", qos_settings,
      std::bind(&CorrectedPosePublisherIMU::imuCallbackLiDAR, this,
                std::placeholders::_1));

  // Publishers for the corrected pose
  vision_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone1/mavros/vision_pose/pose", 10);
  mocap_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone1/mavros/mocap/pose", 10);

  start_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Corrected Pose Publisher IMU node has been "
                                  "started. Calibrating for 5 seconds...");
}

void CorrectedPosePublisherIMU::imuCallbackFCU(
    const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  if (!calibration_done_) {
    tf2::Vector3 accel_fcu(imu_msg->linear_acceleration.x,
                           imu_msg->linear_acceleration.y,
                           imu_msg->linear_acceleration.z);

    sum_accel_fcu_ += accel_fcu;
    imu_received_fcu_ = true;
  }
}

void CorrectedPosePublisherIMU::imuCallbackLiDAR(
    const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  if (!calibration_done_) {
    tf2::Vector3 accel_lidar(imu_msg->linear_acceleration.x,
                             imu_msg->linear_acceleration.y,
                             imu_msg->linear_acceleration.z);

    sum_accel_lidar_ += accel_lidar;
    imu_received_lidar_ = true;
    sample_count_++;

    auto current_time = this->now();
    rclcpp::Duration elapsed_time = current_time - start_time_;

    if (elapsed_time.seconds() >= 5.0) {
      if (sample_count_ == 0 || !imu_received_fcu_ || !imu_received_lidar_) {
        RCLCPP_ERROR(this->get_logger(),
                     "No IMU data received during calibration.");
        calibration_done_ = true;
        return;
      }

      tf2::Vector3 avg_accel_fcu =
          sum_accel_fcu_ / static_cast<double>(sample_count_);
      tf2::Vector3 avg_accel_lidar =
          sum_accel_lidar_ / static_cast<double>(sample_count_);

      double norm_fcu = avg_accel_fcu.length();
      double norm_lidar = avg_accel_lidar.length();

      if (norm_fcu < 1e-6 || norm_lidar < 1e-6) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Average acceleration vector is zero, cannot compute correction.");
        calibration_done_ = true;
        return;
      }

      avg_accel_fcu.normalize();
      avg_accel_lidar.normalize();

      RCLCPP_INFO(this->get_logger(), "Avg Accel FCU: [%.6f, %.6f, %.6f]",
                  avg_accel_fcu.x(), avg_accel_fcu.y(), avg_accel_fcu.z());
      RCLCPP_INFO(this->get_logger(), "Avg Accel LiDAR: [%.6f, %.6f, %.6f]",
                  avg_accel_lidar.x(), avg_accel_lidar.y(),
                  avg_accel_lidar.z());

      correction_quaternion_ =
          computeRotationBetweenVectors(avg_accel_lidar, avg_accel_fcu);

      calibration_done_ = true;
      double roll, pitch, yaw;
      tf2::Quaternion lidar_quaternion;
      lidar_quaternion.setRPY(avg_accel_lidar.x(), avg_accel_lidar.y(),
                              avg_accel_lidar.z());
      tf2::Matrix3x3(lidar_quaternion).getRPY(roll, pitch, yaw);
      RCLCPP_INFO(this->get_logger(),
                  "Calibration completed. Correction quaternion computed. "
                  "LiDAR average angles: roll=%.2f, pitch=%.2f, yaw=%.2f",
                  roll * 180.0 / M_PI, pitch * 180.0 / M_PI,
                  yaw * 180.0 / M_PI);
    }
  }
}

tf2::Quaternion CorrectedPosePublisherIMU::computeRotationBetweenVectors(
    const tf2::Vector3 &vec1, const tf2::Vector3 &vec2) {
  tf2::Vector3 v1 = vec1.normalized();
  tf2::Vector3 v2 = vec2.normalized();

  double dot = v1.dot(v2);
  tf2::Vector3 cross = v1.cross(v2);

  if (cross.length() < 1e-6) {
    if (dot > 0.9999) {
      return tf2::Quaternion::getIdentity();
    } else {
      tf2::Vector3 ortho = tf2::Vector3(1, 0, 0).cross(v1);
      if (ortho.length() < 1e-6) {
        ortho = tf2::Vector3(0, 1, 0).cross(v1);
      }
      ortho.normalize();
      return tf2::Quaternion(ortho, M_PI);
    }
  } else {
    double angle = std::acos(dot);
    tf2::Quaternion q(cross.normalized(), angle);
    q.normalize();
    return q;
  }
}

void CorrectedPosePublisherIMU::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {
  if (!calibration_done_) {
    return;
  }

  tf2::Quaternion q_original;
  tf2::fromMsg(pose_msg->pose.orientation, q_original);

  tf2::Quaternion q_corrected = correction_quaternion_ * q_original;
  q_corrected.normalize();

  tf2::Matrix3x3 rotation_matrix(q_corrected);
  tf2::Matrix3x3 enu_transform(0, 0, 1, 0, 1, 0, -1, 0, 0);

  tf2::Matrix3x3 final_rotation = enu_transform * rotation_matrix;
  tf2::Quaternion q_enu;
  final_rotation.getRotation(q_enu);
  q_enu.normalize();

  if (std::isnan(q_enu.x()) || std::isnan(q_enu.y()) || std::isnan(q_enu.z()) ||
      std::isnan(q_enu.w())) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Corrected orientation contains NaN values. Skipping publish.");
    return;
  }

  geometry_msgs::msg::PoseStamped corrected_pose;
  corrected_pose.header = pose_msg->header;
  corrected_pose.pose.position = pose_msg->pose.position;
  corrected_pose.pose.orientation = tf2::toMsg(q_enu);

  vision_pose_pub_->publish(corrected_pose);
  mocap_pose_pub_->publish(corrected_pose);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q_enu).getRPY(roll, pitch, yaw);
  RCLCPP_INFO(this->get_logger(),
              "Published corrected pose with IMU-based correction and ENU "
              "alignment. Euler angles: roll=%.2f, pitch=%.2f, yaw=%.2f",
              roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CorrectedPosePublisherIMU>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
