// corrected_pose_publisher_imu.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <vector>
#include <Eigen/Dense> // Eigenライブラリのインクルード

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
  rclcpp::Clock steady_clock_; // steady_clockを使用

  // LiDAR IMUデータの処理間隔を制限するための変数
  rclcpp::Time last_lidar_time_;
  double lidar_processing_interval_; // 処理間隔（秒）

  // タイマーオブジェクトをメンバ変数として保持
  rclcpp::TimerBase::SharedPtr timer_;
};

CorrectedPosePublisherIMU::CorrectedPosePublisherIMU()
: Node("corrected_pose_publisher_imu"),
  collecting_data_(true),
  has_fcu_orientation_(false),
  has_lidar_orientation_(false),
  steady_clock_(RCL_STEADY_TIME), // steady_clockで初期化
  lidar_processing_interval_(0.05) // 50ミリ秒間隔で処理
{
  // QoS設定を定義（キューサイズを小さく設定）
  rclcpp::QoS qos(1);
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

  start_time_ = steady_clock_.now();
  last_lidar_time_ = steady_clock_.now();

  // 100ミリ秒ごとにtimerCallbackを呼び出すタイマーを設定し、メンバ変数として保持
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CorrectedPosePublisherIMU::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Node initialized, starting data collection.");
}

void CorrectedPosePublisherIMU::fcuImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // IMUデータの検証
  if (std::isnan(msg->orientation.x) || std::isnan(msg->orientation.y) || std::isnan(msg->orientation.z) || std::isnan(msg->orientation.w))
  {
    RCLCPP_WARN(this->get_logger(), "Received FCU IMU data contains NaN values. Ignoring.");
    return;
  }

  if (collecting_data_)
  {
    tf2::fromMsg(msg->orientation, last_fcu_orientation_);
    has_fcu_orientation_ = true;

    // デバッグ: FCU IMUのオリエンテーションを表示
    RCLCPP_DEBUG(this->get_logger(), "FCU IMU orientation: x=%f, y=%f, z=%f, w=%f", last_fcu_orientation_.x(), last_fcu_orientation_.y(), last_fcu_orientation_.z(), last_fcu_orientation_.w());

    if (has_lidar_orientation_)
    {
      tf2::Quaternion diff = last_fcu_orientation_ * last_lidar_orientation_.inverse();
      diff.normalize(); // 正規化を追加

      // 差分クォータニオンの検証
      if (std::isnan(diff.x()) || std::isnan(diff.y()) || std::isnan(diff.z()) || std::isnan(diff.w()))
      {
        RCLCPP_WARN(this->get_logger(), "Difference quaternion contains NaN values. Ignoring.");
      }
      else
      {
        orientation_differences_.push_back(diff);
        RCLCPP_DEBUG(this->get_logger(), "Stored orientation difference. Total samples: %zu", orientation_differences_.size());
      }

      has_fcu_orientation_ = false;
      has_lidar_orientation_ = false;
    }
  }
}

void CorrectedPosePublisherIMU::lidarImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // IMUデータの検証
  if (std::isnan(msg->orientation.x) || std::isnan(msg->orientation.y) || std::isnan(msg->orientation.z) || std::isnan(msg->orientation.w))
  {
    RCLCPP_WARN(this->get_logger(), "Received LiDAR IMU data contains NaN values. Ignoring.");
    return;
  }

  rclcpp::Time current_time = steady_clock_.now();
  if ((current_time - last_lidar_time_).seconds() < lidar_processing_interval_)
  {
    // 前回の処理から十分な時間が経過していないので、今回のデータは無視
    return;
  }
  last_lidar_time_ = current_time;

  if (collecting_data_)
  {
    tf2::fromMsg(msg->orientation, last_lidar_orientation_);
    has_lidar_orientation_ = true;

    // デバッグ: LiDAR IMUのオリエンテーションを表示
    RCLCPP_DEBUG(this->get_logger(), "LiDAR IMU orientation: x=%f, y=%f, z=%f, w=%f", last_lidar_orientation_.x(), last_lidar_orientation_.y(), last_lidar_orientation_.z(), last_lidar_orientation_.w());

    if (has_fcu_orientation_)
    {
      tf2::Quaternion diff = last_fcu_orientation_ * last_lidar_orientation_.inverse();
      diff.normalize(); // 正規化を追加

      // 差分クォータニオンの検証
      if (std::isnan(diff.x()) || std::isnan(diff.y()) || std::isnan(diff.z()) || std::isnan(diff.w()))
      {
        RCLCPP_WARN(this->get_logger(), "Difference quaternion contains NaN values. Ignoring.");
      }
      else
      {
        orientation_differences_.push_back(diff);
        RCLCPP_DEBUG(this->get_logger(), "Stored orientation difference. Total samples: %zu", orientation_differences_.size());
      }

      has_fcu_orientation_ = false;
      has_lidar_orientation_ = false;
    }
  }
}

void CorrectedPosePublisherIMU::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received pose message.");
  if (!collecting_data_)
  {
    geometry_msgs::msg::PoseStamped corrected_pose = *msg;

    tf2::Quaternion pose_orientation;
    tf2::fromMsg(msg->pose.orientation, pose_orientation);

    // デバッグ: オリジナルのオリエンテーションを表示
    RCLCPP_DEBUG(this->get_logger(), "Original orientation: x=%f, y=%f, z=%f, w=%f", pose_orientation.x(), pose_orientation.y(), pose_orientation.z(), pose_orientation.w());
    RCLCPP_DEBUG(this->get_logger(), "Orientation correction: x=%f, y=%f, z=%f, w=%f", orientation_correction_.x(), orientation_correction_.y(), orientation_correction_.z(), orientation_correction_.w());

    // 補正を適用
    tf2::Quaternion corrected_orientation = orientation_correction_ * pose_orientation;
    corrected_orientation.normalize();

    // 補正後のオリエンテーションにNaNが含まれていないか確認
    if (std::isnan(corrected_orientation.x()) || std::isnan(corrected_orientation.y()) || std::isnan(corrected_orientation.z()) || std::isnan(corrected_orientation.w()))
    {
      RCLCPP_WARN(this->get_logger(), "Corrected orientation contains NaN values. Not publishing.");
      return;
    }

    corrected_pose.pose.orientation = tf2::toMsg(corrected_orientation);

    corrected_pose.header.stamp = this->now(); // タイムスタンプを更新

    corrected_pose_pub1_->publish(corrected_pose);
    corrected_pose_pub2_->publish(corrected_pose);

    RCLCPP_INFO(this->get_logger(), "Published corrected pose.");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Still collecting data, pose correction not applied yet.");
  }
}

void CorrectedPosePublisherIMU::timerCallback()
{
  RCLCPP_DEBUG(this->get_logger(), "Timer callback triggered.");

  rclcpp::Time current_time = steady_clock_.now();
  auto elapsed_time = current_time - start_time_;

  RCLCPP_DEBUG(this->get_logger(), "Timer callback: current time = %.3f, elapsed time = %.3f",
               current_time.seconds(), elapsed_time.seconds());

  if (collecting_data_)
  {
    if (elapsed_time.seconds() >= 15.0)
    {
      collecting_data_ = false;

      if (!orientation_differences_.empty())
      {
        orientation_correction_ = computeAverageQuaternion(orientation_differences_);
        orientation_correction_.normalize();
        RCLCPP_INFO(this->get_logger(), "Orientation correction calculated after collecting %zu samples.", orientation_differences_.size());
        RCLCPP_INFO(this->get_logger(), "Orientation correction: x=%f, y=%f, z=%f, w=%f", orientation_correction_.x(), orientation_correction_.y(), orientation_correction_.z(), orientation_correction_.w());
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Not enough IMU data collected for correction. Using default correction.");
        // 固定の補正クォータニオンを使用
        tf2::Quaternion q_pitch_correction;
        double pitch_correction_radians = -3 * M_PI / 4;  // -135度
        q_pitch_correction.setRPY(0.0, pitch_correction_radians, 0.0);
        orientation_correction_ = q_pitch_correction;
        orientation_correction_.normalize();
      }
    }
  }
}

tf2::Quaternion CorrectedPosePublisherIMU::computeAverageQuaternion(const std::vector<tf2::Quaternion>& quaternions)
{
  if (quaternions.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Quaternion vector is empty. Returning identity quaternion.");
    return tf2::Quaternion::getIdentity();
  }

  // クォータニオンの平均を計算（固有値分解を使用）
  Eigen::MatrixXd A(4, 4);
  A.setZero();
  for (const auto& q : quaternions)
  {
    Eigen::Vector4d q_vec(q.x(), q.y(), q.z(), q.w());
    A += q_vec * q_vec.transpose();
  }

  // 固有値・固有ベクトルを計算
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(A);
  if (eigen_solver.info() != Eigen::Success)
  {
    RCLCPP_ERROR(this->get_logger(), "Eigen decomposition failed. Returning identity quaternion.");
    return tf2::Quaternion::getIdentity();
  }

  // 最大の固有値に対応する固有ベクトルが平均クォータニオン
  Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
  Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();

  // デバッグ: 固有値を表示
  RCLCPP_DEBUG(this->get_logger(), "Eigenvalues: %f, %f, %f, %f", eigenvalues(0), eigenvalues(1), eigenvalues(2), eigenvalues(3));

  Eigen::Vector4d avg_q_vec = eigenvectors.col(3); // 最大の固有値に対応する固有ベクトル

  tf2::Quaternion avg_q(avg_q_vec[0], avg_q_vec[1], avg_q_vec[2], avg_q_vec[3]);
  avg_q.normalize();

  // デバッグ: 平均クォータニオンを表示
  RCLCPP_DEBUG(this->get_logger(), "Average quaternion: x=%f, y=%f, z=%f, w=%f", avg_q.x(), avg_q.y(), avg_q.z(), avg_q.w());

  return avg_q;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CorrectedPosePublisherIMU>();

  // ロギングレベルをデバッグに設定
  if (rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to set logger level.");
  }

  // マルチスレッドエグゼキュータを使用
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
