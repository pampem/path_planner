#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class CorrectedPosePublisher : public rclcpp::Node
{
public:
  CorrectedPosePublisher()
  : Node("corrected_pose_publisher")
  {
    // Subscriber for the /glim_ros/pose topic
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/glim_ros/pose", 10,
      std::bind(&CorrectedPosePublisher::poseCallback, this, std::placeholders::_1));

    // Publishers for the corrected pose
    vision_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone1/mavros/vision_pose/pose", 10);
    mocap_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone1/mavros/mocap/pose", 10);

    RCLCPP_INFO(this->get_logger(), "Corrected Pose Publisher node has been started.");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
  {
    // Original orientation (from /glim_ros/pose)
    tf2::Quaternion q_original;
    tf2::fromMsg(pose_msg->pose.orientation, q_original);

    // Step 1: Create a quaternion for a +45 degrees pitch rotation (around Y axis)
    tf2::Quaternion q_pitch_correction;
    double pitch_correction_radians = - 3 * M_PI / 4;  
    q_pitch_correction.setRPY(0.0, pitch_correction_radians, 0.0);  // Only pitch correction

    // Step 2: Create a quaternion for a 180 degrees yaw rotation (around Z axis)
    tf2::Quaternion q_yaw_rotation;
    double yaw_rotation_radians = 0; 
    q_yaw_rotation.setRPY(0.0, 0.0, yaw_rotation_radians);  // Only yaw rotation (around Z axis)

    // Apply the pitch correction first, then apply the 180 degrees yaw rotation
    tf2::Quaternion q_corrected = q_yaw_rotation * q_pitch_correction * q_original;
    q_corrected.normalize();  // Normalize the quaternion

    // Corrected pose message
    geometry_msgs::msg::PoseStamped corrected_pose;
    corrected_pose.header = pose_msg->header;  // Copy the header
    corrected_pose.pose.position = pose_msg->pose.position;  // Copy the position (no change)
    corrected_pose.pose.orientation = tf2::toMsg(q_corrected);  // Set the corrected orientation

    // Publish the corrected pose to both topics
    vision_pose_pub_->publish(corrected_pose);
    mocap_pose_pub_->publish(corrected_pose);

    RCLCPP_INFO(this->get_logger(), "Published corrected pose with yaw rotation.");
  }

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CorrectedPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
