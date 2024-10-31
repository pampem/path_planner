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
  set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/drone1/mavros/set_mode");
  arming_client_ =
    this->create_client<mavros_msgs::srv::CommandBool>("/drone1/mavros/cmd/arming");
  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/drone1/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  takeoff_client_ =
    this->create_client<mavros_msgs::srv::CommandTOL>("/drone1/mavros/cmd/takeoff");
  land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/drone1/mavros/cmd/land");
  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/drone1/mavros/vision_pose/pose", 10,
    std::bind(&PathPlannerNode::pose_callback, this, std::placeholders::_1));

  gridmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "gridmap", 10, std::bind(&PathPlannerNode::gridmap_callback, this, std::placeholders::_1));

  sethome_cmd_client_ =
    this->create_client<mavros_msgs::srv::CommandLong>("/drone1/mavros/cmd/command");

  waypoints = {{11.6F, 0.0F}, {11.6F, 5.4F}, {0.0F, 5.4F}, {0.0F, 0.0F}};

  waypoint_index = 0;
  loop_count = 0;
  max_loops = 5;
  update_goal();

  // set_home_position();
  // arm_and_takeoff();
}



struct DirectionalPotentialFields
{
  Eigen::MatrixXf potentialFieldX;
  Eigen::MatrixXf potentialFieldY;
};

geometry_msgs::msg::PoseStamped current_pose;

float grid_x = 500;
float grid_y = 300;
float space_x = 100;
float space_y = 60;
float cell_size_x = space_x / grid_x;
float cell_size_y = space_y / grid_y;

float goal_x;
float goal_y;

int grid_x_int = static_cast<int>(grid_x);
int grid_y_int = static_cast<int>(grid_y);

bool land_flag = false;

DirectionalPotentialFields toGoalFields;

std::vector<std::pair<float, float>> waypoints;
size_t waypoint_index;
int loop_count;
int max_loops;

void update_goal()
{
  goal_x = waypoints[waypoint_index].first / cell_size_x + (grid_x / 2);
  goal_y = grid_y - (waypoints[waypoint_index].second / cell_size_y + (grid_y / 2));
  toGoalFields = calculate_to_goal_potential_field(Eigen::MatrixXf::Zero(grid_y_int, grid_x_int));
}

void gridmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  Eigen::MatrixXf gridmap = convert_occupancy_grid_to_eigen_matrix(msg);
  geometry_msgs::msg::Twist velocity_msg;

  float current_x_raw = current_pose.pose.position.x / cell_size_x + (grid_x / 2);
  float current_y_raw = grid_y - (current_pose.pose.position.y / cell_size_y + (grid_y / 2));

  int current_x = static_cast<int>(current_x_raw);
  int current_y = static_cast<int>(current_y_raw);
  RCLCPP_INFO(
    this->get_logger(),
    "Current Pose: [x: %f , y: %f, z: %f, (for gridmap)current_x: %d, current_y: %d]",
    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
    current_x, current_y);

  DirectionalPotentialFields artificial_fields =
    generate_artificial_potential_fields(gridmap, current_y, current_x);
  DirectionalPotentialFields rotational_fields =
    generate_rotational_potential_fields(gridmap, current_y, current_x);

  float weight_to_goal = 0.50;
  float weight_artificial = 0.35;
  float weight_rotational = 0.25;

  adjust_potential_fields(artificial_fields);
  adjust_potential_fields(rotational_fields);
  Eigen::MatrixXf combined_potential_field_x =
    weight_to_goal * toGoalFields.potentialFieldX +
    weight_artificial * artificial_fields.potentialFieldX +
    weight_rotational * rotational_fields.potentialFieldX;
  Eigen::MatrixXf combined_potential_field_y =
    weight_to_goal * toGoalFields.potentialFieldY +
    weight_artificial * artificial_fields.potentialFieldY +
    weight_rotational * rotational_fields.potentialFieldY;

  DirectionalPotentialFields combined_fields;
  combined_fields.potentialFieldX = combined_potential_field_x;
  combined_fields.potentialFieldY = combined_potential_field_y;

  adjust_potential_fields(combined_fields);
  visualize_potential_field(combined_fields, current_y, current_x);

  velocity_msg.linear.x = combined_fields.potentialFieldX(current_y, current_x);
  velocity_msg.linear.y = -1 * combined_fields.potentialFieldY(current_y, current_x);
  velocity_msg.linear.z = 0;
  velocity_publisher_->publish(velocity_msg);

  if (
    current_x >= goal_x - 5 && current_x <= goal_x + 5 && current_y >= goal_y - 5 &&
    current_y <= goal_y + 5) {
    RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", waypoint_index + 1);
    waypoint_index = (waypoint_index + 1) % waypoints.size();
    if (waypoint_index == 0) {
      loop_count++;
      RCLCPP_INFO(this->get_logger(), "Completed loop %d/%d", loop_count, max_loops);
      if (loop_count >= max_loops) {
        RCLCPP_INFO(this->get_logger(), "Completed all loops! Landing the drone.");
        land_drone();
        return;
      }
    }
    update_goal();
  }
}

DirectionalPotentialFields calculate_to_goal_potential_field(
  const Eigen::MatrixXf & gridmap) const
{
  DirectionalPotentialFields fields;
  fields.potentialFieldX = Eigen::MatrixXf::Zero(gridmap.rows(), gridmap.cols());
  fields.potentialFieldY = Eigen::MatrixXf::Zero(gridmap.rows(), gridmap.cols());

  for (int y = 0; y < gridmap.rows(); ++y) {
    for (int x = 0; x < gridmap.cols(); ++x) {
      auto y_float = static_cast<float>(y);
      auto x_float = static_cast<float>(x);
      float dx = goal_x - x_float;
      float dy = goal_y - y_float;
      float magnitude = sqrt(dx * dx + dy * dy);

      if (magnitude > 0) {
        fields.potentialFieldX(y, x) = dx / magnitude;
        fields.potentialFieldY(y, x) = dy / magnitude;
      } else {
        fields.potentialFieldX(y, x) = 0;
        fields.potentialFieldY(y, x) = 0;
      }
    }
  }
  return fields;
}

static DirectionalPotentialFields generate_artificial_potential_fields(
  const Eigen::MatrixXf & gridmap, int current_y, int current_x)
{
  int rows = gridmap.rows();
  int cols = gridmap.cols();

  DirectionalPotentialFields fields;
  fields.potentialFieldX = Eigen::MatrixXf::Zero(rows, cols);
  fields.potentialFieldY = Eigen::MatrixXf::Zero(rows, cols);

  int scan_range = 30;
  int start_row = std::max(current_y - scan_range, 0);
  int end_row = std::min(current_y + scan_range, rows - 1);
  int start_col = std::max(current_x - scan_range, 0);
  int end_col = std::min(current_x + scan_range, cols - 1);

  for (int y = start_row; y <= end_row; ++y) {
    for (int x = start_col; x <= end_col; ++x) {
      if (gridmap(y, x) != 0.0F) {
        for (int dy = -5; dy <= 5; ++dy) {
          for (int dx = -5; dx <= 5; ++dx) {
            int ny = y + dy;
            int nx = x + dx;
            auto dy_float = static_cast<float>(dy);
            auto dx_float = static_cast<float>(dx);
            if (ny >= 0 && ny < rows && nx >= 0 && nx < cols && (dy != 0 || dx != 0)) {
              float influence = 1.0F / std::sqrt(dy_float * dy_float + dx_float * dx_float);
              float potential_y = dy_float * influence;
              float potential_x = dx_float * influence;
              fields.potentialFieldX(ny, nx) += potential_x;
              fields.potentialFieldY(ny, nx) += potential_y;
            }
          }
        }
      }
    }
  }
  return fields;
}

static DirectionalPotentialFields generate_rotational_potential_fields(
  const Eigen::MatrixXf & gridmap, int current_y, int current_x)
{
  int rows = gridmap.rows();
  int cols = gridmap.cols();

  DirectionalPotentialFields rotational_fields;
  rotational_fields.potentialFieldX = Eigen::MatrixXf::Zero(rows, cols);
  rotational_fields.potentialFieldY = Eigen::MatrixXf::Zero(rows, cols);

  int scan_range = 30;
  int start_row = std::max(current_x - scan_range, 0);
  int end_row = std::min(current_x + scan_range, rows - 1);
  int start_col = std::max(current_y - scan_range, 0);
  int end_col = std::min(current_y + scan_range, cols - 1);

  for (int i = start_row; i <= end_row; ++i) {
    for (int j = start_col; j <= end_col; ++j) {
      if (gridmap(i, j) != 0.0F) {
        for (int di = -3; di <= 3; ++di) {
          for (int dj = -3; dj <= 3; ++dj) {
            int ni = i + di;
            int nj = j + dj;
            if (ni >= 0 && ni < rows && nj >= 0 && nj < cols && (di != 0 || dj != 0)) {
              float rotation_strength = std::sqrt(di * di + dj * dj);
              rotational_fields.potentialFieldX(ni, nj) += -dj * rotation_strength;
              rotational_fields.potentialFieldY(ni, nj) += di * rotation_strength;
            }
          }
        }
      }
    }
  }
  return rotational_fields;
}

static void adjust_potential_fields(DirectionalPotentialFields & fields)
{
  auto clamp = [](float & value) { value = std::max(-1.0F, std::min(1.0F, value)); };

  std::for_each(
    fields.potentialFieldX.data(), fields.potentialFieldX.data() + fields.potentialFieldX.size(),
    clamp);
  std::for_each(
    fields.potentialFieldY.data(), fields.potentialFieldY.data() + fields.potentialFieldY.size(),
    clamp);
}

void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_pose = *msg; }

Eigen::MatrixXf convert_occupancy_grid_to_eigen_matrix(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & occupancy_grid) const
{
  Eigen::MatrixXf gridmap = Eigen::MatrixXf::Zero(grid_y_int, grid_x_int);

  for (int y = 0; y < grid_y_int; ++y) {
    for (int x = 0; x < grid_x_int; ++x) {
      int index = y * grid_x_int + x;
      int8_t occupancy_value = occupancy_grid->data[index];

      if (occupancy_value == -1 || occupancy_value == 0) {
        gridmap(y, x) = 0.0F;
      } else {
        gridmap(y, x) = 1.0F;
      }
    }
  }
  return gridmap;
}

void arm_and_takeoff()
{
  while (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for arming service to become available");
  }
  while (!set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for set_mode service to become available");
  }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = "GUIDED";
  auto future = set_mode_client_->async_send_request(request);

  auto arming_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  arming_request->value = true;
  auto arming_future = arming_client_->async_send_request(arming_request);

  rclcpp::sleep_for(std::chrono::seconds(3));

  auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  takeoff_request->altitude = 1.5;
  takeoff_request->latitude = 0;
  takeoff_request->longitude = 0;
  takeoff_request->min_pitch = 0;
  takeoff_request->yaw = 0;
  auto takeoff_future = takeoff_client_->async_send_request(takeoff_request);

  rclcpp::sleep_for(std::chrono::seconds(5));
  RCLCPP_INFO(this->get_logger(), "Drone armed and takeoff");
}

void land_drone()
{
  while (!land_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for land service to become available");
  }

  auto land_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  land_request->altitude = 0;
  land_request->latitude = 0;
  land_request->longitude = 0;
  land_request->min_pitch = 0;
  land_request->yaw = 0;

  auto land_future = land_client_->async_send_request(land_request);

  RCLCPP_INFO(this->get_logger(), "Drone landing initiated");
}

void visualize_potential_field(
  const DirectionalPotentialFields & fields, int current_y, int current_x) const
{
  int rows = fields.potentialFieldX.rows();
  int cols = fields.potentialFieldX.cols();

  cv::Mat image(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
  int step_size = 10;

  for (int y = 0; y < rows; y += step_size) {
    for (int x = 0; x < cols; x += step_size) {
      float vec_x = fields.potentialFieldX(y, x);
      float vec_y = fields.potentialFieldY(y, x);

      cv::Point start_point(x, y);
      cv::Point end_point(x + static_cast<int>(vec_x * 10), y + static_cast<int>(vec_y * 10));

      cv::arrowedLine(image, start_point, end_point, cv::Scalar(255, 255, 255), 1, 8, 0, 0.3);
    }
  }

  cv::circle(image, cv::Point(current_x, current_y), 5, cv::Scalar(0, 0, 255), -1);

  int goal_x_int = static_cast<int>(goal_x);
  int goal_y_int = static_cast<int>(goal_y);
  cv::circle(image, cv::Point(goal_x_int, goal_y_int), 5, cv::Scalar(255, 0, 0), -1);

  cv::imshow("Potential Field Visualization", image);
  cv::waitKey(1);
}

void set_home_position()
{
  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->command = 179;  // MAV_CMD_DO_SET_HOME
  request->param1 = 1;     // Use current position

  // サービスが利用可能になるまで待機
  while (!sethome_cmd_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for set_home_position service to become available");
  }

  auto result = sethome_cmd_client_->async_send_request(request);

  // サービスの応答を待機
  if (
    rclcpp::spin_until_future_complete(shared_from_this(), result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Home position set successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set home position");
  }
  rclcpp::sleep_for(std::chrono::seconds(5));
}

}  // namespace path_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_planner::PathPlanner)
