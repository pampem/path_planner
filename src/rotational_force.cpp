/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#include "path_planner/rotational_force.hpp"

Eigen::Vector2f calculate_rotational_force(
  const Eigen::Vector2f & position, const Eigen::Vector2f & target,
  const nav_msgs::msg::OccupancyGrid & gridmap)
{
  // Validate occupancy grid
  if (gridmap.data.empty()) {
    throw std::invalid_argument("Occupancy grid is empty");
  }

  Eigen::Vector2f force = Eigen::Vector2f::Zero();
  // TODO: Implement rotational force calculation
  return force;
}
