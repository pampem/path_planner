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
  // Validate grid dimensions
  if (gridmap.info.width == 0 || gridmap.info.height == 0) {
    throw std::invalid_argument("Invalid grid dimensions");
  }
  // Validate position is within grid bounds
  if (!is_position_in_grid(position, gridmap)) {
    throw std::out_of_range("Position is outside grid bounds");
  }

  Eigen::Vector2f force = Eigen::Vector2f::Zero();
  // TODO(izumita): Implement rotational force calculation
  return force;
}

bool is_position_in_grid(
  const Eigen::Vector2f & position, const nav_msgs::msg::OccupancyGrid & gridmap)
{
  int x_index =
    static_cast<int>((position.x() - gridmap.info.origin.position.x) / gridmap.info.resolution);
  int y_index =
    static_cast<int>((position.y() - gridmap.info.origin.position.y) / gridmap.info.resolution);

  return x_index >= 0 && x_index < static_cast<int>(gridmap.info.width) && y_index >= 0 &&
    y_index < static_cast<int>(gridmap.info.height);
}
