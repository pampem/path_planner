/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#include "path_planner/rotational_force.hpp"

Eigen::Vector2f calculate_rotational_force(
  const Eigen::Vector2f & position, const Eigen::Vector2f & target,
  const nav_msgs::msg::OccupancyGrid & gridmap)
{
  Eigen::Vector2f force;
  // rotational forceの計算
  return force;
}
