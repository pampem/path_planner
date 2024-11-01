/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#ifndef ROTATIONAL_FORCE_HPP
#define ROTATIONAL_FORCE_HPP

#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>

Eigen::Vector2f calculate_rotational_force(
  const Eigen::Vector2f & position, const Eigen::Vector2f & target,
  const nav_msgs::msg::OccupancyGrid & gridmap);

#endif  // ROTATIONAL_FORCE_HPP
