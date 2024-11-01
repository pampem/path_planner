/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#ifndef ROTATIONAL_FORCE_HPP
#define ROTATIONAL_FORCE_HPP

#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>

/**
 * @brief ロボットの現在位置と目標位置に基づいて回転力を計算
 * 
 * @param position 現在のロボット位置
 * @param target 目標位置
 * @param gridmap 環境のGridMap
 * @return Eigen::Vector2f 計算された回転力ベクトル
 * @throw std::invalid_argument GridMapが無効な場合
 * @throw std::out_of_range Robotの位置がGridMapの範囲外の場合
 */
Eigen::Vector2f calculate_rotational_force(
  const Eigen::Vector2f & position, const Eigen::Vector2f & target,
  const nav_msgs::msg::OccupancyGrid & gridmap);

bool is_position_in_grid(
  const Eigen::Vector2f & position,
  const nav_msgs::msg::OccupancyGrid & gridmap);

#endif  // ROTATIONAL_FORCE_HPP
