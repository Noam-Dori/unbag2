//
// Created by Noam Dori on 06/09/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include "json_pipe.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/msg/odometry.hpp>

#ifndef UNBAG2_ODOM_PIPE_HPP
#define UNBAG2_ODOM_PIPE_HPP

namespace unbag2
{

/**
 * \brief a pipe specializing in converting Odometry messages to JSON files.
 */
class OdomPipe : public JsonPipe<nav_msgs::msg::Odometry>
{
public:
  /**
   * \brief construct a new Odometry pipe. figures.
   */
  OdomPipe();

protected:
  void load_json_params(rclcpp::Node * node) override;

  Json::Value to_json(nav_msgs::msg::Odometry msg) override;
private:
  bool covariance_ = false;
};
}

PLUGINLIB_EXPORT_CLASS(unbag2::OdomPipe, unbag2::Pipe) // NOLINT(cert-err58-cpp)

#endif //UNBAG2_GPS_PIPE_HPP
