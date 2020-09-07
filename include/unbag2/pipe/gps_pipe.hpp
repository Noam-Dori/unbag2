//
// Created by Noam Dori on 31/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include "json_pipe.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#ifndef UNBAG2_GPS_PIPE_HPP
#define UNBAG2_GPS_PIPE_HPP

namespace unbag2
{
/**
 * \brief a pipe specializing in converting GPS messages to JSON files.
 */
class GpsPipe : public JsonPipe<sensor_msgs::msg::NavSatFix>
{
public:
  /**
   * \brief construct a new GPS pipe. figures.
   */
  GpsPipe();

protected:
  void load_json_params(rclcpp::Node * node) override;

  Json::Value to_json(sensor_msgs::msg::NavSatFix msg) override;
private:
  bool covariance_ = false;
};
}

PLUGINLIB_EXPORT_CLASS(unbag2::GpsPipe, unbag2::Pipe) // NOLINT(cert-err58-cpp)

#endif //UNBAG2_GPS_PIPE_HPP
