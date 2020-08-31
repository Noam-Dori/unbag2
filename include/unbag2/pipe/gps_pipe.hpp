//
// Created by Noam Dori on 31/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include "../pipe.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#ifndef UNBAG2_GPS_PIPE_HPP
#define UNBAG2_GPS_PIPE_HPP

namespace unbag2
{
/**
 * \brief
 */
class GpsPipe : public PipeBase<sensor_msgs::msg::NavSatFix>
{
public:
  GpsPipe();

  void load_pipe_params(rclcpp::Node * node) override;

  void process(sensor_msgs::msg::NavSatFix msg, const std::string & topic) override;
};
}

PLUGINLIB_EXPORT_CLASS(unbag2::GpsPipe, unbag2::Pipe) // NOLINT(cert-err58-cpp)

#endif //UNBAG2_GPS_PIPE_HPP
