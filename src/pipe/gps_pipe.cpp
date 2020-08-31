//
// Created by Noam Dori on 31/08/2020.
// Copyright (c) 2020 Perfetto Modular Laboratory. All rights reserved.
//

#include <unbag2/pipe/gps_pipe.hpp>

namespace unbag2
{

GpsPipe::GpsPipe() : PipeBase<sensor_msgs::msg::NavSatFix>("gps_pipe")
{

}
void GpsPipe::load_pipe_params(rclcpp::Node * node)
{

}
void GpsPipe::process(sensor_msgs::msg::NavSatFix msg, const std::string & topic)
{
}
}